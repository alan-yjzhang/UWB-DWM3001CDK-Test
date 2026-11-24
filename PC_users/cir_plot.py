#!/usr/bin/env python3
"""
Live CIR magnitude plot for the initiator's USB (J20) stream.

Reads the binary CIR frames (see Src/UWB/cir_capture.h) from the USB serial port, decodes the
samples into complex numbers, and plots |CIR| of the latest frame from each responder. The plot
is refreshed every --interval seconds (default 1 s). This script is self-contained (it does not
import cir_reader.py).

Frame (little-endian, 64-byte header):
  FF 'C' 'I' 'R' | n u16 | start u16 | seq u32 | fp_raw u32 | pp_raw u32 | ts u64 |
  responder u16 | status u8 | slot u8 | distance_mm i32 | rssi_cdbm i16 | reserved 26B |
  n * (re 3B, im 3B)   18-bit signed real/imag parts

Usage:
  pip install pyserial numpy matplotlib
  python cir_plot.py --port COM5                  # refresh every second
  python cir_plot.py --port COM5 --interval 2     # refresh every 2 seconds
  python cir_plot.py --port COM5 --align-fp       # x axis relative to each frame's first path
  python cir_plot.py --port COM5 --db             # magnitude in dB
  python cir_plot.py --file capture.bin           # replay a raw stream saved to disk
Close the plot window (or Ctrl+C) to stop.
"""
import argparse
import struct
import threading
import time

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ------------------------------------------------------------------------------------------------
#  Frame parsing
# ------------------------------------------------------------------------------------------------

MAGIC = b"\xffCIR"
HDR = struct.Struct("<4sHHIIIQHBBih26x")  # 64 bytes, last 26 reserved
assert HDR.size == 64

RESPONDER_UNKNOWN = 0xFFFF
DISTANCE_INVALID = -2**31
RSSI_INVALID = -2**15


def to_complex(raw, n):
    """n samples of (real 3B LE, imag 3B LE), 18-bit signed -> complex64 array."""
    a = np.frombuffer(raw, dtype=np.uint8, count=n * 6).reshape(n, 6).astype(np.int32)
    re = a[:, 0] | (a[:, 1] << 8) | (a[:, 2] << 16)
    im = a[:, 3] | (a[:, 4] << 8) | (a[:, 5] << 16)

    def s18(x):
        x &= 0x3FFFF
        return np.where(x & 0x20000, x - 0x40000, x)

    return (s18(re) + 1j * s18(im)).astype(np.complex64)


def parse_frames(chunks):
    """Yield one dict per CIR frame found in the byte stream; JSON text between frames is yielded as str."""
    buf = bytearray()
    for chunk in chunks:
        buf += chunk
        while True:
            i = buf.find(MAGIC)
            if i < 0:
                nl = buf.rfind(b"\n")        # keep a possible partial magic after the last line
                if nl >= 0:
                    yield bytes(buf[:nl]).decode(errors="ignore")
                    del buf[:nl + 1]
                break
            if i > 0:
                yield bytes(buf[:i]).decode(errors="ignore")
                del buf[:i]
            if len(buf) < HDR.size:
                break
            (_, n, start, seq, fp, pp, ts, addr, status, slot, dist_mm, rssi_cdbm) = HDR.unpack_from(buf)
            if n == 0 or n > 1024:           # corrupt header: skip this magic and resync
                del buf[:1]
                continue
            total = HDR.size + 6 * n
            if len(buf) < total:
                break
            cir = to_complex(bytes(buf[HDR.size:total]), n)
            del buf[:total]
            yield dict(
                seq=seq, start=start, fp_raw=fp, pp_raw=pp, ts=ts,
                responder=None if addr == RESPONDER_UNKNOWN else addr,
                status=status, slot=slot,
                distance_m=dist_mm / 1000.0 if (status == 0 and dist_mm != DISTANCE_INVALID) else None,
                rssi_dbm=rssi_cdbm / 100.0 if rssi_cdbm != RSSI_INVALID else None,
                cir=cir)


def serial_chunks(port, baud):
    import serial
    with serial.Serial(port, baud, timeout=0.2) as s:
        while True:
            data = s.read(4096)
            if data:
                yield data


def file_chunks(path, size=4096):
    with open(path, "rb") as f:
        while True:
            data = f.read(size)
            if not data:
                return
            yield data


class CirStore:
    """Latest CIR per responder, shared between the reader thread and the plot."""

    def __init__(self):
        self.lock = threading.Lock()
        self.latest = {}        # responder (int, or None if unmatched) -> frame dict
        self.n_frames = 0
        self.error = None

    def put(self, frame):
        with self.lock:
            self.latest[frame["responder"]] = frame
            self.n_frames += 1

    def snapshot(self):
        with self.lock:
            return dict(self.latest), self.n_frames, self.error


def reader_thread(chunks, store, show_json):
    try:
        for item in parse_frames(chunks):
            if isinstance(item, str):
                if show_json:
                    for line in item.splitlines():
                        if line.strip():
                            print(line.strip())
                continue
            # item["cir"] is already a complex64 numpy array (real + 1j*imag)
            store.put(item)
    except Exception as e:  # e.g. serial port unplugged
        with store.lock:
            store.error = e


def label_for(frame):
    who = "unmatched" if frame["responder"] is None else f"0x{frame['responder']:04x}"
    dist = "--" if frame["distance_m"] is None else f"{frame['distance_m']:.2f} m"
    rssi = "--" if frame["rssi_dbm"] is None else f"{frame['rssi_dbm']:.1f} dBm"
    return f"{who}  d={dist}  RSSI={rssi}"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="serial port of the initiator's J20 USB, e.g. COM5")
    src.add_argument("--file", help="replay a raw byte stream saved to disk")
    ap.add_argument("--baud", type=int, default=115200, help="ignored by USB CDC, kept for the driver")
    ap.add_argument("--interval", type=float, default=1.0, help="plot refresh period in seconds (default 1)")
    ap.add_argument("--align-fp", action="store_true", help="x axis = sample index minus first-path index")
    ap.add_argument("--db", action="store_true", help="plot 20*log10(|CIR|) instead of |CIR|")
    ap.add_argument("--show-json", action="store_true", help="also print the JSON ranging lines")
    args = ap.parse_args()

    chunks = serial_chunks(args.port, args.baud) if args.port else file_chunks(args.file)
    store = CirStore()
    threading.Thread(target=reader_thread, args=(chunks, store, args.show_json), daemon=True).start()

    fig, ax = plt.subplots(figsize=(11, 6))
    fig.canvas.manager.set_window_title("DWM3001CDK CIR magnitude")
    ax.set_xlabel("sample - first path" if args.align_fp else "CIR sample index")
    ax.set_ylabel("|CIR| (dB)" if args.db else "|CIR|")
    ax.grid(True, alpha=0.3)
    title = ax.set_title("waiting for CIR frames...")

    lines = {}       # responder -> Line2D
    fp_marks = {}    # responder -> vertical first-path line
    t0 = time.time()

    def update(_):
        latest, n_frames, err = store.snapshot()
        if err is not None:
            title.set_text(f"reader stopped: {err}")
            return
        if not latest:
            return

        for resp in sorted(latest, key=lambda r: (r is None, r or 0)):
            f = latest[resp]
            mag = np.abs(f["cir"])                       # magnitude of the complex samples
            if args.db:
                mag = 20 * np.log10(np.maximum(mag, 1.0))
            fp = f["fp_raw"] / 64.0                     # 10.6 fixed point -> sample index
            x = np.arange(len(mag)) + f["start"]
            if args.align_fp:
                x = x - fp
                fp_x = 0.0
            else:
                fp_x = fp

            if resp not in lines:
                (lines[resp],) = ax.plot(x, mag, lw=1.0)
                fp_marks[resp] = ax.axvline(fp_x, ls="--", lw=0.8, color=lines[resp].get_color())
            lines[resp].set_data(x, mag)
            lines[resp].set_label(label_for(f))
            fp_marks[resp].set_xdata([fp_x, fp_x])

        ax.relim()
        ax.autoscale_view()
        ax.legend(loc="upper right", fontsize=9)
        title.set_text(f"latest CIR per responder  |  {n_frames} frames in {time.time() - t0:.0f} s"
                       f"  (dashed = first path)")

    anim = FuncAnimation(fig, update, interval=max(args.interval, 0.1) * 1000, cache_frame_data=False)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    del anim


if __name__ == "__main__":
    main()
