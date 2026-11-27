#!/usr/bin/env python3
"""
Live CIR magnitude plot, with distance and RSSI taken from the reported JSON block line.

Same plot as cir_plot.py, but the distance, status and RSSI shown for each responder come from the
ranging result line the initiator prints every round, not from the CIR frame header:

  {"Block":12, "results":[{"Addr":"0x0001","Status":"Ok","D_cm":523,...}, ...],
   "RSSI_dBm":"-81.3","NLOS_%":0}

The firmware prints that line first and then sends the round's CIR frames, so each CIR frame is
paired with the most recent block line. The CIR frame header is only used for the responder
address (to look up its D_cm) and the CIR window (start index, first path).

Note on RSSI: the block line has ONE "RSSI_dBm" value per round (diagnostics of the last frame the
initiator received), not one per responder. The same value is shown for every responder of that
round. It is only present with diagnostics on ("DIAG 1", which auto_start.c enables).

Frame (little-endian, 64-byte header):
  FF 'C' 'I' 'R' | n u16 | start u16 | seq u32 | fp_raw u32 | pp_raw u32 | ts u64 |
  responder u16 | status u8 | slot u8 | distance_mm i32 | rssi_cdbm i16 | reserved 26B |
  n * (re 3B, im 3B)   18-bit signed real/imag parts

Usage:
  pip install pyserial numpy matplotlib
  python cir_plot_block.py --port COM5                  # refresh every second
  python cir_plot_block.py --port COM5 --interval 2     # refresh every 2 seconds
  python cir_plot_block.py --port COM5 --align-fp       # x axis relative to each frame's first path
  python cir_plot_block.py --port COM5 --db             # magnitude in dB
  python cir_plot_block.py --file capture.bin           # replay a raw stream saved to disk
Close the plot window (or Ctrl+C) to stop.
"""
import argparse
import json
import struct
import threading
import time

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ------------------------------------------------------------------------------------------------
#  CIR frame parsing
# ------------------------------------------------------------------------------------------------

MAGIC = b"\xffCIR"
HDR = struct.Struct("<4sHHIIIQHBBih26x")  # 64 bytes, last 26 reserved
assert HDR.size == 64

RESPONDER_UNKNOWN = 0xFFFF


def to_complex(raw, n):
    """n samples of (real 3B LE, imag 3B LE), 18-bit signed -> complex64 array."""
    a = np.frombuffer(raw, dtype=np.uint8, count=n * 6).reshape(n, 6).astype(np.int32)
    re = a[:, 0] | (a[:, 1] << 8) | (a[:, 2] << 16)
    im = a[:, 3] | (a[:, 4] << 8) | (a[:, 5] << 16)

    def s18(x):
        x &= 0x3FFFF
        return np.where(x & 0x20000, x - 0x40000, x)

    return (s18(re) + 1j * s18(im)).astype(np.complex64)


def parse_stream(chunks):
    """Yield ("text", line) for each text line and ("cir", dict) for each CIR frame, in stream order."""
    buf = bytearray()

    def lines_of(raw):
        for line in bytes(raw).decode(errors="ignore").splitlines():
            if line.strip():
                yield "text", line.strip()

    for chunk in chunks:
        buf += chunk
        while True:
            i = buf.find(MAGIC)
            if i < 0:
                nl = buf.rfind(b"\n")        # keep a possible partial magic after the last line
                if nl >= 0:
                    yield from lines_of(buf[:nl])
                    del buf[:nl + 1]
                break
            if i > 0:
                yield from lines_of(buf[:i])
                del buf[:i]
            if len(buf) < HDR.size:
                break
            (_, n, start, seq, fp, pp, ts, addr, _st, _slot, _dist, _rssi) = HDR.unpack_from(buf)
            if n == 0 or n > 1024:           # corrupt header: skip this magic and resync
                del buf[:1]
                continue
            total = HDR.size + 6 * n
            if len(buf) < total:
                break
            cir = to_complex(bytes(buf[HDR.size:total]), n)
            del buf[:total]
            yield "cir", dict(seq=seq, start=start, fp_raw=fp, pp_raw=pp, ts=ts,
                              responder=None if addr == RESPONDER_UNKNOWN else addr, cir=cir)


# ------------------------------------------------------------------------------------------------
#  Block line parsing
# ------------------------------------------------------------------------------------------------

def parse_block(line):
    """Return the round's info from a {"Block":...} line, or None if the line is something else.

    {"block": int, "rssi_dbm": float or None, "nlos_pct": int or None,
     "results": {addr: {"status": "Ok"/"Err", "distance_m": float or None}}}
    """
    if not line.startswith("{"):
        return None
    try:
        msg = json.loads(line)
    except json.JSONDecodeError:
        return None
    if not isinstance(msg, dict) or "Block" not in msg:
        return None

    results = {}
    for r in msg.get("results", []):
        try:
            addr = int(r["Addr"], 16)
        except (KeyError, TypeError, ValueError):
            continue
        d_cm = r.get("D_cm")
        ok = r.get("Status") == "Ok" and d_cm is not None
        results[addr] = dict(status=r.get("Status", "?"), distance_m=d_cm / 100.0 if ok else None)

    try:
        rssi = float(msg.get("RSSI_dBm"))    # printed as a string: "-81.3" or "Invalid"
    except (TypeError, ValueError):
        rssi = None
    nlos = msg.get("NLOS_%")
    return dict(block=msg["Block"], rssi_dbm=rssi, nlos_pct=nlos, results=results)


# ------------------------------------------------------------------------------------------------
#  Input sources
# ------------------------------------------------------------------------------------------------

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


# ------------------------------------------------------------------------------------------------
#  Reader thread + plot
# ------------------------------------------------------------------------------------------------

class CirStore:
    """Latest CIR per responder (with its block info), shared between the reader thread and the plot."""

    def __init__(self):
        self.lock = threading.Lock()
        self.latest = {}        # responder (int, or None if unmatched) -> frame dict
        self.n_frames = 0
        self.n_blocks = 0
        self.error = None

    def put(self, frame):
        with self.lock:
            self.latest[frame["responder"]] = frame
            self.n_frames += 1

    def snapshot(self):
        with self.lock:
            return dict(self.latest), self.n_frames, self.n_blocks, self.error


def reader_thread(chunks, store, show_json):
    block = None   # most recent block line; the CIR frames that follow belong to it
    try:
        for kind, obj in parse_stream(chunks):
            if kind == "text":
                if show_json:
                    print(obj)
                b = parse_block(obj)
                if b is not None:
                    block = b
                    with store.lock:
                        store.n_blocks += 1
                continue

            # CIR frame: obj["cir"] is a complex64 array. Take distance / RSSI from the block line.
            res = block["results"].get(obj["responder"]) if block else None
            obj["block"] = block["block"] if block else None
            obj["status"] = res["status"] if res else None
            obj["distance_m"] = res["distance_m"] if res else None
            obj["rssi_dbm"] = block["rssi_dbm"] if block else None
            obj["nlos_pct"] = block["nlos_pct"] if block else None
            store.put(obj)
    except Exception as e:  # e.g. serial port unplugged
        with store.lock:
            store.error = e


def label_for(f):
    who = "unmatched" if f["responder"] is None else f"0x{f['responder']:04x}"
    if f["distance_m"] is not None:
        dist = f"{f['distance_m']:.2f} m"
    elif f["status"] is not None:
        dist = f["status"]                  # e.g. "Err"
    else:
        dist = "--"
    rssi = "--" if f["rssi_dbm"] is None else f"{f['rssi_dbm']:.1f} dBm"
    blk = "" if f["block"] is None else f"  [blk {f['block']}]"
    return f"{who}  d={dist}  RSSI={rssi}{blk}"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="serial port of the initiator's J20 USB, e.g. COM5")
    src.add_argument("--file", help="replay a raw byte stream saved to disk")
    ap.add_argument("--baud", type=int, default=115200, help="ignored by USB CDC, kept for the driver")
    ap.add_argument("--interval", type=float, default=1.0, help="plot refresh period in seconds (default 1)")
    ap.add_argument("--align-fp", action="store_true", help="x axis = sample index minus first-path index")
    ap.add_argument("--db", action="store_true", help="plot 20*log10(|CIR|) instead of |CIR|")
    ap.add_argument("--show-json", action="store_true", help="also print the text / JSON lines")
    args = ap.parse_args()

    chunks = serial_chunks(args.port, args.baud) if args.port else file_chunks(args.file)
    store = CirStore()
    threading.Thread(target=reader_thread, args=(chunks, store, args.show_json), daemon=True).start()

    fig, ax = plt.subplots(figsize=(11, 6))
    fig.canvas.manager.set_window_title("DWM3001CDK CIR magnitude (distance/RSSI from block line)")
    ax.set_xlabel("sample - first path" if args.align_fp else "CIR sample index")
    ax.set_ylabel("|CIR| (dB)" if args.db else "|CIR|")
    ax.grid(True, alpha=0.3)
    title = ax.set_title("waiting for CIR frames...")

    lines = {}       # responder -> Line2D
    fp_marks = {}    # responder -> vertical first-path line
    t0 = time.time()

    def update(_):
        latest, n_frames, n_blocks, err = store.snapshot()
        if err is not None:
            title.set_text(f"reader stopped: {err}")
            return
        if not latest:
            return

        nlos = None
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
            if f["nlos_pct"] is not None:
                nlos = f["nlos_pct"]

        ax.relim()
        ax.autoscale_view()
        ax.legend(loc="upper right", fontsize=9)
        extra = "" if nlos is None else f"  |  NLOS {nlos}%"
        title.set_text(f"latest CIR per responder  |  {n_frames} CIRs, {n_blocks} blocks in "
                       f"{time.time() - t0:.0f} s{extra}  (dashed = first path)")

    anim = FuncAnimation(fig, update, interval=max(args.interval, 0.1) * 1000, cache_frame_data=False)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    del anim


if __name__ == "__main__":
    main()
