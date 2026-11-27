#!/usr/bin/env python3
"""
Live CIR magnitude plot, with distance and RSSI taken from the reported JSON block lines.

The initiator prints one result line per ranging round:

  {"Block":12, "results":[{"Addr":"0x0001","Status":"Ok","D_cm":523,...}, ...],
   "RSSI_dBm":"-81.3","NLOS_%":0}

and, after some of those lines, a binary CIR frame. Not every block line is followed by a CIR
frame, so the two are handled separately:

  - Distances, status and RSSI always come from the MOST RECENT block line, whether or not a CIR
    came with it. They are shown in the text panel and refreshed every update.
  - The CIR curve is the most recent CIR frame. The title says which block line it followed and
    how many block lines have arrived since, so you can see how old the CIR is.
  - A CIR frame with the same seq number as the previous one (the firmware re-sent the buffer
    without a new capture) is counted as a repeat, not as a new CIR.

The CIR frame header is only used for the CIR window (start index, first path) and seq; its
reserved bytes are ignored.

Note on RSSI: the block line has ONE "RSSI_dBm" value per round (diagnostics of the last frame the
initiator received), not one per responder. It is only present with diagnostics on ("DIAG 1").

Frame (little-endian, 64-byte header):
  FF 'C' 'I' 'R' | n u16 | start u16 | seq u32 | fp_raw u32 | pp_raw u32 | ts u64 | reserved 36B |
  n * (re 3B, im 3B)   18-bit signed real/imag parts

Usage:
  pip install pyserial numpy matplotlib
  python cir_plot_block.py --port COM5                  # refresh every second
  python cir_plot_block.py --port COM5 --interval 2     # refresh every 2 seconds
  python cir_plot_block.py --port COM5 --align-fp       # x axis relative to the first path
  python cir_plot_block.py --port COM5 --db             # magnitude in dB
  python cir_plot_block.py --port COM5 --history 5      # also show the 4 previous CIRs, faded
  python cir_plot_block.py --file capture.bin           # replay a raw stream saved to disk
Close the plot window (or Ctrl+C) to stop.
"""
import argparse
import collections
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
HDR = struct.Struct("<4sHHIIIQ36x")  # 64 bytes, last 36 reserved
assert HDR.size == 64


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
            (_, n, start, seq, fp, pp, ts) = HDR.unpack_from(buf)
            if n == 0 or n > 1024:           # corrupt header: skip this magic and resync
                del buf[:1]
                continue
            total = HDR.size + 6 * n
            if len(buf) < total:
                break
            cir = to_complex(bytes(buf[HDR.size:total]), n)
            del buf[:total]
            yield "cir", dict(seq=seq, start=start, fp_raw=fp, pp_raw=pp, ts=ts, cir=cir)


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
    return dict(block=msg["Block"], rssi_dbm=rssi, nlos_pct=msg.get("NLOS_%"), results=results)


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
#  Reader thread
# ------------------------------------------------------------------------------------------------

class Store:
    """State shared between the reader thread and the plot."""

    def __init__(self, history):
        self.lock = threading.Lock()
        self.block = None                                  # latest block line (parsed)
        self.n_blocks = 0
        self.cirs = collections.deque(maxlen=history)      # latest CIR frames, newest last
        self.n_cir = 0                                     # new CIRs (distinct seq)
        self.n_repeat = 0                                  # re-sent CIRs (same seq as before)
        self.error = None

    def snapshot(self):
        with self.lock:
            return dict(block=self.block, n_blocks=self.n_blocks, cirs=list(self.cirs),
                        n_cir=self.n_cir, n_repeat=self.n_repeat, error=self.error)


def reader_thread(chunks, store, show_json):
    try:
        for kind, obj in parse_stream(chunks):
            if kind == "text":
                if show_json:
                    print(obj)
                b = parse_block(obj)
                if b is not None:
                    with store.lock:
                        store.block = b
                        store.n_blocks += 1
                continue

            # CIR frame (obj["cir"] is a complex64 array). Remember which block line it followed.
            with store.lock:
                obj["after_block"] = store.block["block"] if store.block else None
                obj["after_n_blocks"] = store.n_blocks
                if store.cirs and store.cirs[-1]["seq"] == obj["seq"]:
                    store.n_repeat += 1
                    store.cirs[-1] = obj                   # same capture re-sent: just refresh
                else:
                    store.n_cir += 1
                    store.cirs.append(obj)
    except Exception as e:  # e.g. serial port unplugged
        with store.lock:
            store.error = e


# ------------------------------------------------------------------------------------------------
#  Plot
# ------------------------------------------------------------------------------------------------

def block_text(b):
    if b is None:
        return "no block line yet"
    rows = [f"Block {b['block']}"]
    for addr in sorted(b["results"]):
        r = b["results"][addr]
        d = f"{r['distance_m']:7.2f} m" if r["distance_m"] is not None else f"{r['status']:>9}"
        rows.append(f"0x{addr:04x} {d}")
    rows.append("RSSI   " + ("--" if b["rssi_dbm"] is None else f"{b['rssi_dbm']:.1f} dBm"))
    if b["nlos_pct"] is not None:
        rows.append(f"NLOS   {b['nlos_pct']} %")
    return "\n".join(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="serial port of the initiator's J20 USB, e.g. COM5")
    src.add_argument("--file", help="replay a raw byte stream saved to disk")
    ap.add_argument("--baud", type=int, default=115200, help="ignored by USB CDC, kept for the driver")
    ap.add_argument("--interval", type=float, default=1.0, help="plot refresh period in seconds (default 1)")
    ap.add_argument("--align-fp", action="store_true", help="x axis = sample index minus first-path index")
    ap.add_argument("--db", action="store_true", help="plot 20*log10(|CIR|) instead of |CIR|")
    ap.add_argument("--history", type=int, default=1, help="number of recent CIRs to draw (default 1)")
    ap.add_argument("--show-json", action="store_true", help="also print the text / JSON lines")
    args = ap.parse_args()

    chunks = serial_chunks(args.port, args.baud) if args.port else file_chunks(args.file)
    store = Store(max(1, args.history))
    threading.Thread(target=reader_thread, args=(chunks, store, args.show_json), daemon=True).start()

    fig, ax = plt.subplots(figsize=(12, 6))
    fig.subplots_adjust(right=0.78)
    fig.canvas.manager.set_window_title("DWM3001CDK CIR magnitude (distance/RSSI from block line)")
    ax.set_xlabel("sample - first path" if args.align_fp else "CIR sample index")
    ax.set_ylabel("|CIR| (dB)" if args.db else "|CIR|")
    ax.grid(True, alpha=0.3)
    title = ax.set_title("waiting for data...")
    info = fig.text(0.80, 0.88, "", va="top", ha="left", family="monospace", fontsize=10,
                    bbox=dict(boxstyle="round", fc="#f4f4f4", ec="#999999"))
    t0 = time.time()

    def update(_):
        s = store.snapshot()
        info.set_text(block_text(s["block"]))
        if s["error"] is not None:
            title.set_text(f"reader stopped: {s['error']}")
            return
        if not s["cirs"]:
            title.set_text(f"{s['n_blocks']} block lines, no CIR frame yet")
            return

        for ln in list(ax.lines):
            ln.remove()
        k = len(s["cirs"])
        for i, f in enumerate(s["cirs"]):
            newest = (i == k - 1)
            mag = np.abs(f["cir"])                       # magnitude of the complex samples
            if args.db:
                mag = 20 * np.log10(np.maximum(mag, 1.0))
            fp = f["fp_raw"] / 64.0                     # 10.6 fixed point -> sample index
            x = np.arange(len(mag)) + f["start"]
            fp_x = fp
            if args.align_fp:
                x = x - fp
                fp_x = 0.0
            if newest:
                ax.plot(x, mag, lw=1.2, color="C0", label=f"CIR seq {f['seq']}")
                ax.axvline(fp_x, ls="--", lw=0.8, color="C3", label="first path")
            else:
                ax.plot(x, mag, lw=0.8, color="gray", alpha=0.2 + 0.5 * (i + 1) / k)

        ax.relim()
        ax.autoscale_view()
        ax.legend(loc="upper right", fontsize=9)
        f = s["cirs"][-1]
        age = s["n_blocks"] - f["after_n_blocks"]
        after = "before any block line" if f["after_block"] is None else f"after block {f['after_block']}"
        title.set_text(f"CIR seq {f['seq']} ({after}, {age} block lines since)  |  "
                       f"{s['n_cir']} CIRs, {s['n_repeat']} repeats, {s['n_blocks']} blocks in "
                       f"{time.time() - t0:.0f} s")

    anim = FuncAnimation(fig, update, interval=max(args.interval, 0.1) * 1000, cache_frame_data=False)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    del anim


if __name__ == "__main__":
    main()
