#!/usr/bin/env python3
"""
Reader for the initiator's USB (J20) stream: JSON text lines mixed with binary CIR frames.

Binary frame (little-endian), see Src/UWB/cir_capture.h:
  FF 'C' 'I' 'R' | n u16 | start u16 | seq u32 | fp_raw u32 | pp_raw u32 | ts u64 | n * (re 3B, im 3B)

Usage:
  pip install pyserial numpy
  python cir_reader.py --port COM5 --out cir_log.npz
Ctrl+C stops and saves. JSON result lines are printed; CIR frames are summarised.

Use as a library:  for kind, obj in parse_stream(byte_chunks): ...
  kind == "json": obj is the text line;  kind == "cir": obj is a dict (cir = complex64 array).
"""
import argparse, struct, sys
import numpy as np

MAGIC = b"\xffCIR"
HDR = struct.Struct("<4sHHIIIQHBBih26x")  # 64 bytes
assert HDR.size==64


def decode_samples(raw, n):
    a = np.frombuffer(raw, dtype=np.uint8, count=n * 6).reshape(n, 6).astype(np.int32)
    re = a[:, 0] | (a[:, 1] << 8) | (a[:, 2] << 16)
    im = a[:, 3] | (a[:, 4] << 8) | (a[:, 5] << 16)

    def s18(x):
        x &= 0x3FFFF
        return np.where(x & 0x20000, x - 0x40000, x)

    return (s18(re) + 1j * s18(im)).astype(np.complex64)


def parse_stream(chunks):
    buf = bytearray()
    for chunk in chunks:
        buf += chunk
        while True:
            i = buf.find(MAGIC)
            if i < 0:
                # no frame start: emit complete text lines, keep a possible partial magic
                nl = buf.rfind(b"\n")
                if nl >= 0:
                    for line in bytes(buf[:nl]).decode(errors="ignore").splitlines():
                        if line.strip():
                            yield "json", line.strip()
                    del buf[:nl + 1]
                break
            if i > 0:  # text before the frame
                for line in bytes(buf[:i]).decode(errors="ignore").splitlines():
                    if line.strip():
                        yield "json", line.strip()
                del buf[:i]
            if len(buf) < HDR.size:
                break
            _, n, start, seq, fp, pp, ts,addr, status, slot, dist_mm, rssi= HDR.unpack_from(buf)
            total = HDR.size + 6 * n
            if n == 0 or n > 1024:         # corrupt header: skip this magic and resync
                del buf[:1]
                continue
            if len(buf) < total:
                break
            cir = decode_samples(bytes(buf[HDR.size:total]), n)
            del buf[:total]
            yield "cir", dict(seq=seq, start=start, fp_raw=fp, pp_raw=pp, ts=ts, cir=cir)


def serial_chunks(port, baud):
    import serial
    with serial.Serial(port, baud, timeout=0.2) as s:
        while True:
            data = s.read(4096)
            if data:
                yield data


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200, help="ignored by USB CDC, kept for the driver")
    ap.add_argument("--out", default="cir_log.npz")
    args = ap.parse_args()

    seqs, starts, fps, tss, cirs = [], [], [], [], []
    try:
        for kind, obj in parse_stream(serial_chunks(args.port, args.baud)):
            if kind == "json":
                print(obj)
            else:
                seqs.append(obj["seq"]); starts.append(obj["start"]); fps.append(obj["fp_raw"])
                tss.append(obj["ts"]); cirs.append(obj["cir"])
                mag = np.abs(obj["cir"])
                print(f"CIR seq={obj['seq']} start={obj['start']} fp={obj['fp_raw'] / 64:.1f} "
                      f"peak_idx={obj['start'] + int(mag.argmax())} peak={mag.max():.0f}")
    except KeyboardInterrupt:
        pass
    if cirs:
        np.savez(args.out, seq=seqs, start=starts, fp_raw=fps, ts=tss, cir=np.array(cirs))
        print(f"saved {len(cirs)} CIRs to {args.out}")


if __name__ == "__main__":
    main()
