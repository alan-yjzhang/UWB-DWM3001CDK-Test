#!/usr/bin/env python3
"""
Reader for the initiator's USB (J20) stream: JSON text lines mixed with binary CIR frames.

Binary frame (little-endian, 40-byte header), see Src/UWB/cir_capture.h:
  FF 'C' 'I' 'R' | n u16 | start u16 | seq u32 | fp_raw u32 | pp_raw u32 | ts u64 |
  responder u16 | status u8 | slot u8 | distance_mm i32 | rssi_cdbm i16 | reserved u16 |
  n * (re 3B, im 3B)

  responder  short address of the anchor (0xFFFF = could not be matched)
  status     0 = ranging OK, 0xFF = no ranging result, other = stack error code
  distance   mm, only valid when status == 0 (INT32_MIN otherwise)
  rssi       0.01 dBm (INT16_MIN when diagnostics are off)

Usage:
  pip install pyserial numpy
  python cir_reader.py --port COM5 --out cir_log.npz
  python cir_reader.py --file capture.bin           (replay a raw stream saved to disk)
Ctrl+C stops and saves. JSON result lines are printed; CIR frames are summarised.

Use as a library:  for kind, obj in parse_stream(byte_chunks): ...
  kind == "json": obj is the text line;  kind == "cir": obj is a dict (cir = complex64 array,
  distance_m / rssi_dbm are None when not available).
"""
import argparse, struct, sys
import numpy as np

MAGIC = b"\xffCIR"
HDR = struct.Struct("<4sHHIIIQHBBihH")  # 40 bytes
assert HDR.size == 40

RESPONDER_UNKNOWN = 0xFFFF
STATUS_NO_RESULT = 0xFF
DISTANCE_INVALID = -2**31
RSSI_INVALID = -2**15


def decode_samples(raw, n):
    a = np.frombuffer(raw, dtype=np.uint8, count=n * 6).reshape(n, 6).astype(np.int32)
    re = a[:, 0] | (a[:, 1] << 8) | (a[:, 2] << 16)
    im = a[:, 3] | (a[:, 4] << 8) | (a[:, 5] << 16)

    def s18(x):
        x &= 0x3FFFF
        return np.where(x & 0x20000, x - 0x40000, x)

    return (s18(re) + 1j * s18(im)).astype(np.complex64)


def _text_lines(raw):
    for line in bytes(raw).decode(errors="ignore").splitlines():
        if line.strip():
            yield "json", line.strip()


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
                    yield from _text_lines(buf[:nl])
                    del buf[:nl + 1]
                break
            if i > 0:  # text before the frame
                yield from _text_lines(buf[:i])
                del buf[:i]
            if len(buf) < HDR.size:
                break
            (_, n, start, seq, fp, pp, ts,
             addr, status, slot, dist_mm, rssi_cdbm, _res) = HDR.unpack_from(buf)
            total = HDR.size + 6 * n
            if n == 0 or n > 1024:         # corrupt header: skip this magic and resync
                del buf[:1]
                continue
            if len(buf) < total:
                break
            cir = decode_samples(bytes(buf[HDR.size:total]), n)
            del buf[:total]
            yield "cir", dict(
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


def main():
    ap = argparse.ArgumentParser()
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="serial port of the initiator's J20 USB, e.g. COM5")
    src.add_argument("--file", help="replay a raw byte stream saved to disk")
    ap.add_argument("--baud", type=int, default=115200, help="ignored by USB CDC, kept for the driver")
    ap.add_argument("--out", default="cir_log.npz")
    args = ap.parse_args()

    chunks = serial_chunks(args.port, args.baud) if args.port else file_chunks(args.file)

    nan = float("nan")
    log = {k: [] for k in ("seq", "start", "fp_raw", "ts", "responder", "status",
                           "distance_m", "rssi_dbm", "cir")}
    try:
        for kind, obj in parse_stream(chunks):
            if kind == "json":
                print(obj)
                continue
            log["seq"].append(obj["seq"]); log["start"].append(obj["start"])
            log["fp_raw"].append(obj["fp_raw"]); log["ts"].append(obj["ts"])
            log["responder"].append(-1 if obj["responder"] is None else obj["responder"])
            log["status"].append(obj["status"])
            log["distance_m"].append(nan if obj["distance_m"] is None else obj["distance_m"])
            log["rssi_dbm"].append(nan if obj["rssi_dbm"] is None else obj["rssi_dbm"])
            log["cir"].append(obj["cir"])

            mag = np.abs(obj["cir"])
            who = "?" if obj["responder"] is None else f"0x{obj['responder']:04x}"
            dist = "--" if obj["distance_m"] is None else f"{obj['distance_m']:.3f} m"
            rssi = "--" if obj["rssi_dbm"] is None else f"{obj['rssi_dbm']:.1f} dBm"
            print(f"CIR seq={obj['seq']} resp={who} status={obj['status']} d={dist} rssi={rssi} "
                  f"fp={obj['fp_raw'] / 64:.1f} peak_idx={obj['start'] + int(mag.argmax())} "
                  f"peak={mag.max():.0f}")
    except KeyboardInterrupt:
        pass
    if log["cir"]:
        np.savez(args.out, **{k: np.array(v) for k, v in log.items()})
        print(f"saved {len(log['cir'])} CIRs to {args.out}")


if __name__ == "__main__":
    main()
