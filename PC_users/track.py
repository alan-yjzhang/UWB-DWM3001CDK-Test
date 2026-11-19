#!/usr/bin/env python3
"""
UWB tag tracker for the Uberi DWM3001CDK demo firmware.

The tag board runs INITF (initiator) and is attached to this computer over USB (J20).
Anchors run RESPF and only need power. The tag prints one JSON line per ranging block:

  {"Block":12, "results":[{"Addr":"0x0001","Status":"Ok","D_cm":523,...},
                          {"Addr":"0x0002","Status":"Ok","D_cm":871,...}]}

This script reads those lines, drops bad ranges, and solves the tag position by
least squares against anchor coordinates you supply in anchors.json.

Usage:
  pip install pyserial numpy
  python track.py --port COM5 --anchors anchors.json [--log track.csv]
  python track.py --simulate --anchors anchors.json      # no hardware needed
"""
import argparse, csv, json, sys, time
import numpy as np


def load_anchors(path):
    """anchors.json: {"0x0001": [x, y, z], "0x0002": [x, y, z], ...}  (meters)"""
    with open(path) as f:
        raw = json.load(f)
    return {int(k, 16): np.array(v, float) for k, v in raw.items()}


def parse_line(line, max_nlos=None):
    """Return {addr_int: dist_m} for good measurements, or None if not a result line."""
    line = line.strip()
    if not line.startswith("{"):
        return None
    try:
        msg = json.loads(line)
    except json.JSONDecodeError:
        return None
    if "results" not in msg:
        return None
    # Block-level NLOS (only present if diagnostics are enabled on the tag)
    nlos = msg.get("NLOS_%")
    if max_nlos is not None and nlos is not None and nlos > max_nlos:
        return {}
    out = {}
    for r in msg["results"]:
        if r.get("Status") != "Ok" or "D_cm" not in r:
            continue
        out[int(r["Addr"], 16)] = r["D_cm"] / 100.0
    return out


def solve(anchors, ranges, x0=None, min_anchors=4, z_fixed=None):
    """
    Least-squares position. Robust loss (soft_l1) reduces the effect of NLOS outliers.
    If fewer than min_anchors ranges are available returns None.
    z_fixed: if set, solve only x,y at that height (use when anchors are coplanar).
    """
    ids = [a for a in ranges if a in anchors]
    if len(ids) < (3 if z_fixed is not None else min_anchors):
        return None
    A = np.array([anchors[a] for a in ids])
    d = np.array([ranges[a] for a in ids])
    if x0 is None:
        x0 = A.mean(axis=0)

    pos = x0.astype(float).copy()
    if z_fixed is not None:
        pos[2] = z_fixed
    dims = [0, 1] if z_fixed is not None else [0, 1, 2]
    # Iteratively reweighted Gauss-Newton (Huber-style weights) -> robust to NLOS outliers
    for _ in range(30):
        diff = pos - A
        rng_ = np.linalg.norm(diff, axis=1)
        rng_[rng_ < 1e-6] = 1e-6
        r = rng_ - d
        J = (diff / rng_[:, None])[:, dims]
        wts = np.where(np.abs(r) <= 0.3, 1.0, 0.3 / np.abs(r))
        step, *_ = np.linalg.lstsq(J * wts[:, None], -r * wts, rcond=None)
        pos[dims] += step
        if np.linalg.norm(step) < 1e-4:
            break
    resid = np.linalg.norm(A - pos, axis=1) - d
    rms = float(np.sqrt(np.mean(resid ** 2)))
    return pos, rms, len(ids)


def reject_outliers(anchors, ranges, pos, thresh=0.75):
    """Drop ranges whose residual against the current fit is large, if enough anchors remain."""
    if pos is None or len(ranges) <= 4:
        return ranges
    resid = {a: abs(np.linalg.norm(anchors[a] - pos) - d) for a, d in ranges.items() if a in anchors}
    worst = max(resid, key=resid.get)
    if resid[worst] > thresh:
        return {a: d for a, d in ranges.items() if a != worst}
    return ranges


def lines_from_serial(port, baud):
    import serial
    with serial.Serial(port, baud, timeout=1) as s:
        while True:
            raw = s.readline()
            if raw:
                yield raw.decode(errors="ignore")


def lines_simulated(anchors, noise=0.05):
    rng = np.random.default_rng(0)
    t = 0.0
    block = 0
    while True:
        true = np.array([2 + 1.5 * np.cos(t), 1 + 1.0 * np.sin(t), 1.2])
        res = []
        for a, p in anchors.items():
            dist = np.linalg.norm(p - true) + rng.normal(0, noise)
            if rng.random() < 0.05:           # occasional NLOS-style bias
                dist += rng.uniform(0.8, 2.0)
            res.append({"Addr": f"0x{a:04x}", "Status": "Ok", "D_cm": int(dist * 100)})
        yield json.dumps({"Block": block, "results": res}) + "\n"
        block += 1
        t += 0.1
        time.sleep(0.2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--anchors", required=True)
    ap.add_argument("--log")
    ap.add_argument("--simulate", action="store_true")
    ap.add_argument("--max-nlos", type=float, default=None,
                    help="discard blocks whose NLOS_%% exceeds this (needs diagnostics enabled)")
    ap.add_argument("--z", type=float, default=None,
                    help="fix tag height (m) and solve only x,y; use if anchors are coplanar")
    args = ap.parse_args()

    anchors = load_anchors(args.anchors)
    if args.simulate:
        src = lines_simulated(anchors)
    elif args.port:
        src = lines_from_serial(args.port, args.baud)
    else:
        sys.exit("give --port COMx or --simulate")

    logf = open(args.log, "w", newline="") if args.log else None
    w = csv.writer(logf) if logf else None
    if w:
        w.writerow(["time", "x", "y", "z", "rms_m", "n_anchors"])

    last = None
    for line in src:
        ranges = parse_line(line, args.max_nlos)
        if not ranges:
            continue
        sol = solve(anchors, ranges, x0=last, z_fixed=args.z)
        if sol:
            pos, _, _ = sol
            filtered = reject_outliers(anchors, ranges, pos)
            if filtered is not ranges:
                sol = solve(anchors, filtered, x0=pos, z_fixed=args.z) or sol
        if not sol:
            print(f"only {len(ranges)} usable ranges, skipping")
            continue
        pos, rms, n = sol
        last = pos
        print(f"x={pos[0]:7.2f} y={pos[1]:7.2f} z={pos[2]:7.2f}  rms={rms:.3f} m  anchors={n}")
        if w:
            w.writerow([time.time(), *pos.round(3), round(rms, 3), n])
            logf.flush()


if __name__ == "__main__":
    main()
