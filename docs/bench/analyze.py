#!/usr/bin/env python3
"""Benchmark analysis: reads the CSV logs from the robot (/sdcard/FIRST/bench) and prints the
metrics of docs/PATHING-BENCHMARK-PLAN.md section 6 that come from the logs alone.

    python3 analyze.py <folder with CSVs> [sheet.csv]

The optional sheet holds the hand measurements, one row per run:
    test,engine,speed,run,valid,end_x,end_y,end_h_deg,notes
With it, "measured" end errors are added and invalid runs are left out.

Only the Python standard library is used.
"""
import csv
import glob
import math
import os
import statistics
import sys
from collections import defaultdict


def wrap(d):
    return (d + 180) % 360 - 180


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        for k, v in r.items():
            if k not in ("engine", "test"):
                r[k] = float(v) if v not in ("", None) else math.nan
    return rows


# A6 is a quadratic Bézier from the start (72, 24) through the control point (72, 60) to (108, 60),
# as in bench/BenchRoutes.java. Keep the two in step.
A6_CONTROL = (72.0, 60.0)


def planned_path(rows):
    """Points of the planned path: the start, then each step's target in order (A6: the curve)."""
    pts = [(rows[0]["pose_x"], rows[0]["pose_y"])]
    if rows[0]["test"] == "A6":
        (ax, ay), (cx, cy), (bx, by) = pts[0], A6_CONTROL, (rows[0]["target_x"], rows[0]["target_y"])
        for i in range(1, 41):
            t = i / 40
            u = 1 - t
            pts.append((u * u * ax + 2 * u * t * cx + t * t * bx, u * u * ay + 2 * u * t * cy + t * t * by))
        return pts
    for r in rows:
        p = (r["target_x"], r["target_y"])
        if r["step"] >= 0 and p != pts[-1]:
            pts.append(p)
    return pts


def distance_to_polyline(x, y, pts):
    best = math.inf
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0 if L2 < 1e-12 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
        best = min(best, math.hypot(x - (ax + t * dx), y - (ay + t * dy)))
    return best


def metrics(rows):
    """Metrics of one run from its log."""
    start, end = rows[0], rows[-1]
    done = next((r for r in rows if r["busy"] == 0), None)
    tx, ty, th = end["target_x"], end["target_y"], end["target_h_deg"]
    # direction of the last step, for overshoot: from where the robot was when the last step began
    last_step = end["step"]
    begin = next((r for r in rows if r["step"] >= last_step - (1 if done is not None and last_step > 0 else 0)
                  and r["target_x"] == tx and r["target_y"] == ty), start)
    dx, dy = tx - begin["pose_x"], ty - begin["pose_y"]
    length = math.hypot(dx, dy)
    overshoot = max((((r["pose_x"] - tx) * dx + (r["pose_y"] - ty) * dy) / length for r in rows), default=0) \
        if length > 1e-6 else 0
    # distance from the planned path: the straight legs through each step's target, or the curve for A6
    planned = planned_path(rows)
    offline = [distance_to_polyline(r["pose_x"], r["pose_y"], planned) for r in rows] if len(planned) > 1 else [0]
    power = sum((abs(r["power_fwd"]) + abs(r["power_str"]) + abs(r["power_turn"])) * r["loop_ms"] / 1000
                for r in rows)
    loops = [r["loop_ms"] for r in rows[1:]] or [math.nan]
    loops_sorted = sorted(loops)
    # time to settle (from the log): from then on within the distance and angle, slower than 1 in/s
    # and 3°/s, for at least 0.25 s. Fairer than "done": Pedro reports done while still moving, then
    # holds. Two tolerances: the plan's 0.5 in / 1°, and a looser 1 in / 2°.
    def settle_time(dist, ang):
        ok = lambda q: (math.hypot(q["pose_x"] - tx, q["pose_y"] - ty) < dist and abs(wrap(q["pose_h_deg"] - th)) < ang
                        and math.hypot(q["vel_x"], q["vel_y"]) < 1 and abs(q["vel_h_degs"]) < 3)
        good = [ok(r) for r in rows]
        for i in range(len(rows)):
            if all(good[i:]) and end["t_s"] - rows[i]["t_s"] >= 0.25:
                return rows[i]["t_s"]
        return math.nan
    return {
        "time_to_done_s": done["t_s"] if done else math.nan,
        "settle_0.5in_s": settle_time(0.5, 1),
        "settle_1in_s": settle_time(1.0, 2),
        "end_err_logged_in": math.hypot(end["pose_x"] - tx, end["pose_y"] - ty),
        "end_err_logged_deg": abs(wrap(end["pose_h_deg"] - th)),
        "overshoot_in": max(overshoot, 0),
        "offline_max_in": max(offline),
        "offline_rms_in": math.sqrt(sum(o * o for o in offline) / len(offline)),
        "power_used": power,
        "loop_mean_ms": statistics.mean(loops),
        "loop_p95_ms": loops_sorted[int(0.95 * (len(loops_sorted) - 1))],
        "loop_max_ms": max(loops),
        "end_x": end["pose_x"], "end_y": end["pose_y"], "end_h": end["pose_h_deg"],
        "target": (tx, ty, th),
    }


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    sheet = {}
    if len(sys.argv) > 2:
        with open(sys.argv[2], newline="") as f:
            for r in csv.DictReader(f):
                sheet[(r["test"], r["engine"], r["speed"], int(r["run"]))] = r
    groups = defaultdict(list)
    for path in sorted(glob.glob(os.path.join(sys.argv[1], "*.csv"))):
        name = os.path.basename(path)[:-4]
        try:
            test, engine, speed, run = name.rsplit("_", 3)
        except ValueError:
            continue
        rows = load(path)
        if not rows:
            continue
        m = metrics(rows)
        s = sheet.get((test, engine, speed, int(run)))
        if s is not None:
            if s.get("valid", "1").strip().lower() in ("0", "no", "false", "n"):
                continue
            tx, ty, th = m["target"]
            m["end_err_measured_in"] = math.hypot(float(s["end_x"]) - tx, float(s["end_y"]) - ty)
            m["end_err_measured_deg"] = abs(wrap(float(s["end_h_deg"]) - th))
            m["measured_x"], m["measured_y"] = float(s["end_x"]), float(s["end_y"])
        groups[(test, engine, speed)].append(m)

    keys = ["time_to_done_s", "settle_0.5in_s", "settle_1in_s", "end_err_logged_in", "end_err_logged_deg", "end_err_measured_in",
            "end_err_measured_deg", "overshoot_in", "offline_max_in", "power_used", "loop_p95_ms"]
    print("test engine     speed    n  " + "  ".join(f"{k:>22}" for k in keys) + "  repeatability_in")
    for (test, engine, speed), runs in sorted(groups.items()):
        cells = []
        for k in keys:
            vals = [r[k] for r in runs if k in r and not math.isnan(r[k])]
            if not vals:
                cells.append(f"{'-':>22}")
            elif len(vals) == 1:
                cells.append(f"{vals[0]:>22.2f}")
            else:
                worst = max(vals)
                cells.append(f"{statistics.mean(vals):>8.2f} ±{statistics.stdev(vals):5.2f} ({worst:5.2f})")
        # spread of the end positions: measured if available, else logged
        xs = [r.get("measured_x", r["end_x"]) for r in runs]
        ys = [r.get("measured_y", r["end_y"]) for r in runs]
        rep = math.hypot(statistics.pstdev(xs), statistics.pstdev(ys)) if len(runs) > 1 else math.nan
        print(f"{test:<4} {engine:<10} {speed:<7} {len(runs):2d}  " + "  ".join(cells) + f"  {rep:16.2f}")
    print("\nCells: mean ± standard deviation (worst). Measured columns need the hand-measured sheet.")


if __name__ == "__main__":
    main()
