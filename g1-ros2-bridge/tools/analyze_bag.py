#!/usr/bin/env python3
"""
Analyze a rosbag2 recorded during a test of deploy_mujoco_ros.py.

Record (terminal 3, while the simulation and the /cmd_vel publisher run):
    ros2 bag record -o test_forward /cmd_vel /g1/base_twist /g1/policy_time_ms

Analyze:
    source /opt/ros/humble/setup.bash
    python3 analyze_bag.py test_forward                 # defaults
    python3 analyze_bag.py test_forward --skip 4 --out tracking_vx.png

What it does
  1. Tracking: compares the measured base velocity (/g1/base_twist) with the
     commanded one (/cmd_vel, zero-order hold) over a steady-state window and
     prints mean, standard deviation and RMSE.
  2. Timing: statistics of /g1/policy_time_ms (warm-up samples discarded).
  3. Saves two figures: <out> (tracking) and <out>_inference.png (histogram).

Notes
  - The simulation may run slower than real time (e.g. in WSL). The time axis is
    therefore SIMULATION time: telemetry is published once per policy step, so
    sample k of /g1/base_twist is at k * dt_policy (default 0.02 s). Commands
    are mapped to the sample that follows their arrival. The real-time factor
    (simulated seconds per wall-clock second) is printed.
  - The measured linear velocity is only comparable to /cmd_vel if it is
    expressed in the body frame (see the frame fix in the report).
  - The gait is periodic (0.8 s): the steady-state window should cover several
    periods, so use tests of at least ~10 s.
"""
import argparse
import sys

import numpy as np


# ----------------------------------------------------------------------
# Reading
# ----------------------------------------------------------------------
def read_bag(bag_dir):
    """Return dict topic -> (t [s], values [N, k]) from a rosbag2 directory."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_cls = {name: get_message(tp) for name, tp in types.items()}

    raw = {"/cmd_vel": [], "/g1/base_twist": [], "/g1/policy_time_ms": []}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in raw:
            continue
        msg = deserialize_message(data, msg_cls[topic])
        t = t_ns * 1e-9
        if topic in ("/cmd_vel", "/g1/base_twist"):
            raw[topic].append(
                (t, msg.linear.x, msg.linear.y, msg.linear.z,
                 msg.angular.x, msg.angular.y, msg.angular.z)
            )
        else:
            raw[topic].append((t, msg.data))

    out = {}
    for topic, rows in raw.items():
        arr = np.array(rows, dtype=float) if rows else np.zeros((0, 2))
        out[topic] = (arr[:, 0], arr[:, 1:]) if len(arr) else (np.array([]), np.zeros((0, 1)))
    return out


# ----------------------------------------------------------------------
# Analysis (pure numpy, testable without ROS)
# ----------------------------------------------------------------------
def zero_order_hold(t_query, t_src, v_src):
    """Value of a piecewise-constant signal at t_query (0 before first sample)."""
    if len(t_src) == 0:
        return np.zeros((len(t_query), 1))
    idx = np.searchsorted(t_src, t_query, side="right") - 1
    out = v_src[np.clip(idx, 0, None)].copy()
    out[idx < 0] = 0.0
    return out


def rolling_mean_time(t, y, window):
    """Mean of y over the previous `window` seconds at each sample."""
    c = np.concatenate([[0.0], np.cumsum(y)])
    lo = np.searchsorted(t, t - window, side="left")
    n = np.arange(1, len(t) + 1) - lo
    return (c[np.arange(1, len(t) + 1)] - c[lo]) / n


def choose_window(t_cmd, cmd, t_meas, skip, t0=None, t1=None):
    """Steady-state window [a, b] (absolute times)."""
    if t0 is not None and t1 is not None:
        return t0, t1
    nz = np.where(np.any(np.abs(cmd) > 1e-9, axis=1))[0] if len(cmd) else []
    if len(nz) > 0:
        a = t_cmd[nz[0]] + skip
        b = t_meas[-1]
    else:  # standing test: command always zero
        a = t_meas[0] + skip
        b = t_meas[-1]
    return a, b


def tracking_stats(t_meas, meas, cmd_at_meas, a, b):
    m = (t_meas >= a) & (t_meas <= b)
    if m.sum() < 5:
        raise SystemExit("Window contains fewer than 5 samples: lower --skip or record longer.")
    err = meas[m] - cmd_at_meas[m]
    return {
        "n": int(m.sum()),
        "mean_meas": meas[m].mean(axis=0),
        "std_meas": meas[m].std(axis=0),
        "mean_cmd": cmd_at_meas[m].mean(axis=0),
        "rmse": np.sqrt((err ** 2).mean(axis=0)),
    }


def timing_stats(x, warmup, period_ms=20.0):
    x = np.asarray(x, dtype=float)[warmup:]
    if len(x) == 0:
        raise SystemExit("No inference-time samples after warm-up.")
    return {
        "n": len(x),
        "mean": x.mean(),
        "median": np.median(x),
        "p95": np.percentile(x, 95),
        "p99": np.percentile(x, 99),
        "max": x.max(),
        "over_period": float((x > period_ms).mean()),
    }


# ----------------------------------------------------------------------
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="rosbag2 directory")
    ap.add_argument("--skip", type=float, default=2.0,
                    help="simulated seconds discarded after the first non-zero command (default 2)")
    ap.add_argument("--t0", type=float, help="window start, simulated s (overrides --skip)")
    ap.add_argument("--t1", type=float, help="window end, simulated s")
    ap.add_argument("--dt-policy", type=float, default=0.02,
                    help="simulated time between two telemetry messages (default 0.02 s)")
    ap.add_argument("--warmup", type=int, default=20, help="inference samples to discard (default 20)")
    ap.add_argument("--out", default="tracking_vx.png", help="tracking figure file")
    args = ap.parse_args(argv)

    data = read_bag(args.bag)
    t_cmd, cmd = data["/cmd_vel"]
    t_tw, tw = data["/g1/base_twist"]
    t_inf, inf = data["/g1/policy_time_ms"]
    if len(t_tw) == 0:
        sys.exit("No /g1/base_twist messages in the bag.")

    # ---- simulation time axis ----
    wall_tw = t_tw.copy()
    wall_span = wall_tw[-1] - wall_tw[0]
    sim_span = (len(t_tw) - 1) * args.dt_policy
    print(f"\nTelemetry samples: {len(t_tw)}  ->  {sim_span:.1f} s of simulated time "
          f"in {wall_span:.1f} s of wall-clock time")
    if wall_span > 0:
        print(f"Real-time factor: {sim_span / wall_span:.2f}  "
              f"(telemetry rate {(len(t_tw) - 1) / wall_span:.1f} Hz; 50 Hz = real time)")
    t_cmd_wall = t_cmd.copy()
    t_cmd = np.searchsorted(wall_tw, t_cmd_wall, side="left") * args.dt_policy if len(t_cmd) else t_cmd
    t_tw = np.arange(len(wall_tw)) * args.dt_policy
    t_inf = np.arange(len(t_inf)) * args.dt_policy

    # signals: vx (index 0) and wz (index 5 of the 6-vector; cmd has the same layout)
    vx_cmd = zero_order_hold(t_tw, t_cmd, cmd)[:, [0, 5]] if len(t_cmd) else np.zeros((len(t_tw), 2))
    meas = tw[:, [0, 5]]
    cmd_full = cmd[:, [0, 5]] if len(t_cmd) else np.zeros((0, 2))

    a, b = choose_window(t_cmd, cmd_full, t_tw, args.skip, args.t0, args.t1)
    st = tracking_stats(t_tw, meas, vx_cmd, a, b)

    print(f"\nWindow: {a:.1f} s -> {b:.1f} s of simulated time  ({st['n']} samples)")
    for k, name, unit in ((0, "vx", "m/s"), (1, "wz", "rad/s")):
        print(f"  {name}: command {st['mean_cmd'][k]:+.3f} {unit} | measured "
              f"{st['mean_meas'][k]:+.3f} +/- {st['std_meas'][k]:.3f} {unit} | RMSE {st['rmse'][k]:.3f} {unit}")

    ts = None
    if len(inf):
        ts = timing_stats(inf[:, 0], args.warmup)
        print(f"\nInference time ({ts['n']} samples after {args.warmup} warm-up): "
              f"mean {ts['mean']:.2f} ms | median {ts['median']:.2f} | p95 {ts['p95']:.2f} | "
              f"p99 {ts['p99']:.2f} | max {ts['max']:.2f} | > 20 ms: {100*ts['over_period']:.1f} %")

    # ------------------------------ plots ------------------------------
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(2, 1, figsize=(8, 5.5), sharex=True)
    for ax, k, lab, unit in ((axs[0], 0, "$v_x$", "m/s"), (axs[1], 1, "$\\omega_z$", "rad/s")):
        ax.plot(t_tw, meas[:, k], color="#9fb3c8", lw=0.9, label=f"measured {lab} (raw)")
        ax.plot(t_tw, rolling_mean_time(t_tw, meas[:, k], 0.8), color="#192f4a", lw=1.8,
                label="measured, 0.8 s moving average")
        ax.step(t_tw, vx_cmd[:, k], where="post", color="#be4f2d", lw=1.6, label=f"commanded {lab}")
        ax.axvspan(a, b, color="#be4f2d", alpha=0.07)
        ax.set_ylabel(f"{lab} ({unit})")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc="best")
    axs[1].set_xlabel("simulated time (s)")
    axs[0].set_title("Commanded vs measured velocity (shaded: steady-state window)")
    fig.tight_layout()
    fig.savefig(args.out, dpi=200)
    print(f"\nSaved {args.out}")

    if len(inf):
        x = inf[args.warmup:, 0]
        fig2, ax2 = plt.subplots(figsize=(6, 3.5))
        ax2.hist(x, bins=50, color="#192f4a")
        ax2.axvline(20.0, color="#be4f2d", ls="--", label="policy period (20 ms)")
        ax2.set_xlabel("policy inference time (ms)")
        ax2.set_ylabel("count")
        ax2.legend()
        fig2.tight_layout()
        out2 = args.out.rsplit(".", 1)[0] + "_inference.png"
        fig2.savefig(out2, dpi=200)
        print(f"Saved {out2}")

    # LaTeX-ready summary
    print("\nLaTeX table row (vx):")
    print(f"  mean {st['mean_meas'][0]:.3f} $\\pm$ {st['std_meas'][0]:.3f} m/s, "
          f"RMSE {st['rmse'][0]:.3f} m/s")


if __name__ == "__main__":
    main()
