# /// script
# requires-python = ">=3.9"
# dependencies = ["pandas", "matplotlib", "numpy"]
# ///
"""Plot a rustypot soak-test CSV (see examples/soak_test.rs).

Usage:
    uv run plot_soak.py [soak.csv] [-o soak_plot.png]

Produces a 5-panel figure sharing a time axis -- memory, comm errors,
read/write latency, loop-period jitter, and motor temperature -- and prints a
concise health verdict (leak slope, error totals, worst-case latency/jitter,
peak temperature).
"""
import argparse
import sys

import matplotlib

matplotlib.use("Agg")  # headless: save to file, never try to open a window
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def main() -> int:
    ap = argparse.ArgumentParser(description="Plot a rustypot soak-test CSV.")
    ap.add_argument("csv", nargs="?", default="soak.csv", help="input CSV (default: soak.csv)")
    ap.add_argument("-o", "--out", default=None, help="output PNG (default: <csv>.png)")
    args = ap.parse_args()

    out = args.out or args.csv.rsplit(".", 1)[0] + ".png"

    # Skip the leading '#' comment/banner lines the harness prints before the CSV.
    df = pd.read_csv(args.csv, comment="#")
    if df.empty:
        print(f"error: {args.csv} has no data rows", file=sys.stderr)
        return 1

    # Pick a readable time unit based on total run length.
    span = df["elapsed_s"].iloc[-1]
    if span >= 3600:
        t, tlabel = df["elapsed_s"] / 3600.0, "elapsed (hours)"
    elif span >= 120:
        t, tlabel = df["elapsed_s"] / 60.0, "elapsed (minutes)"
    else:
        t, tlabel = df["elapsed_s"], "elapsed (seconds)"

    fig, ax = plt.subplots(5, 1, figsize=(12, 16), sharex=True)
    fig.suptitle(f"rustypot soak test — {args.csv}  ({len(df)} windows, {span/3600:.2f} h)",
                 fontsize=13, fontweight="bold")

    # --- 1. Memory (leak detection) ---
    a = ax[0]
    a.plot(t, df["rss_kb"] / 1024.0, color="tab:blue", label="RSS (MB)")
    # Linear fit -> leak slope in KB/hour.
    slope_kb_h = np.polyfit(df["elapsed_s"], df["rss_kb"], 1)[0] * 3600.0 if len(df) > 1 else 0.0
    a.set_ylabel("RSS (MB)")
    a.set_title(f"Memory   —   trend: {slope_kb_h:+.1f} KB/hour")
    a.grid(True, alpha=0.3)
    a.legend(loc="upper left")

    # --- 2. Communication errors ---
    a = ax[1]
    a.plot(t, df["errs"], color="tab:red", marker=".", label="errors / window")
    for kind, c in [("timeout", "darkred"), ("checksum", "orange"),
                    ("parsing", "purple"), ("incorrect_id", "brown"), ("other", "gray")]:
        if df[kind].sum() > 0:
            a.plot(t, df[kind], marker=".", label=kind, color=c)
    a.set_ylabel("errors / window")
    a.set_title(f"Communication errors   —   total: {int(df['errs'].sum())}, "
                f"max consecutive: {int(df['max_consec_errs'].max())}")
    a.grid(True, alpha=0.3)
    a2 = a.twinx()
    a2.plot(t, df["err_rate"] * 100.0, color="tab:red", alpha=0.3, linestyle="--")
    a2.set_ylabel("err rate (%)", color="tab:red")
    a.legend(loc="upper left")

    # --- 3. Read / write latency ---
    a = ax[2]
    a.fill_between(t, df["read_p50_us"], df["read_p99_us"], alpha=0.15,
                   color="tab:blue", label="read p50–p99")
    a.plot(t, df["read_mean_us"], color="tab:blue", label="read mean")
    a.plot(t, df["read_max_us"], color="tab:blue", alpha=0.4, linestyle=":", label="read max")
    a.plot(t, df["write_mean_us"], color="tab:green", label="write mean")
    a.plot(t, df["write_p99_us"], color="tab:green", alpha=0.4, linestyle=":", label="write p99")
    a.set_ylabel("latency (µs)")
    a.set_title("Bus latency (read waits for replies; write is fire-and-forget)")
    a.grid(True, alpha=0.3)
    a.legend(loc="upper left", ncol=2, fontsize=8)

    # --- 4. Loop-period jitter ---
    a = ax[3]
    lo = df["period_err_mean_us"] - df["period_err_std_us"]
    hi = df["period_err_mean_us"] + df["period_err_std_us"]
    a.fill_between(t, lo, hi, alpha=0.15, color="tab:purple", label="mean ± std")
    a.plot(t, df["period_err_mean_us"], color="tab:purple", label="period err mean")
    a.plot(t, df["period_err_max_us"], color="tab:purple", alpha=0.4, linestyle=":",
           label="period err max")
    a.set_ylabel("period error (µs)")
    a.set_title("Loop-period jitter (deviation from target cycle time)")
    a.grid(True, alpha=0.3)
    a2 = a.twinx()
    a2.plot(t, df["overruns"], color="tab:orange", alpha=0.5, label="overruns")
    a2.set_ylabel("overruns / window", color="tab:orange")
    a.legend(loc="upper left")

    # --- 5. Temperature ---
    a = ax[4]
    a.plot(t, df["temp_max_c"], color="tab:red", marker=".", label="temp max")
    a.plot(t, df["temp_mean_c"], color="tab:orange", marker=".", label="temp mean")
    a.set_ylabel("temperature (°C)")
    a.set_title(f"Motor temperature   —   peak: {df['temp_max_c'].max():.0f} °C")
    a.grid(True, alpha=0.3)
    a2 = a.twinx()
    a2.plot(t, df["err_rate"] * 100.0, color="tab:blue", alpha=0.3, linestyle="--",
            label="err rate")
    a2.set_ylabel("err rate (%)", color="tab:blue")
    a.legend(loc="upper left")
    a.set_xlabel(tlabel)

    fig.tight_layout(rect=(0, 0, 1, 0.99))
    fig.savefig(out, dpi=130)
    print(f"wrote {out}")

    # --- Health verdict ---
    print("\n===== health summary =====")
    print(f"duration        : {span/3600:.2f} h  ({len(df)} windows)")
    leak = "FLAT (no leak)" if abs(slope_kb_h) < 50 else f"RISING {slope_kb_h:+.1f} KB/h <-- investigate"
    print(f"memory          : {leak}  (RSS {df['rss_kb'].iloc[0]/1024:.1f} -> {df['rss_kb'].iloc[-1]/1024:.1f} MB)")
    print(f"errors          : {int(df['errs'].sum())} total, max consecutive {int(df['max_consec_errs'].max())}, "
          f"peak rate {df['err_rate'].max()*100:.4f}%")
    print(f"read latency    : worst-window p99 {df['read_p99_us'].max():.0f} µs, max {df['read_max_us'].max():.0f} µs")
    print(f"period jitter   : worst-window mean {df['period_err_mean_us'].max():.0f} µs, "
          f"max {df['period_err_max_us'].max():.0f} µs, overruns {int(df['overruns'].sum())}")
    print(f"temperature     : peak {df['temp_max_c'].max():.0f} °C")
    return 0


if __name__ == "__main__":
    sys.exit(main())
