"""
Motion data visualizer: reads CSV from record_motion.py and plots target vs actual.

Usage:
  python plot_motion.py [motion.csv]
"""

import sys
import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec


def load_csv(path):
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    t   = np.array([float(r["t"])       for r in rows])
    inp = np.array([float(r["inp_mag"]) for r in rows])
    tgt = np.array([[float(r["tgt_x"]),  float(r["tgt_y"]),  float(r["tgt_z"]),
                     float(r["tgt_rx"]), float(r["tgt_ry"]), float(r["tgt_rz"])] for r in rows])
    act = np.array([[float(r["act_x"]),  float(r["act_y"]),  float(r["act_z"]),
                     float(r["act_rx"]), float(r["act_ry"]), float(r["act_rz"])] for r in rows])
    return t, inp, tgt, act


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "motion.csv"
    t, inp, tgt, act = load_csv(path)

    tgt_mm = tgt[:, :3] * 1000
    act_mm = act[:, :3] * 1000
    err_mm = np.linalg.norm(tgt_mm - act_mm, axis=1)

    fig = plt.figure(figsize=(16, 12))
    fig.suptitle(f"SpaceMouse -> RM75B  Motion Recording\n{path}", fontsize=13)

    gs = gridspec.GridSpec(4, 3, figure=fig, hspace=0.45, wspace=0.35)

    # Row 0: position X Y Z
    for i, label in enumerate(["X (mm)", "Y (mm)", "Z (mm)"]):
        ax = fig.add_subplot(gs[0, i])
        ax.plot(t, tgt_mm[:, i], label="target", color="steelblue", linewidth=1)
        ax.plot(t, act_mm[:, i], label="actual",  color="tomato",    linewidth=1, alpha=0.85)
        ax.set_title(label)
        ax.set_xlabel("time (s)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    # Row 1: orientation RX RY RZ
    for i, label in enumerate(["RX (rad)", "RY (rad)", "RZ (rad)"]):
        ax = fig.add_subplot(gs[1, i])
        ax.plot(t, tgt[:, 3 + i], label="target", color="steelblue", linewidth=1)
        ax.plot(t, act[:, 3 + i], label="actual",  color="tomato",    linewidth=1, alpha=0.85)
        ax.set_title(label)
        ax.set_xlabel("time (s)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    # Row 2: tracking error + input magnitude
    ax_err = fig.add_subplot(gs[2, :])
    ax_err.plot(t, err_mm, color="darkorange", linewidth=1, label="position tracking error (mm)")
    ax_err.set_ylabel("error (mm)")
    ax_err.set_xlabel("time (s)")
    ax_err.set_title("End-effector tracking error  |target - actual|")
    ax_err.grid(True, alpha=0.3)

    ax_inp = ax_err.twinx()
    ax_inp.fill_between(t, inp, alpha=0.15, color="gray", label="mouse input magnitude")
    ax_inp.set_ylabel("SpaceMouse raw value", color="gray")
    ax_inp.tick_params(axis="y", labelcolor="gray")

    lines1, labels1 = ax_err.get_legend_handles_labels()
    lines2, labels2 = ax_inp.get_legend_handles_labels()
    ax_err.legend(lines1 + lines2, labels1 + labels2, fontsize=8, loc="upper left")

    # Row 3: summary stats
    ax_stat = fig.add_subplot(gs[3, :])
    ax_stat.axis("off")

    active_mask = inp > 80
    if active_mask.any():
        err_active = err_mm[active_mask]
        err_idle   = err_mm[~active_mask] if (~active_mask).any() else np.array([0.0])
        stat_text = (
            f"Frames: {len(t)}   Duration: {t[-1]:.1f}s   Rate: {len(t)/t[-1]:.1f}Hz\n"
            f"Tracking error (moving):  mean={err_active.mean():.2f}mm  "
            f"max={err_active.max():.2f}mm  std={err_active.std():.2f}mm\n"
            f"Tracking error (idle):    mean={err_idle.mean():.2f}mm   "
            f"max={err_idle.max():.2f}mm  std={err_idle.std():.2f}mm  <- sensor noise baseline"
        )
    else:
        stat_text = (
            f"Frames: {len(t)}   Duration: {t[-1]:.1f}s\n"
            f"Tracking error:  mean={err_mm.mean():.2f}mm  "
            f"max={err_mm.max():.2f}mm  std={err_mm.std():.2f}mm"
        )

    ax_stat.text(0.01, 0.5, stat_text, transform=ax_stat.transAxes,
                 fontsize=10, verticalalignment="center",
                 family="monospace",
                 bbox=dict(boxstyle="round", facecolor="lightyellow", alpha=0.8))

    out_png = path.replace(".csv", ".png")
    plt.savefig(out_png, dpi=150, bbox_inches="tight")
    print(f"Saved: {out_png}")
    plt.show()


if __name__ == "__main__":
    main()
