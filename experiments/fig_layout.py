"""Figure 0 - system layout drawing (mat, fiducials, pot, scan hits, fitted circle, stations, path)."""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from orbitsim.calibration import tof_scan, calibrate_from_points


def run(cfg, outdir):
    rng = np.random.default_rng(cfg.seed + 5)
    pot_r, off = 65.0, np.array([8.0, -5.0])
    pts = tof_scan(off, pot_r, cfg, rng)
    cal = calibrate_from_points(pts, cfg, rng)
    fig, ax = plt.subplots(figsize=(6.5, 6.5))
    ax.add_patch(plt.Circle((0, 0), cfg.mat_radius, fill=False, lw=1.5))
    for a in np.deg2rad([45, 135, 225, 315]):
        ax.add_patch(plt.Rectangle((285 * np.cos(a) - 12, 285 * np.sin(a) - 12), 24, 24, fc="k"))
    ax.add_patch(plt.Circle(off, pot_r, fc="#d9a074", ec="k"))
    ax.scatter(pts[:, 0], pts[:, 1], s=8, c="tab:blue", label="ToF hit points")
    t = np.linspace(0, 2 * np.pi, 200)
    ax.plot(cal.center[0] + cal.radius * np.cos(t), cal.center[1] + cal.radius * np.sin(t), "r--", lw=1, label="fitted pot circle")
    ax.plot(cal.center[0] + cal.imaging_radius * np.cos(t), cal.center[1] + cal.imaging_radius * np.sin(t), "g:", lw=1, label="imaging orbit")
    ax.scatter(cal.stations[:, 0], cal.stations[:, 1], c="g", s=30, zorder=3, label="stations")
    for i, s in enumerate(cal.stations):
        ax.annotate(str(i), s[:2], textcoords="offset points", xytext=(4, 4), fontsize=8)
    a = np.deg2rad(cfg.dock_angle_deg)
    ax.scatter([cfg.dock_radius * np.cos(a)], [cfg.dock_radius * np.sin(a)], marker="s", c="orange", s=60, label="dock")
    ax.set_aspect("equal"); ax.set_xlim(-330, 330); ax.set_ylim(-330, 330)
    ax.set_title("Layout: mat, fiducials, pot, scan, orbit (simulation)"); ax.legend(loc="upper left", fontsize=7)
    fig.tight_layout(); fig.savefig(os.path.join(outdir, "figures", "fig0_layout.png"), dpi=150); plt.close(fig)
    return dict(est_radius=cal.radius, est_centre=cal.center.tolist(), imaging_radius=cal.imaging_radius)
