"""Experiment 1 - self-calibration accuracy across pot sizes (ToF scan + RANSAC circle fit)."""
import csv, os
from dataclasses import replace
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from orbitsim.calibration import tof_scan, calibrate_from_points
from orbitsim.stats import describe

SUCCESS_MM = 3.0


def one(cfg, rng):
    r = rng.uniform(30, 120)
    off = rng.uniform(-15, 15, 2)
    pts = tof_scan(off, r, cfg, rng)
    rob = calibrate_from_points(pts, cfg, rng, robust=True)
    lsq = calibrate_from_points(pts, cfg, rng, robust=False)
    return r, off, rob, lsq


def run(cfg, outdir, n=500):
    rng = np.random.default_rng(cfg.seed + 1)
    rows = []
    for i in range(n):
        r, off, rob, lsq = one(cfg, rng)
        rows.append(dict(trial=i, r_true=r, off_x=off[0], off_y=off[1],
                         r_robust=rob.radius, r_lsq=lsq.radius,
                         centre_err_robust=float(np.linalg.norm(rob.center - off)),
                         ok_robust=int(rob.ok)))
    with open(os.path.join(outdir, "exp1_calibration_trials.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)

    e_rob = np.array([abs(x["r_robust"] - x["r_true"]) for x in rows])
    e_lsq = np.array([abs(x["r_lsq"] - x["r_true"]) for x in rows])
    c_err = np.array([x["centre_err_robust"] for x in rows])
    summary = dict(
        n_trials=n,
        robust_radius_err_mm=describe(e_rob), lsq_radius_err_mm=describe(e_lsq),
        robust_centre_err_mm=describe(c_err),
        robust_success_rate=float((e_rob <= SUCCESS_MM).mean()),
        lsq_success_rate=float((e_lsq <= SUCCESS_MM).mean()),
        success_definition=f"|radius error| <= {SUCCESS_MM} mm")

    # sensitivity grid: sensor noise x outlier probability (200 trials each)
    grid = []
    for noise in (1.0, 3.0, 6.0, 10.0):
        for outp in (0.0, 0.08, 0.20):
            c2 = replace(cfg, tof_noise_mm=noise, tof_outlier_prob=outp)
            g = np.random.default_rng(cfg.seed + 11)
            er, el = [], []
            for _ in range(200):
                r, off, rob, lsq = one(c2, g)
                er.append(abs(rob.radius - r)); el.append(abs(lsq.radius - r))
            grid.append(dict(noise_mm=noise, outlier_prob=outp,
                             robust_median=float(np.median(er)), robust_p95=float(np.percentile(er, 95)),
                             lsq_median=float(np.median(el)), lsq_p95=float(np.percentile(el, 95))))
    with open(os.path.join(outdir, "exp1_sensitivity.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(grid[0])); w.writeheader(); w.writerows(grid)
    summary["sensitivity"] = grid

    fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
    rt = np.array([x["r_true"] for x in rows])
    ax[0].scatter(rt, [x["r_robust"] for x in rows], s=8, label="robust (RANSAC)")
    ax[0].scatter(rt, [x["r_lsq"] for x in rows], s=8, alpha=0.5, label="plain least squares")
    ax[0].plot([30, 120], [30, 120], "k--", lw=1)
    ax[0].set_xlabel("true pot radius (mm)"); ax[0].set_ylabel("estimated radius (mm)")
    ax[0].set_title("Simulated self-calibration"); ax[0].legend()
    ax[1].hist(e_rob, bins=30, alpha=0.8, label="robust"); ax[1].hist(e_lsq, bins=30, alpha=0.5, label="least squares")
    ax[1].set_xlabel("absolute radius error (mm)"); ax[1].set_ylabel("trials"); ax[1].legend()
    ax[1].set_title("Radius error distribution (simulation)")
    fig.tight_layout(); fig.savefig(os.path.join(outdir, "figures", "fig1_calibration.png"), dpi=150); plt.close(fig)
    return summary
