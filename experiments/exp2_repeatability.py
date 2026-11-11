"""Experiment 2 - station-pose repeatability: dead reckoning vs fiducial-corrected."""
import csv, os
from dataclasses import replace
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from orbitsim.calibration import make_stations
from orbitsim.robot import run_session, pose_errors
from orbitsim.stats import describe

MODES = {"dead_reckoning": False, "marker_corrected": True}


def collect(cfg, rng, use, n_sessions, stations, gyro=None):
    P, H, V = [], [], []
    for _ in range(n_sessions):
        tp = run_session(cfg, rng, stations, use, gyro)
        pe, he = pose_errors(tp, stations)
        P.append(pe); H.append(he); V.append(tp[:, :2] - stations[:, :2])
    return np.array(P), np.array(H), np.array(V)


def inter_session(V):
    """Distance between the true positions of the SAME station in two different sessions."""
    return np.linalg.norm(V[0::2][: len(V) // 2] - V[1::2][: len(V) // 2], axis=2).ravel()


def run(cfg, outdir, n_sessions=200):
    stations = make_stations((0.0, 0.0), 60.0 + cfg.standoff, cfg.n_stations)
    res, raw = {}, {}
    for name, use in MODES.items():
        rng = np.random.default_rng(cfg.seed + 2)
        P, H, V = collect(cfg, rng, use, n_sessions, stations)
        raw[name] = (P, H, V)
        res[name] = dict(position_err_mm=describe(P), heading_err_deg=describe(H),
                         last_station_position_err_mm=describe(P[:, -1]),
                         inter_session_offset_mm=describe(inter_session(V)))
    with open(os.path.join(outdir, "exp2_repeatability_by_station.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["mode", "station", "mean_pos_err_mm", "p95_pos_err_mm", "mean_heading_err_deg"])
        for name, (P, H, V) in raw.items():
            for k in range(P.shape[1]):
                w.writerow([name, k, f"{P[:, k].mean():.3f}", f"{np.percentile(P[:, k], 95):.3f}", f"{H[:, k].mean():.3f}"])

    sens = []
    for gyro in (0.01, 0.03, 0.10):
        for name, use in MODES.items():
            rng = np.random.default_rng(cfg.seed + 22)
            P, H, V = collect(cfg, rng, use, 100, stations, gyro)
            sens.append(dict(sweep="gyro_bias_sigma_dps", value=gyro, mode=name,
                             mean_pos_err=float(P.mean()), p95_pos_err=float(np.percentile(P, 95))))
    for msig in (1.0, 1.5, 3.0, 5.0):
        c2 = replace(cfg, marker_pos_sigma=msig)
        rng = np.random.default_rng(cfg.seed + 23)
        P, H, V = collect(c2, rng, True, 100, stations)
        sens.append(dict(sweep="marker_pos_sigma_mm", value=msig, mode="marker_corrected",
                         mean_pos_err=float(P.mean()), p95_pos_err=float(np.percentile(P, 95))))
    with open(os.path.join(outdir, "exp2_sensitivity.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(sens[0])); w.writeheader(); w.writerows(sens)
    res["sensitivity"] = sens

    fig, ax = plt.subplots(1, 2, figsize=(12, 4.2))
    x = np.arange(cfg.n_stations)
    for name, c in (("dead_reckoning", "tab:red"), ("marker_corrected", "tab:green")):
        P = raw[name][0]
        ax[0].plot(x, P.mean(0), "-o", color=c, label=f"{name} (mean)")
        ax[0].fill_between(x, np.percentile(P, 5, 0), np.percentile(P, 95, 0), color=c, alpha=0.15)
    ax[0].set_xlabel("station index along the orbit"); ax[0].set_ylabel("position error vs ideal station (mm)")
    ax[0].set_title("Error growth along one session (simulation)"); ax[0].legend()
    for name, c in (("dead_reckoning", "tab:red"), ("marker_corrected", "tab:green")):
        d = np.sort(inter_session(raw[name][2]))
        ax[1].plot(d, np.arange(1, len(d) + 1) / len(d), color=c, label=name)
    ax[1].set_xscale("log"); ax[1].set_xlabel("same-station offset between two sessions (mm)")
    ax[1].set_ylabel("cumulative fraction"); ax[1].set_title("Day-to-day repeatability (simulation)"); ax[1].legend()
    fig.tight_layout(); fig.savefig(os.path.join(outdir, "figures", "fig2_repeatability.png"), dpi=150); plt.close(fig)
    return res
