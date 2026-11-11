"""Experiment 3 - does repeatable orbital capture reduce false alarms and keep detection power?"""
import csv, os
import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from orbitsim.calibration import calibrate
from orbitsim.robot import run_session
from orbitsim.plant import make_plant, apply_scenario, render, framing
from orbitsim.detect import compare_station, station_flags, session_alert
from orbitsim.stats import wilson

SCEN = ["none", "growth", "leaf_drop", "yellowing", "wilting"]
NEG, POS = ["none", "growth"], ["leaf_drop", "yellowing", "wilting"]
MODES = ["ideal", "dead_reckoning", "marker_corrected"]


def collect(cfg, outdir, n_trials, seed_offset=4):
    rng = np.random.default_rng(cfg.seed + seed_offset)
    rows, montage, failed = [], {}, 0
    n = cfg.n_stations
    for t in range(n_trials):
        pot_r = rng.uniform(40, 90)
        off = rng.uniform(-12, 12, 2)
        plant0 = make_plant(rng, pot_r)
        cal = calibrate(off, pot_r, cfg, rng)
        if not cal.ok:
            failed += 1
            continue
        Hc, pitch = framing(plant0, cal.imaging_radius)
        plants = {s: apply_scenario(plant0, s, rng) for s in SCEN}
        poses = {"ideal": (cal.stations.copy(), cal.stations.copy())}
        for mode in MODES[1:]:
            use = mode == "marker_corrected"
            poses[mode] = (run_session(cfg, rng, cal.stations, use), run_session(cfg, rng, cal.stations, use))
        for mode in MODES:
            pp, pn = poses[mode]
            g0 = rng.normal(1, cfg.illum_sigma)
            prev = [render(plant0, pp[k, :2] - off, pp[k, 2], Hc, pitch, cfg, rng, g0) for k in range(n)]
            for sc in SCEN:
                g1 = rng.normal(1, cfg.illum_sigma)
                news = [render(plants[sc], pn[k, :2] - off, pn[k, 2], Hc, pitch, cfg, rng, g1) for k in range(n)]
                for k in range(n):
                    m = compare_station(prev[k], news[k])
                    rows.append([mode, sc, t, k, *m])
                if t == 0 and sc == "none":
                    montage[mode] = (prev, news)
    with open(os.path.join(outdir, "exp3_raw_metrics.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["mode", "scenario", "trial", "station", "area_change", "yellow_delta", "iou", "top_drop"])
        w.writerows(rows)
    return rows, montage, failed


def tabulate(rows, cfg, area_t=None, yellow_t=None, top_t=None, k=None):
    k = cfg.k_stations if k is None else k
    by = {}
    for r in rows:
        by.setdefault((r[0], r[1], r[2]), []).append(r[4:8])
    out = {}
    for (mode, sc, t), v in by.items():
        v = np.array(v)
        fl = station_flags(v[:, 0], v[:, 1], v[:, 3], cfg, area_t, yellow_t, top_t)
        out.setdefault((mode, sc), []).append(session_alert(fl, k))
    return out


def run(cfg, outdir, n_trials=40):
    rows, montage, failed = collect(cfg, outdir, n_trials)
    main = tabulate(rows, cfg)
    table, summary = [], {"n_trials_attempted": n_trials, "calibration_failures": failed}
    for (mode, sc), al in sorted(main.items()):
        p, lo, hi = wilson(sum(al), len(al))
        table.append(dict(mode=mode, scenario=sc, role="negative" if sc in NEG else "positive",
                          alert_rate=p, ci_lo=lo, ci_hi=hi, n=len(al)))
    with open(os.path.join(outdir, "exp3_alert_rates.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(table[0])); w.writeheader(); w.writerows(table)
    summary["alert_rates"] = table

    sweep = []
    for a in (0.04, 0.06, 0.08, 0.10):
        for k in (1, 3, 5):
            tb = tabulate(rows, cfg, area_t=a, k=k)
            for mode in MODES:
                neg = sum([tb[(mode, s)] for s in NEG], []); pos = sum([tb[(mode, s)] for s in POS], [])
                sweep.append(dict(area_thresh=a, k=k, mode=mode,
                                  false_alarm_rate=float(np.mean(neg)), detection_rate=float(np.mean(pos))))
    with open(os.path.join(outdir, "exp3_threshold_sweep.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(sweep[0])); w.writeheader(); w.writerows(sweep)
    summary["threshold_sweep"] = sweep

    fig, ax = plt.subplots(figsize=(11, 4.4))
    width = 0.26
    for i, (mode, c) in enumerate(zip(MODES, ("tab:gray", "tab:red", "tab:green"))):
        vals, err = [], [[], []]
        for sc in SCEN:
            r = next(x for x in table if x["mode"] == mode and x["scenario"] == sc)
            vals.append(r["alert_rate"]); err[0].append(r["alert_rate"] - r["ci_lo"]); err[1].append(r["ci_hi"] - r["alert_rate"])
        ax.bar(np.arange(len(SCEN)) + (i - 1) * width, vals, width, yerr=err, color=c, label=mode, capsize=2)
    ax.set_xticks(range(len(SCEN))); ax.set_xticklabels(SCEN)
    ax.set_ylabel(f"session alert rate (>= {cfg.k_stations} of {cfg.n_stations} stations)")
    ax.set_title("Alert rates by scenario - 'none' and 'growth' should stay LOW, others HIGH (simulation)")
    ax.legend(); fig.tight_layout(); fig.savefig(os.path.join(outdir, "figures", "fig3_alert_rates.png"), dpi=150); plt.close(fig)

    th = (106, 80)
    strips = []
    for mode in ("dead_reckoning", "marker_corrected"):
        prev, news = montage[mode]
        for imgs in (prev, news):
            strips.append(np.hstack([cv2.resize(i, th) for i in imgs]))
    cv2.imwrite(os.path.join(outdir, "figures", "fig4_station_montage.png"), np.vstack(strips))
    return summary
