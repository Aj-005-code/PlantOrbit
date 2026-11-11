"""Run every experiment, then write results/summary.json, results/environment.json and results/SUMMARY.md."""
import hashlib, json, os, platform, sys, time
from dataclasses import asdict
import cv2, matplotlib, numpy as np
from orbitsim.config import CFG
from experiments import fig_layout, exp1_calibration, exp2_repeatability, exp3_change_detection

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")


def sha(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def main():
    os.makedirs(os.path.join(OUT, "figures"), exist_ok=True)
    t0 = time.time(); summary = {}
    for name, fn in (("layout", lambda: fig_layout.run(CFG, OUT)),
                     ("exp1_calibration", lambda: exp1_calibration.run(CFG, OUT)),
                     ("exp2_repeatability", lambda: exp2_repeatability.run(CFG, OUT)),
                     ("exp3_change_detection", lambda: exp3_change_detection.run(CFG, OUT))):
        s = time.time(); summary[name] = fn(); print(f"[done] {name} ({time.time() - s:.0f}s)", flush=True)
    json.dump(summary, open(os.path.join(OUT, "summary.json"), "w"), indent=2)
    files = {}
    for root, _, fs in os.walk(OUT):
        for f in sorted(fs):
            if f != "environment.json":
                p = os.path.join(root, f); files[os.path.relpath(p, OUT)] = sha(p)
    json.dump(dict(generated_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), runtime_s=round(time.time() - t0),
                   python=sys.version, platform=platform.platform(), numpy=np.__version__, opencv=cv2.__version__,
                   matplotlib=matplotlib.__version__, config=asdict(CFG), result_sha256=files),
              open(os.path.join(OUT, "environment.json"), "w"), indent=2)
    e1, e2 = summary["exp1_calibration"], summary["exp2_repeatability"]
    L = ["# Simulation summary (auto-generated; SIMULATED data only)", "",
         "## Exp 1 - calibration", f"- success (|radius err| <= 3 mm): robust {e1['robust_success_rate']:.1%}, plain LSQ {e1['lsq_success_rate']:.1%}",
         f"- robust radius error mm: median {e1['robust_radius_err_mm']['median']:.2f}, p95 {e1['robust_radius_err_mm']['p95']:.2f}, max {e1['robust_radius_err_mm']['max']:.2f}", "",
         "## Exp 2 - repeatability (position error vs ideal station, mm)"]
    for m in ("dead_reckoning", "marker_corrected"):
        d = e2[m]
        L.append(f"- {m}: mean {d['position_err_mm']['mean']:.1f}, p95 {d['position_err_mm']['p95']:.1f}; "
                 f"same-station offset between sessions p95 {d['inter_session_offset_mm']['p95']:.1f}")
    L += ["", "## Exp 3 - session alert rates (k=3 stations)", "| mode | scenario | role | alert rate | 95% CI | n |", "|---|---|---|---|---|---|"]
    for r in summary["exp3_change_detection"]["alert_rates"]:
        L.append(f"| {r['mode']} | {r['scenario']} | {r['role']} | {r['alert_rate']:.2f} | {r['ci_lo']:.2f}-{r['ci_hi']:.2f} | {r['n']} |")
    open(os.path.join(OUT, "SUMMARY.md"), "w").write("\n".join(L) + "\n")
    print("\n".join(L))


if __name__ == "__main__":
    main()
