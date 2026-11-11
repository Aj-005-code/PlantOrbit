# PlantOrbit-Sim — simulation of a self-calibrating orbital plant-imaging robot

**Everything in this project is SIMULATED.** No number here was measured on hardware. Never describe
these results as experimental or measured results in a patent specification, pitch or paper.

## 1. What the simulation is meant to support

The narrowed invention (see the earlier discussion) has three linked parts:

1. **Self-calibration** — a wheeled robot measures the pot (ranging-sensor scan + robust circle fit) and
   computes its own orbit radius and station list. No hardcoded pot size.
2. **Repeatable station poses** — the robot returns to the same angular stations every session. Dead
   reckoning drifts, so a fiducial (printed-marker) pose fix on the mat bounds the error.
3. **Multi-view change detection** — images taken at the *same* stations on different days are compared;
   an alert needs agreement from at least *k* of the 12 stations.

The three experiments test one question each:

| Exp | Question | Script |
|---|---|---|
| 1 | Does the scan + fit recover the pot radius for different pot sizes, with noise and leaf outliers? | `experiments/exp1_calibration.py` |
| 2 | Does the fiducial correction keep station poses repeatable where dead reckoning drifts? | `experiments/exp2_repeatability.py` |
| 3 | Does that repeatability reduce false alarms in image-based change detection while keeping detection power? | `experiments/exp3_change_detection.py` |

## 2. Environment setup (step by step)

Requirements: Python 3.10 or newer (built and tested on 3.12), about 500 MB disk, any OS, no GPU.

```bash
# 1. unzip the project and enter it
cd plantorbit-sim

# 2. create an isolated environment
python -m venv .venv
#   Windows PowerShell:  .venv\Scripts\Activate.ps1
#   Windows cmd:         .venv\Scripts\activate.bat
#   macOS / Linux:       source .venv/bin/activate

# 3. install pinned-minimum dependencies
pip install -r requirements.txt

# 4. run the tests (should print: 6 passed)
python -m pytest -q tests

# 5. run everything (about 4 minutes on a laptop CPU)
python run_all.py
```

Outputs appear in `results/`:

- `SUMMARY.md` — readable tables; `summary.json` — all numbers
- `exp*.csv` — raw and aggregated data (every table in your documents should trace to one of these)
- `figures/fig0..fig4*.png` — layout, calibration, repeatability, alert rates, station montage
- `environment.json` — library versions, full config, timestamp, SHA-256 hash of every result file

## 3. Code map

| File | Role |
|---|---|
| `orbitsim/config.py` | **Every assumption in one place** (sensor noise, slip, gyro drift, marker noise, thresholds) |
| `orbitsim/calibration.py` | Simulated ToF scan, least-squares and RANSAC circle fit, imaging radius, station generation |
| `orbitsim/robot.py` | Differential-drive model with separate TRUE and ESTIMATED pose; fiducial correction; session runner |
| `orbitsim/plant.py` | Procedural 3-D plant, five change scenarios, pinhole camera renderer, camera framing |
| `orbitsim/detect.py` | Pot-rim alignment, plant segmentation, per-station metrics, alert rules |
| `orbitsim/stats.py` | Wilson confidence interval, summary statistics |
| `experiments/` | The three experiments plus the layout figure |
| `tests/test_core.py` | Sanity tests (exact circle fit, zero-noise robot hits every station, markers help, determinism) |
| `run_all.py` | Runs everything and records a reproducibility manifest |

To change an assumption, edit `config.py` (for example `gyro_bias_sigma_dps`) and re-run. Every random
draw is seeded from `Config.seed`, so identical code and config give identical results.

## 4. Results from the final run (seed 20261011)

### Experiment 1 — calibration (500 random pots, radius 30–120 mm, offset up to 15 mm; 3 mm noise, 5 % dropout, 8 % leaf outliers)

- Robust (RANSAC) fit: radius error median 0.27 mm, 95th percentile 0.82 mm, max 1.27 mm; 100 % within 3 mm.
- Plain least squares on the same scans: only 69 % within 3 mm, because leaf outliers bias it.
- **Honest finding:** with *no* outliers and high sensor noise (6–10 mm), plain least squares is slightly
  *better* than RANSAC (for example 10 mm noise, no outliers: median 1.0 mm vs 1.95 mm), because the 6 mm
  inlier tolerance throws away good points. The robust fit pays off only when outliers exist. Tune
  `ransac_tol_mm` to your real sensor once measured.

### Experiment 2 — repeatability (200 sessions per mode, 12 stations, assumed hobby-grade robot)

| Mode | Mean position error | 95th pct | Same-station offset between two sessions, 95th pct |
|---|---|---|---|
| Dead reckoning (gyro + encoders, dock-homed) | 7.6 mm | 17.5 mm | 26.4 mm |
| Fiducial-corrected | 1.7 mm | 3.3 mm | 4.7 mm |

- Dead-reckoning error grows along the orbit (Fig. 2); the fiducial-corrected error stays flat.
- Sensitivity: with a 10× worse gyro bias (0.1 °/s), dead-reckoning mean error rises to 15.5 mm (95th pct 46 mm)
  while the corrected mode is unchanged (1.6 mm). The corrected mode scales with marker measurement noise
  instead (1 mm noise → 1.1 mm error; 5 mm → 5.5 mm).
- **Assumption warning:** these numbers depend entirely on the noise model in `config.py`. A better gyro,
  a stepper-based drive or a mechanical guide ring would shrink the dead-reckoning baseline. The
  comparison is "naive dead reckoning vs fiducial correction", not "every possible alternative".

### Experiment 3 — change detection (40 random plants, 5 scenarios, 12 stations, alert needs ≥ 3 stations)

| Scenario | Ideal poses | Dead reckoning | Fiducial-corrected |
|---|---|---|---|
| none (must NOT alert) | 0 % | **45 %** | 0 % |
| growth (must NOT alert) | 0 % | 5 % | 0 % |
| leaf drop (should alert) | 85 % | 90 % | 93 % |
| yellowing (should alert) | 100 % | 100 % | 100 % |
| wilting (should alert) | 100 % | 100 % | 100 % |

- The key result is the **false-alarm rate on an unchanged plant**: 45 % with dead reckoning versus 0/40
  (95 % interval 0–9 %) with fiducial correction.
- Read the dead-reckoning detection rates with care: they are inflated, because that mode raises alerts
  even when nothing changed. Detection power only means something together with the false-alarm rate.
- Threshold sweep (`exp3_threshold_sweep.csv`, area threshold 0.06): requiring more stations to agree cuts
  dead-reckoning false alarms (44 % at k=1, 25 % at k=3, 9 % at k=5) but never reaches the corrected mode's 0 %.
  Corrected-mode detection stays at 92–98 % across k=1–5.
- Leaf-drop detection (about 93 %) is the weakest case, since removing 3 of 22 leaves changes few pixels
  from some angles. Treat 40 trials as a small sample; the intervals in `exp3_alert_rates.csv` are wide.

## 5. Development log (keep this kind of record — it is good practice and honest)

1. **First full run:** the corrected mode and even the *ideal* mode showed 12–17 % false alarms on
   unchanged plants.
2. **Diagnosis** (inspecting `exp3_raw_metrics.csv` and a rendered frame): the camera framing left only a
   thin sliver of the pot in view, so the pot-rim alignment anchor was unstable and sometimes misaligned
   the image pair. This was a flaw in my set-up, not in the idea.
3. **Fixes:** (a) camera height and tilt are now chosen once per plant so canopy and pot rim both fit
   (`plant.framing`); (b) alignment is skipped when fewer than 300 pot pixels are visible or the proposed
   shift exceeds 25 px; (c) focal length changed 250 → 220 px for a wider field of view.
4. **Re-validation:** the fix was checked on a separate development seed (`seed+33`), then the final
   experiment was run on a **fresh test seed (`seed+4`)** so results were not tuned on the data reported.
5. Thresholds (6 % area, 8 % yellow, 8 % canopy-top, k=3) were set a priori, not fitted to results; the
   sweep CSV shows sensitivity to them.

## 6. What this simulation does and does NOT show

**It supports:** the algorithms are logically consistent and workable; fiducial correction should bound
pose error under the assumed noise; bounded pose error should reduce false alarms in multi-view
comparison; calibration adapts to pot size without manual coordinates.

**It does not show:** real sensor behaviour (ToF returns off glossy pots, fiducial detection under glare,
shadows, wind or watering changes); real plant appearance (the plant is a simple procedural model, not a
photograph of a living plant); non-circular pots (square or tapered pots break the circle assumption);
marker visibility from all 12 stations (assumed: at least two fiducials are always visible to a pose
camera, which is a hardware design detail to settle); real drift (measure your own robot).

## 7. Patent-readiness checklist

Not legal advice — confirm every step with a registered patent agent.

1. **Do not disclose before filing.** No GitHub push, video, post or paper until the provisional is filed.
2. **Version control privately:** `git init`, commit this project, `git tag sim-v1-final-run`. Commit
   timestamps give a dated record of what you had and when. Keep the repo private.
3. **Inventor's notebook:** one dated entry per work session (what you tried, what failed, what changed).
   Section 5 above is the model. Record who contributed which idea — inventorship matters.
4. **Check your institution's IP policy** (VIT) before using lab space, funds or staff time.
5. **Archive the run:** keep `results/environment.json` (library versions, config, file hashes) and the
   exact zip you worked from.
6. **Map results to the provisional specification:**

| Specification section | Source in this project |
|---|---|
| Background / problem | Fixed-coordinate systems fail across pot sizes; dead reckoning drifts; single-image checks miss trends |
| Summary of the invention | Sections 1 and 3 of this README |
| Brief description of drawings | `fig0_layout`, flowcharts you draw from `calibration.py` and `robot.py`, block diagram, pin diagram |
| Detailed description / enablement | The algorithms, parameters and assumptions in `config.py`, written out in words and pseudocode |
| Examples | Experiments 1–3, **explicitly labelled as simulation**, with the noise model stated |
| Claims | Method claims: scan → fit → derive orbit; fiducial-corrected station revisit; same-station multi-view comparison with k-of-n agreement. Draft with your agent. |

7. **Plot figures properly:** the PNGs here are evidence figures, not formal patent drawings. Formal drawings
   are black-and-white line drawings; your agent will advise.
8. **Prior art:** the earlier search was informal. A professional IPC-coded search is still needed before
   finalising claims, and linear-rail repeatable-pose systems (found earlier) are the closest art to
   distinguish against.
9. **File order:** DPIIT startup registration → provisional application at ipindiaonline.gov.in →
   12 months to replace simulated evidence with measured data and file the complete specification.

## 8. Replacing each assumption with a measurement (the next 12 months)

| Simulated assumption (`config.py`) | How to measure it on hardware |
|---|---|
| `tof_noise_mm`, dropout, outliers | Scan 5 real pots (matte, glossy, different heights) 50 times each; log error vs a ruler |
| `slip_*`, `lateral_sigma_frac` | Command 1 m straight and a full circle 30 times on your desk; measure the final position |
| `gyro_bias_sigma_dps`, `gyro_arw` | Log the stationary MPU6050 gyro for 10 minutes after calibration; fit drift |
| `dock_pos_sigma` | Dock 30 times; measure offset |
| `marker_pos_sigma`, visibility | Photograph the mat from each station; compare marker-derived pose to a ruler/protractor |
| Plant appearance | Image a real plant over weeks; rerun `detect.py` on real photos |
| Thresholds | Choose from real null (unchanged-plant) image pairs, then test on separate days |

When these are measured, update `config.py`, re-run, and report the measured values alongside the simulated ones.
