import os, sys
from dataclasses import replace
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from orbitsim.config import CFG
from orbitsim.calibration import fit_circle_lsq, calibrate, make_stations
from orbitsim.robot import run_session, pose_errors
from orbitsim.plant import make_plant, apply_scenario, render, framing
from orbitsim.detect import segment_plant, pot_anchor, compare_station

NOISELESS = replace(CFG, tof_noise_mm=0.0, tof_dropout=0.0, tof_outlier_prob=0.0)
ZERO_ROBOT = replace(CFG, slip_bias=0.0, slip_sigma=0.0, lateral_sigma_frac=0.0, gyro_bias_sigma_dps=0.0,
                     gyro_arw_dps_rts=0.0, turn_resid_deg=0.0, dock_pos_sigma=0.0, dock_heading_sigma_deg=0.0)


def test_circle_fit_exact():
    t = np.linspace(0, 2 * np.pi, 40, endpoint=False)
    pts = np.c_[3 + 50 * np.cos(t), -4 + 50 * np.sin(t)]
    c, r = fit_circle_lsq(pts)
    assert abs(r - 50) < 1e-6 and np.allclose(c, [3, -4], atol=1e-6)


def test_calibration_noiseless_recovers_radius():
    rng = np.random.default_rng(0)
    for r in (35, 70, 110):
        cal = calibrate((5.0, -3.0), r, NOISELESS, rng)
        assert cal.ok and abs(cal.radius - r) < 1.0


def test_zero_noise_robot_hits_every_station():
    st = make_stations((0, 0), 240.0, CFG.n_stations)
    tp = run_session(ZERO_ROBOT, np.random.default_rng(0), st, use_markers=False)
    pos, head = pose_errors(tp, st)
    assert pos.max() < 1e-6 and head.max() < 1e-6


def test_markers_reduce_error():
    st = make_stations((0, 0), 240.0, CFG.n_stations)
    rng = np.random.default_rng(1)
    dr = np.mean([pose_errors(run_session(CFG, rng, st, False), st)[0].mean() for _ in range(30)])
    mk = np.mean([pose_errors(run_session(CFG, rng, st, True), st)[0].mean() for _ in range(30)])
    assert mk < 0.5 * dr


def _img(seed):
    rng = np.random.default_rng(seed)
    p = make_plant(rng, 60.0)
    R = 60.0 + CFG.standoff
    Hc, pitch = framing(p, R)
    return p, render(p, (R, 0.0), np.pi, Hc, pitch, CFG, np.random.default_rng(seed + 1)), (R, Hc, pitch)


def test_render_deterministic_and_segmentable():
    _, a, _ = _img(3)
    _, b, _ = _img(3)
    assert np.array_equal(a, b)
    m, _ = segment_plant(a)
    assert m.sum() > 1000 and pot_anchor(a) is not None


def test_unchanged_plant_gives_no_change():
    p, a, (R, Hc, pitch) = _img(4)
    b = render(apply_scenario(p, "none", np.random.default_rng(0)), (R, 0.0), np.pi, Hc, pitch, CFG,
               np.random.default_rng(99))
    area, yellow, iou, top = compare_station(a, b)
    assert abs(area) < 0.03 and yellow < 0.02 and iou > 0.9 and abs(top) < 0.03
