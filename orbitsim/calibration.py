"""Self-calibration: ToF scan -> robust circle fit -> imaging radius -> station list."""
from dataclasses import dataclass
import numpy as np
from .config import Config


def tof_scan(pot_center, pot_r, cfg: Config, rng):
    """Simulate a rotating ToF scan whose beam is aimed at the mat centre.
    Returns an (n,2) array of surface hit points in the mat frame."""
    pot_center = np.asarray(pot_center, float)
    pts = []
    for a in np.deg2rad(np.arange(0.0, 360.0, cfg.scan_step_deg)):
        p = cfg.scan_radius * np.array([np.cos(a), np.sin(a)])
        u = -p / np.linalg.norm(p)
        f = p - pot_center
        b = 2.0 * (u @ f)
        c = f @ f - pot_r ** 2
        disc = b * b - 4.0 * c
        if disc < 0:
            continue                      # beam misses the pot
        t = (-b - np.sqrt(disc)) / 2.0
        if t <= 0 or rng.random() < cfg.tof_dropout:
            continue
        t += rng.normal(0.0, cfg.tof_noise_mm)
        if rng.random() < cfg.tof_outlier_prob:
            t -= rng.uniform(*cfg.tof_outlier_range)   # leaf in the beam
        pts.append(p + t * u)
    return np.array(pts)


def fit_circle_lsq(pts):
    """Algebraic (Kasa) least-squares circle fit."""
    x, y = pts[:, 0], pts[:, 1]
    A = np.c_[2 * x, 2 * y, np.ones(len(x))]
    b = x ** 2 + y ** 2
    cx, cy, c = np.linalg.lstsq(A, b, rcond=None)[0]
    r2 = c + cx ** 2 + cy ** 2
    return np.array([cx, cy]), float(np.sqrt(max(r2, 0.0)))


def circle_from_3(p):
    (x1, y1), (x2, y2), (x3, y3) = p
    A = np.array([[x2 - x1, y2 - y1], [x3 - x1, y3 - y1]])
    if abs(np.linalg.det(A)) < 1e-6:
        return None
    b = 0.5 * np.array([x2 ** 2 + y2 ** 2 - x1 ** 2 - y1 ** 2,
                        x3 ** 2 + y3 ** 2 - x1 ** 2 - y1 ** 2])
    c = np.linalg.solve(A, b)
    return c, float(np.hypot(x1 - c[0], y1 - c[1]))


def fit_circle_ransac(pts, cfg: Config, rng):
    n = len(pts)
    best, best_cnt = None, -1
    for _ in range(cfg.ransac_iters):
        idx = rng.choice(n, 3, replace=False)
        sol = circle_from_3(pts[idx])
        if sol is None:
            continue
        ctr, r = sol
        inl = np.abs(np.linalg.norm(pts - ctr, axis=1) - r) < cfg.ransac_tol_mm
        if inl.sum() > best_cnt:
            best, best_cnt = inl, int(inl.sum())
    if best is None or best_cnt < 8:
        ctr, r = fit_circle_lsq(pts)
        return ctr, r, np.ones(n, bool)
    ctr, r = fit_circle_lsq(pts[best])
    return ctr, r, best


@dataclass
class Calibration:
    center: np.ndarray
    radius: float
    imaging_radius: float
    stations: np.ndarray      # (n,3): x, y, heading (camera faces the pot centre)
    n_points: int
    n_inliers: int
    ok: bool


def make_stations(center, imaging_radius, n):
    ang = 2 * np.pi * np.arange(n) / n
    xy = np.asarray(center) + imaging_radius * np.c_[np.cos(ang), np.sin(ang)]
    return np.c_[xy, ang + np.pi]


def calibrate_from_points(pts, cfg: Config, rng, robust=True) -> Calibration:
    if len(pts) < 8:
        return Calibration(np.zeros(2), 0.0, 0.0, np.zeros((cfg.n_stations, 3)), len(pts), 0, False)
    if robust:
        ctr, r, inl = fit_circle_ransac(pts, cfg, rng)
    else:
        ctr, r = fit_circle_lsq(pts)
        inl = np.ones(len(pts), bool)
    ok = cfg.min_pot_radius <= r <= cfg.max_pot_radius
    R = r + cfg.standoff
    return Calibration(ctr, r, R, make_stations(ctr, R, cfg.n_stations), len(pts), int(inl.sum()), bool(ok))


def calibrate(pot_center, pot_r, cfg: Config, rng, robust=True) -> Calibration:
    return calibrate_from_points(tof_scan(pot_center, pot_r, cfg, rng), cfg, rng, robust)
