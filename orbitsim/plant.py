"""Procedural 3-D plant + pinhole camera renderer (synthetic images for change-detection tests)."""
from dataclasses import dataclass, replace
import copy
import cv2
import numpy as np
from .config import Config

BG_GREY = 175
POT_HSV = (12, 170, 175)
SOIL_BGR = (20, 30, 45)


def hsv2bgr(h, s, v):
    px = np.uint8([[[np.clip(h, 0, 179), np.clip(s, 0, 255), np.clip(v, 0, 255)]]])
    return tuple(int(x) for x in cv2.cvtColor(px, cv2.COLOR_HSV2BGR)[0, 0])


@dataclass
class Leaf:
    azimuth: float
    height: float
    length: float
    width: float
    elev: float
    droop: float
    hue: float
    sat: float
    val: float
    yellow: float = 0.0


@dataclass
class Plant:
    pot_r: float
    pot_h: float
    stem_h: float
    leaves: list


def make_plant(rng, pot_r, n_leaves=22) -> Plant:
    stem_h = 70.0 + 0.9 * pot_r
    leaves = []
    for _ in range(n_leaves):
        leaves.append(Leaf(
            azimuth=rng.uniform(0, 2 * np.pi),
            height=stem_h * rng.uniform(0.3, 0.9),
            length=pot_r * rng.uniform(0.7, 1.1) + 10.0,
            width=0.0,
            elev=np.deg2rad(rng.uniform(-10, 45)),
            droop=0.0,
            hue=rng.normal(55, 4), sat=rng.normal(170, 15), val=rng.normal(140, 18)))
    for lf in leaves:
        lf.width = lf.length * rng.uniform(0.35, 0.5)
    return Plant(pot_r, 1.1 * pot_r, stem_h, leaves)


def apply_scenario(plant: Plant, scenario: str, rng) -> Plant:
    p = copy.deepcopy(plant)
    if scenario == "none":
        pass
    elif scenario == "growth":                         # normal healthy growth (should NOT alert)
        for lf in p.leaves:
            lf.length *= 1.04
            lf.width *= 1.04
        p.stem_h *= 1.02
    elif scenario == "leaf_drop":                      # lose ~15 % of leaves (min 3)
        k = max(3, int(round(0.15 * len(p.leaves))))
        drop = set(rng.choice(len(p.leaves), k, replace=False).tolist())
        p.leaves = [lf for i, lf in enumerate(p.leaves) if i not in drop]
    elif scenario == "yellowing":                      # 30 % of leaves turn yellow
        k = int(round(0.30 * len(p.leaves)))
        for i in rng.choice(len(p.leaves), k, replace=False):
            p.leaves[i].yellow = 0.9
    elif scenario == "wilting":                        # all leaves droop
        for lf in p.leaves:
            lf.droop = 0.9
    else:
        raise ValueError(scenario)
    return p


def leaf_polygon(lf: Leaf, n=10):
    s = np.linspace(0, 1, n)
    h = np.array([np.cos(lf.azimuth), np.sin(lf.azimuth), 0.0])
    z = np.array([0.0, 0.0, 1.0])
    pos = np.zeros((n, 3))
    pos[0] = [0, 0, lf.height]
    step = lf.length / (n - 1)
    for i in range(1, n):
        beta = lf.elev - lf.droop * s[i - 1]
        pos[i] = pos[i - 1] + step * (np.cos(beta) * h + np.sin(beta) * z)
    wid = lf.width * np.sin(np.pi * s) ** 0.8
    nv = np.array([-np.sin(lf.azimuth), np.cos(lf.azimuth), 0.0])
    return np.vstack([pos + 0.5 * wid[:, None] * nv, (pos - 0.5 * wid[:, None] * nv)[::-1]])


def leaf_color(lf: Leaf):
    y = lf.yellow
    return hsv2bgr(lf.hue * (1 - y) + 27 * y, lf.sat * (1 - 0.1 * y), lf.val * (1 + 0.25 * y))


def camera_basis(theta, pitch):
    f = np.array([np.cos(pitch) * np.cos(theta), np.cos(pitch) * np.sin(theta), np.sin(pitch)])
    right = np.array([np.sin(theta), -np.cos(theta), 0.0])
    return f, right, np.cross(right, f)


def project(P, C, f, r, u, cfg: Config):
    d = P - C
    zc, xc, yc = d @ f, d @ r, d @ u
    zs = np.maximum(zc, 1e-6)
    uv = np.stack([cfg.img_w / 2 + cfg.focal_px * xc / zs, cfg.img_h / 2 - cfg.focal_px * yc / zs], 1)
    return uv, zc > 1.0


def background(cfg: Config):
    g = np.linspace(BG_GREY + 12, BG_GREY - 12, cfg.img_h)[:, None]
    return np.repeat(np.repeat(g, cfg.img_w, 1)[:, :, None], 3, 2).astype(np.uint8).copy()


def render(plant: Plant, cam_xy_local, cam_theta, cam_z, pitch, cfg: Config, rng, gain=1.0):
    """Render the plant (standing at the origin) from a camera at cam_xy_local, heading cam_theta."""
    img = background(cfg)
    C = np.array([cam_xy_local[0], cam_xy_local[1], cam_z])
    f, r, u = camera_basis(cam_theta, pitch)

    ang = np.linspace(0, 2 * np.pi, 48, endpoint=False)
    top = np.c_[plant.pot_r * np.cos(ang), plant.pot_r * np.sin(ang), np.zeros_like(ang)]
    bot = np.c_[0.7 * plant.pot_r * np.cos(ang), 0.7 * plant.pot_r * np.sin(ang), -plant.pot_h * np.ones_like(ang)]
    uv, ok = project(np.vstack([top, bot]), C, f, r, u, cfg)
    if ok.all():
        hull = cv2.convexHull(uv.astype(np.float32)).astype(np.int32)
        cv2.fillPoly(img, [hull], hsv2bgr(*POT_HSV), cv2.LINE_AA)
        cv2.fillPoly(img, [uv[:48].astype(np.int32)], SOIL_BGR, cv2.LINE_AA)

    s0, ok0 = project(np.array([[0, 0, 0.0], [0, 0, plant.stem_h]]), C, f, r, u, cfg)
    if ok0.all():
        cv2.line(img, tuple(int(v) for v in s0[0]), tuple(int(v) for v in s0[1]),
                 hsv2bgr(45, 140, 90), 2, cv2.LINE_AA)

    items = []
    for lf in plant.leaves:
        poly = leaf_polygon(lf)
        items.append((float((poly.mean(0) - C) @ f), poly, leaf_color(lf)))
    for _, poly, col in sorted(items, key=lambda t: -t[0]):
        uvp, okp = project(poly, C, f, r, u, cfg)
        if okp.all():
            cv2.fillPoly(img, [uvp.astype(np.int32)], col, cv2.LINE_AA)

    out = img.astype(np.float32) * gain + rng.normal(0, cfg.sensor_noise, img.shape)
    return np.clip(out, 0, 255).astype(np.uint8)


def framing(plant: Plant, imaging_radius: float):
    """Fixed camera height and tilt chosen once per plant at set-up so the canopy and the pot rim
    both fit in frame. Returns (camera_z, pitch)."""
    z_top = 1.08 * max(leaf_polygon(lf)[:, 2].max() for lf in plant.leaves)   # 8 % growth margin
    z_bot = -0.6 * plant.pot_h
    look = 0.5 * (z_top + z_bot)
    cam_z = look + 15.0
    return cam_z, float(np.arctan2(look - cam_z, imaging_radius))
