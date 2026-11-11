"""Per-station change metrics between two sessions (classical image processing only)."""
import cv2
import numpy as np
from .config import Config

K3 = np.ones((3, 3), np.uint8)


def segment_plant(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    m = ((h >= 22) & (h <= 95) & (s > 70) & (v > 40)).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, K3)
    return m.astype(bool), h


def pot_anchor(img):
    """(centre-x, top-y) of the terracotta pot body - the fixed reference used for alignment."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = (hsv[..., 0] >= 5) & (hsv[..., 0] <= 18) & (hsv[..., 1] > 110) & (hsv[..., 2] > 90)
    ys, xs = np.nonzero(m)
    if len(xs) < 300:                  # pot not sufficiently visible -> no reliable anchor
        return None
    return float(xs.mean()), float(np.percentile(ys, 5))


def compare_station(prev, new):
    """Returns (area_change, yellow_delta, iou, top_drop). Negative area_change = canopy shrank."""
    a0, a1 = pot_anchor(prev), pot_anchor(new)
    if a0 is not None and a1 is not None and abs(a0[0] - a1[0]) < 25 and abs(a0[1] - a1[1]) < 25:
        M = np.float32([[1, 0, a0[0] - a1[0]], [0, 1, a0[1] - a1[1]]])
        new = cv2.warpAffine(new, M, (new.shape[1], new.shape[0]), borderValue=(175, 175, 175))
    m0, h0 = segment_plant(prev)
    m1, h1 = segment_plant(new)
    A0, A1 = int(m0.sum()), int(m1.sum())
    area_change = (A1 - A0) / max(A0, 1)
    y0 = float((h0[m0] < 40).mean()) if A0 else 0.0
    y1 = float((h1[m1] < 40).mean()) if A1 else 0.0
    iou = float((m0 & m1).sum() / max((m0 | m1).sum(), 1))
    # canopy-top drop (positive = canopy got LOWER), normalised by prior canopy height above the pot rim
    top0 = float(np.percentile(np.nonzero(m0)[0], 2)) if A0 else 0.0
    top1 = float(np.percentile(np.nonzero(m1)[0], 2)) if A1 else float(prev.shape[0])
    ref = (a0[1] if a0 is not None else prev.shape[0]) - top0
    top_drop = (top1 - top0) / max(ref, 1.0)
    return area_change, y1 - y0, iou, top_drop


def station_flags(area_change, yellow_delta, top_drop, cfg: Config,
                  area_thresh=None, yellow_thresh=None, top_thresh=None):
    ta = cfg.area_thresh if area_thresh is None else area_thresh
    ty = cfg.yellow_thresh if yellow_thresh is None else yellow_thresh
    tt = cfg.top_thresh if top_thresh is None else top_thresh
    return ((np.asarray(area_change) < -ta) | (np.asarray(yellow_delta) > ty)
            | (np.asarray(top_drop) > tt))


def session_alert(flags, k):
    return int(np.sum(flags)) >= k
