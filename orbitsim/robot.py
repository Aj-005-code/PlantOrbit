"""Differential-drive robot with an explicit TRUE pose and a separate ESTIMATED pose.

The controller only ever sees the estimate. Slip makes the true pose deviate; the gyro
drifts, so the estimated heading deviates from the true heading. A fiducial observation
(simulated as a noisy measurement of the TRUE pose) pulls the estimate back.
"""
import numpy as np
from .config import Config


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class RobotSim:
    def __init__(self, cfg: Config, rng, dock_pose, gyro_bias_sigma_dps=None):
        self.cfg, self.rng = cfg, rng
        x, y, th = dock_pose
        self.true_xy = np.array([x, y], float) + rng.normal(0, cfg.dock_pos_sigma, 2)
        self.true_th = th + np.deg2rad(rng.normal(0, cfg.dock_heading_sigma_deg))
        self.est_xy = np.array([x, y], float)          # robot believes it is exactly at the dock
        self.est_th = float(th)
        sig = cfg.gyro_bias_sigma_dps if gyro_bias_sigma_dps is None else gyro_bias_sigma_dps
        self.bias = np.deg2rad(rng.normal(0, sig))     # rad/s
        self.t = 0.0

    def _advance(self, dt):
        self.t += dt
        drift = self.bias * dt + np.deg2rad(self.cfg.gyro_arw_dps_rts) * np.sqrt(dt) * self.rng.normal()
        self.est_th += drift                           # estimate drifts, truth does not

    def turn_to(self, target):
        self._advance(abs(wrap(target - self.est_th)) / self.cfg.turn_rate)
        delta = wrap(target - self.est_th)
        resid = np.deg2rad(self.rng.normal(0, self.cfg.turn_resid_deg))
        self.true_th += delta + resid                  # gyro sees the true rotation
        self.est_th += delta + resid

    def drive(self, d):
        c = self.cfg
        self._advance(d / c.drive_speed)
        eps = c.slip_bias + self.rng.normal(0, c.slip_sigma)
        lat = self.rng.normal(0, c.lateral_sigma_frac * d)
        t, e = self.true_th, self.est_th
        self.true_xy = self.true_xy + d * (1 + eps) * np.array([np.cos(t), np.sin(t)]) \
            + lat * np.array([-np.sin(t), np.cos(t)])
        self.est_xy = self.est_xy + d * np.array([np.cos(e), np.sin(e)])

    def go_to(self, target_xy):
        v = np.asarray(target_xy) - self.est_xy
        dist = float(np.linalg.norm(v))
        if dist < 1e-6:
            return
        self.turn_to(np.arctan2(v[1], v[0]))
        self.drive(dist)

    def observe(self):
        """Fiducial pose fix: noisy measurement of the TRUE pose blended into the estimate."""
        c, r = self.cfg, self.rng
        self._advance(c.observe_s)
        z_xy = self.true_xy + r.normal(0, c.marker_pos_sigma, 2)
        z_th = self.true_th + np.deg2rad(r.normal(0, c.marker_heading_sigma_deg))
        self.est_xy = self.est_xy + c.marker_gain * (z_xy - self.est_xy)
        self.est_th = self.est_th + c.marker_gain * wrap(z_th - self.est_th)

    def reach(self, target_xy, target_th, use_markers):
        target_xy = np.asarray(target_xy, float)
        self.go_to(target_xy)
        if use_markers:
            for _ in range(self.cfg.corr_iters):
                self.observe()
                if np.linalg.norm(self.est_xy - target_xy) < self.cfg.corr_tol:
                    break
                self.go_to(target_xy)
        self.turn_to(target_th)
        if use_markers:
            self.observe()
            self.turn_to(target_th)

    def true_pose(self):
        return np.array([self.true_xy[0], self.true_xy[1], self.true_th])


def run_session(cfg: Config, rng, stations, use_markers, gyro_bias_sigma_dps=None):
    """Visit every station once. Returns (n,3) array of TRUE poses at the moment of capture."""
    a = np.deg2rad(cfg.dock_angle_deg)
    dock = (cfg.dock_radius * np.cos(a), cfg.dock_radius * np.sin(a), a + np.pi)
    robot = RobotSim(cfg, rng, dock, gyro_bias_sigma_dps)
    out = []
    for st in stations:
        robot.reach(st[:2], st[2], use_markers)
        out.append(robot.true_pose())
        robot._advance(cfg.dwell_s)
    return np.array(out)


def pose_errors(true_poses, stations):
    """Position error (mm) and heading error (deg) of each capture versus the ideal station."""
    pos = np.linalg.norm(true_poses[:, :2] - stations[:, :2], axis=1)
    head = np.rad2deg(np.abs(wrap(true_poses[:, 2] - stations[:, 2])))
    return pos, head
