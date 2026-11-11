"""All tunable assumptions live here. Units: millimetres, radians, seconds (unless noted).

Every number below is an ASSUMPTION about a small hobby-grade differential-drive robot.
They are NOT measurements. Replace them with measured values once hardware exists.
"""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class Config:
    seed: int = 20261011

    # ---- world ----
    mat_radius: float = 300.0
    scan_radius: float = 250.0      # radius of the calibration scan circle
    standoff: float = 180.0         # imaging radius = pot radius + standoff
    n_stations: int = 12
    dock_radius: float = 275.0
    dock_angle_deg: float = -15.0

    # ---- ToF ranging sensor (calibration scan) ----
    scan_step_deg: float = 5.0
    tof_noise_mm: float = 3.0
    tof_dropout: float = 0.05        # probability a reading is lost
    tof_outlier_prob: float = 0.08   # probability a leaf intrudes into the beam
    tof_outlier_range: tuple = (10.0, 40.0)   # leaf makes the reading SHORTER by this much
    ransac_tol_mm: float = 6.0
    ransac_iters: int = 300
    min_pot_radius: float = 15.0
    max_pot_radius: float = 150.0

    # ---- robot motion / estimation ----
    drive_speed: float = 100.0                 # mm/s
    turn_rate: float = float(np.deg2rad(60))   # rad/s
    slip_bias: float = 0.005                   # systematic distance scale error (0.5 %)
    slip_sigma: float = 0.015                  # random distance scale error per move (1.5 %)
    lateral_sigma_frac: float = 0.01           # sideways slip as fraction of distance
    gyro_bias_sigma_dps: float = 0.03          # residual gyro bias after calibration, per session
    gyro_arw_dps_rts: float = 0.02             # gyro angle random walk, deg / sqrt(s)
    turn_resid_deg: float = 0.1                # closed-loop turn residual
    dock_pos_sigma: float = 2.0                # docking repeatability (mm)
    dock_heading_sigma_deg: float = 0.5
    dwell_s: float = 2.0
    observe_s: float = 0.5

    # ---- fiducial (marker) pose correction ----
    marker_pos_sigma: float = 1.5
    marker_heading_sigma_deg: float = 0.3
    marker_gain: float = 0.9
    corr_iters: int = 3
    corr_tol: float = 1.0

    # ---- synthetic camera ----
    img_w: int = 320
    img_h: int = 240
    focal_px: float = 220.0
    illum_sigma: float = 0.03        # session-to-session global brightness variation
    sensor_noise: float = 4.0        # gaussian pixel noise (grey levels)

    # ---- change detection ----
    area_thresh: float = 0.06        # alert if projected plant area falls by more than 6 %
    yellow_thresh: float = 0.08      # alert if yellow pixel fraction rises by more than 8 points
    top_thresh: float = 0.08         # alert if canopy top drops by more than 8 % of canopy height
    k_stations: int = 3              # session alert needs >= k stations flagged


CFG = Config()
