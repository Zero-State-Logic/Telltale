"""Batched physics of an SG90-class servo pulling a spring latch.

State per trial: servo output angle theta and rate omega, bolt position x
(0 = thrown/locked, + = retracting) and velocity v.

  servo   J*omega' = N*eta*Kt*i_m - r*F_link - b_g*omega - friction (Coulomb + stiction)
  motor   i_m = (d*V_s - Ke*N*omega) / R          (inductance neglected: L/R ~ 0.1 ms)
  drive   d = clip(kp * (error beyond the deadband), -1, 1)   averaged H-bridge duty
  supply  V_s = V_oc - R_src*I_s,  I_s = I_q + d*i_m          (what the INA219 sees)
  linkage F_link = k_c * (stretch beyond the lost motion) + damping, both directions
  bolt    m*v' = F_link - (F_pre + k_s*x) + F_stops + F_ext - b_b*v - friction

Integrated with semi-implicit Euler at 5 kHz; Karnopp-style stiction on both
the gear train and the bolt. Every parameter is an array, one entry per
trial, so thousands of actuations run in one vectorised loop.
"""
from __future__ import annotations

import numpy as np

from .params import DEG

DT = 2.0e-4             # s, 5 kHz integration step
STEPS_PER_MS = 5
LOCKED_DEG = 10.0       # must match TT_ANGLE_LOCKED in telltale_motion.h
OMEGA_EPS = 0.02        # rad/s, stiction window on the servo
V_EPS = 2.0e-4          # m/s, stiction window on the bolt


def _friction_step(vel, force, f_c, f_s, mass, dt, eps):
    """Advance a velocity under an applied force with Coulomb friction and
    stiction (Karnopp): inside the eps window the body sticks unless the
    applied force beats static friction; a step that would reverse the
    motion stops it instead."""
    moving = np.abs(vel) > eps
    direction = np.where(moving, np.sign(vel), np.sign(force))
    v_new = vel + (force - f_c * direction) / mass * dt
    stick = (~moving) & (np.abs(force) <= f_s)
    v_new = np.where(stick, 0.0, v_new)
    v_new = np.where(moving & (v_new * vel < 0.0), 0.0, v_new)
    return v_new


class Recorder:
    """Samples the supply current like an INA219 would: one reading every
    `period` steps, averaged over the last `avg` steps (conversion window)."""

    def __init__(self, n_trials, n_steps, period, avg, keep_state):
        self.period = period
        self.avg = avg
        self.n_rec = n_steps // period
        self.current = np.zeros((n_trials, self.n_rec), dtype=np.float32)
        self.vbus = np.zeros((n_trials, self.n_rec), dtype=np.float32)
        self.keep_state = keep_state
        if keep_state:
            self.theta = np.zeros((n_trials, self.n_rec), dtype=np.float32)
            self.x = np.zeros((n_trials, self.n_rec), dtype=np.float32)
        self._acc_i = np.zeros(n_trials)
        self._acc_v = np.zeros(n_trials)

    def step(self, k, i_supply, v_supply, theta, x):
        phase = k % self.period
        if phase >= self.period - self.avg:
            self._acc_i += i_supply
            self._acc_v += v_supply
        if phase == self.period - 1:
            j = k // self.period
            if j < self.n_rec:
                self.current[:, j] = self._acc_i * (1000.0 / self.avg)
                self.vbus[:, j] = self._acc_v / self.avg
                if self.keep_state:
                    self.theta[:, j] = theta / DEG
                    self.x[:, j] = x * 1000.0
            self._acc_i[:] = 0.0
            self._acc_v[:] = 0.0


def _ext_force(f_ext, k, n_trials):
    if f_ext is None:
        return 0.0
    if callable(f_ext):
        return f_ext(k)
    return f_ext[:, k]


def simulate(p, cmd_deg, f_ext=None, *, period=STEPS_PER_MS, avg=1,
             relock_step=None, pre_roll=500, init_offset_deg=None,
             keep_state=False):
    """Run the batched model.

    p           parameter dict (arrays of length B), see params.py
    cmd_deg     commanded angle per step, shape (n_steps,) shared or (B, n_steps)
    f_ext       external force on the bolt per step, N (+ = toward retraction):
                None, an array (B, n_steps) or a callable step -> (B,)
    period/avg  sampling period and conversion-averaging window, in steps
    relock_step step index from which the relock jam (p["jam_lo"]) is active
    pre_roll    settling steps at the first command before recording
    Returns a dict with the true supply current in mA (no sensor model yet),
    the servo-side bus voltage and, if keep_state, theta (deg) and x (mm).
    """
    cmd = np.asarray(cmd_deg, dtype=np.float64)
    n_trials = len(p["R"])
    n_steps = cmd.shape[-1]
    shared_cmd = cmd.ndim == 1

    kt_out = p["N"] * p["eta"] * p["Ke"]          # N*m per amp at the output
    r_tot = p["R_src"] + p["shunt"]
    half_db = 0.5 * p["deadband"]
    half_slack = 0.5 * p["slack"]
    theta_ref = LOCKED_DEG * DEG + p["delta"]     # linkage neutral angle
    hi_stop = np.minimum(p["x_max"], p["jam_hi"])
    first = cmd[0] if shared_cmd else cmd[:, 0]

    offset = np.zeros(n_trials) if init_offset_deg is None else init_offset_deg
    theta = (first + offset) * DEG
    omega = np.zeros(n_trials)
    x = np.full(n_trials, -1.0) * p["F_pre"] / p["k_stop"]
    v = np.zeros(n_trials)
    rec = Recorder(n_trials, n_steps, period, avg, keep_state)

    for k in range(-pre_roll, n_steps):
        c = (cmd[max(k, 0)] if shared_cmd else cmd[:, max(k, 0)])
        err = c - theta / DEG
        beyond = np.maximum(np.abs(err) - half_db, 0.0)
        d = np.clip(p["kp"] * np.sign(err) * beyond, -1.0, 1.0)
        w_m = p["N"] * omega
        v_s = (p["Voc"] - r_tot * p["Iq"] + r_tot * d * p["Ke"] * w_m / p["R"]) / (
            1.0 + r_tot * d * d / p["R"])
        i_m = (d * v_s - p["Ke"] * w_m) / p["R"]
        i_supply = p["Iq"] + d * i_m

        stretch = p["r"] * (theta - theta_ref) - x
        over = np.abs(stretch) - half_slack
        engaged = over > 0.0
        f_link = p["k_c"] * np.sign(stretch) * np.maximum(over, 0.0) + \
            p["c_l"] * (p["r"] * omega - v) * engaged

        tau = kt_out * i_m - f_link * p["r"] - p["b_g"] * omega
        omega = _friction_step(omega, tau, p["tau_c"], p["tau_s"], p["J"], DT, OMEGA_EPS)
        theta = theta + omega * DT

        lo_stop = p["jam_lo"] if (relock_step is not None and k >= relock_step) else 0.0
        pen_lo = lo_stop - x
        pen_hi = x - hi_stop
        f_stop = np.where(pen_lo > 0.0, np.maximum(p["k_stop"] * pen_lo - p["c_stop"] * v, 0.0), 0.0)
        f_stop += np.where(pen_hi > 0.0, np.minimum(-p["k_stop"] * pen_hi - p["c_stop"] * v, 0.0), 0.0)
        f_ext_k = _ext_force(f_ext, k, n_trials) if k >= 0 else 0.0
        force = f_link - (p["F_pre"] + p["k_s"] * x) + f_stop + f_ext_k - p["b_b"] * v
        v = _friction_step(v, force, p["Fb_c"], p["Fb_s"], p["m_b"], DT, V_EPS)
        x = x + v * DT

        if k >= 0:
            rec.step(k, i_supply, v_s, theta, x)

    out = {"current_ma": rec.current, "vbus_v": rec.vbus}
    if keep_state:
        out["theta_deg"] = rec.theta
        out["x_mm"] = rec.x
    return out
