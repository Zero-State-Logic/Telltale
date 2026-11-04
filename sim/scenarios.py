"""Scenario builders: lock/unlock cycles, idle prying attacks, benign knocks
and long healthy idle streams, all measured through the INA219 model."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import core, physics, sensor

STEP_US = int(physics.DT * 1e6)            # 200 us
HOLD_MS = 200                               # unlocked hold simulated between windows
IDLE_PERIOD = 50                            # steps per 10 ms idle sample
IDLE_AVG = 43                               # 8.6 ms averaging window (x16 at 12 bit)


@dataclass(frozen=True)
class CycleLayout:
    cmd: np.ndarray        # commanded angle per physics step
    relock_step: int
    unlock_ms: int
    unlock_motion_ms: int
    relock_ms: int
    relock_motion_ms: int

    @property
    def relock_start_ms(self) -> int:
        return self.unlock_ms + HOLD_MS


def cycle_layout() -> CycleLayout:
    """Unlock window, a short unlocked hold, then the relock window. The
    commanded angles come from the firmware's own motion code."""
    u_cmd, u_motion, u_total = core.motion_profile(core.UNLOCK, STEP_US)
    r_cmd, r_motion, r_total = core.motion_profile(core.RELOCK, STEP_US)
    hold = np.full(HOLD_MS * physics.STEPS_PER_MS, u_cmd[-1])
    cmd = np.concatenate([u_cmd, hold, r_cmd])
    return CycleLayout(cmd, len(u_cmd) + len(hold), u_total, u_motion, r_total, r_motion)


def simulate_cycles(p: dict, rng: np.random.Generator, layout: CycleLayout,
                    chunk: int = 2000, keep_state: bool = False) -> dict:
    """Simulate one unlock+relock cycle per trial. Returns measured traces
    (1 kHz, mA) for the unlock and relock windows, plus the true current."""
    n = len(p["R"])
    unlock, relock, true_all, theta, x = [], [], [], [], []
    for start in range(0, n, chunk):
        idx = np.arange(start, min(n, start + chunk))
        q = {k: v[idx] for k, v in p.items()}
        offset = rng.uniform(-0.5, 0.5, len(idx)) * q["deadband"]
        out = physics.simulate(q, layout.cmd, relock_step=layout.relock_step,
                               init_offset_deg=offset, keep_state=keep_state)
        meas = sensor.measure(out["current_ma"], q, rng, "fast")
        a, b = layout.relock_start_ms, layout.relock_start_ms + layout.relock_ms
        unlock.append(meas[:, :layout.unlock_ms])
        relock.append(meas[:, a:b])
        true_all.append(out["current_ma"])
        if keep_state:
            theta.append(out["theta_deg"])
            x.append(out["x_mm"])
    res = {"unlock": np.concatenate(unlock), "relock": np.concatenate(relock),
           "true": np.concatenate(true_all)}
    if keep_state:
        res["theta"] = np.concatenate(theta)
        res["x"] = np.concatenate(x)
    return res


# ------------------------------------------------------------------ idle
def prying_force(onset_s, ramp_s, peak_n, freq_hz, phase, hold_s=2.5):
    """Prying with a lever: force ramps up, then pumps at a few Hz."""
    def force(k):
        tau = k * physics.DT - onset_s
        ramp = np.clip(tau / ramp_s, 0.0, 1.0)
        pump = 1.0 + 0.3 * np.sin(2.0 * np.pi * freq_hz * tau + phase)
        active = (tau >= 0.0) & (tau < hold_s)
        return peak_n * ramp * pump * active
    return force


def slow_push_force(onset_s, ramp_s, peak_n):
    """A deliberately slow push, trying to creep under the baseline tracker."""
    def force(k):
        tau = k * physics.DT - onset_s
        return peak_n * np.clip(tau / ramp_s, 0.0, 1.0)
    return force


def knock_force(onset_s, dur_s, amp_n):
    """A benign knock or slam reaching the bolt: one half-sine pulse."""
    def force(k):
        tau = k * physics.DT - onset_s
        active = (tau >= 0.0) & (tau < dur_s)
        return amp_n * np.sin(np.pi * np.clip(tau / dur_s, 0.0, 1.0)) * active
    return force


def simulate_idle(p: dict, rng: np.random.Generator, force_fn, duration_s: float,
                  chunk: int = 2000) -> dict:
    """Locked and idle at 10 deg with an external force on the bolt. Returns
    true and measured 100 Hz current and the bolt position (mm)."""
    n = len(p["R"])
    n_steps = int(round(duration_s / physics.DT))
    cmd = np.full(n_steps, physics.LOCKED_DEG)
    true_i, meas_i, x = [], [], []
    for start in range(0, n, chunk):
        idx = np.arange(start, min(n, start + chunk))
        q = {k: v[idx] for k, v in p.items()}
        offset = rng.uniform(-0.5, 0.5, len(idx)) * q["deadband"]
        f = force_fn(idx)
        out = physics.simulate(q, cmd, f_ext=f, period=IDLE_PERIOD, avg=IDLE_AVG,
                               init_offset_deg=offset, keep_state=True)
        true_i.append(out["current_ma"])
        meas_i.append(sensor.measure(out["current_ma"], q, rng, "idle"))
        x.append(out["x_mm"])
    return {"true": np.concatenate(true_i), "meas": np.concatenate(meas_i),
            "x_mm": np.concatenate(x)}


def batched_force(maker, **arrays):
    """Bind per-trial force parameters (arrays of length B) to a force maker,
    returning a function idx -> (step -> force[idx])."""
    def for_chunk(idx):
        return maker(**{k: v[idx] for k, v in arrays.items()})
    return for_chunk


def healthy_idle_stream(p_unit: dict, rng: np.random.Generator, hours: float,
                        knock_windows: np.ndarray, knock_rate_hz: float,
                        period_mean_s: float = 600.0) -> tuple[np.ndarray, np.ndarray]:
    """A long healthy idle recording for one unit: quiet holding current,
    benign knocks at Poisson times (true-current windows from the physics),
    a slow INA219 offset drift, and relocks that start new idle periods.
    Returns (measured samples at 100 Hz, boolean period-start markers)."""
    n = int(hours * 3600 * 100)
    true = np.full(n, float(p_unit["Iq"][0]) * 1000.0)
    n_knocks = rng.poisson(knock_rate_hz * hours * 3600)
    win = knock_windows.shape[1]
    for _ in range(n_knocks):
        at = int(rng.integers(0, max(1, n - win)))
        true[at:at + win] = knock_windows[int(rng.integers(0, len(knock_windows)))]
    drift = np.cumsum(rng.normal(0.0, 0.3 / np.sqrt(360000.0), n))   # ~0.3 mA/hour rms
    meas = sensor.measure(true[None, :], {k: v[:1] for k, v in p_unit.items()}, rng,
                          "idle", drift_ma=drift[None, :])[0]
    starts = np.zeros(n, dtype=bool)
    starts[0] = True
    t = 0
    while True:
        t += int(rng.exponential(period_mean_s) * 100) + 200
        if t >= n:
            break
        starts[t] = True
    return meas, starts
