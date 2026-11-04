"""Model parameters, unit-to-unit spread, run-to-run jitter and faults.

Every parameter lives in one flat dict of numpy arrays (one entry per
simulated actuation), so the physics can be evaluated for thousands of
trials at once. docs/MODEL.md lists each value and says whether it comes
from a datasheet, is derived from datasheet numbers, or is assumed.
"""
from __future__ import annotations

import numpy as np

DEG = np.pi / 180.0

# --------------------------------------------------------------- nominal
NOMINAL = {
    # SG90-class micro servo (output-shaft referred where it matters)
    "R": 7.0,             # ohm, motor + driver (assumed; ~0.69 A stall at 4.8 V)
    "Ke": 0.0017,         # V*s/rad motor side, = Kt in SI (derived: 0.1 s/60 deg at 4.8 V)
    "N": 250.0,           # gear ratio (assumed)
    "eta": 0.6,           # gear efficiency (assumed)
    "J": 2.51e-4,         # kg*m^2 at the output: 4e-9 * N^2 + horn (assumed rotor inertia)
    "tau_c": 0.008,       # N*m gear Coulomb friction at the output (assumed)
    "tau_s": 0.015,       # N*m gear static friction / back-drive threshold (assumed)
    "b_g": 2.0e-4,        # N*m*s/rad gear viscous friction (assumed)
    "deadband": 0.9,      # deg total (datasheet: 10 us dead band on a 2000 us/180 deg scale)
    "kp": 0.25,           # duty per degree of error beyond the deadband (assumed)
    "Iq": 0.005,          # A, servo electronics quiescent current (assumed)
    # supply
    "Voc": 5.0,           # V open-circuit
    "R_src": 0.2,         # ohm, supply + wiring (assumed); the 0.1 ohm shunt is added below
    # linkage and latch
    "r": 0.011,           # m, horn radius where the cord attaches (assumed)
    "k_c": 4000.0,        # N/m linkage stiffness (assumed: cord loop / push-pull cable)
    "c_l": 3.0,           # N*s/m linkage damping (assumed)
    "slack": 0.2e-3,      # m lost motion in the linkage (assumed)
    "delta": 0.0,         # rad linkage neutral offset from the locked angle
    "m_b": 0.025,         # kg bolt (assumed)
    "k_s": 250.0,         # N/m latch spring (assumed)
    "F_pre": 1.5,         # N spring preload at full extension (assumed)
    "Fb_c": 0.4,          # N bolt Coulomb friction (assumed)
    "Fb_s": 0.6,          # N bolt static friction (assumed)
    "b_b": 2.0,           # N*s/m bolt viscous friction (assumed)
    "x_max": 18.0e-3,     # m retracted end stop
    "k_stop": 5.0e4,      # N/m end-stop contact stiffness (assumed)
    "c_stop": 20.0,       # N*s/m end-stop damping (assumed)
    "jam_hi": np.inf,     # m: retraction blocked beyond this (unlock jam)
    "jam_lo": 0.0,        # m: extension blocked below this once relock starts
    # INA219 (0.1 ohm shunt)
    "shunt": 0.1,         # ohm
    "ina_offset": 0.0,    # mA
    "ina_gain": 1.0,
    "noise_fast": 1.0,    # mA rms at 9-bit (assumed; includes drive ripple)
    "noise_idle": 0.3,    # mA rms at 12-bit x16 averaging (assumed)
}

# Bolt travel used to place jams. The linkage stretches under the spring
# load, so a healthy unlock retracts the bolt ~15.5 mm (not the 17.3 mm of
# r * 90 deg) and holds it at ~13.7 mm.
TRAVEL_UNLOCK = 15.0e-3
TRAVEL_HOLD = 13.5e-3

FRICTION_KEYS = ("tau_c", "tau_s", "b_g", "Fb_c", "Fb_s", "b_b")
GEAR_FRICTION = ("tau_c", "tau_s", "b_g")
BOLT_FRICTION = ("Fb_c", "Fb_s", "b_b")
SPRING_KEYS = ("k_s", "F_pre")


def broadcast(params: dict, n: int) -> dict:
    """Return a dict where every value is a float64 array of length n."""
    out = {}
    for key, value in params.items():
        arr = np.asarray(value, dtype=np.float64)
        out[key] = np.broadcast_to(arr, (n,)).copy() if arr.ndim == 0 else arr.astype(np.float64).copy()
    return out


def take(params: dict, idx: np.ndarray) -> dict:
    """Select trials idx from a batched parameter dict."""
    return {k: v[idx].copy() for k, v in params.items()}


def concat(batches: list[dict]) -> dict:
    keys = batches[0].keys()
    return {k: np.concatenate([b[k] for b in batches]) for k in keys}


# ------------------------------------------------------- unit variation
def sample_units(rng: np.random.Generator, n_units: int) -> dict:
    """Unit-to-unit spread: manufacturing, assembly and installation."""
    p = broadcast(NOMINAL, n_units)
    u = lambda lo, hi: rng.uniform(lo, hi, n_units)  # noqa: E731
    for key in GEAR_FRICTION:
        p[key] *= u(0.75, 1.25)
    for key in BOLT_FRICTION:
        p[key] *= u(0.75, 1.25)
    spring = u(0.9, 1.1)
    p["k_s"] *= spring
    p["F_pre"] *= u(0.9, 1.1)
    p["Voc"] = u(4.8, 5.2)
    p["R_src"] = u(0.1, 0.3)
    p["k_c"] *= u(0.8, 1.2)
    p["R"] *= u(0.9, 1.1)
    p["Ke"] *= u(0.95, 1.05)
    p["deadband"] *= u(0.8, 1.2)
    p["kp"] *= u(0.8, 1.2)
    p["Iq"] = u(0.004, 0.007)
    p["delta"] = u(-1.0, 1.0) * DEG
    p["slack"] = u(0.1e-3, 0.3e-3)
    p["ina_offset"] = u(-0.5, 0.5)
    p["ina_gain"] = u(0.995, 1.005)
    return p


def apply_run_jitter(p: dict, rng: np.random.Generator, mode: str = "nominal") -> dict:
    """Run-to-run variation of one unit (temperature, supply, lubrication).

    nominal: small jitter around the unit's own values.
    stress:  the full healthy spread (supply 4.8-5.2 V, friction +/-25 %,
             spring +/-10 %) re-drawn on every actuation.
    """
    q = {k: v.copy() for k, v in p.items()}
    n = len(q["Voc"])
    if mode == "nominal":
        q["Voc"] = np.clip(q["Voc"] + rng.normal(0.0, 0.03, n), 4.8, 5.2)
        for key in GEAR_FRICTION:
            q[key] *= rng.normal(1.0, 0.05, n)
        for key in BOLT_FRICTION:
            q[key] *= rng.normal(1.0, 0.05, n)
        q["k_s"] *= rng.normal(1.0, 0.01, n)
    elif mode == "stress":
        q["Voc"] = rng.uniform(4.8, 5.2, n)
        for key in FRICTION_KEYS:
            q[key] *= rng.uniform(0.75, 1.25, n)
        q["k_s"] *= rng.uniform(0.9, 1.1, n)
        q["F_pre"] *= rng.uniform(0.9, 1.1, n)
    else:
        raise ValueError(f"unknown jitter mode {mode!r}")
    q["tau_s"] = np.maximum(q["tau_s"], q["tau_c"] * 1.05)
    q["Fb_s"] = np.maximum(q["Fb_s"], q["Fb_c"] * 1.05)
    return q


# --------------------------------------------------------------- faults
FAULTS = {
    # name: (class, severities, actuation(s) affected)
    "jam_unlock": ("stall", (0.3, 0.5, 0.7, 0.9), "unlock"),
    "jam_relock": ("stall", (0.3, 0.5, 0.7, 0.9), "relock"),
    "linkage_broken": ("linkage", (1.0,), "both"),
    "linkage_slack": ("linkage", (1.5, 3.0, 5.0), "both"),
    "brownout": ("brownout", (4.6, 4.4, 4.2), "both"),
    "friction": ("friction", (1.5, 2.0, 3.0), "both"),
}


def apply_fault(p: dict, fault: str, severity: float, rng: np.random.Generator) -> dict:
    """Return a copy of p with the fault injected at the given severity."""
    q = {k: v.copy() for k, v in p.items()}
    n = len(q["Voc"])
    if fault == "jam_unlock":       # blocked after `severity` of the retraction
        q["jam_hi"] = np.full(n, severity * TRAVEL_UNLOCK)
    elif fault == "jam_relock":     # door ajar: bolt hits the strike face
        q["jam_lo"] = np.full(n, (1.0 - severity) * TRAVEL_HOLD)
    elif fault == "linkage_broken":
        q["k_c"] = np.zeros(n)
        q["c_l"] = np.zeros(n)
    elif fault == "linkage_slack":  # severity in mm of lost motion
        q["slack"] = q["slack"] + severity * 1e-3
    elif fault == "brownout":       # weak supply: low voltage, high impedance
        q["Voc"] = np.full(n, severity)
        q["R_src"] = q["R_src"] * rng.uniform(2.5, 3.5, n)
    elif fault == "friction":       # dry / dirty / iced mechanism
        for key in FRICTION_KEYS:
            q[key] = q[key] * severity
    else:
        raise ValueError(f"unknown fault {fault!r}")
    return q
