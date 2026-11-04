"""INA219 measurement model.

Two ADC settings, matching the firmware (telltale_ina219.c):
  fast: 9-bit shunt conversions read at 1 kHz during motion
  idle: 12-bit with 16-sample averaging read at 100 Hz while locked

Quantisation assumes the shunt register keeps its 10 uV LSB and that the
reduced-resolution modes drop the low bits (10 uV * 2^(12 - bits)): 0.8 mA
steps at 9 bits and 0.1 mA at 12 bits with the 0.1 ohm shunt. Noise levels
are assumptions (docs/MODEL.md). PGA /2 limits the range to +/-80 mV (+/-0.8 A).
"""
from __future__ import annotations

import numpy as np

SHUNT_LSB_MV = 0.01
FULL_SCALE_MV = 80.0


def quant_step_ma(mode: str, shunt_ohm: float = 0.1) -> float:
    bits_dropped = 3 if mode == "fast" else 0
    return SHUNT_LSB_MV * (2 ** bits_dropped) / shunt_ohm


def measure(true_ma: np.ndarray, p: dict, rng: np.random.Generator, mode: str,
            drift_ma: np.ndarray | None = None) -> np.ndarray:
    """Turn true supply current (trials x samples, mA) into INA219 readings."""
    if mode not in ("fast", "idle"):
        raise ValueError(f"unknown INA219 mode {mode!r}")
    shunt = float(np.mean(p["shunt"]))
    sigma = p["noise_fast"] if mode == "fast" else p["noise_idle"]
    y = true_ma * p["ina_gain"][:, None] + p["ina_offset"][:, None]
    if drift_ma is not None:
        y = y + drift_ma
    y = y + rng.standard_normal(true_ma.shape) * sigma[:, None]
    step = quant_step_ma(mode, shunt)
    y = np.round(y / step) * step
    limit = FULL_SCALE_MV / shunt
    return np.clip(y, -limit, limit).astype(np.float32)
