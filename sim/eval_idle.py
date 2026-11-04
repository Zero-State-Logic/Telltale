"""Idle (locked) evaluation: prying attacks versus benign knocks.

Prying is sustained force on the bolt; a knock is a short pulse. Thresholds
for every method are set on validation locks to give zero false alarms in
their healthy idle recordings, then everything is measured on test locks.
"""
from __future__ import annotations

import numpy as np

from . import core, params as P, scenarios as S, sensor
from .metrics import percentiles, rate

SAMPLE_MS = 10
ONSET_S = 1.5                   # quiet before the attack (blanking + anchoring + armed)
WINDOW_S = 5.0
DEBOUNCE = 30                   # samples (300 ms) for the debounced-threshold baseline
STRIKE_MM = 10.0                # bolt retraction at which the door could open (assumed)
H_GRID = np.arange(200.0, 5001.0, 50.0)
H_MARGIN = 1.2


def _knock_windows(units, n_units, per_unit, rng):
    idx = np.repeat(np.arange(n_units), per_unit)
    p = P.apply_run_jitter(P.take(units, idx), rng, "nominal")
    n = len(idx)
    fn = S.batched_force(S.knock_force, onset_s=np.full(n, 0.2),
                         dur_s=rng.uniform(0.02, 0.12, n), amp_n=rng.uniform(2.0, 15.0, n))
    out = S.simulate_idle(p, rng, fn, 1.0)
    return idx, out["true"]


def _learned_detector(p_unit, rng, h_hi):
    det = core.Detector()
    det.idle_cfg.h_hi = h_hi
    quiet = np.full((1, 2200), float(p_unit["Iq"][0]) * 1000.0)
    det.idle_enter()
    det.idle_block(sensor.measure(quiet, p_unit, rng, "idle")[0])
    if not det.idle_learned():
        raise RuntimeError("idle baseline did not finish learning")
    return det


def _stream_alarms(det, stream, starts):
    edges = list(np.flatnonzero(starts)) + [len(stream)]
    alarms = 0
    for a, b in zip(edges[:-1], edges[1:]):
        det.idle_enter()
        ev = det.idle_block(stream[a:b])
        alarms += int(np.sum((ev == core.IDLE_FORCED) | (ev == core.IDLE_POWER_LOSS)))
    return alarms


def healthy_streams(units, n_units, rng, hours, knocks_per_unit, knock_rate):
    k_idx, k_win = _knock_windows(units, n_units, knocks_per_unit, rng)
    streams = []
    for u in range(n_units):
        p_u = P.take(units, np.array([u]))
        streams.append(S.healthy_idle_stream(p_u, rng, hours, k_win[k_idx == u], knock_rate))
    return streams


def pick_thresholds(units, n_units, streams, rng):
    """Smallest settings with zero alarms on validation, plus a margin."""
    worst_h = H_GRID[0]
    for u, (stream, starts) in enumerate(streams):
        p_u = P.take(units, np.array([u]))
        base = _learned_detector(p_u, rng, H_GRID[-1])
        for h in H_GRID:
            if h < worst_h:
                continue
            det = base.copy()
            det.idle_cfg.h_hi = float(h)
            if _stream_alarms(det, stream, starts) == 0:
                worst_h = max(worst_h, float(h))
                break
        else:
            worst_h = float(H_GRID[-1])
    all_s = np.concatenate([s for s, _ in streams])
    t_inst = float(all_s.max()) + 1.0
    t_deb = float(max(_rolling_min(s, DEBOUNCE).max() for s, _ in streams)) + 1.0
    return {"cusum_h_hi": round(worst_h * H_MARGIN, 1), "instant_threshold_ma": round(t_inst, 2),
            "debounced_threshold_ma": round(t_deb, 2), "debounce_samples": DEBOUNCE}


def _rolling_min(x, w):
    if len(x) < w:
        return np.array([x.min()])
    view = np.lib.stride_tricks.sliding_window_view(x, w)
    return view.min(axis=1)


def _first_index(mask):
    hits = np.flatnonzero(mask)
    return int(hits[0]) if len(hits) else -1


def _first_debounced(x, thr, w):
    above = x > thr
    run = 0
    for i, a in enumerate(above):
        run = run + 1 if a else 0
        if run >= w:
            return i
    return -1


def prying_trials(units, n_units, rng, forces, trials, thresholds):
    n_f = len(forces)
    idx = np.repeat(np.arange(n_units), n_f * trials)
    force = np.tile(np.repeat(np.asarray(forces, float), trials), n_units)
    n = len(idx)
    p = P.apply_run_jitter(P.take(units, idx), rng, "nominal")
    fn = S.batched_force(S.prying_force, onset_s=np.full(n, ONSET_S),
                         ramp_s=rng.uniform(0.2, 1.0, n), peak_n=force,
                         freq_hz=rng.uniform(1.5, 4.0, n), phase=rng.uniform(0, 2 * np.pi, n))
    out = S.simulate_idle(p, rng, fn, WINDOW_S)
    onset = int(ONSET_S * 1000 / SAMPLE_MS)
    results = {f: {"cusum": [], "instant": [], "debounced": [], "bolt2mm": [], "strike": []}
               for f in forces}
    dets = {}
    for i in range(n):
        u = int(idx[i])
        if u not in dets:
            dets[u] = _learned_detector(P.take(units, np.array([u])), rng, thresholds["cusum_h_hi"])
        det = dets[u].copy()
        det.idle_enter()
        ev = det.idle_block(out["meas"][i])
        meas = out["meas"][i]
        a_c = _first_index(ev == core.IDLE_FORCED)
        a_i = _first_index(meas > thresholds["instant_threshold_ma"])
        a_d = _first_debounced(meas, thresholds["debounced_threshold_ma"], DEBOUNCE)
        b2 = _first_index(out["x_mm"][i] > 2.0)
        bs = _first_index(out["x_mm"][i] > STRIKE_MM)
        r = results[float(force[i])]
        for key, a in (("cusum", a_c), ("instant", a_i), ("debounced", a_d)):
            r[key].append(a - onset if a >= onset else (None if a < 0 else -1))
        r["bolt2mm"].append(b2 - onset if b2 >= 0 else None)
        r["strike"].append(bs - onset if bs >= 0 else None)
    return _summarise_prying(results)


def _summarise_prying(results):
    summary = {}
    for f, r in results.items():
        entry = {}
        for key in ("cusum", "instant", "debounced"):
            lat = [v for v in r[key] if v is not None and v >= 0]
            early = sum(1 for v in r[key] if v == -1)   # fired before the attack began
            entry[key] = {"detected": rate(len(lat), len(r[key])),
                          "latency_ms": {k: (None if v is None else v * SAMPLE_MS)
                                         for k, v in percentiles(lat).items()},
                          "before_bolt_moved_2mm": rate(_before(r[key], r["bolt2mm"]), len(r[key])),
                          "before_bolt_cleared_strike": rate(_before(r[key], r["strike"]), len(r[key])),
                          "fired_before_onset": early}
        entry["bolt_moved_2mm"] = rate(sum(b is not None for b in r["bolt2mm"]), len(r["bolt2mm"]))
        entry["bolt_cleared_strike"] = rate(sum(b is not None for b in r["strike"]), len(r["strike"]))
        summary[f"{f:g}N"] = entry
    return summary


def _before(alarms, events):
    """Alarms raised no later than the event (or with no event at all)."""
    return sum(1 for v, b in zip(alarms, events) if v is not None and v >= 0 and (b is None or v <= b))


def healthy_false_alarms(units, n_units, streams, rng, thresholds):
    hours = sum(len(s) for s, _ in streams) / (100.0 * 3600.0)
    c_alarms = 0
    for u, (stream, starts) in enumerate(streams):
        det = _learned_detector(P.take(units, np.array([u])), rng, thresholds["cusum_h_hi"])
        c_alarms += _stream_alarms(det, stream, starts)
    inst = sum(int(np.sum(np.diff(np.concatenate([[0], (s > thresholds["instant_threshold_ma"]).astype(int)])) == 1))
               for s, _ in streams)
    deb = sum(int(np.sum(_rolling_min(s, DEBOUNCE) > thresholds["debounced_threshold_ma"]) > 0)
              for s, _ in streams)
    return {"hours": round(hours, 2), "cusum_alarms": c_alarms, "instant_alarms": inst,
            "debounced_units_with_alarm": deb}


def slow_push(units, n_units, rng, thresholds, ramp_s=20.0, peak_n=30.0):
    """A deliberately slow push that tries to drag the baseline tracker."""
    idx = np.arange(n_units)
    p = P.apply_run_jitter(P.take(units, idx), rng, "nominal")
    fn = S.batched_force(S.slow_push_force, onset_s=np.full(n_units, ONSET_S),
                         ramp_s=np.full(n_units, ramp_s), peak_n=np.full(n_units, peak_n))
    out = S.simulate_idle(p, rng, fn, ONSET_S + ramp_s + 2.0)
    onset = int(ONSET_S * 1000 / SAMPLE_MS)
    lat, before = [], 0
    for u in range(n_units):
        det = _learned_detector(P.take(units, np.array([u])), rng, thresholds["cusum_h_hi"])
        det.idle_enter()
        a = _first_index(det.idle_block(out["meas"][u]) == core.IDLE_FORCED)
        b = _first_index(out["x_mm"][u] > 2.0)
        if a >= onset:
            lat.append(a - onset)
            before += (b < 0 or a <= b)
    return {"ramp_s": ramp_s, "peak_n": peak_n, "detected": rate(len(lat), n_units),
            "before_bolt_moved_2mm": rate(before, n_units),
            "force_at_alarm_n": percentiles([peak_n * min(1.0, l * SAMPLE_MS / 1000 / ramp_s) for l in lat])}
