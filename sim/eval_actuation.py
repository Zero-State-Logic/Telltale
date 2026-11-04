"""Per-actuation evaluation: learn each simulated lock's healthy signature
with the C detector, then score healthy cycles and injected faults."""
from __future__ import annotations

import numpy as np

from . import core, params as P, scenarios as S
from .metrics import auc, rate, threshold_for_fa

FA_TARGET_VAL = 0.0005          # threshold chosen for <= 0.5 FA / 1000 on validation
PEAK_SIGMA_FLOOR = 2.0          # mA, for the overcurrent baseline

# Ablations: which features feed the score (indices into core.FEATURES).
ABLATIONS = {
    "telltale_default": None,                       # the C detector's own score
    "no_template_residual": (0, 1, 2, 4, 5, 7),
    "peak_and_mean_only": (0, 1),
}


def _healthy_batch(units, n_units, n_runs, rng, mode):
    idx = np.repeat(np.arange(n_units), n_runs)
    return idx, P.apply_run_jitter(P.take(units, idx), rng, mode)


def _fault_batch(units, n_units, trials, rng, mode):
    batches, labels = [], []
    for fault, (cls, severities, _) in P.FAULTS.items():
        for sev in severities:
            idx = np.repeat(np.arange(n_units), trials)
            q = P.apply_run_jitter(P.take(units, idx), rng, mode)
            batches.append(P.apply_fault(q, fault, sev, rng))
            labels += [(fault, sev, cls, int(u)) for u in idx]
    return P.concat(batches), labels


def _peak_stats(learn_peaks):
    arr = np.asarray(learn_peaks)
    return arr.mean(axis=0), np.maximum(arr.std(axis=0), PEAK_SIGMA_FLOOR)


def run_population(units, n_units, layout, rng, *, learn, healthy, fault_trials=0,
                   mode="nominal", threshold=None, alpha=None, sim_chunk=2000):
    """Simulate and score a population of locks. Returns per-actuation
    healthy scores/z and per-event fault records."""
    h_idx, h_par = _healthy_batch(units, n_units, learn + healthy, rng, mode)
    h_sim = S.simulate_cycles(h_par, rng, layout, chunk=sim_chunk)
    if fault_trials:
        f_par, f_lab = _fault_batch(units, n_units, fault_trials, rng, mode)
        f_sim = S.simulate_cycles(f_par, rng, layout, chunk=sim_chunk)
    else:
        f_lab, f_sim = [], None

    h_scores, h_z, h_base, h_peak, faults = [], [], [], [], []
    for u in range(n_units):
        det = core.Detector()
        if threshold is not None:
            det.act_cfg.threshold = threshold
        if alpha is not None:
            det.act_cfg.ewma_alpha = alpha
        rows = np.where(h_idx == u)[0]
        learn_peaks = []
        for j, r in enumerate(rows):
            ru = det.actuation(core.UNLOCK, h_sim["unlock"][r], layout.unlock_motion_ms)
            rr = det.actuation(core.RELOCK, h_sim["relock"][r], layout.relock_motion_ms)
            if j < learn:
                learn_peaks.append((ru["features"][0], rr["features"][0]))
                continue
            if j == learn:
                mu, sd = _peak_stats(learn_peaks)
            h_scores.append((ru["score"], rr["score"]))
            h_z.append((ru["z"], rr["z"]))
            h_base.append(((ru["features"][0] - mu[0]) / sd[0], (rr["features"][0] - mu[1]) / sd[1]))
            h_peak.append((ru["features"][0], rr["features"][0]))
        for k, (fault, sev, cls, uu) in enumerate(f_lab):
            if uu != u:
                continue
            d2 = det.copy()
            ru = d2.actuation(core.UNLOCK, f_sim["unlock"][k], layout.unlock_motion_ms)
            rr = d2.actuation(core.RELOCK, f_sim["relock"][k], layout.relock_motion_ms)
            faults.append({
                "fault": fault, "sev": sev, "cls": cls,
                "score": (ru["score"], rr["score"]), "z": (ru["z"], rr["z"]),
                "status": (ru["status"], rr["status"]), "pred": (ru["fault"], rr["fault"]),
                "base": ((ru["features"][0] - mu[0]) / sd[0], (rr["features"][0] - mu[1]) / sd[1]),
                "peak": (ru["features"][0], rr["features"][0]),
            })
    return {"h_scores": np.array(h_scores), "h_z": np.array(h_z),
            "h_base": np.array(h_base), "h_peak": np.array(h_peak), "faults": faults}


def masked_score(z: np.ndarray, mask) -> np.ndarray:
    """Score from a subset of feature z-values (residual counted one-sided)."""
    zz = np.array(z, dtype=np.float64)
    zz[..., 6] = np.maximum(zz[..., 6], 0.0)
    return np.sqrt(np.sum(zz[..., list(mask)] ** 2, axis=-1))


def _event_class(rec, threshold):
    """The first anomalous actuation of the cycle names the fault (the
    firmware reacts to the unlock before it ever relocks)."""
    if rec["score"][0] > threshold:
        return rec["pred"][0]
    if rec["score"][1] > threshold:
        return rec["pred"][1]
    return "none"


def summarise(val, test, layout_info):
    """Thresholds from validation, everything else measured on test."""
    thr = threshold_for_fa(val["h_scores"].ravel(), FA_TARGET_VAL)
    k_base = threshold_for_fa(val["h_base"].ravel(), FA_TARGET_VAL)
    t_fix = threshold_for_fa(val["h_peak"].ravel(), FA_TARGET_VAL)
    hs, hb, hp = test["h_scores"], test["h_base"], test["h_peak"]
    out = {
        "threshold": round(thr, 3),
        "baseline_peak_sigma": round(k_base, 3),
        "baseline_fixed_peak_ma": round(t_fix, 1),
        "healthy": {
            "actuations": int(hs.size),
            "telltale_false_alarms": rate(int((hs > thr).sum()), int(hs.size)),
            "overcurrent_false_alarms": rate(int((hb > k_base).sum()), int(hb.size)),
            "fixed_peak_false_alarms": rate(int((hp > t_fix).sum()), int(hp.size)),
            "score_median": round(float(np.median(hs)), 3),
            "score_p999": round(float(np.percentile(hs, 99.9)), 3),
        },
        "layout": layout_info,
    }
    h_event = hs.max(axis=1)
    hb_event = hb.max(axis=1)
    hp_event = hp.max(axis=1)
    per, cls_conf = {}, {}
    for rec_key in sorted({(r["fault"], r["sev"]) for r in test["faults"]},
                          key=lambda t: (list(P.FAULTS).index(t[0]), t[1])):
        recs = [r for r in test["faults"] if (r["fault"], r["sev"]) == rec_key]
        ev = np.array([max(r["score"]) for r in recs])
        evb = np.array([max(r["base"]) for r in recs])
        evp = np.array([max(r["peak"]) for r in recs])
        per[f"{rec_key[0]}@{rec_key[1]:g}"] = {
            "fault": rec_key[0], "severity": rec_key[1], "class": recs[0]["cls"],
            "telltale": {"detected": rate(int((ev > thr).sum()), len(ev)),
                         "auc": round(auc(ev, h_event), 4)},
            "overcurrent": {"detected": rate(int((evb > k_base).sum()), len(evb)),
                            "auc": round(auc(evb, hb_event), 4)},
            "fixed_peak": {"detected": rate(int((evp > t_fix).sum()), len(evp)),
                           "auc": round(auc(evp, hp_event), 4)},
        }
        for r in recs:
            pred = _event_class(r, thr)
            if pred != "none":
                cls_conf.setdefault(r["cls"], {}).setdefault(pred, 0)
                cls_conf[r["cls"]][pred] += 1
    out["per_fault"] = per
    out["per_class"] = _per_class(test, thr, k_base, t_fix, h_event, hb_event, hp_event)
    out["all_faults"] = _all_faults(test, thr, k_base, t_fix)
    out["classification"] = _classification(cls_conf)
    out["ablations"] = _ablations(val, test)
    return out


def _per_class(test, thr, k_base, t_fix, h_event, hb_event, hp_event):
    res = {}
    for cls in ("stall", "linkage", "brownout", "friction"):
        recs = [r for r in test["faults"] if r["cls"] == cls]
        ev = np.array([max(r["score"]) for r in recs])
        evb = np.array([max(r["base"]) for r in recs])
        evp = np.array([max(r["peak"]) for r in recs])
        res[cls] = {
            "telltale": {"detected": rate(int((ev > thr).sum()), len(ev)), "auc": round(auc(ev, h_event), 4)},
            "overcurrent": {"detected": rate(int((evb > k_base).sum()), len(evb)), "auc": round(auc(evb, hb_event), 4)},
            "fixed_peak": {"detected": rate(int((evp > t_fix).sum()), len(evp)), "auc": round(auc(evp, hp_event), 4)},
            "original_lock": {"detected": rate(0, len(ev))},
        }
    return res


def _all_faults(test, thr, k_base, t_fix):
    recs = test["faults"]
    n = len(recs)
    return {
        "telltale": rate(sum(max(r["score"]) > thr for r in recs), n),
        "overcurrent": rate(sum(max(r["base"]) > k_base for r in recs), n),
        "fixed_peak": rate(sum(max(r["peak"]) > t_fix for r in recs), n),
        "original_lock": rate(0, n),
    }


def _classification(conf):
    total = sum(sum(v.values()) for v in conf.values())
    correct = sum(conf.get(c, {}).get(c, 0) for c in conf)
    return {"confusion": conf, "accuracy_on_detected": round(correct / total, 4) if total else None,
            "n": total}


def _ablations(val, test):
    res = {}
    for name, mask in ABLATIONS.items():
        if mask is None:
            v, t = val["h_scores"], test["h_scores"]
            ft = np.array([max(r["score"]) for r in test["faults"]])
        else:
            v, t = masked_score(val["h_z"], mask), masked_score(test["h_z"], mask)
            ft = np.array([masked_score(np.array(r["z"]), mask).max() for r in test["faults"]])
        thr = threshold_for_fa(v.ravel(), FA_TARGET_VAL)
        res[name] = {"threshold": round(thr, 3),
                     "false_alarms": rate(int((t > thr).sum()), int(t.size)),
                     "faults_detected": rate(int((ft > thr).sum()), len(ft)),
                     "auc_all_faults": round(auc(ft, t.max(axis=1)), 4)}
    return res


def drift_experiment(rng, layout, n_units, n_runs, learn, threshold):
    """Friction creeps up 40 % over n_runs actuations (wear, dust). Compare
    the gated adaptive model with a frozen one; then a sudden 2x friction
    jump at the end must still be caught by the adaptive model."""
    units = P.sample_units(rng, n_units)
    idx = np.repeat(np.arange(n_units), learn + n_runs + 1)
    par = P.apply_run_jitter(P.take(units, idx), rng, "nominal")
    step = np.tile(np.arange(learn + n_runs + 1), n_units)
    creep = 1.0 + 0.40 * np.clip((step - learn) / n_runs, 0.0, 1.0)
    jump = np.where(step == learn + n_runs, 2.0, 1.0)
    for key in P.FRICTION_KEYS:
        par[key] = par[key] * creep * jump
    sim = S.simulate_cycles(par, rng, layout)
    res = {}
    for label, alpha in (("adaptive", None), ("frozen", 0.0)):
        fa, n, caught = 0, 0, 0
        for u in range(n_units):
            det = core.Detector()
            det.act_cfg.threshold = threshold
            rows = np.where(idx == u)[0]
            for j, r in enumerate(rows):
                if j == learn and alpha is not None:
                    det.act_cfg.ewma_alpha = alpha
                ru = det.actuation(core.UNLOCK, sim["unlock"][r], layout.unlock_motion_ms)
                rr = det.actuation(core.RELOCK, sim["relock"][r], layout.relock_motion_ms)
                if learn <= j < learn + n_runs:
                    fa += (ru["score"] > threshold) + (rr["score"] > threshold)
                    n += 2
                elif j == learn + n_runs:
                    caught += max(ru["score"], rr["score"]) > threshold
        res[label] = {"false_alarms_during_creep": rate(fa, n),
                      "sudden_2x_friction_detected": rate(caught, n_units)}
    res["setup"] = {"units": n_units, "creep_runs": n_runs, "friction_creep": "+40 % linear"}
    return res
