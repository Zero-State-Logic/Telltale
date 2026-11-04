"""TELLTALE evaluation: python -m sim.evaluate [--quick]

Simulates populations of locks, runs the firmware's C detector (ctypes) on
the simulated INA219 readings and writes:
  results/eval.json          every number quoted in the README
  results/footprint.json     RAM per struct and host time per sample
  docs/assets/results.svg    the results chart
Thresholds are always chosen on validation locks and reported on separate
test locks. Everything here is simulated; nothing is a hardware measurement.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

from . import core, eval_actuation as EA, eval_idle as EI, params as P, scenarios as S
from .svgchart import chart_from_results

ROOT = Path(__file__).resolve().parent.parent

FULL = dict(val_units=40, test_units=80, learn=30, healthy=125, fault_trials=5,
            stress_val=20, stress_test=40, stress_healthy=50, stress_trials=3,
            drift_units=15, drift_runs=400,
            idle_val=40, idle_test=80, idle_hours=0.5, knocks=12, knock_rate=1 / 120,
            forces=(4.0, 8.0, 15.0, 30.0, 60.0), pry_trials=4, slow_units=40)
QUICK = dict(val_units=6, test_units=8, learn=30, healthy=30, fault_trials=2,
             stress_val=4, stress_test=4, stress_healthy=20, stress_trials=1,
             drift_units=3, drift_runs=60,
             idle_val=4, idle_test=4, idle_hours=0.05, knocks=4, knock_rate=1 / 120,
             forces=(8.0, 30.0), pry_trials=2, slow_units=2)


def log(msg):
    print(f"[eval {time.strftime('%H:%M:%S')}] {msg}", flush=True)


def actuation_block(cfg, layout, mode, n_val, n_test, healthy, trials, seed):
    rng = np.random.default_rng(seed)
    val_units = P.sample_units(rng, n_val)
    test_units = P.sample_units(rng, n_test)
    val = EA.run_population(val_units, n_val, layout, rng, learn=cfg["learn"],
                            healthy=healthy, mode=mode)
    thr = EA.threshold_for_fa(val["h_scores"].ravel(), EA.FA_TARGET_VAL)
    test = EA.run_population(test_units, n_test, layout, rng, learn=cfg["learn"],
                             healthy=healthy, fault_trials=trials, mode=mode, threshold=thr)
    info = {"unlock_ms": layout.unlock_ms, "unlock_motion_ms": layout.unlock_motion_ms,
            "relock_ms": layout.relock_ms, "relock_motion_ms": layout.relock_motion_ms,
            "val_units": n_val, "test_units": n_test, "learn_runs": cfg["learn"],
            "healthy_cycles_per_unit": healthy, "fault_trials_per_unit_per_severity": trials,
            "variation": mode}
    return EA.summarise(val, test, info), thr


def idle_block(cfg, seed):
    rng = np.random.default_rng(seed)
    val_units = P.sample_units(rng, cfg["idle_val"])
    test_units = P.sample_units(rng, cfg["idle_test"])
    log("idle: validation streams")
    val_streams = EI.healthy_streams(val_units, cfg["idle_val"], rng, cfg["idle_hours"],
                                     cfg["knocks"], cfg["knock_rate"])
    thresholds = EI.pick_thresholds(val_units, cfg["idle_val"], val_streams, rng)
    log(f"idle: thresholds {thresholds}")
    test_streams = EI.healthy_streams(test_units, cfg["idle_test"], rng, cfg["idle_hours"],
                                      cfg["knocks"], cfg["knock_rate"])
    fa = EI.healthy_false_alarms(test_units, cfg["idle_test"], test_streams, rng, thresholds)
    log(f"idle: healthy false alarms {fa}")
    pry = EI.prying_trials(test_units, cfg["idle_test"], rng, cfg["forces"], cfg["pry_trials"],
                           thresholds)
    slow = EI.slow_push(test_units, min(cfg["slow_units"], cfg["idle_test"]), rng, thresholds)
    return {"thresholds": thresholds, "healthy": fa, "prying": pry, "slow_push": slow,
            "setup": {"val_units": cfg["idle_val"], "test_units": cfg["idle_test"],
                      "hours_per_unit": cfg["idle_hours"], "knock_rate_per_min": cfg["knock_rate"] * 60,
                      "knock_force_n": [2.0, 15.0], "knock_duration_ms": [20, 120],
                      "prying_trials_per_force_per_unit": cfg["pry_trials"],
                      "prying_ramp_s": [0.2, 1.0], "prying_pump_hz": [1.5, 4.0],
                      "prying_hold_s": 2.5}}


def footprint():
    exe = ROOT / "build" / ("bench.exe" if sys.platform == "win32" else "bench")
    if not exe.exists():
        return {"error": "build/bench not found; run tools/build.sh"}
    out = subprocess.run([str(exe)], capture_output=True, text=True, check=True)
    return json.loads(out.stdout)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--quick", action="store_true", help="small populations (smoke test)")
    ap.add_argument("--out", default=str(ROOT / "results"))
    ap.add_argument("--chart", default=str(ROOT / "docs" / "assets" / "results.svg"))
    args = ap.parse_args(argv)
    cfg = QUICK if args.quick else FULL
    t0 = time.time()
    layout = S.cycle_layout()

    log("actuation: nominal variation")
    nominal, thr = actuation_block(cfg, layout, "nominal", cfg["val_units"], cfg["test_units"],
                                   cfg["healthy"], cfg["fault_trials"], seed=2026)
    log(f"actuation: threshold {thr:.3f}, healthy FA {nominal['healthy']['telltale_false_alarms']}")
    log("actuation: stress variation (full healthy spread on every run)")
    stress, _ = actuation_block(cfg, layout, "stress", cfg["stress_val"], cfg["stress_test"],
                                cfg["stress_healthy"], cfg["stress_trials"], seed=7)
    log("actuation: drift experiment")
    drift = EA.drift_experiment(np.random.default_rng(11), layout, cfg["drift_units"],
                                cfg["drift_runs"], cfg["learn"], thr)
    log("idle: prying vs knocks")
    idle = idle_block(cfg, seed=4242)

    results = {
        "about": "TELLTALE simulated evaluation. All numbers come from the physics model "
                 "in sim/ and the C detector in firmware/telltale/. Not hardware measurements.",
        "quick": bool(args.quick),
        "fa_target_validation_per_actuation": EA.FA_TARGET_VAL,
        "actuation": nominal,
        "actuation_stress": stress,
        "drift": drift,
        "idle": idle,
        "detector_defaults": {
            "learn_runs": 30, "features": list(core.FEATURES),
            "scored_features": [f for f in core.FEATURES if f != "t_peak"],
        },
        "runtime_s": None,
    }
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    results["runtime_s"] = round(time.time() - t0, 1)
    name = "eval_quick.json" if args.quick else "eval.json"
    (out_dir / name).write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    if not args.quick:
        (out_dir / "footprint.json").write_text(json.dumps(footprint(), indent=2) + "\n",
                                                encoding="utf-8")
        Path(args.chart).parent.mkdir(parents=True, exist_ok=True)
        chart_from_results(results, args.chart)
    log(f"done in {results['runtime_s']} s -> {out_dir / name}")
    return results


if __name__ == "__main__":
    main()
