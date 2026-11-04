"""The firmware's C detector, driven through ctypes by simulated traces."""
import hashlib
import hmac
import os
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from sim import core, metrics, params as P, scenarios as S  # noqa: E402
from sim import eval_idle as EI  # noqa: E402

LEARN = 30
HEALTHY = 12
FAULTS = [("jam_unlock", 0.5, "stall"), ("jam_relock", 0.5, "stall"),
          ("linkage_broken", 1.0, "linkage"), ("friction", 2.0, None),
          ("brownout", 4.2, None)]


class ActuationDetectorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rng = np.random.default_rng(99)
        cls.layout = S.cycle_layout()
        unit = P.sample_units(rng, 1)
        healthy = P.apply_run_jitter(P.take(unit, np.zeros(LEARN + HEALTHY, int)), rng, "nominal")
        faults = [P.apply_fault(P.apply_run_jitter(P.take(unit, np.zeros(1, int)), rng, "nominal"),
                                f, s, rng) for f, s, _ in FAULTS]
        cls.sim = S.simulate_cycles(P.concat([healthy] + faults), rng, cls.layout)
        cls.det = core.Detector()
        cls.healthy_scores = []
        for i in range(LEARN + HEALTHY):
            ru = cls.det.actuation(core.UNLOCK, cls.sim["unlock"][i], cls.layout.unlock_motion_ms)
            rr = cls.det.actuation(core.RELOCK, cls.sim["relock"][i], cls.layout.relock_motion_ms)
            if i < LEARN:
                assert ru["status"] == "learning" and rr["status"] == "learning"
            else:
                cls.healthy_scores += [ru["score"], rr["score"]]

    def score_fault(self, k):
        d = self.det.copy()
        row = LEARN + HEALTHY + k
        ru = d.actuation(core.UNLOCK, self.sim["unlock"][row], self.layout.unlock_motion_ms)
        rr = d.actuation(core.RELOCK, self.sim["relock"][row], self.layout.relock_motion_ms)
        return ru, rr

    def test_learning_finishes_after_30_runs(self):
        self.assertFalse(self.det.learning(core.UNLOCK))
        self.assertFalse(self.det.learning(core.RELOCK))

    def test_healthy_actuations_stay_below_threshold(self):
        thr = self.det.act_cfg.threshold
        self.assertLess(max(self.healthy_scores), thr)

    def test_every_fault_is_flagged(self):
        thr = self.det.act_cfg.threshold
        for k, (fault, sev, _) in enumerate(FAULTS):
            ru, rr = self.score_fault(k)
            with self.subTest(fault=fault, severity=sev):
                self.assertGreater(max(ru["score"], rr["score"]), thr)

    def test_fault_classes(self):
        for k, (fault, sev, expected) in enumerate(FAULTS):
            if expected is None:
                continue
            ru, rr = self.score_fault(k)
            first = ru if ru["status"] == "anomaly" else rr
            with self.subTest(fault=fault):
                self.assertEqual(first["fault"], expected)

    def test_anomalies_do_not_update_the_model(self):
        ru, rr = self.score_fault(2)
        self.assertFalse(ru["updated"])

    def test_layouts_match_firmware_profiles(self):
        self.assertEqual(self.layout.unlock_ms, 1080)
        self.assertEqual(self.layout.unlock_motion_ms, 880)
        self.assertEqual(self.layout.relock_ms, 968)
        self.assertEqual(self.layout.relock_motion_ms, 768)


class IdleDetectorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rng = np.random.default_rng(5)
        cls.unit = P.sample_units(rng, 1)
        cls.rng = rng
        n = 3
        p = P.take(cls.unit, np.zeros(n, int))
        peak = np.array([15.0, 0.0, 0.0])
        knock = np.array([0.0, 12.0, 0.0])

        def force_for(idx):
            pry = S.prying_force(1.5, 0.4, peak[idx], 2.5, 0.0)
            hit = S.knock_force(2.0, 0.08, knock[idx])
            return lambda k: pry(k) + hit(k)

        cls.out = S.simulate_idle(p, rng, force_for, 4.0)

    def run_detector(self, i):
        det = EI._learned_detector(self.unit, self.rng, 1000.0)
        det.idle_enter()
        return det.idle_block(self.out["meas"][i])

    def test_prying_raises_forced_while_locked(self):
        ev = self.run_detector(0)
        hits = np.flatnonzero(ev == core.IDLE_FORCED)
        self.assertTrue(len(hits) > 0)
        self.assertGreater(hits[0], 150)    # never before the attack starts (1.5 s)

    def test_knock_and_quiet_do_not_alarm(self):
        for i in (1, 2):
            ev = self.run_detector(i)
            self.assertFalse(np.any((ev == core.IDLE_FORCED) | (ev == core.IDLE_POWER_LOSS)))

    def test_power_loss(self):
        det = EI._learned_detector(self.unit, self.rng, 1000.0)
        det.idle_enter()
        ev = det.idle_block(np.zeros(400, dtype=np.float32))
        self.assertTrue(np.any(ev == core.IDLE_POWER_LOSS))


class CryptoBindingTest(unittest.TestCase):
    def test_hmac_matches_python(self):
        rng = np.random.default_rng(1)
        for n_key, n_msg in ((0, 0), (16, 3), (64, 64), (65, 200), (131, 1000)):
            key = bytes(rng.integers(0, 256, n_key, dtype=np.uint8))
            msg = bytes(rng.integers(0, 256, n_msg, dtype=np.uint8))
            self.assertEqual(core.hmac_sha256(key, msg), hmac.new(key, msg, hashlib.sha256).digest())

    def test_sha256_matches_python(self):
        for n in (0, 1, 55, 56, 63, 64, 65, 1000):
            msg = os.urandom(n)
            self.assertEqual(core.sha256(msg), hashlib.sha256(msg).digest())


class MetricsTest(unittest.TestCase):
    def test_auc(self):
        self.assertAlmostEqual(metrics.auc([3, 4], [1, 2]), 1.0)
        self.assertAlmostEqual(metrics.auc([1, 2], [3, 4]), 0.0)
        self.assertAlmostEqual(metrics.auc([1, 2], [1, 2]), 0.5)

    def test_threshold_for_fa(self):
        scores = np.arange(1000, dtype=float)
        t = metrics.threshold_for_fa(scores, 0.005)
        self.assertLessEqual(np.mean(scores > t), 0.005)
        self.assertGreater(np.mean(scores > t - 1.0), 0.005)

    def test_wilson_contains_rate(self):
        lo, hi = metrics.wilson(5, 100)
        self.assertLess(lo, 0.05)
        self.assertGreater(hi, 0.05)


if __name__ == "__main__":
    unittest.main()
