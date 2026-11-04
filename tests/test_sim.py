"""Physics and sensor-model sanity checks (python -m unittest discover -s tests -v)."""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from sim import params as P, physics as PH, scenarios as S, sensor  # noqa: E402


def nominal(n=1, **overrides):
    p = P.broadcast(P.NOMINAL, n)
    for key, value in overrides.items():
        p[key] = np.broadcast_to(np.asarray(value, dtype=float), (n,)).copy()
    return p


class ServoPhysicsTest(unittest.TestCase):
    def test_no_load_speed_matches_datasheet(self):
        # Linkage removed, 4.8 V, command steps 10 -> 70 deg: SG90 datasheet 0.1 s / 60 deg.
        p = nominal(Voc=4.8, R_src=0.0, shunt=0.0, k_c=0.0, c_l=0.0)
        cmd = np.concatenate([np.full(5, 10.0), np.full(2500, 70.0)])
        out = PH.simulate(p, cmd, keep_state=True, pre_roll=100)
        theta = out["theta_deg"][0]
        t_reach = np.argmax(theta >= 69.0)   # ms
        self.assertGreater(t_reach, 80)
        self.assertLess(t_reach, 135)

    def test_stall_current_is_about_v_over_r(self):
        p = nominal(jam_hi=1.0e-3)   # bolt blocked after 1 mm
        cmd = np.concatenate([np.full(100, 10.0), np.full(2500, 60.0)])
        out = PH.simulate(p, cmd)
        i_end = out["current_ma"][0, -50:].mean()
        v_s = out["vbus_v"][0, -50:].mean()
        expected = (v_s / P.NOMINAL["R"]) * 1000.0 + P.NOMINAL["Iq"] * 1000.0
        self.assertAlmostEqual(i_end, expected, delta=0.05 * expected)
        self.assertGreater(i_end, 550.0)

    def test_idle_current_is_quiescent(self):
        p = nominal()
        out = PH.simulate(p, np.full(5000, PH.LOCKED_DEG), period=50, avg=43)
        np.testing.assert_allclose(out["current_ma"][0, 5:], P.NOMINAL["Iq"] * 1000.0, atol=0.05)

    def test_holding_current_grows_with_force_on_bolt(self):
        forces = np.array([0.0, 6.0, 10.0, 14.0])
        p = nominal(len(forces))
        out = PH.simulate(p, np.full(10000, PH.LOCKED_DEG),
                          f_ext=lambda k: forces * (k > 2000), period=50, avg=43)
        settled = out["current_ma"][:, -20:].mean(axis=1)
        self.assertTrue(np.all(np.diff(settled) > 0), settled)
        self.assertAlmostEqual(settled[0], 5.0, delta=0.1)

    def test_healthy_cycle_shape(self):
        layout = S.cycle_layout()
        rng = np.random.default_rng(0)
        res = S.simulate_cycles(nominal(2), rng, layout, keep_state=True)
        unlock = res["unlock"]
        self.assertEqual(unlock.shape[1], layout.unlock_ms)
        self.assertGreater(unlock[:, :layout.unlock_motion_ms].mean(), 60.0)
        self.assertLess(unlock.max(), 700.0)
        # bolt retracts ~15 mm on unlock and returns to the extended stop on relock
        self.assertGreater(res["x"].max(), 13.0)
        self.assertLess(abs(res["x"][0, -1]), 0.2)

    def test_jams_and_broken_linkage_change_the_trace(self):
        layout = S.cycle_layout()
        rng = np.random.default_rng(1)
        base = nominal(3)
        p = P.concat([P.take(base, np.array([0])),
                      P.apply_fault(P.take(base, np.array([1])), "jam_unlock", 0.5, rng),
                      P.apply_fault(P.take(base, np.array([2])), "linkage_broken", 1.0, rng)])
        res = S.simulate_cycles(p, rng, layout)
        plateau = res["unlock"][:, -100:].mean(axis=1)
        self.assertGreater(plateau[1], 3 * plateau[0])     # stalled against the jam
        self.assertLess(plateau[2], 0.3 * plateau[0])      # nothing to hold


class SensorModelTest(unittest.TestCase):
    def test_quantisation_steps(self):
        self.assertAlmostEqual(sensor.quant_step_ma("fast"), 0.8)
        self.assertAlmostEqual(sensor.quant_step_ma("idle"), 0.1)

    def test_measure_adds_offset_and_quantises(self):
        p = nominal(2, ina_offset=[0.0, 0.5], noise_fast=0.0)
        rng = np.random.default_rng(0)
        y = sensor.measure(np.full((2, 4), 100.0), p, rng, "fast")
        self.assertTrue(np.allclose(y[0], 100.0))
        self.assertTrue(np.allclose(np.round(y / 0.8) * 0.8, y, atol=1e-4))
        self.assertGreater(y[1, 0], y[0, 0])

    def test_range_is_clipped(self):
        p = nominal(1, noise_fast=0.0)
        y = sensor.measure(np.full((1, 2), 5000.0), p, np.random.default_rng(0), "fast")
        self.assertTrue(np.all(y <= 800.0))


class ParamsTest(unittest.TestCase):
    def test_unit_spread_within_brief(self):
        u = P.sample_units(np.random.default_rng(3), 500)
        self.assertTrue(np.all((u["Voc"] >= 4.8) & (u["Voc"] <= 5.2)))
        ratio = u["k_s"] / P.NOMINAL["k_s"]
        self.assertTrue(np.all((ratio >= 0.9) & (ratio <= 1.1)))
        fr = u["tau_c"] / P.NOMINAL["tau_c"]
        self.assertTrue(np.all((fr >= 0.75) & (fr <= 1.25)))

    def test_unknown_fault_rejected(self):
        with self.assertRaises(ValueError):
            P.apply_fault(nominal(), "gremlins", 1.0, np.random.default_rng(0))


if __name__ == "__main__":
    unittest.main()
