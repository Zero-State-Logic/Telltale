# TELLTALE simulation model

Everything in `results/eval.json` comes from this model (`sim/physics.py`, `sim/params.py`, `sim/sensor.py`) driven by the firmware's own motion profiles and scored by the firmware's own C detector. Nothing here has been checked against a real lock yet. Treat the numbers as a statement about this model, and the model as a hypothesis to test on hardware.

## Mechanism

An SG90-class micro servo turns a horn (11 mm radius). A cord loop / push-pull cable joins the horn to a spring-loaded bolt. Unlike the original Phantom Door's single pull cord, the linkage must transmit force **both ways**: prying the bolt back has to push on the horn, or the servo cannot feel it. With a pull-only cord, prying just slackens the cord. Jams, linkage loss, brown-out and friction are still visible that way; forced-while-locked is not.

```
J*w'  = N*eta*Kt*i_m - r*F_link - b_g*w - gear friction (Coulomb + stiction)
i_m   = (d*V_s - Ke*N*w) / R                 inductance neglected (L/R ~ 0.1 ms << 1 ms samples)
d     = clip(kp * (error beyond deadband))   averaged H-bridge duty from the servo's P-controller
V_s   = V_oc - (R_src + R_shunt) * I_s
I_s   = I_q + d*i_m                          supply current seen by the INA219
m*x'' = F_link - (F_pre + k_s*x) + F_stops + F_ext - b_b*x' - bolt friction (Coulomb + stiction)
F_link = k_c * (stretch beyond the lost motion) + damping
```

Semi-implicit Euler at 5 kHz, Karnopp stiction on the gear train and the bolt. The servo sees a new command only once per 20 ms frame, as a real 50 Hz servo does, so a slow ramp turns into a staircase of short current pulses.

## Parameters

| Parameter | Value | Source |
| --- | --- | --- |
| Stall torque | 1.8 kgf·cm (0.176 N·m) at 4.8 V | SG90 datasheet; reproduced by R, Kt, N, eta below |
| No-load speed | 0.1 s / 60° at 4.8 V | SG90 datasheet; reproduced (test in `tests/test_sim.py`) |
| Dead band | 10 µs → 0.9° | SG90 datasheet (2000 µs over 180°) |
| Motor + driver resistance R | 7.0 Ω | assumed: gives ~0.69 A stall at 4.8 V, in line with commonly reported SG90 stall currents |
| Ke = Kt | 0.0017 V·s/rad | derived from the no-load speed with N = 250 |
| Gear ratio N, efficiency eta | 250, 0.6 | assumed |
| Output inertia J | 2.5e-4 kg·m² | assumed rotor inertia 4e-9 kg·m² × N² |
| Gear friction (Coulomb / static / viscous) | 0.008 / 0.015 N·m / 2e-4 N·m·s | assumed |
| P-gain | 0.25 duty per degree | assumed (saturates at ~4.5° error) |
| Quiescent current I_q | 4–7 mA | assumed |
| Horn radius r | 11 mm | assumed |
| Linkage stiffness / damping / lost motion | 4 N/mm / 3 N·s/m / 0.1–0.3 mm | assumed |
| Bolt mass, spring, preload | 25 g, 0.25 N/mm, 1.5 N | assumed |
| Bolt friction (Coulomb / static) | 0.4 / 0.6 N | assumed |
| Supply | 4.8–5.2 V, 0.1–0.3 Ω + 0.1 Ω shunt | assumed USB-style 5 V supply |
| INA219 quantisation | 0.8 mA (9-bit), 0.1 mA (12-bit ×16) | assumed: 10 µV LSB × 2^(12−bits) with the 0.1 Ω shunt |
| INA219 noise | 1.0 mA rms (9-bit), 0.3 mA rms (12-bit ×16) | assumed |
| INA219 offset / gain error | ±0.5 mA / ±0.5 % per unit | assumed (order of the datasheet limits) |

## Variation

* **Unit to unit** (each simulated lock): gear and bolt friction ±25 %, spring and preload ±10 %, supply 4.8–5.2 V, supply resistance 0.1–0.3 Ω, linkage stiffness ±20 %, R ±10 %, Ke ±5 %, dead band and gain ±20 %, linkage neutral offset ±1°, INA219 offset and gain.
* **Run to run, nominal**: supply ±30 mV, friction ±5 %, spring ±1 %, random rest position inside the dead band, sensor noise.
* **Run to run, stress**: the full healthy spread (supply 4.8–5.2 V, friction ±25 %, spring ±10 %) re-drawn on every actuation. Reported separately as `actuation_stress`.

## Faults and attacks

| Scenario | Severities | How it is injected |
| --- | --- | --- |
| Jam on unlock | 30 / 50 / 70 / 90 % of travel | a hard stop on the bolt's retraction |
| Jam on relock (door ajar) | 30 / 50 / 70 / 90 % of travel | a hard stop on the bolt's extension once relock starts |
| Linkage broken | — | linkage stiffness and damping set to zero |
| Linkage slack | +1.5 / 3 / 5 mm lost motion | wider dead zone in the linkage |
| Brown-out | 4.6 / 4.4 / 4.2 V | low open-circuit voltage, supply resistance ×2.5–3.5 |
| Added friction | ×1.5 / 2 / 3 | all gear and bolt friction terms scaled |
| Prying while locked | 4 / 8 / 15 / 30 / 60 N peak at the bolt | ramp 0.2–1.0 s, then pumped ±30 % at 1.5–4 Hz for 2.5 s |
| Slow push | 30 N over 20 s | linear ramp, to test baseline tracking |
| Benign knock | 2–15 N, 20–120 ms | half-sine pulse at random times, ~1 per 2 minutes |

## Not modelled (yet)

* The 50 Hz pulse-stretched drive of analog servo ICs (an averaged duty is used instead), PWM current ripple beyond a noise term, motor inductance and cogging.
* Temperature, gear wear, backlash and play in the gear train, gear stripping under heavy prying, servo heating during long holds.
* Lateral loads on the bolt (door pushed against the strike), door-frame vibration, real knock spectra.
* ESP32 timing jitter and I2C errors; INA219 conversion details beyond quantisation and averaging.
* The fingerprint sensor and Wi-Fi stack (they do not touch the current signature).
