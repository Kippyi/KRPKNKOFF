# Milestone 3 – Multiple Kart Classes

All three classes run on the **same code**. Each one is only a JSON file in `Content/KartData/Karts/`.
Class-specific behaviour (clutch type, gearbox, starter, brakes) comes from generic components
selected by configuration.

## New generic mechanisms (core)
* **Engine running state:** it stalls below `stallRpm` and fires above `fireRpm` when turned (starter, push, or
  through the drivetrain). A stopped engine is pure drag.
* **Onboard starter:** `starterTorque` > 0 (the Rotax has one, VERIFIED: Rotax operator manual).
* **Push start:** `KartInputs::PushForce`, an external force along the heading (a helper pushing).
* **Sequential dog box:** a shift passes through neutral for `shiftCutTime`. The speed mismatch on engagement is
  absorbed through clutch slip. Lifting the throttle is the driver's job (or the auto-shift aid's).
* **Driver aids (input only):** `KartSim/DriverAids.h` holds auto-clutch, auto-shift and auto-push. The game and the
  automated tests use the same code.

## 125cc KZ2 – IAME Screamer KZ (`KZ2_IAMEScreamer.json`)
| Item | Value | Class | Source |
|---|---|---|---|
| Minimum weight (kart + driver) | 175.0 kg | VERIFIED | CIK-FIA Technical Regulations 2025 |
| Brakes | four-wheel | VERIFIED | CIK-FIA TR 2025: "4WP B4 or BRKF + BRKR" in KZ2 |
| Engine | water-cooled 125 cm³ reed-valve, gearbox inseparable | VERIFIED | CIK-FIA TR 2025 §9.10.1 |
| Gearbox | 6-speed, hand-operated mechanical | VERIFIED | CIK-FIA TR 2025 |
| Primary | 19/75 | VERIFIED | CIK homologation form 22/M/24 |
| Gears | 13/32, 16/29, 18/27, 22/27, 22/23, 26/24 | VERIFIED | Form 22/M/24. Exact match with the form's check angles (test `KZ2_GearboxMatchesHomologationForm`) |
| Wheelbase range | 1010–1070 mm | VERIFIED | CIK-FIA TR 2025 (Group 2) |
| Power | ~35 kW @ 13,000 rpm | ESTIMATE | IAME: "over 40 hp" (lower bound). Class references ~48–52 hp are unverified |
| Final drive 19/28, clutch capacity 70 Nm, front brake 150 Nm/wheel, inertia, idle, limiter | — | ASSUMPTION | |
| Tyres | Rotax/Mojo D5 parameters | ASSUMPTION | No KZ tyre data |

## 100cc direct drive – Yamaha KT100 (`KT100_DirectDrive.json`)
| Item | Value | Class | Source |
|---|---|---|---|
| Displacement | 97.6 cm³ (52 × 46 mm) | VERIFIED | Yamaha 2005 spec as cited by Wikipedia |
| Power | 11 kW (15 hp) @ 10,000 rpm | VERIFIED | Same |
| Direct drive, push start, stalls when stopped | — | VERIFIED (configuration) | KartPulse and TKART descriptions of direct-drive KT100 |
| Torque curve shape, gearing 11/75, weight 160 kg, inertia | — | ESTIMATE / ASSUMPTION | Class weights vary by series |

## Validation (automated, `Tests/ClassTests.cpp` and `KartSimTests --laps`)
| Check | Rotax | KZ2 | KT100 |
|---|---|---|---|
| 0–100 km/h | 10.0 s | 4.35 s (shifting through 6 gears) | — (top speed about 92 km/h on the circuit) |
| Driver bot, 1.4 g target | 22.80 s, 0 spins | 22.37 s, 0 spins | 24.06 s, 0 spins |
| Class-specific | clutch engages ≤ 4,000 rpm (RMC rule) | stalls if the clutch is dropped at idle | stopped at rest, push start fires at 3.4 m/s, stalls when stopped |

### Class comparison (`KartSimTests --classes`; model output to compare with real data)
| Class | 0–60 | 0–100 | Top (30 s) | 100→0 locked | 100→0 best pedal |
|---|---|---|---|---|---|
| KT100 direct drive | 11.4 s (from push-start fire) | — | 98 km/h | 49.1 m | 49.0 m |
| Rotax Max | 6.8 s | 10.0 s | 111 km/h | 49.2 m | 48.8 m |
| KZ2 | 2.1 s | 4.0 s | 142 km/h | 24.1 m | 24.0 m |

The KZ2's four-wheel brakes halve the stopping distance (about 1.6 g against 0.8 g for rear-only karts). The KT100
launch is dominated by the ESTIMATED low-rpm torque and gearing.

### The 5.47 g anomaly: resolved
* **Cause:** the KZ2 bot (1.8 g target) slid at about 72 km/h and 25–30° sideslip under full throttle. Sliding friction
  (μ ≈ 1.8) at ground level gives a roll moment near the tip threshold (t/2h ≈ 2.3 g), so the kart rocked and hopped and
  landed while still sliding. That gave 4.65 g vertical and μ × that lateral. This is physically consistent.
* **No numerical energy injection:** the new test `Energy_SidewaysSlideNeverGainsEnergy` audits total energy
  (kinetic, potential, frame twist, tyre radial springs, tyre carcass deflection and spin) in free slides at 30/60/90°.
  Energy never rises by more than 1 J per step, and free slides never leave the ground.
* **Bugs found and fixed:**
  1. `SetRearAxleOmega()` left the per-wheel spin copy stale until the next substep (an inconsistent state).
  2. **The auto-shift aid downshifted on engine rpm**, which collapses during wheel hop. It went 4→3→2→1 at 67 km/h.
     Downshifts now use road speed and only happen if the lower gear stays below the upshift point. Regression test
     `KZ2_AutoShiftNeverOverRevsOnDownshift`. KZ2 spins at the 1.8 g target fell from 8 to 5.

**Known issues:** at a 1.8 g target the KZ2 bot still spins 5 times. A naive full-throttle bot overdrives a 47 hp kart,
so this is not a regression-test target. KZ2 and KT100 tyre and engine-shape data are estimates.

## Frame model study: torsional chain (2026-11-17)

**Question:** literature reports that total front-to-rear frame stiffness matters more than its distribution,
but in this model the front/rear split mattered a lot.

**Implementation:** `ChassisModel` is now a general torsional chain. Stations (nodes) along the wheelbase each
carry [position, roll inertia, mass, mass height], joined by uniform-GJ segments or explicit
`segmentStiffness`/`segmentDamping`. The rigid roll is the inertia-weighted mean; node twists are relative
(Σ J·φ = 0 exactly). Each node mass contributes its d'Alembert roll load m·z·f_y. The loader validates that the
nodes reproduce the total mass, CoG height and weight distribution.

**Findings:**
1. **Distributed-mass layout (research config `Rotax125MaxSenior_DistributedFrame.json`):** with ESTIMATED
   masses (85 kg driver/seat high and close to the rear axle) the kart becomes power-oversteer prone. At
   full throttle 20° of steering gives 18° sideslip, and 30° spins. It still laps cleanly with a modulating
   driver (0 spins, 1.66 g). Real karts don't behave like that, so the mass layout, the seat compliance or the tyre
   data is wrong or missing. **Not adopted** until corner weights, driver position and a twist-rig measurement exist.
2. **Defaults use the lumped configuration** (front section | body with all mass at the CoG | rear section), the
   chain equivalent of the validated Milestone 2 model.
3. **Bugs found on the way:**
   * **Tyre deflection memory (energy injection).** The relaxation-length states could integrate far past
     the sliding boundary, so at low speed a sliding tyre kept pushing after the slip reversed (+6–12 J/step
     in slow spins). Fix: carcass deflection capped at brush λ ≤ 1 (force unchanged at the boundary). The energy
     audit now shows at most 0.3 J/step (was 0.8).
   * **A misdiagnosis, corrected:** I first blamed the energy gain on the twist-axis height and moved the axis
     to the CoG. That exaggerated twist excitation 2.7× and broke the drivability tests. I reverted it; the
     frame-rail axis is physically right.
4. A fragile test (steering "lightening" judged on a non-steady, rotating case) was replaced by its physical
   origin: `Tyre_AligningMomentCollapsesAtSaturation` (Mz peaks at 3° slip at 57 % of max Fy and is 0 when fully sliding).

**Side effect:** the KZ2 bot at a 1.8 g target spins 9 times (5 before the cap). That target is beyond grip for a naive
full-throttle bot and is not a regression target. All 29 tests pass.

## In game
Keys **1/2/3** switch Rotax / KZ2 / KT100. **E/Q** shift, **Left Shift** clutch, **B** starter (Rotax),
hold **P** to push start (KT100), **H** toggles auto-shift (default off), **J** toggles auto-clutch (default on).
Command line: `-KartClass=KZ2_IAMEScreamer.json`.
