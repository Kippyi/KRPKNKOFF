# Milestone 2 – Physics Refinement: Log

## 2026-11-17 – Drivability (user report: "can't drive straight and turn without spinning")

### Reproduction
* Open-loop maneuvers (`KartSimTests --maneuvers`): 10° column at 50 km/h with full throttle spun the
  kart, and so did 50 % brake with 10° of steer.
* Closed-loop driver bot (`KartSimTests --laps`): a careful 1.0 g-target driver spun **16 times in 3 laps**.
  That makes it a model problem, not only a keyboard problem.

### Root causes found (in order of impact)
1. **Chassis compliance topology (model flaw).** All frame torsion sat at the front, so the rear axle was
   rigid to the main body and took about 90 % of lateral load transfer. Fix: a rear-section twist DOF
   (PhysicsModel §2.2). Result: 0 spins up to a 1.8 g target.
2. **Tyre thermal window inconsistent with the thermal model.** The assumed 70 °C optimum was never reached
   (the model runs at 35–60 °C), so grip was cut by up to 20 % all the time. No published D5 window exists (Mojo:
   "not possible to recommend a perfect tyre temperature"). Fix: optimum 50 °C, window 40 °C, max penalty 8 %
   until measured.
3. **Front/rear tyre parameters ignored the width difference** (4.5" vs 7.1"). Fix: rear cornering stiffness
   36/rad vs front 26/rad; load sensitivity rear 0.07 vs front 0.16.
4. **Sliding friction drop 18 %** made the rear snap. Fix: 8 % with a 6 m/s reference speed.
5. **Grip level** recalibrated to μ₀ = 1.95, targeting the 1.5–2.2 g band reported for karts (data-logger vendor
   guide plus logged-data discussions; low-confidence sources, see RotaxMaxBaseline §7).
6. **Driver-bot quality.** Pure pursuit ignored the slip-angle demand; yaw-rate feedback was added. This is a test-tool fix.

### Input layer (keyboard / gamepad only; physics untouched)
* Steering lock limited to L·a/v² + slip allowance (1.4 g + 5°), i.e. about 33° of column at 50 km/h.
* Keyboard brake capped at 0.40 (rear-only brakes lock above about 0.45 at speed), with slower ramps.

### Results after the changes
| Check | Before | After |
|---|---|---|
| Driver bot 1.0 g target | 16 spins | 0 spins, max off-line 0.8 m |
| Driver bot 1.8 g target | 35 spins | 0 spins, 1.71 g peak, best lap 21.2 s |
| 50 km/h, 20° column, full throttle | spin | 1.33 g, 2.5° sideslip |
| Steady-state max lateral | 1.14 g | 1.58 g |
| Braking 90→0 km/h | 55 m | 40–42 m |

## 2026-11-17 – Camber thrust (user: "better, a little hard to turn")

* The keyboard steering limit was loosened from 1.4 g to 1.6 g plus 5° of slip (input only).
* **Camber thrust added to the tyre model.** The inclination angle is computed from the contact geometry
  (γ = asin(−axle·n)), and thrust enters the brush as an equivalent lateral slip, ratio·γ, where
  ratio = C_γ/C_y = 0.15 (ASSUMPTION: car-tyre range 0.1–0.2, no kart data).
* Front camber emerges from caster and KPI: at 90° of column the inside front leans 7.2° and the outside 3.8°,
  both into the turn. Test `Steering_FrontCamberFromCasterAtLock`.
* No regressions: 18/18 tests pass and driver-bot lap times are unchanged (small steer angles on this circuit).

## 2026-11-17 – Steering torque validation, tyre enveloping, penetration rate

User decision: no FFB wheel is available, so wheel/FFB work is deferred and physics continues.

**Steering torque (future FFB signal), test `Steering_ColumnTorqueIsSelfCentring`:**
* Self-centring at standstill (weight jacking, about −3 Nm at 60° column) and in corners (caster trail, −3 to −8 Nm).
* It goes light past front saturation (30 km/h: −7.9 Nm at 30° column, −6.0 Nm at 60°). That is the understeer cue.
* It reverses only once the kart is spinning, which is physically correct (the wheel snaps into opposite lock).
* In game: +7.0 Nm centring at 81 km/h and 1.0 g.

**Tyre enveloping (effective road plane):** 7 ground samples per wheel (5 along ±0.5 R, 2 across ±0.35 W),
weighted plane fit. Penetration comes from the effective *height*; the slope only tilts the force direction.
The first version measured penetration along the tilted normal, which added spurious penetration at edges. It was found by A/B test and fixed.
Cost in UE: about 0.39 ms of physics per frame. Switch: `TyreModel::bEnableEnveloping`.

**Penetration rate bug fixed:** the tyre damper used only the wheel-centre velocity and ignored the road
rising under the wheel, so steps produced zero damper response. The rate is now the change in penetration per step.

**Sharp 25 mm × 1 m block at 54 km/h (A/B):** single-point contact gives a 12 g vertical spike; enveloping gives 7.9 g.
In both modes the kart briefly leaves the ground (about 0.3 s). That is governed by tyre vertical
stiffness and damping (ESTIMATE/ASSUMPTION), so it is not tuned without measured data.

## 2026-11-17 – Regulation research (engine/clutch)

* The text of the **Global RMC Technical Regulation 2025 v3** was extracted (the PDF has a text layer; the fetch tool
  could not read it directly).
* §2.2.1: Senior MAX clutch engagement ≤ 4,000 rpm, defined as "the kart without driver must start to move".
  The model's 3,800 rpm engagement start is inside this bound. The procedure is reproduced as test
  `Clutch_RegulationEngagementAt4000Rpm` (an 85 kg kart without driver moves at 4,000 rpm and stays still at 3,600 rpm).
* §2.2.2 clutch dimensions and §2.6.5 exhaust valve settings A/B were recorded. The regulation states no exhaust-valve
  opening rpm and no rev limit, so the torque-curve shape and the 14,800 rpm limiter stay ESTIMATEs.

**Tension with literature (unresolved):** a multibody kart study reports that total front-to-rear frame stiffness
matters more than its distribution along the frame. In this model, moving the compliance from front-only to
split front/rear changed handling drastically, even though the total stiffness was identical. The likely reason is that this model
lumps all non-axle mass into one rigid body between the two springs, while a real frame carries its mass along its
length. A multi-segment frame (or comparison with the paper's data) is a candidate M2/M3 item. A go-kart patent
cites a torsional stiffness design target of 339–791 Nm/deg, higher than the 314 Nm/deg FEA value used here.

### Still open
* Inside-rear lift no longer occurs on the test circuit (0 % of lap time), only above about 1.75 g with
  large steering. Real karts reportedly lift in slow corners. This needs logged data (steering angle and lift
  video) before changing the jacking geometry, CoG height or the frame split.
* The front/rear frame stiffness split is an assumption. A twist-rig measurement would settle it.
* The wheel/pedal DirectInput layer and FFB output are not started yet.
