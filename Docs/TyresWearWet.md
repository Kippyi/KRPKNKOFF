# Tyre compounds, wet tyres, wear and changing weather (2026-11-18)

## What changed
* **Wet grip is now a tyre property.** The track conditions give each contact a *wetness* (0 dry .. 1 saturated
  water film, `GroundHit::Wetness`); the tyre turns that into grip with its compound's `wet.gripFloor`:
  mu x (1 - (1 - gripFloor) x wetness). The slick keeps the old value (0.60), so slick behaviour and all old
  results are unchanged. The `WetAsphalt` surface now means wetness 1 (grip factor 1.0 x the tyre's floor).
  Rubber on a wet line is still extra slippery (road factor in TrackConditions).
* **Two tyre sets per class:** `tyres` (dry, required) and `wetTyres` (optional), each `{name, front, rear}`.
  Every class now has a wet set (Mojo W5). Choose in the garage / Tab setup: **Tyres (0 = slick, 1 = wet)**.
  It is saved with the setup.
* **Wet cooling:** water on the tread cools it (`wet.cooling`, W/K at full wetness, ASSUMPTION 8 for both).
* **Wear model:** `Wear` = fraction of the tread used; shown as mm left.
  dWear = rate x sliding power x dt x 2^((T - optimum) / tempDoubling) (clamped 0.25..16).
  Grip = 1 - (1 - minGripFactor) x Wear^2: little loss while there is tread, then it falls away.
* **Used tyre sets carry over.** The player's wear per compound is saved to `Saved/Setups/<classId>.tyres.json`
  (every 15 s, on setup apply, class switch and exit) and refitted next session or launch. A kart reset
  (put back on track) keeps its tyres. **Garage > Fit new tyres** fits a new set of the edited compound.
  AI karts and `-KartDefaultSetup` always run new tyres, so regression runs are unaffected.
* **AI:** at spawn and at each session start the AI fits the set with more grip for the wetness on the racing
  line (`ChooseTyreCompound`). Its cornering target is scaled by the grip of its tyres on the current line
  (`CompoundGripRatio`): soaked track on slicks 1.4 g -> 0.84 g, on W5 -> 1.07 g.
* **Changing weather:** new weather preset **Shower** (`-KartWeather=Shower`, key N, menu). Dry and clouding
  over for 6 min, rain 8 mm/h from minute 7 to 15, then it clears. It runs on the conditions clock, so
  `-KartWeatherTimeScale` speeds it up. Each multiplayer machine runs it from when it applied the preset, so a
  late joiner is behind.
* **HUD:** bottom-left panel shows the fitted set and, per wheel, temperature (blue cold / green in window /
  red hot, relative to that compound's window) and tread left. "grip x" under the kart now includes the fitted
  compound's wet loss. Telemetry wheel boxes also show mm.

## Data and provenance
| Item | Value | Status | Source |
|---|---|---|---|
| D5 rear 7.1/11.0-5: tread depth | 3.5 mm +-0.5 | VERIFIED | CIK homologation form 050-TP-45 |
| D5 rear: service pressure | 0.6 bar +-0.3 | VERIFIED | 050-TP-45 |
| D5 rear: diameter / tread width / hardness | 275 mm / 135 mm / 39.3 IRHD | VERIFIED | 050-TP-45 |
| D5 rear: deflection under 40 kg | V 1.0 mm, L 12.0 mm (+-2 / +-1) | VERIFIED | 050-TP-45 |
| W5 rear 11x6.00-5: tread depth | 5 mm +-0.5 | VERIFIED | CIK homologation form 050-TW-77 |
| W5 rear: service pressure | 1.0 bar +-0.3 | VERIFIED | 050-TW-77 |
| W5 rear: diameter / tread width / hardness | 276 mm / 125 mm / 45.0 IRHD | VERIFIED | 050-TW-77 |
| W5 rear: deflection under 40 kg | V 2.0 mm, L 11.0 mm | VERIFIED | 050-TW-77 |
| W5 front size 10x4.50-5 | | VERIFIED | Rotax W5 product page |
| W5 rear model radius 0.138 m | 276/2 | VERIFIED | 050-TW-77 |
| W5 rear model width 0.167 m | 0.180 x 125/135 | ESTIMATE | tread-width ratio to the D5 |
| W5 front tread depth 5 mm | | ASSUMPTION | same as the rear (front form not read) |
| W5 radial stiffness = D5 | | ASSUMPTION | V 1.0 vs 2.0 mm is within the +-2 mm tolerance |
| W5 mu0 x0.90, slip stiffness x0.85, slideMuDrop 0.12 | | ASSUMPTION | no wet-tyre force data; tread-block flex |
| W5 wet grip floor 0.85 (slick 0.60) | | ASSUMPTION | gives wets ~1.3x slick grip when soaked; crossover at wetness 0.38 |
| W5 thermal window 35 +-30 C, heat fraction 0.6 | | ASSUMPTION | |
| Wear rate 4.0e-7 /J (both sets), doubling 15 C slick / 10 C wet | | ESTIMATE / ASSUMPTION | calibrated: most-worn slick uses ~2/3 of its tread in ~1 h at autopilot pace (forum reports of 60-80 good laps; not a Mojo figure) |
| Wear grip loss 15 % with the tread gone, quadratic | | ASSUMPTION | |

Findings not applied (they would change the dry physics and the regression; need a decision and data):
* The D5 rear form gives a vertical deflection of 1.0 mm under 40 kg (~390 kN/m at the test conditions,
  pressure not stated). The model uses 190 kN/m (contact-patch estimate). The +-2 mm tolerance makes the form
  value very coarse.
* The D5 rear diameter is 275 mm (model 279.4 mm from the nominal 11 in).
* W5 rear weighs 1450 g vs 1695 g for the D5 rear (rear axle inertia not changed per set).

## Results (standalone tests, `Tests/TyreWearTests.cpp`, Rotax, test circuit)
* Soaked track, driver target 1.1 g: **W5 clean, 24.75 s; slicks 11 spins**, 28.7 s.
* Dry, 8 laps at 1.4 g: W5 used **4.5x more tread** than slicks (0.32 vs 0.07 mm) and ran hotter relative to its
  window. On a dry track wets have 10 % less peak grip.
* Worn slicks (85 % of the tread used) at a 1.6 g target: 21.91 s vs 21.77 s new, 0.7 m further off line.
* Slick life: ~0.67 of the tread per hour of running on the most-worn tyre.
* Shower timeline: dry -> rain (line wetness 1.0, the AI picks W5) -> water goes down after it stops. Without
  karts the line dries slowly (still 0.75 wetness after an hour). Traffic clears the line much faster (squeegee).
* Regression: Autopilot 1.4 g, 22.683 s, unchanged. 68/68 tests pass.

## Not done / limitations
* No aquaplaning (water depth above saturation has no extra effect), no puddles.
* No graining, blistering, heat cycles, compound hardening, or a scrub-in (bedding) phase for new tyres.
* No tyre pressure setting (both sets run their nominal stiffness).
* No visual difference between slick and wet tyres on the kart model.
* No pit lane: change sets in the garage, which resets the kart where it stands.
