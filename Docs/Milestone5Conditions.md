# Milestone 5 – Simulation Features

M5 items in the brief: setup adjustments and lap timing (both done during M3), **dynamic track grip,
rubber build-up, weather, practice/qualifying/race sessions, telemetry analysis** (this document).
A single kart only: opponents need the Milestone 6 AI, so a race is currently against the clock.

**Every coefficient below is an ASSUMPTION.** There is no measured rubber-build, wet-grip or drying data. The
tests pin down the model's *structure* (where rubber goes, how water comes and goes, that grip reaches the
tyres), not its magnitudes.

## Track conditions (`KartSim/TrackConditions.h`, engine-free)
A grid on the track centre line: 1 m along × 0.5 m across, ±6 m (468 × 24 cells on the test circuit).
Each cell holds **rubber** r ∈ [0,1] and **water film** w (mm).

| Process | Model |
|---|---|
| Rubber deposit | ∝ tyre **sliding power** at each contact patch (`RubberPerJoule` 2e-5 /J) |
| Rubber loss | 2 %/h natural, 5 % per mm of rain (washing) |
| Rain | +RainRate mm/h on every cell, capped at 2 mm (drainage limit) |
| Drainage / evaporation | drain 1 mm/h × w/2 mm; evaporation 0.3 mm/h at 20 °C track, ×2 per +15 °C |
| Tyre squeegee | each tyre pass removes 4 % of the cell's water → **a dry line appears** |
| Track temperature | relaxes (τ 600 s) to air + 25 °C × (1−cloud) (×0.1 while raining) |

Grip multiplier per cell (reaches the tyre via `GroundHit::GripMultiplier`, so the tyre model is unchanged):
`dry = 1 + 0.08 r`; `wet = 1 − 0.4 · min(1, w / 0.5 mm)`; wet rubber is extra slippery: `× (1 − 0.15 r · wetFrac)`.
Limits: green dry 1.00, fully rubbered 1.08 (= the earlier RubberedAsphalt assumption), saturated 0.60, saturated
rubber 0.51. Only paved surfaces (asphalt variants, kerbs) are affected. The weather's track temperature
also feeds the tyre thermal model.

**Rubber rate:** one kart deposits only about 0.1 % of full rubber per lap (4 autopilot laps → 0.5 %). That's
the intended realism: tracks rubber in over hundreds of kart-laps. Sessions therefore start from a preset:
**Green** (r = 0) or **Rubbered** (r = 0.7 in a ±1 m band on the centre line, tapering over another metre; the
shared Autopilot drives the centre line). Racing-line rubber is not modelled separately from the centre line.

### In game (`AKartConditionsActor`)
* `-KartWeather=Dry|Hot|Overcast|Rain|Drying` (key **N** cycles), `-KartTrack=Green|Rubbered` (key **M**).
* `-KartWeatherTimeScale=x` speeds up the weather and drying; `-KartConditions=0` disables the system.
* Visuals: track-wide wet look (darker, glossier paved surfaces from the mean water depth), sun dimmed by cloud,
  thicker haze in overcast and rain. **Not visualised:** the per-cell dry line and rubber (needs a dense mesh or a
  runtime texture), rain particles, spray, cloud coverage itself.

## Sessions (M5: `KartSim/Session.h` + `FKartRaceControl`; superseded in M6 by the multi-kart `KartSim/RaceDirector.h` and `AKartRaceWorld`, same rules)
Key **Y** starts the next session (practice → qualifying → race), moving the kart to the grid.
Command line: `-KartSession=Practice|Qualifying|Race`, `-KartRaceLaps=n` (10), `-KartQualiMinutes=m` (8).
* **Practice:** laps counted, best valid lap.
* **Qualifying:** countdown; at zero the chequered flag is out and the lap in progress still counts.
* **Race:** N laps. **Start procedure by class**:
  * gearbox classes (KZ) take a **standing start**: 3 s lights, held on the brakes, green.
  * single-speed classes (Rotax, KT100) take a **rolling start**: a formation lap, with green at the line. This
    matches their usual real-world procedure; a direct-drive KT100 can't be held on the brakes without
    stalling, and the Rotax's centrifugal-clutch launch is slow.
* Laps with track-limit violations are counted (no penalty model).

## Telemetry analysis (`KartSim/LapDelta.h`)
Time and speed are sampled every 2 m of centre-line distance (`KartSim/TrackPath.h` projection). The best
valid full lap is the reference. HUD: **live delta** (green = gaining) and **predicted lap**, always on. The
telemetry view (T) adds a **speed-vs-distance trace** of the best lap (grey) against the current lap (yellow).
The CSV recorder (K) is unchanged for offline analysis.

## Verification
* Standalone tests 47/47. New: grip limits, rain/drying and temperature, rubber only where tyres ran
  (track edges stay clean), dry line forming on a wet track, wet grip reaching the kart (dry pace on a
  saturated track: 7 spins; wet pace 0.8 g: clean, 27.92 s vs 22.68 s dry), qualifying flag rule, race
  distance/total (analytic to 1 ms), rolling start, lap delta (−0.556 s measured vs −0.557 s actual),
  centre-line projection.
* In engine (autopilot, Rotax, 3 laps):

| Scenario | Lap |
|---|---|
| `-KartConditions=0` | 22.683 s (identical to before M5) |
| Dry, green (default) | 22.682–22.683 s |
| Dry, rubbered | 22.666–22.669 s (the autopilot drives to a fixed g target, so it only gains in braking/traction) |
| Rain, 0.8 g target | 27.921 s (= the standalone test) |
| Race 2 laps, Rotax rolling | 45.365 s = 22.683 + 22.682 |
| Race 2 laps, KZ standing | 46.095 s (launch from the lights ≈ +1.7 s) |
| Qualifying 0.6 min | flag during the first timed lap; that lap counted |

## Not done / limitations
* **No wet tyres.** Karts run slicks in all weather. Rain tyres (and tyre choice) need compound data.
* No aquaplaning or standing-water depth effects beyond the grip floor; no puddle geometry (no drainage slope).
* Rubber marbles off-line are not modelled; there is no per-cell temperature (the track temperature is uniform).
* Sessions are single-kart; no opponents, grid positions or penalties until M6.
