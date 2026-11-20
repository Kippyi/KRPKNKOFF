# Kart Setup (Milestone 5 item, started early)

**Code:** `KartSim/Setup.h` (engine-free parameter table with legal ranges, JSON save/load). In game: **Tab**.
Setups are saved per class to `Saved/Setups/<classId>.json` and applied automatically when the class loads.
Backspace in the menu restores the class file's values.

| Parameter | Range | Basis of range |
|---|---|---|
| Engine sprocket | 8–24 T | covers direct-drive / TaG (11–14 T) and KZ (19–20 T) practice; ASSUMPTION |
| Axle sprocket | 20–95 T | covers KZ (~26–30 T) and TaG (~76–88 T); ASSUMPTION |
| Front track | 0.95–1.20 m | typical spacer range; keeps the front axle inside 1400 mm overall |
| Rear track | 1.10–1.22 m | **1.22 m = CIK 1400 mm max overall width (VERIFIED) − 180 mm rear tyre** |
| Caster | 8–20° | typical eccentric-bush range; ASSUMPTION |
| Front toe | −2 to +2° | ASSUMPTION |
| Front camber | −3 to +1° | ASSUMPTION |
| Ackermann | 0–1 | model blend factor |
| Steering ratio | 2.8–4.5 : 1 | tie-rod hole positions; ASSUMPTION |

## Correction made while adding ranges
All class configs had a rear track of 1.240 m. With the 180 mm rear tyre that is **1.42 m overall, wider than the CIK
limit**, even though the config comment described the correct derivation. It was corrected to 1.220 m in every class. The
test suite and driver-bot results were unaffected within reporting precision.

## Verified effects (automated, `Tests/SetupTests.cpp`)
| Change | Result |
|---|---|
| Axle sprocket 76 → 88 (Rotax 12 T) | 0–60 km/h 7.38 → 5.84 s; top speed 115.1 → 103.6 km/h |
| Caster 8° → 20° | inside-rear load drop at 90° column 104 → 237 N (more jacking) |
| Rear track 1.22 → 1.10 m | rear outside–inside load difference 473 → 518 N per g |
| Steering ratio 4.2 → 3.0 | yaw rate at 15° column, 36 km/h: 16.6 → 23.3 deg/s |

These confirm the setup changes act in the physically expected direction. The magnitudes inherit the model's
ESTIMATE/ASSUMPTION parameters.
