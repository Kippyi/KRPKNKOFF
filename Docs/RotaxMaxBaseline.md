# 125cc Rotax Max (Senior MAX EVO) – Baseline Specification

Each value is classified as one of:

* **VERIFIED**: from a manufacturer or regulatory source (cited).
* **ESTIMATE**: an engineering derivation (method shown).
* **ASSUMPTION**: a placeholder with no supporting data yet. It must be replaced by measurement.

The config file `Content/KartData/Karts/Rotax125MaxSenior.json` mirrors this table.

## 1. Engine

| Parameter | Value | Class | Source / method |
|---|---|---|---|
| Displacement | 124.8 cm³ | VERIFIED | Bore 54.00 mm × stroke 54.50 mm (Rotax / dealer specs) |
| Max power | 22 kW (30 hp) @ 11,500 rpm | VERIFIED | Rotax Senior MAX datasheet (ISO 15550 / ISO 4106) |
| Max torque | 21 Nm @ 9,000 rpm | VERIFIED | Rotax Senior MAX datasheet |
| Torque at 11,500 rpm | 18.27 Nm | ESTIMATE (derived) | P/ω = 22,000 / (11,500·2π/60) |
| Torque curve 5,000–14,000 rpm | see config | VERIFIED (digitised) | Read from the vector paths of the official datasheet power graph (2026-11-19): power rises to 22.1 kW at 11,100–11,500 rpm, then collapses past 12,000 rpm (11.6 kW at 13,150 rpm). The table anchors are kept; the graph's 21.7 Nm at 8,500 rpm is clamped to the stated 21 Nm maximum. Below 5,000 rpm and above 14,000 rpm: ESTIMATE |
| Rev limit | 14,800 rpm | ESTIMATE | Two secondary sources agree (Wikipedia "max rpm 14,800"; a Rotax distributor: "limiter at around 14,800 rpm"). Not Rotax-official |
| Idle | 2,500 rpm | ASSUMPTION | |
| Rotating inertia (crank, balance shaft, ignition rotor, clutch hub) | 0.0030 kg·m² | ASSUMPTION | Gives a free rev of about 0.2 s to the limiter at WOT. **Needs measurement** |
| Closed-throttle drag | 0.6 Nm + 0.0025 Nm/(rad/s) | ASSUMPTION | About 3.7 Nm at 12,000 rpm. Two-stroke engine braking is weak. **Needs coast-down data** |
| Throttle lag | 25 ms | ASSUMPTION | Carburettor transport + cable |

## 2. Clutch and drivetrain

| Parameter | Value | Class | Source / method |
|---|---|---|---|
| Clutch type | Dry centrifugal, single-speed | VERIFIED | Rotax regulations (centrifugal clutch section) |
| Engagement start | 3,800 rpm | ASSUMPTION within a VERIFIED bound | Global RMC Technical Regulation 2025 v3, §2.2.1: "Engagement speed of centrifugal clutch at maximum 4.000 rpm (the kart without driver must start to move)". Reproduced by test `Clutch_RegulationEngagementAt4000Rpm` |
| Clutch drum inner diameter | ≤ 84.90 mm | VERIFIED (limit) | Same regulation, §2.2.2. Shoe thickness ≥ 24.10 mm |
| Exhaust valve | electronically timed, settings A/B | VERIFIED (exists) | §2.6.5. The opening rpm is not stated, so the torque-curve shape stays an ESTIMATE |
| Full lock-up | 5,500 rpm | ASSUMPTION | |
| Torque capacity at lock-up | 55 Nm | ASSUMPTION | About 2.6 × peak engine torque. Must not slip at WOT once locked |
| Engine sprocket | 12 T | ASSUMPTION (setup) | Typical range 11–13 T. This is a track-dependent setup choice |
| Axle sprocket | 80 T | ASSUMPTION (setup) | Typical range 76–88 T |
| Chain efficiency | 0.95 | ESTIMATE | Roller-chain literature 0.94–0.98. Kart chains run dry or dirty |
| Rear axle assembly inertia (axle, hubs, rims, tyres, sprocket, disc, carrier) | 0.075 kg·m² | ESTIMATE | Two rear wheels at about 2.4 kg, r_g about 0.11 m, plus a 50 mm hollow axle and a ventilated disc |

## 3. Chassis (CIK-style senior chassis + driver)

| Parameter | Value | Class | Source / method |
|---|---|---|---|
| Minimum mass, kart + driver | 162 kg | VERIFIED (series-dependent) | One Senior MAX championship rule states 162 kg. Other series differ, so it is configurable |
| Wheelbase | 1.040 m | ESTIMATE | Within the CIK homologation range (101–107 cm). 1040 mm is common |
| Front track (tyre contact centres) | 1.100 m | ASSUMPTION (setup) | Adjustable with spacers |
| Rear track (tyre contact centres) | 1.240 m | ESTIMATE | Overall width ≤ 1.40 m minus rear tyre width (about 0.16 m) |
| Weight distribution, front | 42 % | ASSUMPTION | Typical guidance is 40–44 % front. **Measure on corner scales** |
| CoG height (kart + driver) | 0.27 m | ASSUMPTION | **Needs tilt-table or pendulum measurement** |
| Roll / pitch / yaw inertia | 14 / 24 / 32 kg·m² | ESTIMATE | Radii of gyration 0.29 / 0.385 / 0.445 m |
| Chassis torsional stiffness | 18,000 Nm/rad (≈ 314 Nm/deg) | ESTIMATE | FEA of a tubular kart frame reports 316.9 Nm/deg (UFSC monograph). **Real chassis need a twist-rig test** |
| Torsional damping | 120 Nm·s/rad | ASSUMPTION | |
| Front-section roll inertia (twist DOF) | 1.2 kg·m² | ESTIMATE | Front wheels, spindles and front frame, about 10 kg at about 0.35 m |
| Aerodynamic CdA | 0.60 m² | ASSUMPTION | Kart + seated driver Cd·A ≈ 0.55–0.7 m² is a commonly cited range. **Coast-down test needed** |

## 4. Steering and brakes

| Parameter | Value | Class | Source / method |
|---|---|---|---|
| Steering ratio (column : mean road wheel) | 3.6 : 1 | ASSUMPTION (setup) | Tie-rod hole selection changes it |
| Column lock | ±100° | ASSUMPTION | |
| Ackermann factor | 0.7 | ASSUMPTION (setup) | |
| Caster | 14° | ASSUMPTION (setup) | Adjustable via eccentric bushes |
| Kingpin inclination (KPI) | 10° | ASSUMPTION | |
| Kingpin to wheel-centre plane (spindle offset) | 0.085 m | ASSUMPTION | |
| Static toe / camber | 0° / 0° | ASSUMPTION (setup) | |
| Brake | Rear only, hydraulic disc | VERIFIED (class) | Senior MAX uses rear braking. The DD2 class adds front brakes |
| Max rear brake torque | 320 Nm | ESTIMATE | Must exceed the rear lock torque ≈ μ·F_z,rear·R ≈ 1.5·940·0.139 ≈ 196 Nm with margin |

## 5. Tyres – Mojo D5

| Parameter | Front | Rear | Class | Source / method |
|---|---|---|---|---|
| Size | 10×4.50-5 | 11×7.10-5 | VERIFIED | Rotax Mojo D5 product page / homologation sheet |
| Cold pressure | 0.60 bar | 0.60 bar | VERIFIED (recommended) | Rotax Mojo D5 page |
| Unloaded radius | 0.127 m | 0.1397 m | ESTIMATE | From the nominal 10" and 11" diameters |
| Radial stiffness | 110 kN/m | 190 kN/m | ESTIMATE | Contact area ≈ F_z/p; patch length from width; δ = a²/(2R) → k ≈ F_z/δ |
| Radial damping | 600 N·s/m | 900 N·s/m | ASSUMPTION | |
| Peak friction μ₀ at reference load | 1.65 | 1.65 | ASSUMPTION | Target about 2 g steady cornering on rubbered asphalt |
| Reference load F_z0 | 330 N | 460 N | ESTIMATE | Static corner load |
| Load sensitivity k_μ | 0.12 | 0.12 | ASSUMPTION | |
| Cornering stiffness coefficient c_y (C_y / F_z0) | 28 /rad | 30 /rad | ASSUMPTION | Peak lateral slip angle about 9–10° |
| Longitudinal stiffness coefficient c_x | 24 | 26 | ASSUMPTION | |
| Relaxation length σ_x / σ_y | 0.08 / 0.14 m | 0.08 / 0.16 m | ASSUMPTION | |
| Optimum temperature / window | 70 °C ± 35 °C | 70 °C ± 35 °C | ASSUMPTION | |
| Rolling resistance C_rr | 0.020 | 0.020 | ASSUMPTION | |
| Wheel + tyre spin inertia | 0.014 kg·m² | (part of axle) | ESTIMATE | |

## 6. Measurements needed to calibrate (priority order)

1. **Tyre force and moment data** for the Mojo D5 (F_y versus α at 3 loads, F_x versus κ, M_z) from a flat-track
   or trailer rig. This is the single largest source of uncertainty.
2. **Corner weights and CoG height** of a representative kart + 80 kg driver.
3. **Chassis torsional stiffness**: front axle twisted with the rear clamped, Nm/deg.
4. **Engine dyno curve** (brake torque versus rpm at WOT, 3,000–14,500 rpm) and motoring torque.
5. **Clutch engagement and lock-up rpm** (data-logger rpm versus axle speed from a standing start).
6. **Coast-down test** (speed versus time on the level) to separate CdA from rolling resistance.
7. **Logged lap data** (rpm, speed, lateral g, steering angle) for overall validation, e.g. from
   a MyChron or AiM logger.
8. Caster, KPI, spindle offset and steering ratio from a real chassis's geometry.

## 6b. Model results to compare against real data (scenario harness, 2026-11-17)

| Scenario | Model result |
|---|---|
| Standing start, WOT | clutch slips at about 4,100 rpm, locks at 4.0 s; 0–60 km/h 6.8 s, 0–100 km/h 10.0 s |
| Top speed (12/80) | about 108 km/h, at the 14,800 rpm limiter |
| Braking 90→0 km/h (rear only) | 55 m, mean 0.57 g, peak 0.71 g; rear axle locks at ≥ 45 % pedal |
| Coast-down from 90 km/h | 54 km/h at 5.5 s, 36 km/h at 10.0 s |
| Steady-state ramp (25° column) | max 1.14 g, then oversteer. The inside rear is unloaded to 121 N but not lifted, and the solid axle forces about 9 % longitudinal slip on it |
| 30 mm × 1 m kerb at 54 km/h | the whole kart leaves the ground briefly; rear peak Fz 2.5 kN per tyre |

The lateral-g capability and the inside-rear lift threshold are the most important open
validation items. I found no published Rotax Senior peak lateral-g figure, so grip has **not**
been tuned to a guessed target.

## 7. Sources

* Global RMC Technical Regulation 2025 Version 3 (copy hosted by Estonian federation): https://autosport.ee/wp-content/uploads/2025/04/Rotax-MAX-Engines-2024-Technical-Specifications2025.pdf
* Go-kart patent (torsional stiffness design target 3,000–7,000 in-lb/deg ≈ 339–791 Nm/deg; design guidance, not a measurement): https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/4682668
* Multi-body elastic simulation of a go-kart (frame torsion tests; total front-to-rear stiffness matters more than its distribution): https://www.researchgate.net/publication/226692235_Multi-body_elastic_simulation_of_a_go-kart_Correlation_between_frame_stiffness_and_dynamic_performance

* Kart data/limiter context (distributor article): https://kartclass.com/blogs/news/understanding-data-loggers

* Rotax Senior MAX datasheet: https://www.rotax-racing.com/assets/uploads/Engines/2021_Datasheet_Senior_Printpreview.pdf
* Rotax 125 Senior MAX EVO product page: https://rotax-racing.com/products/125-senior-max-evo
* Rotax Mojo D5 product page: https://www.rotax-racing.com/products/mojo-d5
* Mojo D5 rear homologation sheet: https://www.rotax-racing.com/assets/uploads/MOJO/Homologation-24/D5_MOJO-050-TP-45_rear.pdf
* Senior MAX championship overview (162 kg minimum weight): https://kart.co.za/championships/senior-max/
* Kart frame torsional stiffness FEA (UFSC): https://repositorio.ufsc.br/handle/123456789/274335
* Rotax Max (Wikipedia, for the unconfirmed rev figure): https://en.wikipedia.org/wiki/Rotax_Max
