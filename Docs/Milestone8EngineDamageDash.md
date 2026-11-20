# Milestone 8: engine tuning, mechanical damage, Mychron dash

## Engine tuning and condition (`KartSim/EngineTune.h`, class-file block `"tuning"`)

| Piece | Model | Provenance |
|---|---|---|
| Air | dry-air density from temperature, pressure and humidity (Buck vapour pressure) | physics |
| Power vs air | rated curve x ISO 1585 SI correction inverted: (p_dry/99 kPa)^1.2 (298 K/T)^0.6 | exponents as widely quoted, ESTIMATE until checked against the standard |
| Best jet vs air | BestJet = BestJetRef (rho_dry/rho_ref)^0.70 | reproduces IAME X30 Shifter manual "+2-3 points per 6 degC" (VERIFIED, published) and RADNE's Rotax EVO steps 122/125/128 at >=18/<18/<10 degC (third party) |
| Richness | (Jet/BestJet)^2 for main jets (number = bore), ^1 for the KT100 needle (turns) | ESTIMATE |
| Power vs mixture | peak at r = 0.96 (+0.5 %), lean side falls fast, rich side slowly; 1.0 at r = 1 | ASSUMPTION |
| Cooling | one node (water + block, or cylinder head for the KT100), heated by the burnt charge, cooled by (base + per-speed) x (1 - radiator cover) x coolant | windows: Rotax 50-55 degC (AiM), IAME 50-58 degC (IAME manual), KT100 CHT 175-215 degC (forum, ESTIMATE); coefficients ESTIMATE |
| Piston / seizure | crown index = node + rise x charge load x (1 + 4 x leanness); above the limit the seize level grows (power loss = tightening), 1 = seized (engine locks: 80 Nm drag at the crank). Limit drops when the node is cold (piston expands faster than the bore) or > window + 35 degC (oil film) | ASSUMPTION (gameplay-calibrated, see tests) |
| EGT | idle..full by load/rpm, +1200 degC per unit leanness, -800 per unit richness, 1 s probe lag | full-load values: Rotax 550-650 (AiM), KZ 600-700 (forums) |

Setup page: **Main jet** (Rotax 110-140, KZ2 135-175), **Carb needle** (KT100, 1.5-3.5 turns in 1/16), **Radiator cover** (0-90 %).
The garage shows today's air, the power vs rated, and the jetting-chart jet. AI mechanics auto-jet and tape the
radiator (`AutoTune`). Weather presets now carry pressure and humidity; new **Cold** preset (6 degC).
Command line: `-KartAirTemp= -KartPressure= -KartHumidity=`.

Behaviour (Tests/EngineDamageTests.cpp, TestCircuit, 22 degC):
* jet 122 22.713 s; 120 (2 points lean) 22.708 s and hotter piston; 130 (rich) 22.788 s; 106 seizes on lap 2.
* water: open 49 degC, 20 % cover 55 degC (in the window), 90 % covered boils and seizes.
* hopelessly lean straight-line running seizes in ~4 s; the locked engine drags the rear tyres.

**Regression change:** the TestCircuit autopilot lap is now **22.712-22.713 s** (was 22.734) because power is
corrected for the air (+1.8 % at 22 degC / 1013 hPa / 50 % RH vs the ISO reference). Verified in the packaged
game (4 laps) and in the standalone tests.

## Mechanical damage (`KartSim/KartDamage.h`, class-file block `"damage"`)

From kart-to-kart contact forces (zones in the body frame) and tyre load spikes (kerb strikes / landings):
* nose / front corners: CIK front fairing displaced (> 4.5 kN rearward), bent steering arm = toe change per side
* rear bumper: bent axle (Coulomb drag 3 Nm/deg), chain thrown (one hit > 300 N s over 6 kN, or 1.5 deg bend)
* radiator side (left on Rotax / KZ): holed radiator, coolant leak -> overheating -> seizure
* front wheel load > 4.5 kN / rear > 6.5 kN: steering arm / axle bend. Clean laps peak at 0.6-0.8 kN (tests) and the
  user's driving telemetry at 2.4 kN front / 4.6 kN rear.

Thresholds are ASSUMPTIONS: a 1 m/s nudge does nothing, 3 m/s pushes the fairing and bends 0.2 deg, 6 m/s bends
~1 deg and throws the chain. A six-kart AI race does ~0.01 deg of damage in total.

Damage and a seized engine survive a reset in the core; in the game **R (reset) is a trip to the pits and repairs
everything**, as does a session start (grid placement). Settings page: Mechanical damage / Engine seizure on-off
(`-KartNoDamage`, `-KartNoFailures`). Not drawn on the model: the fairing displacement (the imported bodywork
merges the nose with the front panel), bent parts.

## Mychron dash (`KartDash.h/.cpp`, `Tools/CreateDashMaterial.py` -> `M_KartDash`)

A canvas render target on a quad over the measured front face of the rig's "display" part (the Mychron on the
steering wheel), redrawn at 25 Hz in the cockpit camera. Same drawing as a HUD overlay in the chase camera
(Settings toggle). Shows shift LEDs (from 70 % of the limiter, blue on the limiter), rpm bar and number, speed,
gear (KZ), lap time held 5 s after the line (green = best), lap number, best, live delta, water / CHT (blue cold,
yellow / red hot), EGT, and alarms (seized, chain off, tightening, coolant leak, water high / low, damage events).
Brightness `-KartDashNits=` (default 5000, physical units). Procedural-kart rigs have no display part: overlay only.
Verified on screenshots (cockpit at 22 and 4 degC, chase overlay).

Telemetry CSV (K) has the new channels: WaterTemp, PistonTemp, Egt, Richness, SeizeLevel, PowerFactor, Coolant,
AirDensity, DamageToeL/R, AxleBend, ChainOff, FairingOff.

Not verified: driving feel of the damage effects by a human; the damage model in multiplayer (each kart's damage is
computed where it is simulated; proxies do not show it).

## Revision 2 (user feedback: "hit someone slightly and my whole steering is gone")

* **Damage toned down** (all ASSUMPTIONS): contact threshold 6 -> 9 kN, toe 0.004 -> 0.0012 deg/N s (max 2.5 deg),
  axle 0.0025 -> 0.0015 deg/N s, chain off 300 -> 400 N s, wheel-load bends 4.5/6.5 -> 6/8 kN at a third of the rate.
  **Bug fixed:** a nose hit bent BOTH front steering arms; now the nose only releases the fairing and only front-CORNER
  hits bend steering. Rear-end at 3 m/s: fairing released, steering untouched; 9 m/s: axle 0.9 deg, chain off.
* **Fairing moves on the model:** Tools/SplitNose.py (Blender) splits the nose out of Body_skinfront (loose part ahead of
  x 1.25 m), Tools/ImportNose.py imports it, MakeBogKartRig.py tags it `"fairing": true`; the pawn pushes it 6 cm back
  and 2.5 deg nose-down when released (screenshots compared). `-KartTestFairing` releases it 1 s in.
* **Mychron:** damage messages removed (only sensor alarms: water / CHT, EGT). New **LAMBDA** tile (wide-band sensor:
  0.86 / richness under power, ESTIMATE best-power lambda; reads lean off throttle, 0.15 s lag). **[ / ]** change the jet
  (or 1/16 needle turn) one size leaner / richer on track; saved with the setup and shown on the dash.
* **Wheel-to-wheel and 3-D contact (KartContact):** tyre discs vs tyre discs (tread or sidewall, rotation-aware friction,
  spin torques), tyre ellipsoids vs the other kart's body boxes (floor, sidepods, tapered nose measured on the model,
  front panel, rear bumper, driver, helmet), bumper boxes split into main / nose / rear bumper (narrower than the rear
  tyres) and fading as karts separate vertically. Body points vs the ground (helmet, shoulders, steering wheel, nose,
  sidepods, bumper, seat, frame rails) let a kart roll over and lie on its side / roof.
  "Tread engagement" mass (12 kg) is an explicit ASSUMPTION: with a free, light front wheel the disc model alone only
  gives a 3 cm hop; real tyres interlock and deform.
  Tests: wheel on the outer part of a rear tyre at 3 / 6 / 9 m/s closing lifts the follower's wheel 0.43 / 0.48 / 0.55 m
  (tilt up to 22 deg) and it lands; dropped on its side / roof it comes to rest without exploding; a T-bone into a
  sidepod shoves (a 25 cm wheel cannot climb a 30 cm wall).
  The six-kart AI race now has up to 2 spins and 81 mm max bodywork overlap (was 0 / 55 mm): front wheels outside the
  nose rub other karts' sidepods. No AI launches in it. TestCircuit regression unchanged: 22.712-22.713 s.

## Revision 3 (user: "bodywork wasn't moving, idk about wheel to wheel")

The core sim released the fairing on every rear hit from 1.5 m/s (test Damage_RearHitMatrix), so the likely cause was
visibility: 6 cm / 2.5 deg is hidden from the chase and cockpit cameras. Now:
* nose 9 cm back + 5 deg down; **sidepods** (Tools/SplitSidepods.py splits Body_skinside into L / R) hang out and down
  7 deg when a side impact > 5.5 kN knocks them; **rear bumper** (Body_skinrear) pushed in 3 cm and drooping 6 deg on a
  rear impact > 6.5 kN. Rig key `"panel": nose | sidepodL | sidepodR | bumper`. `-KartTestDamage` knocks all of them.
  Screenshots checked (sidepod and nose clearly displaced).
* The game log now records `DAMAGE <driver>: ...`, `AIRBORNE <driver>: ...` and `WHEEL CONTACT <driver>: ...` lines for
  feedback sessions.
* Wheel-to-wheel only happens where tyres are exposed: the rear bumper covers the rear wheels from directly behind
  (as the CIK bumper is meant to), so a launch needs your front wheel on the OUTER edge of their rear tyre (~1.15 m
  lateral offset in the matrix test: wheel thrown up 0.14-0.27 m), or wheels banging side by side.

## Revision 4 (user: "chain pops off too much, bodywork doesn't really wiggle, the front bar gets pushed in too far, wheel-to-wheel still a bit iffy")

Diagnosis from the user's Saved/Logs/KRPKNKOFF.log: in the turn-1 pile-ups, several karts went from 0 to 1.7-3.0 deg axle bend in a
single frame, then CHAIN OFF. That needs a >100 kN contact force or a ~28 kN tyre load for one step: numerical spikes
of the penalty contacts, not loads that bend steel.

* **Damage inputs filtered** (KartSimulation): tyre loads 20 ms low-pass, kart-kart forces summed per channel (nose,
  rear, left, right, left/right front corner) and 15 ms low-pass. New `damage.maxBendPerHit` 1.0 deg: one strike / hit
  adds at most that much toe or axle bend. `chainOffBend` 1.5 -> 2.5 deg (about three hard hits), `chainOffImpulse`
  400 -> 350 N s of the *filtered* rear force (a ~9 m/s / 32 km/h rear-ending still throws it; 5 m/s does not). ESTIMATES.
  Test `Damage_PileUpAtTurnOne` (5-kart train at 20 m/s, leader brakes, late reactions): no chains off, max 0.16 deg toe.
* **Box contact damping** 2800 -> 4500 N s/m (~0.65 critical), faded in over the first 1 cm of overlap (no force step at
  first touch). Side-by-side banging (`Contact_SideBySideBangingStaysPlanted`): the 4 m/s / 0.55 m stagger case went from
  7 contact on/off cycles and 20 all-four-tyres-unloaded steps to 1 and 0. Rear bump at 5 m/s still dissipates ~55 % KE.
  AI race: max overlap 81 -> 52 mm.
* **Panel wobble** (pawn): every panel is a damped spring driven by the kart's specific force (minus static 1 g) plus
  white-noise vibration growing with speed and rpm. Mounted: 12 Hz, 20 % damping, 8 mm stop (in game 1-3 mm shimmer).
  Knocked loose: 4 Hz, 8 % damping, 45 mm stop (in game +-2-4 cm flapping). Vertical motion also rotates the panel about
  its hinge (1.5 deg/cm). `-KartPanelLog` logs the offsets twice a second. All ESTIMATES (no reference footage measured).
* **Nose pose** 9 cm / 5 deg -> 4 cm / 2.5 deg back and down.
* AIRBORNE log lines only for real flights (CoG peak > 3 cm).

Regression: TestCircuit autopilot 22.712 s (unchanged). 95/95 standalone tests pass.

Rev 4b (user: loose bodywork "jumps around too much, looks unrealistic"): loose panels 4 Hz / 8 % / 1.0 g noise / 45 mm stop
-> 7 Hz / 30 % / 0.35 g / 15 mm, hinge rotation 0.8 deg/cm. In game (-KartTestDamage -KartPanelLog, one lap): max 0.86 cm,
rms 0.28 cm, mostly a ~0.5-0.7 cm sway in corners (was +-2-4 cm). ESTIMATES.
