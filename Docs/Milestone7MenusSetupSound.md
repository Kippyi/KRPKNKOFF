# Milestone 7 - Menus, full setup garage, sound (in progress)

## Main menu / pause menu (SKartMenu, Slate, no UI assets)
Startup shows the main menu (unless the game was launched with test flags such as -KartAutopilot / -KartAI /
-KartSession / -KartNoMenu, or as a multiplayer client). Pages: Practice, Race weekend, Multiplayer (host /
join by IP), Garage, Settings (sound), Controls, Quit. Esc in game opens the pause menu (pauses single player).
Menu choices travel to the session through UKartGameInstance (command-line flags still win, so the
automated tests are unaffected).

## Setup garage
All setup values as spin boxes with a one-line explanation, live total weight / weight split / final drive /
CoG height, Apply + save / Revert / Class defaults; saved per class (Saved/Setups/<class>.json), loaded at start.
New adjustments (tests: 57/57):
* Ballast 0-25 kg, mounted low (ASSUMPTION 0.15 m) - adds mass, lowers CoG, adds pitch/yaw inertia.
* Seat height (CoG height; frame mass heights follow) - test: 0.24 -> 0.30 m gives +22 % rear load transfer per g.
* Seat fore/aft (front weight %).
* Chassis stiffness 70-130 % (frame torsional stiffness + damping).
* Front brake share (only classes with front brakes - KZ); total brake torque preserved.
Not adjustable yet (no physics for them): tyre pressures, rear axle hardness, ride height, carburettor.

## Sound
* Recorded engine layers are supported and preferred: 16-bit WAV loops on_<rpm>.wav / off_<rpm>.wav in
  Content/KartData/Sounds/<classId>/ (rotax_max_senior, kz2_iame, kt100_dd), pitch-shifted by rpm and crossfaded
  by rpm and throttle (Content/KartData/Sounds/README.txt). No free (CC0) kart recordings were found; options are
  recording your own onboard audio or licensed packs.
* Fallback synthesis per class: harmonics of the firing frequency (rpm/60, one firing per revolution)
  shaped by fixed resonances, cycle-to-cycle variation, overrun misfire/pops. Class characters (ASSUMPTIONS):
  Rotax muffled (mandatory intake + exhaust silencers), KZ louder/brighter with crackle (the CIK 108 dB limit
  was aimed at KZ), KT100 thinner with pipe "ring". The user rated the first synth version "horrendous";
  the second version is untested by ear.
* Tyre scrub from sliding power, contact thumps, wind (own kart). Other karts are 3-D (5 m / 65 m).
* Settings page: master, own engine, other karts, tyres/contacts, wind, mute (GameUserSettings.ini [KartSound]).
* Verification: -KartRecordAudio=<s> records the game output to Saved/AudioCapture/KartAudio.wav and logs the
  player's mean rpm; checked level (-20 dBFS Rotax), no clipping, no underruns with 6 karts, dominant
  frequency consistent with the firing frequency.

## Wheel, pedals, force feedback (FKartWheelInput, DirectInput 8)
* All attached game controllers are read each frame (8 axes, 128 buttons). The first force-feedback device is
  opened exclusive with a constant-force effect (autocentre off).
* Menu > Wheel & pedals: devices, live readout, Detect buttons (steering, throttle, brake, clutch, shift up/down),
  wheel rotation (match the wheel software, e.g. SimPro Manager), FFB strength, invert, soft-lock strength,
  Rescan devices. Saved in GameUserSettings.ini [KartWheel].
* Steering 1:1 (wheel angle = column angle, no assists); soft lock at the kart's steering lock (~100 deg).
  Force = simulated column torque x strength + soft lock. Sign: + sim force = wheel left; DirectInput +X = right
  (ASSUMPTION about the device convention: Invert exists for the opposite case).
* Verified on this PC: SIMAGIC Alpha EVO Wheelbase detected, constant-force effect created ("force feedback
  ACTIVE"), steering auto-bound to its X axis. NOT yet verified: feel/direction/strength of the force and pedal
  detection (needs the user driving).
* Note: the wheel must be connected to the PC running the game (Parsec forwards gamepads, not wheels).
* -KartDefaultSetup ignores saved setups (regression runs; the user's saved setup changed the autopilot lap).

## Bug: "all four tyres spazzing / death wobble" (user report, fixed)
* The user's telemetry CSV (35 s) had 7 episodes, every one with a tyre on GRASS or a KERB. They showed tyre
  load flickering every 4 ms at the 5 mm asphalt/grass lip, and one launch at 73 km/h (FR over grass into a
  kerb END: 1.2 kN, then RR 3.5 kN at 3.6 g, all four wheels airborne).
* Cause 1 (physics): the tyre damper used the raw change in penetration, so any ground STEP read as metres per
  second for one step. Fix: the road part of the rate is low-passed over the time the contact patch takes to
  roll across it (contact length / speed); the wheel's own motion stays exact. Test
  Tyre_GroundStepDoesNotSpikeTheDamper: 5 mm step down 339 N, step up 935 N (= the rear tyre spring's
  k*h of 950 N, which is physical).
* Cause 2 (track): procedural kerbs started and ended with a vertical 25 mm face, and corners were split into
  fragments with many ends. Fix: gaps up to 3 m are merged, and every kerb run tapers from flush to full height
  over 2 m.
* A wobble detector stays in the game: a tyre-load jump over 500 N in one step logs the context and saves 5 s of
  telemetry to Saved/Wobble/.
* Regression unchanged: 22.683 s (-KartDefaultSetup). Safety: -KartNoWheel keeps automated runs from driving a
  direct-drive base unattended.

## Bug: KZ "lifting on all four, slamming down, repeat" (user report, fixed)
* User telemetry (KZ, 99 s): 50 all-four-airborne moments, all on plain asphalt mid-corner at 70-105 km/h in a big
  slide (~48 deg column, 10-19 deg sideslip). The kart rolled onto the outside tyres (up to 3.9 kN, 2 cm
  deflection), the barely damped tyre springs (~10 % critical) threw it into the air for 0.1-0.15 s, it slammed
  down, and this repeated about 5 times a second.
* Missing physics: the seated driver (~40 % of the mass) is a strongly damped body, but the model coupled it rigidly.
  Fix: new required tyre field bodyDamping (ASSUMPTION, = the carcass value), acting on the wheel's own vertical
  motion only, so road bumps are not made harsher. The carcass damping still acts on the road profile.
* Tests: Kart_SlidingKZDoesNotPogoHop (the same manoeuvre that gave 5 hops: 0 now). Step-test bounds were reset to
  catch the old ~4.3 kN damper bug with margin. Smooth-driving baselines barely move: Rotax 22.682 s, KZ 22.21 s,
  KT100 24.01 s. Riverside KZ AI race: no hop events.

## Tracks (layout files)
* Layouts are data: Content/KartData/Tracks/<file>.json with name, width, kerbRadius and controlPoints (a closed
  Catmull-Rom centre line in metres; start/finish at the first point). Ground size follows the layout bounds.
  The AI racing line, timing lines, conditions grid and grid slots all derive from the centre line.
* Choose a track in the menu (Practice / Race weekend / Host), or with -KartCircuit=<file>. The default with no
  choice is TestCircuit.json (regression: 22.683 s unchanged). Multiplayer clients rebuild their track to the
  host's layout (replicated; OnLayoutChanged re-reads timing, conditions and the race world).
* Riverside Kart Park (new, fictional): 1014 m, 8 m wide; T1 hairpin R12/173 deg after a 161 m straight, esses, fast
  sweepers, technical complex (R12 / R13 / R17), long fast left, tight R10 final corner; min clearance between
  sections 35 m. Designed with a checker that reproduces the game's spline (length, radii, clearance, map).
  Verified: 6-kart AI race, all laps valid, about 47-48 s laps, no wobble events.

## Track detail: camber, embankments, kerb types (Crestwood Raceway)
* Control points may carry camber in degrees ([x, y, camber]; + banked with the outside raised, - off-camber),
  smoothly blended. The asphalt is tilted across its width; the lower edge stays flush with the grass, and
  the raised edge gets a grass verge (max 15 % slope, 1.5-5 m wide). The physics needs no special code:
  ground traces and tyre enveloping see the real surface.
* Explicit kerbs per control-point range: side inside / outside / both; type flat (25 mm) / rumble (saw-tooth
  ridges 0.25 m apart, 8-24 mm) / high (rounded 60 mm hump, 0.8 m wide). All runs are tapered. Tracks without a
  kerb list keep the automatic flat kerbs (Test Circuit lap unchanged at 22.682 s; surface self-test PASS).
* Crestwood Raceway (new): 951 m; T1 banked 5 deg, top sweeper 4 deg, final corner 3 deg; off-camber (-3 deg)
  hairpin exit; chicane with high kerbs on the insides; rumble strips in the fast corners. AI races: Rotax laps
  ~49-50 s, KZ ~48 s, all valid, no wobble events.
* High-kerb bench (left wheels over the 60 mm hump at 40-80 km/h): 1.5-1.9 g jolt, ~3 deg roll, two wheels light
  for up to 0.08 s, the kart knocked 0.2-0.4 m off line; never all four wheels airborne.
* Elevation (v2): a 4th control-point value (metres), Catmull-Rom smoothed. The grass is a 2.5 m terrain grid:
  8 cm under the track/shoulders, blending over 25 m into gentle hills around the mean track height. Both sides
  get a 2.5 m grass shoulder flush with the edge (the old 5 mm lip is gone). Barriers sit on the terrain; the
  centre line, start and timing lines carry the elevation.
* "Whole track feels like a rumble strip" (user, Rotax on Crestwood v1): the asphalt was one quad per 0.75 m.
  With a changing bank the quads are not planar, so the diagonals fold by a few mm: in the wobble dump, 90-200 N
  of tyre-load ripple at ~25 Hz on banked asphalt and 16 % FFB ripple. Fix: 4 rows per sample x 8 strips, each
  vertex exact. Re-measured on Crestwood v2: banked 10-20 N, FFB 1.9 %.
* High kerb softened to a 40 mm hump, 1.0 m wide (60 mm was too violent at speed). Exit kerbs (outside, end of
  corner) on every corner.
* Crestwood v2: elevation -1.5..+6 m, max grade 5.2 %, tightest crest R 218 m (0.37 g unloading at 100 km/h).
  AI laps valid: Rotax ~49.8 s, KZ ~48.3 s.
* Found, NOT fixed yet: KZ standing launch. With the auto clutch it reaches 2.3 g (the traction limit of the assumed
  mu 1.95); dumping the clutch at revs gives unphysical 4.3 g spikes and a 10 deg wheelie with the fronts up for 1 s.
  This causes start-line pile-ups in KZ races (19.5 kN contact on Crestwood v2).

## Still to do (asked for)
* Packaged build + F8 feedback bundle for testers.

## Kart and driver model v2 (2026-11-18)
* Source/KRPKNKOFF/Private/KartVehicleModel.cpp (BuildVisuals moved out of the pawn). Visual only; physics regression
  unchanged (22.682 s).
* New mesh tools in FKartMeshBuilder: AddLoft (smooth surface through rings, livery per vertex), SuperEllipse,
  AddTaperTube, AddEllipsoid (with patches).
* Bodywork: CIK-style four-piece kit shaped after current kits of the Eurostar Dynamica type (front fairing with
  raised side ends, curved nassau panel, sidepods raised at the rear, full-width rear bumper). ESTIMATED proportions
  from CIK rules / typical kits, not measured; no logos. Livery = BodyColor + pearl + black.
* Seat bucket, fuel tank, radiator + hoses, airbox, exhaust (header, chamber, silencer), chain guard, brake disc +
  caliper, bearing cassettes, tie rods; per class: KZ gearbox + gear lever, KT100 finned air-cooled barrel.
* Driver: lofted torso, rib protector, neck collar, two-bone IK arms (hands at 9 and 3) and legs (knees splayed round
  the wheel), boots, gloves, helmet with centre stripe and dark visor. Steering wheel with grips, hub, display.
* Debug: -KartShowroom=<cm> orbit camera (-KartShowroomYaw=, -KartShowroomPitch= for a fixed view), -KartShots=t1,t2
  timed in-game screenshots, -KartCockpit starts in cockpit view, -KartRecord records telemetry from the start.

## Kart and driver model v3 proportions (2026-11-18, user reference photo of a CIK senior kart)
* User: "the current ones are too small and the driver looks squished". The driver's height was right (helmet top
  ~1.0 m); the kart around them was short (1.85 m), with low sidepods (0.15-0.28 m), a 0.21 m fairing and a low steering wheel.
* Kart (visual only, physics unchanged): ~2.0 m overall (ESTIMATE, typical senior kart with CIK fairing and rear
  protection), front fairing 1.12 m wide, nose 0.60 m ahead of the front axle, top kept below the top of the front
  tyres; sidepods 0.20-0.275 m (up to the tyre-top plane); taller front panel; deeper rear bumper 0.33 m behind the rear
  tyres; steering wheel centre 0.53 m. Limits VERIFIED in Motorsport UK NCR (follows FIA karting bodywork rules): front
  fairing not above the plane through the top of the front wheels, lateral bodywork not cutting the plane through the
  tops of the front and rear tyres, fairing within 180 mm of the front tyres.
* Driver: torso reclined ~13 deg with the seat, bulkier suit (adult proportions), feet on pedals 7 cm ahead of the front
  axle, knees splayed, helmet top ~1.03 m. Cockpit eye moved to (0.39 m ahead of the rear axle, 0.95 m up).
* Previous model kept for comparison: no (git-less project; the v2 file is in the session scratchpad only).

## Kart and driver model v4: detail pass (2026-11-18)
* The user asked for "more detailed, like real 3D models". Free-model search: about 60 downloadable Sketchfab karts (no CC0
  CIK-style kart; the one good CC-BY candidate needs a logged-in download). The user chose to push the procedural model.
  The user's bodyworks.pkz is an encrypted Kart Racing Pro package (entropy 7.99 bits/byte, no zip headers): not extracted
  (third-party protected content).
* New mesh-builder tools: AddCylinder (capped), AddTorus, AddRoundBox, SharpenColors (tried for hard livery edges; it gave
  sawtooth edges on lofted panels, so it is unused).
* Driver: lofted helmet (longer back, chin bar, gloss visor band, centre and side stripes, top vents, visor pivots, rear
  spoiler); gloves (cuff, back of hand, four fingers wrapped round the grip, thumb); elbows, knee pads, boots with soles;
  rib protector straps, zip, neck brace; suit side band and waist seam.
* Steering wheel: flat three-spoke wheel, contoured grips, boss and bolts, a generic data display with a screen and six shift LEDs.
* Engine bay: crankcase on a mount plate with rail clamps; clutch drum; starter motor; water-cooled cylinder, head and bolts,
  plug cap, HT lead and coil (KT100: finned barrel); carburettor with intake rubber to a rounded silencer with two snorkels;
  header, lofted expansion chamber with joint springs, silencer and stay; radiator with core, 26 fins, side frame,
  tanks, shutter, stays and hoses.
* Chassis: 48-tooth axle sprocket, engine sprocket and chain; chain guard; ventilated brake disc (two plates + 24 vanes),
  carrier, caliper, bracket, hose and master cylinder; rear hubs with bolts; seat struts; tie-rod ends; steering column
  support; battery (Rotax); fuel cap, fuel line and throttle cable.
* Race numbers: yellow plates with seven-segment digits on the front panel, rear bumper and both sidepods; number from the
  driver name (AI karts differ).
* Wheels: tyre with a bulged sidewall and rounded shoulder (72 segments); rim with flanges, outer lip, dished face, six vent
  holes, three hub nuts and a valve stem.
* Bodywork mesh density x2-2.5; smoother, brighter metals (less noise). Fairing livery bands use absolute heights.
* Physics unchanged: regression 22.682 / 22.682 s. A run with 3 AI karts shows no errors.

## Driver v5: reclined and animated (2026-11-18)
* The user said the driver was "too upright, it looks weird". The seat and spine now recline ~25 deg from vertical (was ~12),
  and the torso rings are built normal to the spine. The head stays level, looking forward.
* Animated (procedural rig, every frame in ApplyRender, all karts including AI, network proxies, replay puppets and the ghost):
  * the torso (TorsoMesh) pivots at the pelvis: sways to the outside of the corner (6 deg/g, max 10) and forward
    under braking (5 deg/g);
  * the helmet pivots at the neck: keeps 70 % of the roll level and turns into the corner with the steering (0.18 x
    column angle, max 14 deg);
  * the hands stay on the rim and turn with the steering wheel; the forearms and upper arms are rigid segments placed by
    two-bone IK from the leaning shoulders;
  * accelerations come from the chassis velocity (finite difference, smoothed 0.12 s), so the rig works without physics data;
  * the cockpit camera follows the head position only (no roll or yaw, for comfort).
* The steering wheel angle now comes from the simulated front wheels, so AI and replay karts turn their wheels too
  (before, only the local player's wheel turned).
* Not animated: legs and feet (pedal presses), no skeletal mesh (procedural segments).
* Physics unchanged (regression 22.682 s).

## KZ standing start fixed (2026-11-18)
* Reproduced in core tests (Tests/LaunchTests.cpp). A clutch release over 0.15 s from 11-13.5k rpm gave 3.8-4.9 g on the
  accelerometer channel, 4.3 kN rear load (2.5 x the kart's weight), a 10 deg wheelie with the front up 0.9 s, and
  power hop (the rear tyres leaving the ground at ~10 Hz, the drivetrain winding up). A dump from 11k: 2.3 g, then a bog
  to 1,900 rpm (24 km/h after 1 s).
* Diagnosis: the physics is consistent (traction limit ~2.3 g with the assumed mu, front lift above ~1.55 g =
  m g x front fraction x wheelbase / CoG height). The 4.9 g peaks are hop impacts seen at 10 deg pitch. The defect was
  the shared driver aid: the auto clutch engaged fully by 9,000 rpm, i.e. a clutch dump at every AI / keyboard start.
* Fix (KartSim/DriverAids.h, used by AI and the keyboard auto-clutch): launch control slips the clutch to hold 75 % of
  the rev limit, with clutch torque capped at 80 % of the front-lift force (a slipping clutch transmits exactly its
  capacity, so the drive force is set directly); it keeps slipping until the slip is < 300 rpm (smooth lock-up). It eases
  the throttle when the front unloads in first gear, and the auto-upshift half-dips the clutch until the new gear's
  slip closes.
* Result (Launch_KzLaunchControlIsCleanAndFast): 0-60 km/h 1.76 s, 0-100 3.69 s (was 3.97; real KZ ~3-3.5 s, ESTIMATE
  from secondary sources); first gear 1.28 g, no front lift, never below 11,365 rpm; upshift kick 1.62 g (was 2.3),
  0.03 s front lift; rear load never below 790 N (no hop). 76/76 tests pass; regression 22.683 s.
* In game: two 8-kart KZ standing-start races on Crestwood: 0.0 s contact (before: pile-ups up to 19.5 kN).
* A full manual clutch dump with a pedal still wheelies and hops (realistic in kind; the magnitude follows the assumed mu).

## Imported driver suit: texture and clean-up (2026-11-19)
Pipeline: Tools/DriverSuit/make_driver.py (Blender, needs DriverRig.json from -KartDumpDriverRig) -> Art/Driver ->
Tools/DriverSuit/import_driver.py (also creates M_DriverSuit) -> Tools/MakeBogKartRig.py -> Tools/PackageGame.bat.
* Collar peak fixed: the fused helmet's lower edge is not level (the chin bar starts ~3 cm below the narrowest neck slice
  at the front, the rear rim ends ~2 cm above it; measured from the radius and texture-colour profile round the neck).
  The level vertex cut left a chin flap and a rim flap. It is now a plane bisect sloping 0.25 m/m towards the front.
* Shoulder streaks fixed: the masked distance weights gave a hard arm/torso seam, and posing the arms forward stretched
  the shoulder-blade faces. Weights are now diffused over the mesh edge graph (~6 cm, about 107 iterations), keeping the
  top 4 influences, and the armature uses dual-quaternion (preserve volume) skinning.
* Wrist fixed: the cuff detector found the finger roots (0.79 m in the T-pose) instead of the wrist (0.684 m, where the
  glove gauntlet narrows). Hands are now cut geometrically at the wrist, so the forearms end in the black gauntlet and
  the procedural gloves take over.
* Suit texture: Tripo's base colour, normal and roughness maps go into M_DriverSuit (two-sided). Metallic = 0, because
  the supplied metallic map (mean 0.37) is non-physical for fabric or leather. Roughness is remapped to 0.55-1.0
  (ASSUMPTION, visual: a matte fabric). Material classes are suittex/boottex. Human drivers get M_DriverSuit; AI drivers
  fall back to the flat suit in their kart colour, with plastic boots.
* Verified in the packaged game: showroom shots from front-right and above-left; in the cockpit view (-KartCockpit) the
  Arai helmet is hidden and the textured sleeve and gauntlet are visible. Not checked on screen: an AI driver's fallback
  suit.
* Known texture limits (from the AI-generated source): there is soft smearing on the shoulder caps, and the texture has
  some baked-in shading.

### Follow-up (user: "the helmet is still in the body there is no neck and the suit looks horrendous")
* Cause of the sunken helmet: the suit's hip->shoulder length (0.558 m) is 7.8 cm longer than the game rig's seat
  pelvis->shoulder (0.480 m along the spine), so the collar ended ~12 cm up inside the helmet. Now the spine and chest
  bones drop 7.8 cm along the spine (computed, not hand-tuned) so the suit's shoulder line lands on the game's shoulders.
* Neck: a tapered matte-black balaclava tube (r 0.060 m, ESTIMATE from an adult neck circumference) runs from inside the
  collar into the helmet (part Driver_Neck on the torso). The helmet mesh sits 2.5 cm higher along the spine (visual
  only; the cockpit eye is unchanged).
* Texture: Tools/DriverSuit/clean_suit_texture.py turns the AI base colour into flat panels (classify, majority filter,
  drop wide accent blobs and short fragments, merge small specks). Output: Art/Driver/suit_basecolor_clean.png. Racing
  blue sRGB (0.10, 0.22, 0.52), black side panels, long red and white piping only. The AI normal map is no longer
  connected, because it caused dark mottling.
* Verified on 4 packaged-game showroom shots (side, front, above-front, three-quarter).

### Follow-up 2 (user: "helmet need to come up a bit more and the shoulders seem really chunky")
* Measured: the overall shoulder width was 0.49 m (a normal adult bideltoid breadth), but the suit was still 0.39 m wide
  2.5 cm below the collar. The AI mesh's shoulder tops ran flat and level out to the arms (box / shoulder pads), and the
  sleeves were baggy.
* make_driver.py tuning (env-overridable, used for A/B previews): ARM_SLIM 0.82 (radial sleeve scale, ramping in over
  the shoulder cap), SHOULDER_DROP 0.055 m (top surface lowered from the neck outward, fading along the upper arm;
  0.035 still read boxy), DQ off (linear-blend skinning; dual quaternion did not help the bulge), HELMET_LIFT 0.05 m
  (written to driver_layout.json; MakeBogKartRig.py applies it; the neck tube is extended to match).
* Verified in the packaged game (front, three-quarter and wide shots). The cleaned suit texture is kept: with the clean
  palette it no longer looks dirty, and dropping it would lose the panels and piping.
* Follow-up 3 (user: shoulders "a bit funky, too slim"): backed off to ARM_SLIM 0.90 and SHOULDER_DROP 0.045, chosen from
  a side-by-side front preview of 0.82/0.055, 0.90/0.045 and 0.95/0.045. Verified in the packaged game.

## Engine sound v2: physically structured two-stroke synth (2026-11-19)
* `kartsim::EngineSoundSynth` (Source/KartSimCore/.../EngineSound.h/.cpp, engine-free) replaces the harmonic-table
  fallback in UKartSynthWave (recorded WAV layers in Content/KartData/Sounds/<class>/ still take priority).
  Path: exhaust blowdown pulse per revolution (shape in crank degrees, cycle-to-cycle amplitude / timing variation,
  misfires on overrun, occasional pops) -> tuned-pipe waveguide (negative end reflection; first resonance = firing
  frequency at the tuned rpm, shorter round trip with hot gas) -> silencer (resonant LP + chamber peak) -> radiation HP;
  + intake pulse through the airbox (Helmholtz BP) and snorkel with flow roar; + Rotax EVO exhaust valve (sharper /
  louder above 8,400 rpm, ASSUMPTION); + piston noise near TDC, chain whine (rear axle rev/s x sprocket teeth),
  centrifugal clutch rattle at idle (Rotax only; KT100 is direct drive, KZ has a lever clutch), limiter cut.
  Per-class profiles are ESTIMATES / ASSUMPTIONS (tuned rpm = peak-power rpm, sprocket teeth typical).
* Game inputs added: bLimiter, bClutchLocked, AxleHz (KartAudioComponent).
* Offline renderer: `KartSimTests --sound <KartConfig.json> <out.wav> [laps]` (2.5 s idle, autopilot laps, 3 s
  coast-down) -> Saved/SoundRenders/{rotax,kz2,kt100}_v2.wav.
* Verified: offline spectra (no clipping, idle / WOT / overrun levels), in-game capture (-KartRecordAudio) matches the
  offline render. NOT verified: that it sounds realistic - needs the user's ears.
* Gotcha: game windows launched from the terminal are unfocused and UE mutes them, so -KartRecordAudio captured pure
  silence. Add `-ini:Engine:[Audio]:UnfocusedVolumeMultiplier=1.0`.

## Rotax torque curve from the official datasheet (2026-11-19)
* The user added 2021_Datasheet_Senior_Printpreview.pdf (BRP-Rotax). The Senior power curve was read from the PDF's
  vector paths (PyMuPDF get_drawings, calibrated on the grid lines) -> torqueCurve 5,000-14,000 rpm in
  Rotax125MaxSenior.json. Versus the old estimate: more low-rpm torque (13.1 vs 10.8 Nm @ 5,000) and a collapse past
  12,000 rpm (8.6 vs 13.8 Nm @ 13,000). Table anchors kept (21 Nm @ 9,000; 22 kW @ 11,500).
* Regression reference changes: TestCircuit autopilot **22.682 -> 22.734 s** (in-game = standalone).
* An AI fix was needed: Ai_FasterKartPassesSlowerKartCleanly failed (the 1.4 g kart sat 4-9 m behind for 144 s,
  because the following-gap speed cap stayed active during a pass). While overtaking, the desired gap is now
  0.6 + 0.05 v. 76/76 tests pass.

## Engine sound v3 + frame-drop fix (2026-11-19)
* User: v2 "too buzzy, not accurate", plus FPS drops while driving.
* **FPS:** a CSV profile (`-ExecCmds="r.GPUCsvStatsEnabled 1,csvprofile frames=N"`) showed GPU/LumenSceneUpdate at 6.9 ms
  median, 51 ms max, about half the frame. It was independent of the AI count, the scenery and the kart moving; freezing the
  surface cache removed it. Fix: `r.LumenScene.SurfaceCache.MeshCardsMinSize=100` ([SystemSettings], DefaultEngine.ini):
  meshes under 1 m (barrier tyre instances, kart parts, fence posts) are not captured into Lumen cards (they are still
  lit). Packaged build, 1080p, 6 karts: frame median 15.4 -> 8.1 ms, p99 23.8 -> 10.1 ms. Lighting looks unchanged.
* **Sound v3:** wider pulse (42 deg rise), cycle-to-cycle timing (7 deg) and pulse-width (22 %) variation so the harmonics
  smear into bands instead of razor lines, two-stage silencer ~1 kHz, a broadband exhaust roar band, near-linear output,
  quieter chain. At WOT the spectral centroid drops from 345 to 251 Hz and the harmonic peaks above 800 Hz merge into the roar.
  Clips: Saved/SoundRenders/*_v3.wav. Realism is still the user's call.
