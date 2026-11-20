# Milestone 4 – Visual Development

**Scope note:** the brief asks for photorealistic karts, drivers and environments. Photoreal assets come from
artists or scanned/licensed content (Blender, Fab/Megascans, packs) and can't be generated in code. With the
user's agreement, this milestone delivers (1) an **asset pipeline** so real assets drop in without code changes
and (2) a much better **procedural placeholder** set.
**Nothing visual is read by the physics.** In-engine autopilot laps are identical to the millisecond before and
after this milestone (Rotax 22.683 s, KZ2 22.214 s, KT100 24.012 s). The 37 standalone tests still pass.

## Materials
`Content/KartVisuals/M_KartSurface` is generated headlessly by `Tools/CreateKartMaterials.py`:
`UnrealEditor-Cmd.exe KRPKNKOFF.uproject -run=pythonscript -script="Tools/CreateKartMaterials.py"`.
It is a two-sided master with two-scale world-space noise (patches plus aggregate), noise-driven roughness,
metallic and optional vertex colour. Runtime instances provide asphalt, grass, kerbs, paint, rubber, alloy,
plastic, the driver suit and the helmet.

## Procedural kart (`KartProceduralVisuals.*`, `AKartVehiclePawn::BuildVisuals`)
* Tubular frame from **config geometry**: wheelbase, tracks, kingpin position and caster angle. The frame
  follows setup changes.
* Main rails, cross members, kingpin brackets, side bars, front and rear bumpers, seat stays, steering column,
  pedals.
* Bodywork (nose, front panel, sidepods, rear bumper), seat, floor tray, radiator, engine and exhaust.
* Seated driver (torso, arms to the wheel, legs to the pedals) and a separate helmet (hidden in cockpit view).
* **Tyres and rims revolved from a tyre cross-section**, sized from the tyre config (5 in rims). Right-hand wheels
  are mirrored so the rims face outboard. A marker sector makes wheel spin visible.
* The steering wheel turns with the column input; the rear axle spans the hubs (follows frame twist).
* Cockpit camera: eye point 0.92 m, pitched 24° down, FOV 90°, so the wheel rim and front panel are in view.

## Track and environment
Noise-textured asphalt and grass, kerbs in red/white, **tyre-wall barriers** on corner outsides (instanced),
volumetric clouds, and an unbound post-process volume (auto-exposure bias −1.3: auto exposure over-brightened
the physically dark asphalt). **The barriers are visual only:** kart-barrier collisions are not simulated (they
sit beyond 7 m of grass).

## Asset pipeline
### Kart visual rig (`Content/KartData/Visuals/*.json`)
Selected by `"visualFile"` in a kart config or by `-KartVisual=<file>`. See `ExampleEngineShapes.json`:
* `hide`: procedural placeholders to hide: `frame`, `bodywork`, `driver`, `steeringWheel`, `axle`, `wheels`.
* `parts`: `mesh` (static mesh asset path) and `attach` = `chassis` | `driver` (location in the **kart frame**:
  metres forward of the rear axle, y left, z up from the ground) or `wheelFL/FR/RL/RR` | `steeringWheel`
  (local offset in metres; the part spins/steers with its parent). Also `rotationDeg` [pitch, yaw, roll] and `scale`.
* Verified: the example rig loads 5 parts and hides the bodywork (screenshot check).

### Track surfaces for imported art
Resolution order in `FKartUnrealTrackQuery`:
1. component tag `KartSurface.<Surface>`
2. **static mesh asset name containing `KartSurface_<Surface>`** (e.g. `SM_Circuit_KartSurface_Grass_02`)
3. asphalt

Surfaces: Asphalt, RubberedAsphalt, DirtyAsphalt, WetAsphalt, Kerb, Grass, Gravel.
Imported track meshes need **complex collision** (the ground query traces complex).
**Self-test:** `-KartSurfaceSelfTest` probes the track centre, run-off, kerb and a test pad built from
`/Game/KartVisuals/Test/SM_Pad_KartSurface_Gravel` (created by `Tools/CreateSurfaceTestAsset.py`). Result:
**PASS** (Asphalt/Grass/Kerb/Gravel).

### Lessons recorded
* UE 5.8 creates physics bodies **asynchronously**: a trace in the same frame as a component's registration
  misses it. The self-test probes about 1.5 s later.
* The engine cube has only simple collision. The test pad asset is set to "use simple as complex".
* A physical-material convention was tried first; its failures traced back to the async timing above, so it is
  unverified rather than impossible. The naming convention is used because it is verified and needs no import
  settings.

## Still placeholder / not done
* No real kart, driver or track art (needs assets). No driver animation (needs a skeletal mesh and rig).
* No barrier collision physics, no trackside detail beyond tyre walls, no time-of-day or weather visuals.

## Imported kart: "Bog kart" (2026-11-18)
* Source: `kips bog bog kart.fbx` (the user's Blender kitbash, 61 objects, ~166k triangles). Textures are **not embedded**:
  the paths point to another PC, so materials are reduced to classes and coloured at runtime.
* `Tools/SplitBogKart.py` (headless Blender) measures hubs and the steering column, groups the parts (body / 4 wheels /
  steering wheel) and splits them by material class, re-expressed in the kart frame, into `Art/BogKart/*.fbx`. Any
  offensive object or material names in the source are dropped.
* `Tools/ImportBogKart.py` (UnrealEditor-Cmd pythonscript) imports them to `/Game/KartVisuals/BogKart`;
  `Tools/CheckBogKart.py` logs their bounds (axis check: X fwd, Y right, Z up).
* Rig `Content/KartData/Visuals/BogKart.json` is the default `visualFile` of every class. New rig keys:
  `kartAligned` (wheel / steering parts given in the kart frame), `fitRadius` (scale the wheel to the configured tyre
  radius), `material` (body | chassis | chrome | alloy | anodized | graphite | plastic | rubber | display; `body` = the
  kart's livery colour), `steeringColumn` {hub, axis} (moves the steering wheel and the driver's grips).
* Measured from the model: wheelbase 1.035 m, front track 1.116 m, rear track 1.174 m (the physics uses 1.22 m, so the
  rear wheels sit about 2 cm wider than the model's hubs), tyre radii 0.128 / 0.131 m (fitted to the config).
* Verified: 22 parts load, showroom shots look right (tyres on the ground, hands on the rim), and the autopilot
  regression is unchanged at 22.682 s.
* Not done: textures/decals, steered spindles (they are static on the body), a spinning rear disc (static).
* Licensing: the source names show parts from third-party KRP mod packs (R2S Bodywork Pack, OTK, Sodi, Mychron
  meshes). Fine for personal use; get permission before distributing builds that contain them.

## Realism pass 1 (2026-11-18)
* **Kart paint:** `/Game/KartVisuals/M_KartPaint` (Tools/CreateKartPaint.py): clear-coat shading with an object-space
  livery (no UVs or textures): primary = kart colour, pearl lower band, black pinstripe and swoosh. Used for the
  `body` material class. Clear-coat inputs are not reachable from Python, so the engine defaults are used (1.0 / 0.1).
* **Ground:** CC0 photo-scanned PBR sets from ambientCG (Asphalt033, Grass005, Concrete034; `Art/Textures/CC0`,
  licence file included) imported by Tools/CreateGroundMaterials.py into `M_KartGround` (world-projected top-down,
  anti-tiling macro variation, `Tint`, `Wetness`) + MI_Ground_Asphalt/Grass/Concrete. Kerbs and lines = tinted concrete.
  Rain drives `Wetness` (darker albedo, roughness toward a 0.08 water film; reflections via Lumen).
* **Lighting / post:** sun source angle 0.53 deg + contact shadows; Lumen final gather / reflection / scene detail 2;
  bloom 0.45, vignette 0.28, grain 0.04, motion blur 0.35, AO 0.6, slight fringe. Exposure unchanged.
* Verified: dry and rain screenshots; autopilot regression unchanged at 22.682 s.
* Next candidates: trackside scenery (trees, fences, pit building, grandstands), rubbered racing line and skid marks
  in the ground material, tyre sidewall detail, a proper driver model (suit, helmet), time of day.

## Realism pass 2: trackside scenery (2026-11-18)
* `KartScenery.cpp` (`AKartTestTrackActor::BuildScenery`, rebuilt with every layout; `-KartNoScenery` skips it):
  deterministic placement (fixed seed, identical on every multiplayer machine), HISM instances, **no collision**.
  * Pit / paddock on the start straight (side with more space): Jersey-barrier pit wall, 36 m pit building with roller
    doors and roof overhang, 6-row covered grandstand, floodlights, street lamps, benches, barrels, utility boxes.
  * Catch fence both sides 11 m out (skipped where too close to another part of the track and in the pit zone):
    Poly Haven posts + galvanised rails + tension cables (engine cylinders).
  * 420 trees (jacaranda, fir saplings, small tree) beyond the fence; ~1000 shrubs in clusters behind it.
* Assets: CC0 Poly Haven glTF (Tools/FetchPolyHaven.py -> Art/Models/PolyHaven, licence file) imported by
  Tools/ImportScenery.py into /Game/Scenery (added to DirectoriesToAlwaysCook).
* Fixes needed for Poly Haven glTF (in order found):
  1. Leaves / wire import opaque, and the glTF has JPG colour maps with **no alpha**: the separate alpha maps are
     downloaded and wired by Tools/FixSceneryMasked.py (masked two-sided master M_SceneryMasked, OpenGL normals, ARM).
  2. Trees rendered bare: leaves are millions of tiny real triangles, eroded by Nanite simplification. Fixed with
     Nanite **Preserve Area** (Tools/FixSceneryFoliage.py; set via set_editor_property + save_loaded_asset(False) -
     attribute assignment + save_asset did NOT persist, and StaticMeshEditorSubsystem is None in commandlets).
  3. The chain-link panel's wire quad rendered as solid black slabs in the packaged game whatever was tried (mask,
     distance fade, no shadows, linear alpha): the panel mesh was dropped for rails + cables.
* Lesson: `-KartShots=<t>` takes capture TIMES; shots at 1 s show unstreamed Nanite / textures.
* Verified: screenshots (pit straight, infield, wide, trackside); autopilot regression 22.682 s.
* Not measured: frame rate with full scenery (420 Nanite trees + 1000 shrubs) - check on the user's PC.

## Realism pass 3: photographed sky, physical lighting, driver proportions (2026-11-18)
* Sky: CC0 Poly Haven pure-sky HDRIs (Art/Sky: kloofendal_48d_partly_cloudy = dry, overcast_soil = rain), converted
  to EXR by Blender, M_KartSky (Tools/CreateSky.py): IsSky unlit lat-long lookup on a 5 km sphere (tag KartSky),
  photo sun disc clamped, horizon row repeated below the horizon, Overcast blend driven by cloud cover
  (AKartConditionsActor). SkyAtmosphere / volumetric clouds removed (fallback to atmosphere if the material is missing).
* Sun matched to the photo (pitch -47.9, yaw 34.2), 90 klx; sky ~6000 nits per HDR unit; auto exposure limited to
  EV100 12-14.6; height fog in physical luminance (3600 x colour) - the old 0-1 colour rendered as a black horizon band.
* Ground tints retuned for daylight (grass 0.50/0.52/0.40, asphalt 0.46/0.47/0.52).
* Driver (procedural, user's choice over MetaHuman): 50th-percentile proportions - spine 52 cm, shoulders ~44 cm,
  slimmer limbs, suit-coloured knees, grips on the imported wheel rim (15.5 cm).
* Verified: dry / rain / wide screenshots; autopilot regression 22.682 s.
* Open: shaded faces (pit building back wall) read very dark - check whether the sky light's real-time capture
  sees the sky sphere; the driver is still primitive-based (tubes), not photoreal.

## R2S livery skin (2026-11-18)
* User-supplied TGAs (project root) wired by Tools/SplitBogKart.py (UV-preserving skin* / sw* classes),
  Tools/CreateKartSkin.py (T_Skin_* + M_KartSkin, clear coat) and the rig (regenerated from BogKartLayout.json).
* Human karts get the livery; AI karts keep their own colour paint (field stays distinguishable).
* Verified: front / side / rear / chase screenshots, UVs correct. Not supplied: KG508 / M10 front panel textures,
  tank, seat textures (those parts stay painted / plain).

## Realism pass 4: working sky light / GI, track wear, turf (2026-11-19)
* **Bug found:** the sky light contributed nothing (ShowFlag.SkyLighting 0 vs 1 gave identical frames) and Lumen
  showed no bounce, so every shaded face was black (pit building, kart shadow side).
  * `r.Lumen.HardwareRayTracing=True` (DefaultEngine.ini): Lumen now sees the procedural track/grass meshes (they have
    no distance fields, so software Lumen never bounced light off the ground).
  * Sky light spawned deferred with Movable + real-time capture set before registration (KartGameMode.cpp).
  * Lumen + sky capture still start stale with the runtime-built scene: one `FGlobalComponentRecreateRenderStateContext`
    1.5 s after load fixes it (found by toggling the HWRT cvar at runtime). `-KartNoLightRefresh` skips it for A/B.
* **Asphalt** `M_KartAsphalt` / `MI_Track_Asphalt` (Tools/CreateTrackMaterials.py): asphalt vertex colours carry
  R = racing-line position (the AI's minimum-curvature line, sign matched to curvature), G = half width / 10,
  B = corner / braking-zone intensity -> per-pixel rubbered line (streaky, slightly glossier), tyre marks in braking
  zones, dusty ragged edges, off-line grime. Params: LineWidth, RubberAmount, RubberDarken, TyreMarks, EdgeDust, BaseTint.
* **Grass** `M_KartGrass` / `MI_Track_Grass`: Grass005 at two scales/rotations, fBm patches (dry / lush, brightness),
  faint mowing stripes, turf tint. Both react to Wetness. The old MI_Ground_* remain as the fallback.
* Verified: screenshots (showroom with / without the refresh, chase cam on TestCircuit). Visual only.
  NOT re-verified: the autopilot lap-time regression, frame rate with Lumen HWRT.
* Pass 4b (same day): **tyre walls** = real used road tyres (Tools/MakeBarrierTyre.py -> Art/Track/BarrierTyre.fbx ->
  /Game/KartVisuals/Track/SM_BarrierTyre, 0.62 x 0.20 m, open centre; ESTIMATE size), 3 high, two staggered rows,
  random yaw / jitter (fixed seed), alternating white / red painted top tyres on the front row; M_BarrierTyre reads
  per-instance custom data (paint, random fade/dust). **Verge**: the shoulder strip uses MI_Track_Verge (M_KartGrass
  with VergeWear = 1): a ragged, patchy dirt band at the track edge. **Kerbs**: UVs now in metres (x from the track
  edge, y along); M_KartKerb = paint over concrete, rubber smudges on the inner ~0.35 m, chipped paint, glossy when wet.
  **Grading**: WhiteTemp 6150, contrast / saturation 1.06 / 1.07, FilmToe 0.58.
* Verified: chase-cam screenshots; **autopilot regression 22.682 s** [autopilot] (TestCircuit, unchanged); frame time
  at 1080p with 6 karts on an RTX 4070 SUPER: median 15.4 ms / p95 21.1 ms with Lumen HWRT, 16.0 / 21.8 ms without
  (HWRT costs nothing measurable). Rain look with the new materials not checked on screen.
