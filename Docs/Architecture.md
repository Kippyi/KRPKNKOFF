# KRPKNKOFF – Software Architecture

Status: Milestone 1 (physics prototype). Last revised 2026-11-17.

## 1. Environment (inspected 2026-11-17)

| Item | Found | Notes |
|---|---|---|
| Unreal Engine | 5.8.4 (`E:\UE_5.8`, launcher build) | Project `KRPKNKOFF.uproject` is associated with 5.8 |
| Visual Studio | 2022 Community 17.14, 2026 Community 18.11 | **Initially had no MSVC / Windows SDK.** VS2022 C++ workloads, MSVC 14.44, Win11 SDK 22621 and Clang were installed for this project |
| UE toolchain requirements (from `Engine/Config/Windows/Windows_SDK.json`) | MSVC 14.44.35211 or later (VS2022) or 14.50.35723 or later (VS2026), Windows SDK 10.0.22621 | Versions 14.44.0–14.44.35210 are banned by UE |
| Hardware | i7-14700, 32 GB, RTX 4070 SUPER | |
| Workflow note | Claude Code runs in the editor's Terminal plugin | Builds use `-NoHotReloadFromIDE` (safe while the open editor has no project DLLs loaded). Runtime tests use a separate `-game` process |

## 2. Guiding principles

1. **The simulation is a plain C++ library with no Unreal dependency.** `Source/KartSimCore` contains
   no `UObject`, no `FVector` and no engine headers outside the module boilerplate file.
   It compiles both as a UE module and as a standalone executable (`Tests/`), so physics can be
   unit-tested, batch-run and profiled without the editor.
2. **Unreal is the host.** It drives the fixed-step clock and provides ground queries (line traces),
   input devices, rendering, audio and the HUD. It reads the sim state and never writes it,
   except through the defined input API.
3. **All vehicle behaviour comes from data.** Each kart class is a JSON file
   (`Content/KartData/Karts/*.json`). The code has no class-specific branches, only
   generic component types (centrifugal clutch, manual clutch, direct drive; N-speed gearbox and so on).
4. **Every parameter carries provenance.** Config comments tag each value `VERIFIED` (with source),
   `ESTIMATE` (engineering derivation documented) or `ASSUMPTION` (placeholder awaiting data).
5. **SI units and the ISO 8855 axis system inside the sim:** x forward, y left, z up, in metres,
   kilograms, seconds and radians. The conversion to UE (cm, left-handed, y right) happens only in
   `KartUnrealConversions.h`.

## 3. Module layout

```
KRPKNKOFF/
├─ KRPKNKOFF.uproject
├─ Source/
│  ├─ KRPKNKOFF.Target.cs / KRPKNKOFFEditor.Target.cs
│  ├─ KartSimCore/                 UE module, pure C++ (no UObjects)
│  │  ├─ Public/KartSim/
│  │  │  ├─ Math.h                 Vec3, Quat (double precision)
│  │  │  ├─ Json.h                 minimal JSON/JSONC reader for configs
│  │  │  ├─ KartConfig.h           all parameter structs + loader
│  │  │  ├─ TrackSurface.h         ITrackQuery, surface types, analytic test tracks
│  │  │  ├─ TyreModel.h            brush tyre model, transient slip, thermal, wear
│  │  │  ├─ EngineModel.h          torque map, inertia, idle control, limiter
│  │  │  ├─ TransmissionModel.h    clutch (centrifugal/manual/direct), gearbox, chain
│  │  │  ├─ BrakeModel.h           brake torque, temperature, fade
│  │  │  ├─ SteeringModel.h        Ackermann, caster/KPI kinematics, kingpin torque
│  │  │  ├─ ChassisModel.h         rigid body + torsional twist DOF
│  │  │  ├─ KartPhysicsCore.h      KartSimulation – orchestrates one kart
│  │  │  ├─ Telemetry.h            telemetry frame, ring buffer, CSV export
│  │  │  └─ ForceFeedback.h        steering torque → FFB command processing
│  │  └─ Private/…                 implementations
│  └─ KRPKNKOFF/                   UE game module
│     ├─ KartVehiclePawn           owns KartSimulation, fixed-step loop, interpolation
│     ├─ KartUnrealTrackQuery      ITrackQuery via line traces + surface tags
│     ├─ KartInputComponent        Enhanced Input (runtime-built), device abstraction
│     ├─ KartForceFeedbackOutput   IFFBDevice interface (null/log device in M1)
│     ├─ KartTestTrackActor        procedural test circuit (asphalt/kerb/grass)
│     ├─ KartHUD                   telemetry overlay + physics debug drawing
│     └─ KartGameMode
├─ Content/KartData/Karts/         kart class JSON configs
├─ Tests/                          standalone test runner + scenario harness (no UE)
└─ Docs/                           architecture, physics model, baselines, ADRs
```

### Why one core module rather than eleven UE modules

The brief lists KartPhysicsCore, KartTyreModel, KartChassisModel and others. Each of these is a
**separate class with its own header/source and no cyclic dependencies**, but they share one UE
module. Splitting them into eleven UE modules would multiply Build.cs files and DLL boundaries
without adding any isolation, because they are already engine-free plain C++. The UE-facing parts
(input, FFB device and telemetry display) live in the game module. See ADR-0002.

## 4. Runtime data flow

```
            ┌─────────────── UE game thread, Tick (variable dt) ───────────────┐
 Devices ──►│ KartInputComponent ─► KartInputs (raw, normalised)                 │
            │ accumulator += dt                                                  │
            │ while accumulator ≥ 1/240:                                         │
            │     KartSimulation::Step(1/240, inputs, trackQuery)                │
            │         └─ N substeps (default 4 → 960 Hz internal)                │
            │     telemetry.Push(frame); ffb.Update(frame)                       │
            │ alpha = accumulator / (1/240)                                      │
            │ render pose = interpolate(prevState, currState, alpha)             │
            └────────────────────────────────────────────────────────────────────┘
```

* The **outer tick is 240 Hz**, and it is the rate for input sampling, telemetry and FFB.
* The **inner integration runs at 4 substeps (960 Hz)**, because the stiffest modes
  (front wheel spin coupled to tyre longitudinal stiffness, about 40 Hz, and the tyre vertical and chassis twist modes)
  need ω·dt ≲ 0.3 for semi-implicit Euler. This is measured by tests, not assumed (see `Tests/`).
* There is a frame-time cap of 0.1 s. If the game hitches, the sim slows down instead of spiralling.
* No multiplayer determinism is claimed. The sim is single-threaded and deterministic for a
  given input sequence on the same binary. That claim has not been validated across machines or compilers.

## 5. Key class responsibilities

| Class | Responsibility | Depends on |
|---|---|---|
| `KartSimulation` | Owns state; per substep: kinematics → contacts → tyre forces → drivetrain → integrate | all models |
| `ChassisModel` | 6-DOF rigid body (CoG frame) + front-section torsional twist DOF; wheel hardpoints | Math |
| `SteeringModel` | Column → road-wheel angles (Ackermann); wheel pose from kingpin rotation (caster, KPI, spindle offset); kingpin & column torque | Math |
| `TyreModel` | Contact geometry (disc vs plane), vertical force, transient slip, combined brush forces, aligning moment, temperature, wear | TrackSurface |
| `EngineModel` | WOT torque map, closed-throttle drag, throttle lag, idle control, rev limiter | Math |
| `TransmissionModel` | Clutch capacity & stick/slip, gearbox ratios & shifting, chain ratio/efficiency | Engine |
| `BrakeModel` | Pedal → torque, Coulomb lock handling, disc temperature & fade | – |
| `TrackSurface` | Surface catalogue (asphalt, rubbered, dirty, wet, grass, kerb) and `ITrackQuery` | Math |
| `Telemetry` | Fixed-layout frame, ring buffer, CSV writer | – |
| `ForceFeedback` | Kingpin torque → device command: gain, smoothing, slew limit, clipping metering | – |

## 6. Architecture decisions (ADR index)

* ADR-0001 – Custom physics, not Chaos Vehicles (required by brief; Chaos assumes suspension/differential).
* ADR-0002 – Engine-free simulation core in a single UE module.
* ADR-0003 – Brush tyre model with transient slip (see `PhysicsModel.md` §3).
* ADR-0004 – 240 Hz outer / 960 Hz inner fixed step.
* ADR-0005 – JSONC kart configs with per-value provenance tags.
* ADR-0006 – Chassis compliance as a front-section torsional DOF in series with tyre radial stiffness.

These are recorded in `Docs/ADR/`.
