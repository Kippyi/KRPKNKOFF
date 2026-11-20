# Milestone 1 – Physics Prototype: Implementation Plan

Goal: one drivable 125cc Rotax Max kart on a basic test track, running custom C++ physics at a
fixed 240 Hz, with a telemetry overlay and a force-feedback architecture.

## Work packages (in build order)

| # | Package | Deliverable | Done when |
|---|---|---|---|
| 1 | Toolchain | VS2022 + MSVC 14.44 + Win SDK 22621 + Clang | UBT builds an empty C++ module |
| 2 | Core math & config | `Math.h`, `Json.h`, `KartConfig.h`, Rotax JSON | Config loads; unit tests pass |
| 3 | Tyre model | Contact geometry, transient slip, brush forces, M_z, thermal, wear | Tyre unit tests pass (slope, saturation, friction circle, convergence) |
| 4 | Powertrain | Engine map, centrifugal clutch stick/slip, chain, solid axle | Clutch/engine tests pass |
| 5 | Brakes | Coulomb-clamped rear brake + thermal | Lock test passes |
| 6 | Steering | Ackermann + kingpin kinematics + kingpin torque | Jacking direction test passes |
| 7 | Chassis & orchestration | Rigid body, twist DOF, `KartSimulation::Step` | Static load test, coast-down, timestep convergence |
| 8 | Scenario harness | Standalone runner: launch, top speed, braking, steady circle | CSV output; physically plausible metrics reported |
| 9 | UE integration | Pawn with fixed-step accumulator, interpolation, line-trace ground query | Kart sits on track in PIE, no jitter |
| 10 | Input | Enhanced Input built at runtime (keyboard, gamepad, generic wheel axes) | Drivable in PIE |
| 11 | Test track | Procedural circuit: asphalt, kerbs, grass run-off | Lap is possible |
| 12 | Telemetry overlay & debug draw | HUD text, bars, per-wheel loads/slip/temp, force vectors, CoG | Visible in PIE; CSV export toggle |
| 13 | FFB architecture | `ForceFeedbackProcessor` (core) + `IKartFFBDevice` (UE, null device) | Torque signal logged in telemetry |

## Status (2026-11-17)

| # | Status | Evidence |
|---|---|---|
| 1 | Done | MSVC 14.44.35229 + SDK 22621 + Clang 19.1.5 installed; UBT builds both modules |
| 2–8 | Done | 14/14 automated tests pass (`Tests/build_tests.bat`, then `Intermediate\KartSimTests\KartSimTests.exe`); 6 scenarios write CSVs |
| 9 | Done | Runs in a `-game` process: 240 Hz measured, 0.1–0.2 ms physics per frame, interpolated rendering |
| 10 | Done (keyboard + XInput gamepad) | Wheel/pedal devices pending the DirectInput layer (M2) |
| 11 | Done | Procedural ~520 m circuit, kerbs as real geometry, grass run-off |
| 12 | Done | HUD panel, per-wheel grid, g-g plot, debug draw (G), CSV recording (K) |
| 13 | Done (architecture) | Column torque → `ForceFeedbackProcessor` → `IKartFFBDevice`; only the null device exists, so no hardware output yet |

**Open M1 issues, carried into M2:**
* Peak lateral grip (about 1.1 g before solid-axle binding causes oversteer) and the inside-rear lift threshold need
  validation against logged data. Grip has not been tuned to a guessed target.
* Kerb response seems violent with the assumed tyre damping. It needs comparison against real kerb strikes.
* The standing-start launch is dominated by the estimated low-rpm torque and the assumed clutch engagement.
* Not yet verified in PIE inside the editor (only in `-game`), because the editor has to restart to load
  the new C++ modules.

Not in M1 (deliberately): DirectInput constant-force output, chassis multi-mode flex, camber
thrust, KZ/100cc classes, art assets, audio, sessions, AI and multiplayer.

## Risks

* **Unvalidated tyre parameters.** The kart will be drivable and qualitatively right, but grip
  levels are estimates until tyre data or logged laps exist.
* **UE 5.8 API drift.** The game-module code is kept thin, so fixing compile errors stays local.
