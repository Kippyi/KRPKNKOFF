# Architecture Decision Records – Milestone 1

## ADR-0001: Custom kart physics, not Chaos Vehicles
**Status:** accepted. **Context:** the brief forbids Chaos Vehicles as the primary model. Chaos
Wheeled Vehicle also assumes suspension, a differential and a car-style drivetrain. Karts have
none of these: a solid rear axle, no suspension, and chassis flex as the compliance.
**Decision:** a dedicated simulation in `KartSimCore`. Unreal provides only ground queries,
input and rendering.

## ADR-0002: Engine-free simulation core in a single UE module
**Status:** accepted. **Decision:** all physics is plain C++17 (no UObject/FVector) in
`Source/KartSimCore`. Each brief-listed subsystem is its own class and file. They share one UE module
because splitting into eleven DLLs adds boundaries without adding isolation. `KARTSIM_API` maps to
`KARTSIMCORE_API` in UE and to nothing standalone. **Consequence:** the same code is unit-tested
by `Tests/build_tests.bat` in about 10 s with no editor.

## ADR-0003: Brush tyre model with transient slip
**Status:** accepted for M1, to be revisited in M2. **Context:** no kart tyre force/moment data exists to
fit a Magic Formula. **Decision:** a combined-slip brush model (parabolic pressure) with
physically meaningful parameters (μ, slip stiffnesses, load sensitivity, sliding friction
drop), plus relaxation-length transient slip that stays valid at standstill. **Revisit when:** tyre
rig data or logged laps are available. An MF 6.x fit then becomes possible behind the same interface.

## ADR-0004: 240 Hz outer step, 4 × 960 Hz substeps
**Status:** accepted. **Context:** the stiffest modes are wheel spin with tyre slip stiffness (about 54 Hz),
tyre radial stiffness (about 10 Hz) and chassis twist (about 40 Hz). **Decision:** input, FFB and telemetry run at 240 Hz,
and integration runs at 960 Hz. `Kart_TimestepConvergence` checks that 4 versus 16 substeps agree within 1 %.
Cost in UE is about 0.1–0.2 ms per frame.

## ADR-0005: JSONC kart configs with per-value provenance
**Status:** accepted. **Decision:** one JSON-with-comments file per kart class. Every field is
required, so a typo can never silently fall back to a default. Each value carries a
VERIFIED, ESTIMATE or ASSUMPTION tag. The configs are staged as non-UFS files for packaging.

## ADR-0006: Chassis compliance as a front-section torsional DOF
**Status:** accepted for M1. **Decision:** the front section twists about the body x-axis
against the frame stiffness k_t. This puts it in series with the front tyre radial stiffness.
Inside-rear unloading emerges from lateral transfer plus caster/KPI jacking, so lift is never scripted.
**Known limit:** there is a single mode. Rear-section and seat compliance are planned for M2.

## ADR-0007: Runtime-built input, analog rest guard
**Status:** accepted. **Decision:** Enhanced Input actions and mappings are created in C++ at runtime (no
assets). Analog axes are ignored until they have been seen at rest once. That protects against virtual or
stuck controllers; this machine has an XBOX synthetic gamepad bus. Wheel and pedal hardware will
go through a DirectInput layer in M2, together with FFB output.
