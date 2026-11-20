# Replays, ghost lap and lap analysis (2026-11-18)

Visual / analysis features only: the physics is untouched (regression 22.682 s, 72/72 core tests).

## Core (engine-free, `KartSim/Replay.h`, `KartSim/LapDelta.h`)
* `ReplayFrame` (72 bytes): pose (position, quaternion), velocity, steer angles, wheel spin, rpm, throttle,
  brake, gear, engine running. `MakeReplayFrame` / `LerpFrame` (linear + quaternion nlerp) /
  `ApplyReplayFrame` (poses a sim with `SetKinematicPose`, the multiplayer-proxy path).
* `ReplayTrack` per kart (name, class file, colour, frames) with interpolated `Sample(t)`.
* `ReplayRecording`: circuit, info, lap time (ghosts), tracks; binary `.krpl` save/load (versioned; a
  truncated or foreign file is rejected without touching the target), `Trim(seconds)`.
* `GhostKeeper`: records the current lap at 30 Hz from the lap timer and keeps the fastest VALID WHOLE lap.
* `LapDelta` traces now carry speed, throttle, brake, steering, rpm, gear, lateral/longitudinal g and the
  CoG position per 2 m station, and keep a history of every whole lap (max 200). `DeltaTrace(lap, ref)`.

Measured (`Tests/ReplayTests.cpp`): 30 Hz replay vs the real 240 Hz path: max 2.0 mm error; ~140 KB per
kart-minute; posed kart = source to 2e-7 m (frame twist is not replayed: ~5 mm at a rear wheel); ghost
= best lap exactly, its start and end 0.4 m apart (same line, one lap apart).

## In game
* **Recording:** the race world records every kart at 30 Hz, always (last 30 minutes kept). A new track
  layout starts a new recording.
* **F5 replay** (single player): the race freezes, the live karts are hidden and display-only "puppet"
  karts are posed from the recording, so the live simulation is never touched and saved files play the same
  way. Starts 60 s before the end. Space pause, Left/Right -/+5 s (Shift 30 s), Up/Down speed
  (0.1x-4x), V next kart, X camera: chase / cockpit / TV (trackside cameras every ~60 m on the outside of
  the bend, 4 m up, zooming to keep the kart about 6 m tall in the picture). F5 back to driving. Replay
  engine sound comes from the recorded rpm/throttle; the hidden live karts are muted. Force feedback stops.
* **F7** saves `Saved/Replays/<circuit>_<date-time>.krpl`. `-KartReplay=<file>` (relative = Saved/Replays)
  opens one at start; `-KartReplayAt=<s>`, `-KartReplayCamera=0|1|2`, `-KartReplayFocus=<i>` for tests.
  `-KartSaveReplayOnExit` writes `Saved/Replays/LastSession.krpl`; `-KartQuitAfter=<s>` exits.
* **Ghost (F6 on/off):** your fastest valid lap per circuit + class, saved to
  `Saved/Ghosts/<circuit>_<classId>.krpl` and loaded at start / class switch / layout change. Shown as a
  cyan, silent puppet running on your lap clock; hidden within 2.5 m of you (it would sit inside your kart)
  and after it finishes its lap. It is set by any valid lap, autopilot included.
* **L lap analysis:** full-screen viewer. Six traces vs distance (speed; throttle green / brake red;
  steering; lateral g; rpm + gear; time delta) for the selected lap (yellow) against the reference (grey:
  best lap, or any lap with PgUp/PgDn), values at the cursor, a track map coloured green where the lap
  gained and red where it lost time against the reference, and the lap list. Driving inputs are suspended
  (brake held) while it is open, like the setup page. `-KartAnalysisAfterLaps=<n>` opens it (tests).

## Verified in the packaged build
* Session with 3 AI, 5 autopilot laps: ghost saved on new bests (23.244 then 23.241 s), replay saved
  on exit (145 s, 4 karts, 1.3 MB).
* `-KartReplay=LastSession.krpl -KartReplayCamera=2`: 4 puppets posed, TV camera cuts and zooms, overlay.
* Slower laps (1.1 g, 24.64 s) against the 23.24 s ghost: ghost visible ahead.
* Lap analysis opened after lap 2: traces, cursor values, map and lap list draw.
* Regression with all of this active: 22.682 / 22.682 / 22.680 s (unchanged).

## Limitations
* Replay is single player (in multiplayer F5 says so): pausing a shared race needs a design.
* The ghost is opaque (no translucent material yet) and is not shown in multiplayer races' other karts.
* Not replayed: frame twist, tyre smoke/sound of scrub, weather/water changes (the scenery stays as now).
* Replays of another track show a warning: puppets are posed at the recorded positions regardless.
* No replay browser in the menu yet (command line / F7 files only).
