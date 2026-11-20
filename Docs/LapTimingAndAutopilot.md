# Lap Timing and Autopilot

## Lap timing (`KartSim/LapTiming.h`)
* Start/finish plus 2 sector lines from the track actor (`AKartTestTrackActor::GetTimingLines`), spanning the
  track, kerbs and 2 m of run-off.
* Forward-direction crossing of the CoG path only. The crossing time is **interpolated within the 240 Hz step**
  (the analytic test hits a 2.5 s sector to 1e-9 s).
* **Track limits:** a lap is invalid if all four wheels are on grass or gravel at the same time (kerbs are track).
  Invalid laps are excluded from best lap and best sectors.
* A reset aborts the lap in progress; switching class clears the history.
* HUD: current lap, completed sectors (green = session best, yellow = slower), last and best. Each completed lap
  is written to the log.

## Autopilot (`KartSim/Autopilot.h`)
The same modulating test driver as the automated drivability tests: pure pursuit with yaw-rate feedback, a
speed profile from a target lateral g, and braking held below rear lock, with all driver aids (clutch, shifting,
push start). In game: key **O** toggles it, `-KartAutopilot=<g>` enables it at launch, and `-KartQuitAfterLaps=<n>`
exits after n laps (unattended runs). It is a verification and demo tool, **not** the Milestone 6 racing AI.

## End-to-end verification (Unreal track vs standalone physics)
`UnrealEditor.exe KRPKNKOFF.uproject -game -KartClass=<file> -KartAutopilot=1.4 -KartQuitAfterLaps=3`

| Class | In-engine lap (3 laps) | Standalone test | Difference |
|---|---|---|---|
| Rotax Max | 22.683 / 22.682 / 22.683 s | 22.68 s | < 3 ms |
| KZ2 | 22.214 / 22.214 / 22.213 s | 22.21 s | < 4 ms |
| KT100 | 24.012 / 24.012 / 24.012 s | 24.01 s | < 2 ms |

This confirms the line-trace ground queries, coordinate conversion, fixed-step loop and tyre enveloping on real
mesh geometry against the analytic test plane, and shows the simulation is repeatable lap to lap.
**Not** claimed: cross-machine or network determinism, which hasn't been tested.
