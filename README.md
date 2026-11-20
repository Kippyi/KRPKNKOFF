# KRPKNKOFF – Kart Racing Simulator (Milestone 1: physics prototype)

A kart simulator with a dedicated, engine-independent C++ physics core hosted in Unreal Engine 5.8.

* Architecture: `Docs/Architecture.md`
* Physics equations & assumptions: `Docs/PhysicsModel.md`
* Rotax Max baseline (verified / estimated / assumed values, missing measurements): `Docs/RotaxMaxBaseline.md`
* Plan & status: `Docs/Milestone1Plan.md`, decisions: `Docs/ADR/`
* Visuals & asset pipeline (kart visual rigs, track surface naming): `Docs/Milestone4Visuals.md`
* Track conditions, weather, sessions, live delta: `Docs/Milestone5Conditions.md`
* Menus, setup garage, sound: `Docs/Milestone7MenusSetupSound.md`
* AI opponents, kart contact, multiplayer: `Docs/Milestone6AiMultiplayer.md`
  (`-KartAI=n`, `-KartAIStrength=f`, `-KartPlayerAI`; host `/Engine/Maps/Entry?listen -game`, client `<host IP> -game`)

## Build

Requirements: UE 5.8, Visual Studio 2022 with MSVC 14.44.35211 or later, Windows SDK 10.0.22621.

```bat
:: Physics tests only (no Unreal, ~10 s)
Tests\build_tests.bat
Intermediate\KartSimTests\KartSimTests.exe                           :: unit tests
Intermediate\KartSimTests\KartSimTests.exe --scenarios Saved\SimRuns :: scenario CSVs

:: Unreal modules (add -NoHotReloadFromIDE while an editor without the modules loaded is open)
E:\UE_5.8\Engine\Build\BatchFiles\Build.bat KRPKNKOFFEditor Win64 Development -Project="%CD%\KRPKNKOFF.uproject" -WaitMutex
```

## Run

```bat
E:\UE_5.8\Engine\Binaries\Win64\UnrealEditor.exe "%CD%\KRPKNKOFF.uproject" -game -windowed -ResX=1600 -ResY=900
```

Unattended verification run: add `-KartClass=KZ2_IAMEScreamer.json -KartAutopilot=1.4 -KartQuitAfterLaps=3` (laps are written to the log).

Or restart the editor and press Play: the `KartGameMode` is the global default. It builds the test
track, lighting and player start on any map.

## Controls

| Action | Keyboard | Gamepad |
|---|---|---|
| Throttle / brake | W / S (or arrows) | RT / LT |
| Steer | A / D (or arrows) | Left stick |
| Clutch (manual-clutch karts) | Left Shift | LB |
| Shift up / down (gearbox karts) | E / Q | RB / X |
| Kart class: Rotax / KZ2 / KT100 | 1 / 2 / 3 | |
| Starter (Rotax) | B | B |
| Push start (KT100 direct drive) | hold P | |
| Auto-shift / auto-clutch aids | H / J | |
| Kart setup menu (gearing, tracks, caster, toe, camber, Ackermann, steering ratio) | Tab, then arrows / Enter / Backspace | |
| Autopilot (test/demo driver) | O | |
| Weather preset: Dry / Hot / Overcast / Rain / Drying | N | |
| Track rubber: Green / Rubbered | M | |
| Start next session: Practice / Qualifying / Race (host) | Y | |
| AI strength 70 / 80 / 90 / 100 % (host, next session) | U | |
| Reset kart | R | Menu |
| Camera chase / cockpit | C | Y |
| Physics debug draw | G | |
| Telemetry overlay | T | |
| Record telemetry CSV (`Saved/Telemetry/`) | K | |

Keyboard and gamepad inputs are ramped, and their steering lock is reduced with speed. This
shapes the input only; the physics is identical for every device. Steering wheels and pedals arrive with the DirectInput
layer in Milestone 2.
