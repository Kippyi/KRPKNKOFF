KRPKNKOFF - kart racing simulator (test build)
==============================================

INSTALL
1. Unzip the whole folder anywhere (keep the folder structure).
2. Run KRPKNKOFF.exe.
   - Windows may show "Windows protected your PC" (the game is not code-signed):
     click "More info" -> "Run anyway".
   - If it complains about missing DLLs (VCRUNTIME / MSVCP), run
     Engine\Extras\Redist\en-us\vc_redist.x64.exe once, then start the game again.
3. Requirements: Windows 10/11 64-bit, a DirectX 12 graphics card, ~3 GB free disk space.

GRAPHICS / LOW FRAME RATE (laptops)
  The first start runs a short hardware test and picks a quality preset (Low / Medium / High / Epic).
  Change it in Esc > Settings > GRAPHICS: Quality, Render scale, Frame limit, VSync, Display, Show FPS.
  - Low / Medium switch off Lumen lighting and ray tracing (several times faster on the GPU).
    High / Epic need a ray-tracing graphics card (RTX / RX 6000 or newer).
  - Still slow: lower the Render scale (e.g. 50-60 %).
  - Laptops with two GPUs: Windows often runs games on the weak built-in one. Windows Settings >
    System > Display > Graphics > add KRPKNKOFF.exe (KRPKNKOFF\Binaries\Win64) > High performance.
    Plug the charger in (battery mode slows the GPU a lot).

CONTROLS (keyboard / Xbox pad)
  Throttle / brake   W / S        Steer            A / D
  Clutch             Left Shift   Shift up / down  E / Q (KZ gearbox)
  Starter / push     B / hold P   Reset + repair   R
  Camera             C            Telemetry        T
  Cockpit view       Page Up/Down tilt, Home/End field of view (saved)
  Quick setup        Tab          Menu             Esc
  Next session (host)  Y          AI strength      U        Weather N / rubber M

TRACKS
  Choose the track in the menu (Practice / Race weekend / Multiplayer): Test Circuit (520 m),
  Riverside Kart Park (1014 m) or Crestwood Raceway (951 m: banked sweepers, off-camber hairpin,
  a chicane with high kerbs you can jump, rumble strips).

TYRES AND RAIN
  Menu -> Garage (or Tab): "Tyres" 0 = slicks, 1 = wets (grooved). Wets are much better on a wet
  track, but on a dry line they overheat and wear fast. Your tyres wear as you drive and the used
  set is kept between sessions: Garage -> "Fit new tyres" for a fresh set. The HUD (bottom left)
  shows each tyre's temperature (blue cold, green good, red hot) and the tread left in mm.
  Weather "Shower" (key N or the menu): rain arrives a few minutes in and stops later.

ENGINE TUNING, DAMAGE AND THE MYCHRON
  Garage (or Tab): "Main jet" (Rotax / KZ2) or "Carb needle" turns (KT100), and "Radiator cover".
  On track: [ = one size leaner, ] = one size richer (saved). The Mychron shows LAMBDA: green
  0.84-0.89 is best power, yellow/red = lean (hot, seizure risk), blue = rich (slow).
  The garage shows today's air and the jetting-chart jet. Cold, dense air needs a bigger jet; a jet
  that is too small (lean) is a touch faster but runs hot and can SEIZE the engine. On cold days tape
  the radiator (cover) until the water sits in its window (Rotax 50-55 C); too much and it boils.
  Weather "Cold" (6 C) makes this matter.
  Contact and big kerb hits can bend the steering (the kart pulls), bend the rear axle (slower),
  throw the chain, hole the radiator (it overheats) or push the front fairing back (you see it move).
  Wheels touch for real: a front wheel on someone's rear wheel can launch you, and karts can roll over.
  R = back to the pits: resets the kart AND repairs it. Turn damage / seizure off on the Settings page.
  The Mychron on the steering wheel (cockpit view; bottom right on the HUD in the chase view) shows
  rpm and shift lights, lap / best / delta, water (or CHT) and EGT, and warnings.

REPLAYS, GHOST AND LAP ANALYSIS
  F5   watch a replay of the session (the race pauses; single player). In replay:
       Space pause, Left/Right -/+5 s (Shift: 30 s), Up/Down speed, V next kart,
       X camera (chase / cockpit / trackside TV), F5 back to driving.
  F7   save the replay (KRPKNKOFF\Saved\Replays\) - send it with feedback.
  F6   ghost on/off: your fastest valid lap on this track with this kart (kept between runs).
  L    lap analysis: speed, throttle/brake, steering, lateral g, rpm and time delta against
       distance, for any lap against your best (or another lap), plus a track map coloured
       where you gained (green) or lost (red) time. Up/Down lap, PgUp/PgDn reference lap,
       Left/Right cursor.

STEERING WHEEL
  Menu -> Wheel & pedals. Set the wheel rotation to match your wheel software, then click
  Detect for steering (turn right a quarter turn), throttle, brake, clutch and the paddles.
  If the wheel pulls AWAY from centre, turn on "Invert force feedback".
  Close other apps that use force feedback. Wheels in DirectInput mode are supported
  (Simagic, Logitech, Thrustmaster, Fanatec, Moza ...).

MULTIPLAYER (no port forwarding, no VPN)
  Needs Steam running and signed in (a free account is enough; you don't need to own anything).
  Host:  Menu -> Multiplayer -> type a server name -> HOST. Your server appears in everyone's list.
  Join:  Menu -> Multiplayer -> pick a server in the SERVER LIST -> JOIN  (Refresh to update).
         Or in Steam: right-click your friend -> "Join game".
  Connections go through Steam's relay network, so nobody has to open router ports.
  While testing, the game uses Steam's public test app, so Steam shows you as "playing Spacewar".
  Everyone must run the same build (servers from other builds are hidden).
  Fallback without Steam: "Direct connect (IP)" - this one needs UDP port 7777 forwarded on the host.

FEEDBACK - please send:
  1. What happened and what you expected (and what you were doing: class, session, wheel/keyboard).
  2. The log file:  KRPKNKOFF\Saved\Logs\KRPKNKOFF.log   (from the game folder, after quitting)
  3. A screenshot or short clip if it is visual.
  4. Your setup if it is about handling:  KRPKNKOFF\Saved\Setups\<class>.json
  Telemetry: press K while driving to record a CSV (KRPKNKOFF\Saved\Telemetry\).

KNOWN LIMITATIONS
  - Placeholder graphics and synthesised engine sound (real recordings to come).
  - AI is weak at overtaking in a pack.
  - Kart-to-barrier collisions are not simulated.
