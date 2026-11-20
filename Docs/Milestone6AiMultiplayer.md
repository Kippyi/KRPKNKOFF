# Milestone 6 – AI and Multiplayer

## Part 1: AI racing

### Kart-to-kart contact (`KartSim/KartContact.h`)
* Each kart is a box: CIK maximum length 1820 mm and width 1400 mm (VERIFIED limits), centred on the
  wheelbase, active from the ground to 0.6 m (a kart airborne above another does not touch it).
* 2-D separating-axis test on the yaw-only footprints → minimum-penetration normal and depth. The contact
  point is the mean of the corners inside the other box, at bumper height.
* Penalty spring-damper with regularised Coulomb friction: k = 1.5e5 N/m, c = 2800 N s/m, μ = 0.35.
  **All ASSUMPTIONS** (no bumper data). A 5 cm overlap gives 7.5 kN; damping is ~0.4 of critical.
* Applied through `KartSimulation::SetExternalLoad` (force plus moment about the CoG, held for one step),
  equal and opposite. All local karts are stepped in **lockstep** and contacts are evaluated every step.
* Test: a 5 m/s rear-end bump lasts 75 ms with 71 mm maximum overlap, loses half the kinetic energy
  (inelastic, as plastic bodywork is) and stays straight.
* **Not modelled:** wheel-to-wheel interlocking (the classic kart launch), barrier collisions, damage.

### Slipstream (`KartSim/Slipstream.h`)
A kart close behind another and in line gets less drag (`Settings.DragScale`): up to 45% less at zero gap,
decaying over 4 m and fading to nothing at 1 m lateral offset. **ASSUMPTION**: needs straight-line draft
data. A solo kart is unaffected.

### Racing line (`KartSim/RacingLine.h`)
A minimum-curvature line inside the paved width (kart CoG kept 0.9 m from the edge), solved by
Gauss-Seidel on the discrete curvature, coarse to fine (point strides 32 → 1; plain sweeps converge
hopelessly slowly). It uses the full width (3.1 m offsets) and cuts ∫κ² by 15%. It clips the inside of the
long carousel, as a minimum-curvature line should. It is the usual AI baseline, not a true minimum-time line.

### AI driver (`KartSim/AiDriver.h`)
* The same simulated kart as the player, with no grip or power help. It drives with the shared Autopilot
  control law on the racing line.
* Autopilot gained two inputs that do nothing at their defaults (regression laps unchanged): a lateral
  offset (with the speed profile using the offset path's curvature κ/(1−κo)) and a speed cap.
* **Skill:** grip target (lateral g), braking, lap-to-lap consistency, and aggression (following gap and
  patience before an overtake).
* **Traffic:** speed-matching behind a kart in its lane at a speed-dependent gap.
* **Overtaking:** after following closely it passes, preferring the inside of the next corner, and commits
  until the passed kart is behind. If its side runs out of room before it is ahead, it aborts and yields
  (drops back).
* **Side by side:** keeps 1.6 m lateral clearance from karts alongside, which takes priority over the track
  clamp (up to 0.8 m of kerb).
* Lap times: racing line at 1.4 g gives 20.65 s, against 22.68 s for the centre-line Autopilot.

### Race-craft bugs found by testing
* **The AI ran off track while overtaking** (laps invalidated in-engine). The lateral offset was clamped to the
  room at the kart's position, but the Autopilot applies it ~10 m ahead. It is now clamped to the minimum room
  over the next 15 m.
* **Squeezed off around the outside.** A trace showed a kart passing on the outside running out of room as the
  line swung across, then steering into the kart alongside (300–1400 N for 1.5 s) and off the track. This is now
  fixed by the inside-line preference, the abort-and-yield, and the clearance priority above.
* The AI tests now also check track limits (the standalone track is an infinite plane, which is why this was
  missed before).
* A progress-based abort ("not gaining in 3 s") was tried and **reverted**: it broke one-on-one passing.

### Race world and sessions (`AKartRaceWorld`, `KartSim/RaceDirector.h`)
* `AKartRaceWorld` steps every local kart in lockstep at 240 Hz, applies contacts and slipstream, drives the
  AI and runs the sessions. Lap timing uses one common race clock.
* `RaceDirector` replaces M5's single-kart `Session` and keeps its rules (its tests were ported):
  practice/qualifying by best valid lap; the race classified by laps and distance, with gaps and laps down.
  Before green the order is the grid/formation order.
  * **Rolling start** (Rotax, KT100): the formation lap is limited to 54 km/h for every kart, the player
    included (a throttle-cut limiter). Green comes when the first kart reaches the line; times run from that
    common green.
  * **Standing start** (KZ): lights, then green.
  * The race grid comes from the last qualifying result, otherwise AI first and humans at the back.
* `-KartAI=n` (default 5, max 11), `-KartAIStrength=f` (0.9; key **U** cycles 70–100%), `-KartPlayerAI` (an AI
  drives your kart: demo and test). The HUD has a timing tower (position, name, gap or best lap, AI state, contacts/s).

### Verification (AI)
* 54/54 standalone tests, including:
  * contact;
  * racing line;
  * a faster AI passing a slower one cleanly while staying on track;
  * a 6-kart race (no spins, on track, all complete the distance);
  * the race director (rolling/standing start, flag rules, lapped karts).
* In engine:
  * solo regression `-KartAI=0 -KartAutopilot=1.4` gives 22.683 s, unchanged.
  * 6-kart Rotax race (rolling start) and KZ race (standing start) run start to finish with every kart
    classified.
  * 8 karts: ~1.9 ms physics per frame.

### Known limitations (AI)
* **Overtaking in a pack is weak.** Faster karts often stall behind slower ones: in the six-kart test a 1.42 g
  kart stayed behind a 1.25 g kart for the whole 144 s. The test reports order changes instead of asserting
  them. The following logic caps speed to the kart ahead, so the AI can't build a draft run, and passing on
  a non-optimal offset line rarely gains enough. One-on-one passing works and is tested.
* The AI does not defend, make mistakes beyond its consistency spread, or react to track conditions (it drives
  to a fixed grip target; on a wet track lower the strength with U).
* AI karts may use the kerbs when passing (|offset| up to ~5.2 m; track limits start at ~5.6 m).

## Part 2: Multiplayer (verified on one PC over loopback)

### How to play
* Host: `UnrealEditor.exe KRPKNKOFF.uproject /Engine/Maps/Entry?listen -game` (plus any of the usual flags,
  e.g. `-KartAI=3 -KartClass=KZ2_IAMEScreamer.json`).
* Client: `UnrealEditor.exe KRPKNKOFF.uproject <host IP> -game` (127.0.0.1 on the same PC). UDP port 7777 must
  be reachable.
* The host controls sessions (Y), AI strength (U) and conditions (N/M); everyone sees the same timing tower.

### Model (the usual racing-sim approach)
* **Each kart is simulated where it is driven:** a human's kart on their own machine (no input latency), AI
  karts on the host. That machine sends the kart's state at **30 Hz** in sim coordinates (position, orientation,
  velocities, steer, wheel speeds, track-limits flag): owner → host → multicast.
* **Everywhere else the kart is a kinematic proxy** (`KartSimulation::SetKinematicPose`), interpolated **100 ms
  behind** the server clock (extrapolated up to 0.25 s if packets are late).
* **Contacts:** every machine runs contacts against all karts but only moves its own karts, so each driver feels
  the hit against the other kart's (100 ms old) proxy. Symmetric at low latency; at high latency contacts become
  approximate (no rollback).
* **Timing and sessions are host-authoritative:** the host times remote karts on its own race clock from their
  proxy paths. Track limits are judged on the driver's machine and sent as a flag. The session, phase,
  lights, weather preset and classification are replicated (`AKartRaceWorld`). The start lights and the
  formation limiter apply to every machine's own kart.
* The environment (track, sky, conditions) is created locally on every machine. Conditions follow the host's
  presets but evolve locally (water and rubber grids are not synchronised cell by cell).

### Verification (multiplayer)
* Two processes on one PC (loopback): a host with 3 AI and a client joining mid-session. The session restarts
  with all 5 karts, and the client's view shows all karts, the replicated tower and AI states.
* Remote timing check: the client's kart (same AI pace) best lap as timed by the host, 21.691 s, against the
  host's own kart, 21.692 s.
* KZ standing start with a remote player: held on the grid by the replicated lights, classified normally.
  Class changes replicate (proxies switch to the KZ).
* Solo regression unchanged (22.683 s).

### Known limitations (multiplayer)
* **Tested only on loopback (≈0 ms latency).** Behaviour at real internet latency and packet loss is unverified.
* Listen server only (the host plays); no dedicated server, lobby, server browser or reconnection. A player
  joining or leaving restarts the session.
* The owner is trusted with its kart's state (no anti-cheat or server-side validation).
* Kart class is per player; AI follow the host's class.

### Bug found on the way
`AKartVehiclePawn` had `AutoPossessPlayer = Player0`, so **every AI kart spawned after the player stole the
local player's controller and camera** (the first race screenshot was following an AI kart). In multiplayer
the host would also have possessed every client's kart. It is now disabled; the GameMode possesses each
player's pawn.

## Online: Steam server list + relay (2026-11-18)
* The user found port forwarding and ZeroTier too painful and wanted a KRP-style server list. Backend: Steam with App
  ID 480 (Valve's public test app "Spacewar": free, testing only; releasing needs your own App ID, a $100 Steam
  Direct fee, recouped after $1,000 of sales).
* Plugins: OnlineSubsystem, OnlineSubsystemUtils, OnlineSubsystemSteam, SteamSockets. GameNetDriver = SteamSocketsNetDriver
  (Steam relay network: no port forwarding), falling back to IpNetDriver when Steam is not running.
* UKartGameInstance: HostOnline (advertises a Steam lobby: game tag KRPKNKOFF, build date, server name, track, class;
  8 players), then opens the track as a listen server; RefreshServers (lobby search filtered by the game tag; other
  builds are hidden and counted); JoinServer (JoinSession, then travel to the resolved steam address); Steam "Join game" /
  invites; LeaveOnlineSession when going to the main menu or quitting. Without Steam: LAN sessions + direct IP.
* Menu > Multiplayer: status line, server list (name / host / track / class / drivers / JOIN, Refresh), host box with
  server name and session options, direct IP fallback.
* Race world: human drivers take their Steam names (player state) when Steam is online.
* Verified (this PC, Steam not running): the game starts, Steam fails to initialise cleanly, LAN fallback; the menu
  page renders; direct-IP loopback host + client still connects (IpNetDriver fallback, "Player 2 joined (remote)").
* NOT yet verified: anything with Steam running (lobby creation, list, relay join). A full test needs two Steam
  accounts on two PCs; the same account cannot join itself.
