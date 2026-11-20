# Engine sound v4: physically modelled two-stroke (2026-11-19)

User: v1-v3 "don't sound like the real engines". v1-v3 shaped pulses with filters; the analysis below shows why they
sounded dull. v4 simulates the engine's gas dynamics and acoustics: `KartSim/TwoStrokeAcoustics.h/.cpp`, used by
`EngineSoundSynth` (`EngineSoundProfile::bPhysical`, default on; game flag `-KartOldEngineSound` = v3 for A/B).

## Model (per audio sample, 48 kHz)
* Cylinder 0-D gas (mass, temperature, fresh fraction), slider-crank volume, Wiebe combustion of the trapped fresh
  charge (cycle-to-cycle burn and timing variation, misfire probability from the synth: overrun / limiter / idle),
  wall heat loss. 4 sub-steps per sample (2 for other karts).
* Exhaust port uncovered by the piston (Rotax EVO valve: later opening below the switch rpm), compressible orifice flow
  solved against the wave arriving from the pipe (bisection, 8 iterations); transfers from the crankcase.
* Crankcase pumping, reed (one way), carburettor throat area = throttle, airbox Helmholtz volume + snorkels.
* Expansion chamber = Kelly-Lochbaum waveguide, one section per sample of LOCAL wave travel; gas hottest at the port and
  cooling along the pipe (temperature follows the exhaust gas, so the tune shifts with load); reflections from area AND
  temperature changes. Lengths from the tuning rule at the class's peak-power rpm (`PipeFromTuning`).
* Straight-through silencer (packing absorbs above PackingHz), open outlet (reflection -0.95, low-passed).
  Radiated exhaust = d(outlet flow)/dt, intake = d(snorkel flow)/dt; mixed per listener (own kart: airbox close;
  other karts: exhaust dominant).

## Data (honesty)
* VERIFIED: Rotax bore / stroke 54 / 54.5 mm (Junior MAX homologation, same family), IAME Screamer 54.00 / 54.40 mm
  (IAME homologation), Rotax peak power 22 kW at 11,500 rpm (datasheet, used for the pipe tune).
* Secondary: KT100 52 / 46 mm.
* ESTIMATES: port timings (Rotax 82 deg ATDC valve open / 92 closed; KZ 79; KT100 88), all diameters, volumes, losses,
  temperature profile, scavenging (mixing-displacement, fresh-out fraction Fc^2). The Rotax homologation appendix with
  the port timing could not be fetched (HTTP 403).

## Verification
* `Acoustics_RotaxPhysicsSweep` (averaged cycles): peak cylinder pressure 47-63 bar, port pressure swings ~0.55-1.6 bar,
  suction wave at the port while the transfers flow (cycle trace), closed throttle -> ~1/3 of the charge.
* KNOWN LIMITATION: the model's engine is weaker than the real one (indicated ~15 kW at 11,000 rpm vs ~25 needed for
  the datasheet's 22 kW brake) and its power does not peak at 11,500 rpm: the linear-acoustic pipe gives weak pressure
  waves. Only the sound uses it (the kart's power comes from the datasheet torque curve), but the "on the pipe" effect
  on loudness is weaker than real. Fixing it means a non-linear gas-dynamics pipe (method of characteristics).
* Spectra (`KartSimTests --soundsweep <cfg> <wav>`, analysis script in the session scratchpad): v3 Rotax had its energy
  almost only in the firing fundamental (spectral centroid 150-300 Hz, a muffled hum). v4: strong 4th-7th harmonics
  that move with rpm (pipe resonances), centroid ~800-1,250 Hz at full throttle, noisy crackle on the overrun. Levels:
  WOT -8 to -10 dB, idle 14-19 dB below, overrun in between; clipping <0.1 % (KZ launch only).
* In game (6 karts, -KartRecordAudio): no dropouts, no clipping, -24 dBFS, centroid 838 Hz.
* Cost: 21 ms per second of audio (full), 14 ms (other karts) -> 6 karts ~9 % of one core.
* NOT verified: that it sounds like a real Rotax / KZ / KT100. Needs the user's ears.

## Listening files
`Saved/SoundRenders/`: `v4_lap_<class>.wav` (an autopilot lap), `sweep_v4_<class>.wav` vs `sweep_v3_<class>.wav`
(idle, blip, 8 s full-throttle sweep to the limiter, lift-off coast, half throttle).

## v4.1 (user: "sounds horrible now like a very loud mosquito")
Cause (measured): the far-field d/dt radiation plus a leaky silencer put the energy in the 4th-7th harmonics
(centroid 0.8-1.2 kHz, where the ear is most sensitive); the airbox Helmholtz was under-damped and droned at low rpm.
* Listener tone filter: 2nd-order low-pass at ToneHz (Rotax 650, KZ 620, KT100 560 Hz at 10,000 rpm), corner
  proportional to rpm (same harmonic balance at every rpm, so the engine gets louder and brighter as it revs).
  Net slope above the corner -6 dB/oct. Stands for helmet / body shielding / distance (ESTIMATE).
* Silencer packing: Rotax 700 Hz / 12 % straight through, KZ 850 / 15 %, KT100 800 / 15 % (were 1,200-1,800 / 30-45 %).
* AirboxDamping 0.25 -> 0.70 (filter + foam): intake now about -13 dB at all rpm (was -6.6 dB at 5,000 rpm).
* Levels re-set: WOT rises with rpm (Rotax -14 -> -11 dB, KZ -17 -> -11.6, KT100 -19 -> -14.5), no clipping.
* Firing fundamental now the strongest component; Rotax centroid 560 Hz at 14,000 rpm; in game 433 Hz (was 838).
* Settings > Sound > Engine tone (Darker / Brighter, 0.5-1.6 x the corner), live.
