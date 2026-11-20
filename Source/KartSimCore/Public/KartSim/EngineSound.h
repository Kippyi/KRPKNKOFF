// Two-stroke kart engine sound, synthesised from the engine's physical layout (engine-free, used by the game's
// UKartSynthWave and by the offline renderer `KartSimTests --sound`).
//
// Signal path (per sample, 48 kHz):
//   crank angle -> EXHAUST blowdown pulse at port opening (shape fixed in crank DEGREES, so it gets shorter and
//   brighter with rpm; amplitude from throttle / load with cycle-to-cycle variation, misfires off throttle)
//     -> TUNED PIPE: a delay line with a negative end reflection = resonances at odd multiples of 1/(2*D). D is set so
//        the first resonance equals the firing frequency at the pipe's tuned rpm: "on the pipe" the firing harmonics
//        line up with the pipe and the note hardens; off the pipe they don't (muddy at low rpm, thin past the peak).
//        Hot gas = faster sound = shorter D at load.
//     -> SILENCER (resonant low-pass + chamber peak) -> tailpipe radiation (high-pass).
//   + INTAKE pulse (reed opening, ~half a turn after the exhaust) through the airbox (Helmholtz band-pass) and its
//     snorkel; strongest at wide-open throttle. On a Rotax the airbox sits next to the driver's ear.
//   + EXHAUST VALVE (Rotax EVO): opens above ValveOpenRpm -> sharper blowdown, louder.
//   + MECHANICAL: piston / crank noise near TDC, chain mesh whine (rear axle rev/s x sprocket teeth),
//     centrifugal-clutch drum rattle at idle, rev-limiter ignition cut.
//   -> gentle saturation.
// All profile numbers are ESTIMATES / ASSUMPTIONS from the engines' layout (see EngineSound.cpp), not measurements.
//
// v4 (bPhysical, default): the exhaust and intake come from TwoStrokeAcoustics (cylinder gas, ports, expansion
// chamber waveguide, silencer, crankcase / reed / airbox); the pulse model above remains as the fallback
// (bPhysical = false, game: -KartOldEngineSound) for A/B listening. Mechanical / chain / clutch / pops are shared.
#pragma once

#include "KartSim/Export.h"
#include "KartSim/TwoStrokeAcoustics.h"

#include <cstdint>
#include <string>

namespace kartsim
{
	struct EngineSoundProfile
	{
		float TunedRpm = 11500.0f;      // pipe tuned to this rpm (first resonance = firing frequency)
		float PipeReflection = 0.62f;   // |end reflection| incl. losses (0..0.9); sharper "on pipe" note when higher
		float PipeLossHz = 2500.0f;     // high frequencies lose more per round trip
		float HotGasShrink = 0.06f;     // round-trip time shrinks by this fraction at full load (hotter gas)
		float RiseDeg = 22.0f;          // blowdown pulse rise (crank degrees); exhaust valve open uses ValveRiseDeg
		float ValveRiseDeg = 16.0f;
		float ValveOpenRpm = 0.0f;      // 0 = no exhaust valve
		float ValveGain = 1.25f;        // level factor with the valve open
		float BlowdownNoise = 0.38f;    // turbulent component of the blowdown
		float SilencerHz = 1500.0f;     // silencer low-pass
		float SilencerQ = 0.8f;
		float ChamberHz = 650.0f;       // silencer chamber resonance (peaking)
		float ChamberGainDb = 4.0f;
		float IntakeLevel = 0.55f;      // relative to exhaust
		float IntakeOffsetDeg = 190.0f; // intake pulse phase after the exhaust pulse
		float AirboxHz = 170.0f;        // airbox Helmholtz resonance
		float SnorkelHz = 420.0f;       // intake snorkel / silencer tube resonance
		float Roughness = 0.10f;        // cycle-to-cycle amplitude variation
		float ShapeVariation = 0.15f;   // cycle-to-cycle pulse-width variation (smears harmonics: real combustion is never identical)
		float TimingJitterDeg = 1.5f;   // cycle-to-cycle ignition timing variation
		float OverrunFireProb = 0.45f;  // fraction of cycles that fire on a closed throttle above idle
		float RearTeeth = 80.0f;        // chain mesh whine = rear axle rev/s x rear sprocket teeth
		float ChainLevel = 0.035f;
		float MechLevel = 0.05f;
		float ClutchRattle = 0.08f;     // centrifugal clutch drum rattle at idle (direct drive / Rotax); 0 = none
		float ClutchEngageRpm = 4000.0f;
		float Level = 0.55f;
		float Drive = 1.6f;             // output saturation
		// v4 physical model
		bool bPhysical = true;
		TwoStrokeGeometry Geo;
		float ExhaustNorm = 1.0f / 400.0f, IntakeNorm = 1.0f / 8.0f; // model units -> ~unit level at full power
		float ExhaustMix = 0.75f, IntakeMix = 0.45f;                 // listener balance (driver's ear: airbox close)
		// Listener tone: 2nd-order roll-off above ToneHz (helmet, body shielding, distance; the far-field d/dt radiation
		// alone sounded like "a very loud mosquito" - user, v4 first build). ToneScale = the player's Engine tone setting.
		float ToneHz = 650.0f;
		float ToneScale = 1.0f;

		static EngineSoundProfile ForClass(const std::string& ClassId);
	};

	struct EngineSoundInput
	{
		float Rpm = 0.0f;
		float Throttle = 0.0f;   // effective 0..1
		float Load = 0.0f;       // 0..1 torque fraction
		bool bRunning = true;
		bool bLimiter = false;   // rev limiter cutting
		bool bClutchLocked = true;
		float AxleHz = 0.0f;     // rear axle rev/s (chain whine)
	};

	class KARTSIM_API EngineSoundSynth
	{
	public:
		void SetProfile(const EngineSoundProfile& P, float SampleRateHz);
		// Renders N mono samples (-1..1). Targets are smoothed per sample (no zipper noise).
		void Render(float* Out, int N, const EngineSoundInput& Target);
		const EngineSoundProfile& Profile() const { return Prof; }
		void SetToneScale(float S) { Prof.ToneScale = S < 0.3f ? 0.3f : (S > 2.0f ? 2.0f : S); } // applied within 64 samples

	private:
		struct Biquad
		{
			float B0 = 1, B1 = 0, B2 = 0, A1 = 0, A2 = 0, Z1 = 0, Z2 = 0;
			float Run(float X) { const float Y = B0 * X + Z1; Z1 = B1 * X - A1 * Y + Z2; Z2 = B2 * X - A2 * Y; return Y; }
			void LowPass(float Fs, float F, float Q);
			void HighPass(float Fs, float F, float Q);
			void BandPass(float Fs, float F, float Q);   // 0 dB peak
			void Peaking(float Fs, float F, float Q, float GainDb);
		};
		float Noise();
		float Pulse(float PhiDeg, float RiseDeg) const;

		EngineSoundProfile Prof;
		float Fs = 48000.0f;
		// smoothed inputs
		float SRpm = 0.0f, SThr = 0.0f, SLoad = 0.0f, SRun = 0.0f, SLocked = 1.0f, SValve = 0.0f;
		// crank
		double Theta = 0.0;               // degrees, 0..360
		float CycleAmp = 0.0f, CycleJitter = 0.0f, CycleRise = 1.0f;
		bool bValveOpen = false;
		// pipe
		static constexpr int PipeMax = 2048;
		float Pipe[PipeMax] = {};
		int PipeW = 0;
		float PipeLp = 0.0f;
		// filters
		Biquad SilLp, SilLp2, SilPeak, RadHp, AirBp, SnorkBp, IntLp, NoiseHp, MechBp, Rattle[3], OutDc;
		float ChainPhase = 0.0f, ChainAm = 0.0f;
		float RattleKick = 0.0f;
		float PopEnv = 0.0f;
		TwoStrokeAcoustics Acoustics;
		Biquad ToneLp;
		float LastCrank = 0.0f;
		void RenderPhysical(float* Out, int N, const EngineSoundInput& Target);
		uint32_t Rng = 0x9E3779B9u;
		int BlockCount = 0;
	};
}
