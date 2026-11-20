#include "KartSim/EngineSound.h"

#include <algorithm>
#include <cmath>

namespace kartsim
{
	namespace
	{
		constexpr float Pi = 3.14159265358979f;
		float Clamp01(float X) { return X < 0.0f ? 0.0f : (X > 1.0f ? 1.0f : X); }
		float Wrap360(double D) { D = std::fmod(D, 360.0); return static_cast<float>(D < 0.0 ? D + 360.0 : D); }
		float SmoothStep(float A, float B, float X) { const float T = Clamp01((X - A) / (B - A)); return T * T * (3.0f - 2.0f * T); }
	}

	EngineSoundProfile EngineSoundProfile::ForClass(const std::string& ClassId)
	{
		EngineSoundProfile P;
		if (ClassId.find("kz") != std::string::npos)
		{
			// IAME Screamer KZ2: reed-valve 125 shifter, no exhaust valve, the loudest class. ESTIMATES: pipe tuned near
			// its ~13,500 rpm power peak, lighter silencing, rougher combustion, lever clutch (no centrifugal rattle).
			P.TunedRpm = 13500.0f; P.PipeReflection = 0.70f; P.PipeLossHz = 3200.0f;
			P.RiseDeg = 30.0f; P.ValveOpenRpm = 0.0f; P.BlowdownNoise = 0.5f; P.ShapeVariation = 0.2f;
			P.SilencerHz = 1800.0f; P.SilencerQ = 1.0f; P.ChamberHz = 900.0f; P.ChamberGainDb = 2.0f;
			P.IntakeLevel = 0.45f; P.AirboxHz = 190.0f; P.SnorkelHz = 500.0f;
			P.Roughness = 0.18f; P.TimingJitterDeg = 6.0f; P.OverrunFireProb = 0.35f;
			P.RearTeeth = 30.0f; P.ChainLevel = 0.005f; P.MechLevel = 0.06f;
			P.ClutchRattle = 0.0f; P.ClutchEngageRpm = 0.0f;
			P.Level = 0.8f; P.Drive = 1.0f;
			// v4 geometry. Bore 54.00 / stroke 54.40 mm: IAME Screamer homologation (VERIFIED). Port timing, pipe
			// diameters, silencer, airbox: ESTIMATES for a 125 KZ (exhaust ~202 deg, CIK KZ silencer is less restrictive).
			P.Geo.Bore = 0.054f; P.Geo.Stroke = 0.0544f; P.Geo.ConRod = 0.105f; P.Geo.CompressionRatio = 13.0f;
			P.Geo.EpoDeg = 79.0f; P.Geo.EpoValveClosedDeg = 0.0f; P.Geo.TpoDeg = 114.0f; P.Geo.ExhaustWidth = 0.85f;
			P.Geo.CarbDiameter = 0.030f; P.Geo.ReedArea = 1.0e-3f; P.Geo.TunedRpm = 13500.0f;
			P.Geo.HeaderD = 0.042f; P.Geo.BellyD = 0.110f; P.Geo.StingerD = 0.024f; P.Geo.TailD = 0.032f;
			P.Geo.PackingHz = 850.0f; P.Geo.PackingDry = 0.15f;
			P.ToneHz = 620.0f;
			P.Geo.AirboxVolume = 0.0035f; P.Geo.SnorkelD = 0.030f;
			P.ExhaustMix = 0.85f; P.IntakeMix = 0.35f;
			P.Level = 1.0f; // v4 with the tone filter (measured on the sweep render: no clipping)
		}
		else if (ClassId.find("kt100") != std::string::npos)
		{
			// Yamaha KT100 direct drive: piston-port, header + can silencer (a weaker "pipe"), no clutch at all.
			// ESTIMATES: tuned near its ~11,000 rpm peak, small airbox.
			P.TunedRpm = 11000.0f; P.PipeReflection = 0.52f; P.PipeLossHz = 2800.0f;
			P.RiseDeg = 38.0f; P.ValveOpenRpm = 0.0f; P.BlowdownNoise = 0.5f; P.ShapeVariation = 0.2f; P.TimingJitterDeg = 6.0f;
			P.SilencerHz = 1300.0f; P.SilencerQ = 0.8f; P.ChamberHz = 800.0f; P.ChamberGainDb = 3.0f;
			P.IntakeLevel = 0.5f; P.AirboxHz = 210.0f; P.SnorkelHz = 520.0f;
			P.Roughness = 0.12f; P.OverrunFireProb = 0.5f;
			P.RearTeeth = 63.0f; P.ChainLevel = 0.006f;
			P.ClutchRattle = 0.0f; P.ClutchEngageRpm = 0.0f;
			P.Level = 0.9f; P.Drive = 0.8f;
			// v4 geometry. Bore 52 / stroke 46 mm (secondary source). Piston-port intake approximated by the reed model.
			// Port timing (exhaust ~184 deg), small 24 mm Walbro-type carb, pipe and airbox: ESTIMATES.
			P.Geo.Bore = 0.052f; P.Geo.Stroke = 0.046f; P.Geo.ConRod = 0.095f; P.Geo.CompressionRatio = 11.5f;
			P.Geo.EpoDeg = 88.0f; P.Geo.EpoValveClosedDeg = 0.0f; P.Geo.TpoDeg = 120.0f; P.Geo.ExhaustWidth = 0.70f; P.Geo.TransferWidth = 1.1f;
			P.Geo.CarbDiameter = 0.024f; P.Geo.ReedArea = 6.0e-4f; P.Geo.TunedRpm = 11000.0f;
			P.Geo.HeaderD = 0.034f; P.Geo.BellyD = 0.080f; P.Geo.StingerD = 0.020f; P.Geo.TailD = 0.026f;
			P.Geo.PackingHz = 800.0f; P.Geo.PackingDry = 0.15f;
			P.ToneHz = 560.0f;
			P.Geo.AirboxVolume = 0.0015f; P.Geo.SnorkelD = 0.022f; P.Geo.SnorkelCount = 1;
			P.ExhaustMix = 0.8f; P.IntakeMix = 0.4f;
			P.Level = 1.4f;
		}
		else
		{
			// Rotax 125 MAX EVO (Senior): reed valve, electronically controlled exhaust valve (VERIFIED, datasheet),
			// mandatory intake silencer (airbox) and exhaust silencer, balance shaft (smooth), centrifugal clutch.
			// TunedRpm = datasheet peak-power rpm (11,500) - ASSUMPTION that the pipe is tuned there.
			// ValveOpenRpm 7,600 - the lower of the two selectable EVO settings (7,600 / 7,900); secondary source (KartPulse / dealer
			// write-ups of the EVO ECU), not the Rotax manual itself.
			// ClutchEngageRpm 4,000 = the regulation maximum (Global RMC tech regs, VERIFIED bound).
			// v3 (user: v2 "too buzzy"): wide soft pulse, strong cycle variation, heavy two-stage silencing ~1 kHz.
			P.TunedRpm = 11500.0f; P.PipeReflection = 0.40f; P.PipeLossHz = 1800.0f;
			P.RiseDeg = 42.0f; P.ValveRiseDeg = 32.0f; P.ValveOpenRpm = 7600.0f; P.ValveGain = 1.25f; // 7,600 / 7,900 rpm selectable (Senior; DD2 8,800 / 9,100): secondary source, see Docs
			P.BlowdownNoise = 0.55f;
			P.SilencerHz = 1000.0f; P.SilencerQ = 0.7f; P.ChamberHz = 450.0f; P.ChamberGainDb = 3.0f;
			P.IntakeLevel = 0.8f; P.AirboxHz = 150.0f; P.SnorkelHz = 380.0f;
			P.Roughness = 0.16f; P.TimingJitterDeg = 7.0f; P.ShapeVariation = 0.22f; P.OverrunFireProb = 0.5f;
			P.RearTeeth = 80.0f; P.ChainLevel = 0.005f;
			P.ClutchRattle = 0.06f; P.ClutchEngageRpm = 4000.0f;
			P.Level = 0.75f; P.Drive = 0.8f;
			// v4 geometry. Bore 54 / stroke 54.5 mm (Rotax MAX homologation, VERIFIED for the Junior MAX of the same
			// family). Exhaust port opens 82 deg ATDC with the valve open, 92 with it closed; Dellorto VHSB 34 carb;
			// pipe lengths from the tuning rule at 11,500 rpm (datasheet peak power). Other numbers: ESTIMATES.
			P.Geo.EpoDeg = 82.0f; P.Geo.EpoValveClosedDeg = 92.0f; P.Geo.TunedRpm = 11500.0f;
			P.Geo.PackingHz = 700.0f; P.Geo.PackingDry = 0.12f; // mandatory Rotax silencer: well packed
			P.ToneHz = 650.0f;
			P.ExhaustMix = 0.7f; P.IntakeMix = 0.5f;
			P.Level = 1.3f;
		}
		return P;
	}

	void EngineSoundSynth::Biquad::LowPass(float Sr, float F, float Q)
	{
		F = std::min(F, 0.45f * Sr);
		const float W = 2.0f * Pi * F / Sr, C = std::cos(W), Al = std::sin(W) / (2.0f * Q), A0 = 1.0f + Al;
		B0 = (1.0f - C) * 0.5f / A0; B1 = (1.0f - C) / A0; B2 = B0; A1 = -2.0f * C / A0; A2 = (1.0f - Al) / A0;
	}
	void EngineSoundSynth::Biquad::HighPass(float Sr, float F, float Q)
	{
		F = std::min(F, 0.45f * Sr);
		const float W = 2.0f * Pi * F / Sr, C = std::cos(W), Al = std::sin(W) / (2.0f * Q), A0 = 1.0f + Al;
		B0 = (1.0f + C) * 0.5f / A0; B1 = -(1.0f + C) / A0; B2 = B0; A1 = -2.0f * C / A0; A2 = (1.0f - Al) / A0;
	}
	void EngineSoundSynth::Biquad::BandPass(float Sr, float F, float Q)
	{
		F = std::min(F, 0.45f * Sr);
		const float W = 2.0f * Pi * F / Sr, C = std::cos(W), Al = std::sin(W) / (2.0f * Q), A0 = 1.0f + Al;
		B0 = Al / A0; B1 = 0.0f; B2 = -Al / A0; A1 = -2.0f * C / A0; A2 = (1.0f - Al) / A0;
	}
	void EngineSoundSynth::Biquad::Peaking(float Sr, float F, float Q, float GainDb)
	{
		F = std::min(F, 0.45f * Sr);
		const float A = std::pow(10.0f, GainDb / 40.0f);
		const float W = 2.0f * Pi * F / Sr, C = std::cos(W), Al = std::sin(W) / (2.0f * Q), A0 = 1.0f + Al / A;
		B0 = (1.0f + Al * A) / A0; B1 = -2.0f * C / A0; B2 = (1.0f - Al * A) / A0; A1 = -2.0f * C / A0; A2 = (1.0f - Al / A) / A0;
	}

	void EngineSoundSynth::SetProfile(const EngineSoundProfile& P, float SampleRateHz)
	{
		Prof = P;
		Fs = SampleRateHz;
		SilLp.LowPass(Fs, Prof.SilencerHz, Prof.SilencerQ);
		SilLp2.LowPass(Fs, Prof.SilencerHz * 1.4f, 0.6f);
		SilPeak.Peaking(Fs, Prof.ChamberHz, 1.2f, Prof.ChamberGainDb);
		RadHp.HighPass(Fs, 90.0f, 0.707f);
		AirBp.BandPass(Fs, Prof.AirboxHz, 1.6f);
		SnorkBp.BandPass(Fs, Prof.SnorkelHz, 3.0f);
		IntLp.LowPass(Fs, 1800.0f, 0.707f);
		NoiseHp.BandPass(Fs, 700.0f, 0.45f);   // exhaust roar band (was white hiss above 600 Hz)
		MechBp.BandPass(Fs, 3600.0f, 1.5f);
		Rattle[0].BandPass(Fs, 2350.0f, 30.0f);
		Rattle[1].BandPass(Fs, 3720.0f, 35.0f);
		Rattle[2].BandPass(Fs, 5650.0f, 40.0f);
		OutDc.HighPass(Fs, 25.0f, 0.707f);
		Acoustics.Init(Prof.Geo, Fs);
		LastCrank = 0.0f;
		ToneLp.LowPass(Fs, Prof.ToneHz * Prof.ToneScale, 0.6f);
	}

	float EngineSoundSynth::Noise()
	{
		Rng ^= Rng << 13; Rng ^= Rng >> 17; Rng ^= Rng << 5;
		return static_cast<float>(Rng) / 2147483648.0f - 1.0f;
	}

	float EngineSoundSynth::Pulse(float PhiDeg, float RiseDeg) const
	{
		// x e^(1-x): fast rise to 1 at PhiDeg = RiseDeg, exponential blowdown decay; cut at 8 x the rise.
		if (PhiDeg <= 0.0f || PhiDeg > 8.0f * RiseDeg) { return 0.0f; }
		const float X = PhiDeg / RiseDeg;
		return X * std::exp(1.0f - X);
	}

	void EngineSoundSynth::RenderPhysical(float* Out, int N, const EngineSoundInput& T)
	{
		const float KIn = 1.0f / (0.004f * Fs);
		const float KRun = 1.0f / (0.05f * Fs);
		const float TargetRun = T.bRunning ? 1.0f : 0.0f;
		const float TargetLocked = T.bClutchLocked ? 1.0f : 0.0f;
		for (int I = 0; I < N; ++I)
		{
			SRpm += (T.Rpm - SRpm) * KIn;
			SThr += (T.Throttle - SThr) * KIn;
			SLoad += (T.Load - SLoad) * KIn;
			SRun += (TargetRun - SRun) * KRun;
			SLocked += (TargetLocked - SLocked) * KRun;
			if ((BlockCount & 63) == 0)
			{
				// Tone corner proportional to the firing frequency: the same harmonic balance at every rpm, so the engine gets
				// louder and brighter as it revs (a fixed corner made it quieter at high rpm).
				const float Track = std::max(SRpm, 1500.0f) / 10000.0f;
				ToneLp.LowPass(Fs, Prof.ToneHz * Prof.ToneScale * std::min(std::max(Track, 0.45f), 1.45f), 0.6f);
			}
			if ((BlockCount++ & 63) == 0 && Prof.ValveOpenRpm > 0.0f)
			{
				if (!bValveOpen && SRpm > Prof.ValveOpenRpm + 150.0f) { bValveOpen = true; }
				if (bValveOpen && SRpm < Prof.ValveOpenRpm - 150.0f) { bValveOpen = false; }
			}
			SValve += ((bValveOpen ? 1.0f : 0.0f) - SValve) * (1.0f / (0.08f * Fs));
			const float Rpm = std::max(SRpm, 0.0f);

			// Firing probability: misfires on the overrun, the rev limiter's ignition cut, now and then at idle.
			const bool bOverrun = SThr < 0.06f && Rpm > std::max(Prof.ClutchEngageRpm, 3500.0f) + 500.0f;
			float PFire = 1.0f;
			if (T.bLimiter) { PFire = 0.35f; }
			else if (bOverrun) { PFire = Prof.OverrunFireProb; }
			else if (Rpm < 3500.0f) { PFire = 0.85f; }
			TwoStrokeInput In;
			In.Rpm = Rpm;
			In.Throttle = Clamp01(SThr);
			In.FireProb = T.bRunning ? PFire : 0.0f;
			In.Roughness = Prof.Roughness;
			In.JitterDeg = Prof.TimingJitterDeg;
			In.ValveOpen = Prof.ValveOpenRpm > 0.0f ? SValve : 1.0f;
			const TwoStrokeOutput O = Acoustics.Step(In);

			// Cycle events (crank wrap): clutch drum rattle, pops after misfires.
			if (O.CrankDeg < LastCrank && Rpm > 150.0f)
			{
				if (Prof.ClutchRattle > 0.0f && SLocked < 0.5f && Rpm < Prof.ClutchEngageRpm && 0.5f * (Noise() + 1.0f) < 0.45f)
				{
					RattleKick = 0.5f + 0.5f * std::fabs(Noise());
				}
				if (T.bRunning && (bOverrun || T.bLimiter) && 0.5f * (Noise() + 1.0f) < 0.006f) { PopEnv = 0.12f + 0.06f * Noise(); }
			}
			LastCrank = O.CrankDeg;

			const float Ex = O.Exhaust * Prof.ExhaustNorm;
			const float Inl = O.Intake * Prof.IntakeNorm;
			// Mechanical: piston / crank noise near the exhaust opening, chain whine, clutch rattle.
			const float TdcEnv = Rpm > 150.0f ? Pulse(Wrap360(O.CrankDeg - 262.0), 9.0f) : 0.0f;
			const float Mech = MechBp.Run(Noise()) * TdcEnv * Prof.MechLevel * Clamp01(Rpm / 12000.0f) * 3.0f;
			const float ChainHz = std::max(T.AxleHz, 0.0f) * Prof.RearTeeth;
			ChainPhase += ChainHz / Fs;
			ChainPhase -= std::floor(ChainPhase);
			ChainAm += (std::fabs(Noise()) - ChainAm) * 0.002f;
			const float ChainLvl = Prof.ChainLevel * std::pow(Clamp01(ChainHz / 2500.0f), 1.3f) * (0.6f + 0.8f * ChainAm);
			const float Chain = ChainLvl * (std::sin(2.0f * Pi * ChainPhase) + 0.35f * std::sin(4.0f * Pi * ChainPhase));
			const float Kick = RattleKick;
			RattleKick = 0.0f;
			const float RattleS = Prof.ClutchRattle * 25.0f * (Rattle[0].Run(Kick) + 0.7f * Rattle[1].Run(Kick) + 0.5f * Rattle[2].Run(Kick));
			float Pop = 0.0f;
			if (PopEnv > 0.001f) { Pop = PopEnv * Noise(); PopEnv *= 0.9988f; }

			// The model keeps pumping while the engine is stopped and spinning down; SRun fades it with the engine.
			const float RunGate = std::max(SRun, Clamp01(Rpm / 1500.0f) * 0.3f);
			const float Engine = ToneLp.Run(Prof.ExhaustMix * Ex + Prof.IntakeMix * Inl);
			float S = RunGate * Prof.Level * Engine + SRun * (Mech + RattleS + Pop) + Chain;
			S = std::tanh(Prof.Drive * S) / std::tanh(Prof.Drive);
			Out[I] = OutDc.Run(S);
		}
	}

	void EngineSoundSynth::Render(float* Out, int N, const EngineSoundInput& T)
	{
		if (Prof.bPhysical) { RenderPhysical(Out, N, T); return; }
		const float KIn = 1.0f / (0.004f * Fs);          // ~4 ms input smoothing
		const float KRun = 1.0f / (0.05f * Fs);
		const float TargetRun = T.bRunning ? 1.0f : 0.0f;
		const float TargetLocked = T.bClutchLocked ? 1.0f : 0.0f;
		for (int I = 0; I < N; ++I)
		{
			SRpm += (T.Rpm - SRpm) * KIn;
			SThr += (T.Throttle - SThr) * KIn;
			SLoad += (T.Load - SLoad) * KIn;
			SRun += (TargetRun - SRun) * KRun;
			SLocked += (TargetLocked - SLocked) * KRun;

			// Slow parameters every 64 samples: exhaust valve (with hysteresis), silencer brightness.
			if ((BlockCount++ & 63) == 0)
			{
				if (Prof.ValveOpenRpm > 0.0f)
				{
					if (!bValveOpen && SRpm > Prof.ValveOpenRpm + 150.0f) { bValveOpen = true; }
					if (bValveOpen && SRpm < Prof.ValveOpenRpm - 150.0f) { bValveOpen = false; }
				}
				SilLp.LowPass(Fs, Prof.SilencerHz * (0.7f + 0.4f * SThr), Prof.SilencerQ);
			}
			SValve += ((bValveOpen ? 1.0f : 0.0f) - SValve) * (1.0f / (0.08f * Fs)); // the valve moves in ~80 ms

			const float Rpm = std::max(SRpm, 0.0f);
			const bool bTurning = Rpm > 150.0f;
			// --- crank ---
			if (bTurning)
			{
				Theta += 360.0 * (Rpm / 60.0) / Fs;
				if (Theta >= 360.0)
				{
					Theta -= 360.0;
					// New cycle: does it fire, and how strongly?
					const bool bOverrun = SThr < 0.06f && Rpm > std::max(Prof.ClutchEngageRpm, 3500.0f) + 500.0f;
					float PFire = 1.0f;
					if (T.bLimiter) { PFire = 0.35f; }
					else if (bOverrun) { PFire = Prof.OverrunFireProb; }
					else if (Rpm < 3500.0f) { PFire = 0.8f; }              // two-strokes 4-stroke now and then at idle
					const bool bFire = T.bRunning && 0.5f * (Noise() + 1.0f) < PFire;
					const float Strength = (0.18f + 0.82f * std::pow(Clamp01(SThr), 0.7f)) * (0.75f + 0.25f * Clamp01(SLoad));
					CycleAmp = bFire ? Strength * (1.0f + Prof.Roughness * Noise()) * (bOverrun ? 0.35f + 0.3f * Noise() : 1.0f) : 0.04f;
					CycleJitter = Prof.TimingJitterDeg * Noise();
					CycleRise = 1.0f + Prof.ShapeVariation * Noise();
					// Unburnt mixture lighting in the pipe after a misfire: a pop (overrun / limiter).
					if (!bFire && T.bRunning && (bOverrun || T.bLimiter) && 0.5f * (Noise() + 1.0f) < 0.012f) { PopEnv = 0.16f + 0.08f * Noise(); }
					// Centrifugal clutch disengaged at idle: the drum rattles on its bearing now and then.
					if (Prof.ClutchRattle > 0.0f && SLocked < 0.5f && Rpm < Prof.ClutchEngageRpm && 0.5f * (Noise() + 1.0f) < 0.45f)
					{
						RattleKick = 0.5f + 0.5f * std::fabs(Noise());
					}
				}
			}

			// --- exhaust: blowdown pulse -> tuned pipe -> silencer -> radiation ---
			const float Rise = Prof.RiseDeg + (Prof.ValveRiseDeg - Prof.RiseDeg) * SValve;
			const float PEx = bTurning ? CycleAmp * Pulse(Wrap360(Theta - CycleJitter), Rise * CycleRise) : 0.0f;
			const float Turb = Prof.BlowdownNoise * (0.25f * CycleAmp + PEx) * NoiseHp.Run(Noise()) * 2.0f;  // roar: blowdown + steady flow
			const float X = PEx + Turb;
			const float D = std::min(static_cast<float>(PipeMax - 2), (30.0f / Prof.TunedRpm) * (1.0f - Prof.HotGasShrink * Clamp01(SLoad)) * Fs);
			const float ReadPos = static_cast<float>(PipeW) - D;
			const float Rp = ReadPos < 0.0f ? ReadPos + PipeMax : ReadPos;
			const int R0 = static_cast<int>(Rp);
			const float Fr = Rp - static_cast<float>(R0);
			const float Delayed = Pipe[R0 % PipeMax] * (1.0f - Fr) + Pipe[(R0 + 1) % PipeMax] * Fr;
			const float LossK = 1.0f - std::exp(-2.0f * Pi * Prof.PipeLossHz / Fs);
			PipeLp += (Delayed - PipeLp) * LossK;
			const float Refl = Prof.PipeReflection * (0.75f + 0.25f * Clamp01(SThr)); // a cold, lean-flowing pipe rings less
			const float Y = X - Refl * PipeLp;
			Pipe[PipeW] = Y;
			PipeW = (PipeW + 1) % PipeMax;
			const float Ex = RadHp.Run(SilPeak.Run(SilLp2.Run(SilLp.Run(Y)))) * (1.0f + (Prof.ValveGain - 1.0f) * SValve);

			// --- intake: reed opening ~half a turn later, through the airbox and snorkel ---
			const float IntAmp = (0.3f + 0.7f * Clamp01(SThr)) * (CycleAmp > 0.05f ? 1.0f : 0.6f);
			const float PIn = bTurning && T.bRunning ? IntAmp * Pulse(Wrap360(Theta - Prof.IntakeOffsetDeg - 0.7f * CycleJitter), 55.0f * CycleRise) : 0.0f;
			const float PInR = PIn * (1.0f + 0.7f * Noise());  // intake roar: turbulent flow through the reed / carb throat
			const float In = IntLp.Run(AirBp.Run(PInR) * 2.2f + SnorkBp.Run(PInR) * 0.9f) * Prof.IntakeLevel;

			// --- mechanical ---
			const float TdcEnv = bTurning ? Pulse(Wrap360(Theta - 262.0), 9.0f) : 0.0f;  // exhaust opens ~98 deg after TDC
			const float Mech = MechBp.Run(Noise()) * TdcEnv * Prof.MechLevel * Clamp01(Rpm / 12000.0f) * 3.0f;
			const float ChainHz = std::max(T.AxleHz, 0.0f) * Prof.RearTeeth;
			ChainPhase += ChainHz / Fs;
			ChainPhase -= std::floor(ChainPhase);
			ChainAm += (std::fabs(Noise()) - ChainAm) * 0.002f;
			const float ChainLvl = Prof.ChainLevel * std::pow(Clamp01(ChainHz / 2500.0f), 1.3f) * (0.6f + 0.8f * ChainAm);
			const float Chain = ChainLvl * (std::sin(2.0f * Pi * ChainPhase) + 0.35f * std::sin(4.0f * Pi * ChainPhase));
			const float Kick = RattleKick;
			RattleKick = 0.0f;
			const float RattleS = Prof.ClutchRattle * 25.0f * (Rattle[0].Run(Kick) + 0.7f * Rattle[1].Run(Kick) + 0.5f * Rattle[2].Run(Kick));
			float Pop = 0.0f;
			if (PopEnv > 0.001f) { Pop = PopEnv * Noise(); PopEnv *= 0.9988f; }

			float S = SRun * (Prof.Level * 0.42f * (Ex + In) + Mech + RattleS + Pop) + Chain;
			S = std::tanh(Prof.Drive * S) / std::tanh(Prof.Drive);
			Out[I] = OutDc.Run(S);
		}
	}
}
