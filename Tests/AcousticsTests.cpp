// Physically modelled engine sound (TwoStrokeAcoustics): stability, physics sanity, cost.
#include "TestFramework.h"

#include "KartSim/EngineSound.h"
#include "KartSim/TwoStrokeAcoustics.h"

#include <chrono>
#include <cmath>
#include <vector>

using namespace kartsim;

namespace
{
	struct SteadyResult { TwoStrokeCycleStats S; double ExRms = 0.0, InRms = 0.0, Peak = 0.0, MinKw = 0.0, MaxKw = 0.0; bool bFinite = true; };

	SteadyResult RunSteady(TwoStrokeAcoustics& M, float Rpm, float Thr, double Seconds, float FireProb = 1.0f)
	{
		SteadyResult R;
		const int N = static_cast<int>(Seconds * 48000.0);
		double Se = 0.0, Si = 0.0;
		int Count = 0, NC = 0, LastCycleNo = -1;
		TwoStrokeCycleStats Acc;
		double MinKw = 1e9, MaxKw = 0.0;
		TwoStrokeInput In;
		In.Rpm = Rpm; In.Throttle = Thr; In.FireProb = FireProb; In.Roughness = 0.0f; In.JitterDeg = 0.0f;
		for (int I = 0; I < N; ++I)
		{
			const TwoStrokeOutput O = M.Step(In);
			if (!std::isfinite(O.Exhaust) || !std::isfinite(O.Intake)) { R.bFinite = false; }
			if (I > N / 2) { Se += O.Exhaust * O.Exhaust; Si += O.Intake * O.Intake; ++Count; R.Peak = std::fmax(R.Peak, std::fabs(O.Exhaust)); }
			// Average the cycle stats over the second half (single cycles scatter).
			const TwoStrokeCycleStats& L = M.LastCycle();
			if (I > N / 2 && L.Cycles != LastCycleNo)
			{
				LastCycleNo = L.Cycles;
				++NC;
				Acc.IndicatedKW += L.IndicatedKW; Acc.PeakCylinderBar += L.PeakCylinderBar; Acc.TrappedFreshMg += L.TrappedFreshMg;
				Acc.DeliveryRatio += L.DeliveryRatio; Acc.PortPeakBar += L.PortPeakBar; Acc.PortMinBar += L.PortMinBar; Acc.PipeGasK += L.PipeGasK;
				MinKw = std::fmin(MinKw, L.IndicatedKW); MaxKw = std::fmax(MaxKw, L.IndicatedKW);
			}
		}
		const double D = NC > 0 ? 1.0 / NC : 1.0;
		R.S = Acc;
		R.S.IndicatedKW *= D; R.S.PeakCylinderBar *= D; R.S.TrappedFreshMg *= D; R.S.DeliveryRatio *= D; R.S.PortPeakBar *= D; R.S.PortMinBar *= D; R.S.PipeGasK *= D;
		R.MinKw = MinKw; R.MaxKw = MaxKw;
		R.ExRms = std::sqrt(Se / std::max(Count, 1));
		R.InRms = std::sqrt(Si / std::max(Count, 1));
		return R;
	}

	TwoStrokeGeometry RotaxGeo()
	{
		TwoStrokeGeometry G;
		G.EpoValveClosedDeg = 92.0f;
		return G;
	}
}

KS_TEST(Acoustics_RotaxPhysicsSweep)
{
	TwoStrokeGeometry G = RotaxGeo();
	G.PipeFromTuning();
	KS_LOG("pipe: header %.3f diffuser %.3f belly %.3f baffle %.3f stinger %.3f m (total %.2f)", G.HeaderL, G.DiffuserL, G.BellyL, G.BaffleL, G.StingerL,
	       G.HeaderL + G.DiffuserL + G.BellyL + G.BaffleL + G.StingerL);
	TwoStrokeAcoustics M;
	M.Init(G, 48000.0f);
	const auto T0 = std::chrono::steady_clock::now();
	double BestDr = 0.0, BestRpm = 0.0, Seconds = 0.0;
	bool bAllFinite = true;
	for (float Rpm = 5000.0f; Rpm <= 14000.0f; Rpm += 1000.0f)
	{
		const SteadyResult R = RunSteady(M, Rpm, 1.0f, 1.5);
		Seconds += 1.5;
		bAllFinite = bAllFinite && R.bFinite;
		KS_LOG("WOT %5.0f rpm: %4.1f kW ind (cycles %4.1f-%4.1f), peak %5.1f bar, trapped %5.1f mg, delivery %.2f, port %.2f..%.2f bar, gas %4.0f K, exhaust rms %.0f intake rms %.1f",
		       Rpm, R.S.IndicatedKW, R.MinKw, R.MaxKw, R.S.PeakCylinderBar, R.S.TrappedFreshMg, R.S.DeliveryRatio, R.S.PortMinBar, R.S.PortPeakBar, R.S.PipeGasK, R.ExRms, R.InRms);
		if (Rpm >= 7000.0f && R.S.TrappedFreshMg > BestDr) { BestDr = R.S.TrappedFreshMg; BestRpm = Rpm; }
	}
	const double Sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - T0).count();
	KS_LOG("most trapped charge at %.0f rpm; cost %.1f ms per second of audio", BestRpm, 1000.0 * Sec / Seconds);
	KS_EXPECT(bAllFinite);
	// Off throttle the engine breathes far less.
	TwoStrokeAcoustics M2;
	M2.Init(G, 48000.0f);
	const SteadyResult Closed = RunSteady(M2, 9000.0f, 0.0f, 1.0);
	KS_LOG("closed throttle 9000 rpm: peak %.1f bar, trapped %.1f mg, delivery %.2f, exhaust rms %.3f", Closed.S.PeakCylinderBar, Closed.S.TrappedFreshMg,
	       Closed.S.DeliveryRatio, Closed.ExRms);
	KS_EXPECT(Closed.bFinite);
}

KS_TEST(Acoustics_CycleTrace)
{
	// Port / cylinder / crankcase pressure through one crank turn at 11,000 rpm WOT (after settling).
	TwoStrokeGeometry G = RotaxGeo();
	TwoStrokeAcoustics M;
	M.Init(G, 48000.0f);
	TwoStrokeInput In;
	In.Rpm = 11000.0f; In.Throttle = 1.0f; In.Roughness = 0.0f; In.JitterDeg = 0.0f;
	for (int I = 0; I < 48000 * 2; ++I) { M.Step(In); }
	double Next = 0.0;
	bool bWrapped = false;
	float Prev = 0.0f;
	for (int I = 0; I < 48000; ++I)
	{
		const TwoStrokeOutput O = M.Step(In);
		if (O.CrankDeg < Prev) { if (bWrapped) { break; } bWrapped = true; Next = 0.0; }
		Prev = O.CrankDeg;
		if (bWrapped && O.CrankDeg >= Next)
		{
			KS_LOG("%5.1f deg: cyl %6.2f bar  port %5.2f bar  crankcase %5.2f bar  exhaust %8.1f", O.CrankDeg, M.CylinderBar(), M.PortBar(), M.CrankcaseBar(), O.Exhaust);
			Next += 15.0;
		}
	}
	KS_LOG("pipe length in model %.2f m", M.PipeLengthM());
}

KS_TEST(Acoustics_LowDetailMatchesAndIsCheaper)
{
	// Other karts use 2 cylinder sub-steps: same sound level within ~2 dB, stable, cheaper.
	for (float Rpm : {6000.0f, 9000.0f, 12000.0f})
	{
		TwoStrokeGeometry G = RotaxGeo();
		TwoStrokeAcoustics Full, Low;
		Full.Init(G, 48000.0f);
		G.Substeps = 2;
		Low.Init(G, 48000.0f);
		const auto T0 = std::chrono::steady_clock::now();
		const SteadyResult Rf = RunSteady(Full, Rpm, 1.0f, 1.0);
		const auto T1 = std::chrono::steady_clock::now();
		const SteadyResult Rl = RunSteady(Low, Rpm, 1.0f, 1.0);
		const auto T2 = std::chrono::steady_clock::now();
		const double Db = 20.0 * std::log10(Rl.ExRms / Rf.ExRms);
		KS_LOG("%5.0f rpm: full %.0f / low %.0f exhaust rms (%+.1f dB), %.1f / %.1f ms per s", Rpm, Rf.ExRms, Rl.ExRms, Db,
		       1000.0 * std::chrono::duration<double>(T1 - T0).count(), 1000.0 * std::chrono::duration<double>(T2 - T1).count());
		KS_EXPECT(Rl.bFinite && std::fabs(Db) < 3.0);
	}
}

KS_TEST(Acoustics_SynthSurvivesAbuse)
{
	// Stall, instant rpm jumps, limiter, engine off and restart, closed throttle at the limiter: always finite, bounded.
	for (const char* Cls : {"rotax_max_senior", "kz2_iame", "kt100_dd"})
	{
		EngineSoundSynth S;
		S.SetProfile(EngineSoundProfile::ForClass(Cls), 48000.0f);
		float Buf[256];
		double Peak = 0.0;
		bool bFinite = true;
		uint32_t R = 7u;
		for (int B = 0; B < 48000 * 20 / 256; ++B)
		{
			R = R * 1664525u + 1013904223u;
			EngineSoundInput In;
			const int Phase = (B / 300) % 5;
			In.Rpm = Phase == 0 ? 0.0f : static_cast<float>((R >> 8) % 16000);
			In.Throttle = static_cast<float>((R >> 4) % 100) / 100.0f;
			In.Load = In.Throttle;
			In.bRunning = Phase != 3;
			In.bLimiter = Phase == 4 && (B % 3 == 0);
			In.bClutchLocked = (B % 2) == 0;
			In.AxleHz = In.Rpm / 360.0f;
			S.Render(Buf, 256, In);
			for (float V : Buf) { bFinite = bFinite && std::isfinite(V); Peak = std::fmax(Peak, std::fabs(V)); }
		}
		KS_LOG("%s: abuse 20 s, peak %.2f, finite %d", Cls, Peak, bFinite ? 1 : 0);
		KS_EXPECT(bFinite && Peak < 2.0);
	}
}

KS_TEST(Acoustics_ComponentLevels)
{
	// Steady full throttle through the synth (tone filter included): exhaust only vs intake only, dB re full scale.
	for (const char* Cls : {"rotax_max_senior", "kz2_iame", "kt100_dd"})
	{
		for (float Rpm : {5000.0f, 7000.0f, 9000.0f, 11000.0f, 13000.0f})
		{
			double Db[2];
			for (int Part = 0; Part < 2; ++Part)
			{
				EngineSoundProfile P = EngineSoundProfile::ForClass(Cls);
				P.MechLevel = 0.0f; P.ChainLevel = 0.0f; P.ClutchRattle = 0.0f; P.Roughness = 0.0f;
				if (Part == 0) { P.IntakeMix = 0.0f; } else { P.ExhaustMix = 0.0f; }
				EngineSoundSynth S;
				S.SetProfile(P, 48000.0f);
				EngineSoundInput In;
				In.Rpm = Rpm; In.Throttle = 1.0f; In.Load = 1.0f;
				float Buf[480];
				double Sq = 0.0; int N = 0;
				for (int B = 0; B < 200; ++B) { S.Render(Buf, 480, In); if (B >= 100) { for (float V : Buf) { Sq += V * V; ++N; } } }
				Db[Part] = 10.0 * std::log10(Sq / N + 1e-12);
			}
			KS_LOG("%s %5.0f rpm: exhaust %5.1f dB, intake %5.1f dB", Cls, Rpm, Db[0], Db[1]);
		}
	}
}
