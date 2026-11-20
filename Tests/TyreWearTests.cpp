// Tyre compounds, wet response and wear. Coefficients are ASSUMPTIONS/ESTIMATES (Docs/TyresWearWet.md);
// these tests pin the BEHAVIOUR: which compound wins where, overheating wets, wear costing lap time.
#include "DriverBot.h"
#include "TestFramework.h"
#include "TestHelpers.h"

#include "KartSim/KartPhysicsCore.h"
#include "KartSim/Setup.h"

#include <cmath>

using namespace kartsim;

namespace
{
	KartConfig WithCompound(KartConfig C, int Compound) { C.FitTyres(Compound); return C; }
	double Sum(const std::vector<double>& V, size_t From) { double S = 0; for (size_t I = From; I < V.size(); ++I) { S += V[I]; } return S; }
	void Soak(TrackConditions& T)
	{
		BuildTestConditions(T);
		T.Weather = {16.0, 1.0, 0.0, 16.0};
		T.SetUniform(0.0, 1.5);
		T.Params.SqueegeePerPass = 0.0; T.Params.DrainPerHour = 0.0; T.Params.EvaporationPerHour = 0.0;
	}
}

KS_TEST(Tyre_EveryClassHasAWetSet)
{
	for (const char* F : {"Rotax125MaxSenior.json", "KZ2_IAMEScreamer.json", "KT100_DirectDrive.json", "Rotax125MaxSenior_DistributedFrame.json"})
	{
		KartConfig C = kstest::LoadKart(F);
		KS_EXPECT(C.HasWetTyres());
		KS_EXPECT(C.Compound == 0);
		KS_EXPECT_NEAR(C.FrontTyre.WetGripFloor, 0.60, 1e-12);
		// The setup page switches the set and it round-trips through a saved setup.
		int N = 0;
		const SetupParam* P = GetSetupParams(N);
		int Idx = -1;
		for (int I = 0; I < N; ++I) { if (std::string(P[I].Key) == "tyreCompound") { Idx = I; } }
		KS_EXPECT(Idx >= 0);
		SetSetupValue(C, Idx, 1.0);
		KS_EXPECT(C.Compound == 1 && C.RearTyre.WetGripFloor > 0.8 && C.RearTyre.TreadDepth == 5.0);
		KartConfig D = kstest::LoadKart(F);
		std::string Err;
		KS_EXPECT(ApplySetupJson(SerializeSetup(C), D, Err));
		KS_EXPECT(D.Compound == 1 && D.RearTyre.UnloadedRadius == C.RearTyre.UnloadedRadius);
	}
}

KS_TEST(Tyre_WetGripIsACompoundProperty)
{
	const KartConfig C = kstest::LoadRotax();
	TyreModel Slick, Wet;
	const KartConfig CW = WithCompound(C, 1);
	Slick.Init(C.RearTyre);
	Wet.Init(CW.RearTyre);
	KS_LOG("wet factor at saturation: slick %.2f, W5 %.2f; dry: both %.2f/%.2f", Slick.WetGripFactor(1.0), Wet.WetGripFactor(1.0), Slick.WetGripFactor(0.0), Wet.WetGripFactor(0.0));
	KS_EXPECT_NEAR(Slick.WetGripFactor(1.0), 0.60, 1e-12);
	KS_EXPECT(Wet.WetGripFactor(1.0) * CW.RearTyre.Mu0 > 1.15 * Slick.WetGripFactor(1.0) * C.RearTyre.Mu0);
	KS_EXPECT(CW.RearTyre.Mu0 < C.RearTyre.Mu0); // on a dry track the slick has more grip
	// Wear doubles per WearTempDoubling above the optimum; wear grip loss is gentle early, full at the end.
	KS_EXPECT_NEAR(Wet.WearTempFactor(CW.RearTyre.OptimumTemp + 10.0), 2.0, 1e-9);
	Slick.SetWear(0.5);
	KS_EXPECT_NEAR(Slick.WearGripFactor(), 1.0 - 0.25 * (1.0 - C.RearTyre.MinWearGripFactor), 1e-12);
	KS_EXPECT_NEAR(Slick.GetTreadRemaining(), 1.75, 1e-12);
}

KS_TEST(Tyre_WetsWinInTheWetSlicksSpin)
{
	// Saturated track, same driver target: the W5 set is clean, the slicks are not (or are much slower).
	const KartConfig C = kstest::LoadRotax();
	TrackConditions A, B;
	Soak(A);
	Soak(B);
	const LapResult Wet = DriveLaps(WithCompound(C, 1), 1.1, 3, nullptr, nullptr, &A);
	const LapResult Slk = DriveLaps(C, 1.1, 3, nullptr, nullptr, &B);
	KS_LOG("soaked @1.1 g: W5 laps %d best %.3f spins %d off %.1f m | slick laps %d best %.3f spins %d off %.1f m",
	       Wet.LapsDone, Wet.BestLap, Wet.Spins, Wet.MaxCrossTrack, Slk.LapsDone, Slk.BestLap, Slk.Spins, Slk.MaxCrossTrack);
	KS_EXPECT(Wet.LapsDone == 3 && Wet.Spins == 0);
	KS_EXPECT(Slk.Spins > 0 || Slk.MaxCrossTrack > Wet.MaxCrossTrack + 1.0);
	KS_LOG("tyre temps (mean RL): W5 %.1f C, slick %.1f C (water cooling)", Wet.MeanTemp[2], Slk.MeanTemp[2]);
}

KS_TEST(Tyre_WetsOverheatAndShredOnADryTrack)
{
	const KartConfig C = kstest::LoadRotax();
	const LapResult Slk = DriveLaps(C, 1.4, 8, nullptr);
	const LapResult Wet = DriveLaps(WithCompound(C, 1), 1.4, 8, nullptr);
	double WS = 0, WW = 0;
	for (int W = 0; W < 4; ++W) { WS += Slk.Wear[W] * C.RearTyre.TreadDepth; WW += Wet.Wear[W] * 5.0; }
	KS_LOG("dry, 8 laps @1.4 g: slick tread used %.3f mm (mean of 4), max T %.1f C | W5 tread used %.3f mm, max T %.1f C",
	       WS / 4, Slk.MaxTemp[0], WW / 4, Wet.MaxTemp[0]);
	KS_EXPECT(Slk.LapsDone == 8 && Wet.LapsDone == 8);
	KS_EXPECT(Wet.MaxTemp[0] > C.FrontTyre.OptimumTemp - 10.0); // the wet runs hot relative to ITS window...
	KS_EXPECT(WW > 3.0 * WS);                                    // ...and loses tread several times faster
}

KS_TEST(Tyre_WornTyresCostLapTime)
{
	const KartConfig C = kstest::LoadRotax();
	// At a target near the limit, a worn set (85 % of the tread used) runs wider / slower than a new set.
	const LapResult New = DriveLaps(C, 1.6, 4, nullptr, nullptr, nullptr, 0.0);
	const LapResult Old = DriveLaps(C, 1.6, 4, nullptr, nullptr, nullptr, 0.85);
	KS_LOG("@1.6 g: new set best %.3f s spins %d off %.1f m | worn (85 %%) best %.3f s spins %d off %.1f m",
	       New.BestLap, New.Spins, New.MaxCrossTrack, Old.BestLap, Old.Spins, Old.MaxCrossTrack);
	KS_EXPECT(New.Spins == 0);
	KS_EXPECT(Old.Spins > 0 || Old.MaxCrossTrack > New.MaxCrossTrack + 0.3 || Old.BestLap > New.BestLap + 0.05);
}

KS_TEST(Tyre_WearSurvivesKartReset)
{
	const KartConfig C = kstest::LoadRotax();
	KartSimulation Sim;
	Sim.Initialize(C);
	Sim.SetTyreWear(2, 0.4);
	Sim.Reset(Vec3(), 0.0);
	KS_EXPECT_NEAR(Sim.GetTyreModel(2).GetWear(), 0.4, 1e-12);
	Sim.Initialize(C); // a new set
	KS_EXPECT_NEAR(Sim.GetTyreModel(2).GetWear(), 0.0, 1e-12);
}

KS_TEST(Tyre_SlickLifeEstimate)
{
	// Calibration record (ESTIMATE): the slick wear rate was set so that a D5 set at autopilot pace has used
	// ~2/3 of its tread after ~1 h of running (forum reports: 60-80 good laps of ~50 s). Pinned loosely.
	const KartConfig C = kstest::LoadRotax();
	const LapResult R = DriveLaps(C, 1.4, 10, nullptr);
	const double Driven = Sum(R.LapTimes, 0);
	double MaxWear = 0;
	for (int W = 0; W < 4; ++W) { MaxWear = R.Wear[W] > MaxWear ? R.Wear[W] : MaxWear; }
	const double HourWear = MaxWear * 3600.0 / Driven;
	KS_LOG("10 laps (%.0f s): max wear %.4f -> %.2f per hour of running (%.2f mm of %.1f mm)", Driven, MaxWear, HourWear, HourWear * 3.5, 3.5);
	KS_EXPECT(HourWear > 0.4 && HourWear < 1.0);
}

KS_TEST(Tyre_AiPicksTheRightSetAndScalesPace)
{
	KartConfig C = kstest::LoadRotax();
	KS_EXPECT(ChooseTyreCompound(C, 0.0) == 0);
	KS_EXPECT(ChooseTyreCompound(C, 1.0) == 1);
	double Cross = 0.0;
	for (double W = 0.0; W <= 1.0; W += 0.01) { if (ChooseTyreCompound(C, W) == 1) { Cross = W; break; } }
	KS_EXPECT_NEAR(CompoundGripRatio(C, 0.0), 1.0, 1e-12);
	const double SlickSoaked = CompoundGripRatio(C, 1.0);
	C.FitTyres(1);
	const double WetSoaked = CompoundGripRatio(C, 1.0), WetDry = CompoundGripRatio(C, 0.0);
	KS_LOG("wets beat slicks above wetness %.2f; grip ratio soaked: slick %.2f, W5 %.2f; W5 on dry %.2f -> AI 1.4 g becomes %.2f / %.2f g",
	       Cross, SlickSoaked, WetSoaked, WetDry, 1.4 * SlickSoaked, 1.4 * WetSoaked);
	KS_EXPECT(Cross > 0.2 && Cross < 0.6);
	KS_EXPECT(WetSoaked > SlickSoaked && WetDry < 1.0);
}
