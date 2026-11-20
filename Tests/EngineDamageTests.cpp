// Engine tuning (jetting, air, cooling, seizure) and mechanical damage (Milestone 8).
// The coefficients are ESTIMATES / ASSUMPTIONS (see EngineTune.h, KartDamage.h); these tests pin the BEHAVIOUR
// and the few published anchors (jet steps vs temperature, water windows).
#include "DriverBot.h"
#include "TestFramework.h"
#include "TestHelpers.h"

#include "KartSim/EngineTune.h"
#include "KartSim/KartContact.h"
#include "KartSim/KartPhysicsCore.h"
#include "KartSim/Setup.h"

#include <cmath>
#include <memory>
#include <string>

using namespace kartsim;
using kstest::kTick;

namespace
{
	AirState Air(double T, double P = 101.325, double H = 0.5) { AirState A; A.TempC = T; A.PressureKPa = P; A.Humidity = H; return A; }

	struct LapSummary
	{
		LapResult R;
		double WaterEnd = 0.0, WaterMax = 0.0, EgtMax = 0.0, PistonMax = 0.0, MaxFzFront = 0.0, MaxFzRear = 0.0;
		bool bSeized = false, bDamaged = false;
		double SeizedAt = -1.0;
	};

	LapSummary Drive(const KartConfig& C, int Laps)
	{
		LapSummary S;
		S.R = DriveLaps(C, 1.4, Laps, nullptr, nullptr, nullptr, 0.0, [&S](const KartSimulation& Sim, double T)
		{
			const TelemetryFrame& F = Sim.GetTelemetry();
			S.WaterEnd = F.WaterTemp;
			S.WaterMax = std::fmax(S.WaterMax, F.WaterTemp);
			S.EgtMax = std::fmax(S.EgtMax, F.Egt);
			S.PistonMax = std::fmax(S.PistonMax, F.PistonTemp);
			S.MaxFzFront = std::fmax(S.MaxFzFront, std::fmax(F.FzFL, F.FzFR));
			S.MaxFzRear = std::fmax(S.MaxFzRear, std::fmax(F.FzRL, F.FzRR));
			if (Sim.GetEngineCondition().IsSeized() && !S.bSeized) { S.bSeized = true; S.SeizedAt = T; }
			S.bDamaged = S.bDamaged || Sim.GetDamage().Any();
		});
		return S;
	}
}

KS_TEST(Tune_AirDensityAndIsoCorrection)
{
	// ISA sea level, dry: 1.225 kg/m^3.
	KS_EXPECT_NEAR(MoistAirDensity(Air(15.0, 101.325, 0.0)), 1.225, 0.002);
	// Humid air is lighter and carries less oxygen.
	KS_EXPECT(DryAirDensity(Air(30.0, 101.325, 0.9)) < DryAirDensity(Air(30.0, 101.325, 0.1)));
	// ISO 1585 reference (dry 99 kPa, 298 K) gives exactly the rated power.
	const double Pv = 0.5 * SaturationVapourPressureKPa(25.0);
	KS_EXPECT_NEAR(IsoPowerCorrection(Air(24.85, 99.0 + Pv, 0.5)), 1.0, 1e-3);
	// Hot, high and humid costs power; cold dense air makes more.
	KS_EXPECT(IsoPowerCorrection(Air(35.0, 95.0, 0.8)) < 0.95);
	KS_EXPECT(IsoPowerCorrection(Air(8.0, 102.0, 0.5)) > 1.04);
}

KS_TEST(Tune_BestJetFollowsPublishedTemperatureSteps)
{
	const KartConfig Rx = kstest::LoadRotax();
	// RADNE Rotax EVO table (third party): 122 at >= 18 degC, 125 below 18, 128 below 10.
	const double J22 = BestJet(Rx.Tuning, Air(22.0)), J14 = BestJet(Rx.Tuning, Air(14.0)), J6 = BestJet(Rx.Tuning, Air(6.0));
	KS_LOG("Rotax best jet: 22 C %.1f, 14 C %.1f, 6 C %.1f; 35 C %.1f; 22 C at 900 m (91 kPa) %.1f", J22, J14, J6,
	       BestJet(Rx.Tuning, Air(35.0)), BestJet(Rx.Tuning, Air(22.0, 91.0)));
	KS_EXPECT_NEAR(J22, 122.0, 0.01);
	KS_EXPECT_NEAR(J14, 125.0, 1.5);
	KS_EXPECT_NEAR(J6, 128.0, 1.5);
	// IAME Shifter manual: +2-3 main-jet points per 6 degC drop.
	const KartConfig Kz = kstest::LoadKart("KZ2_IAMEScreamer.json");
	const double D = BestJet(Kz.Tuning, Air(16.0)) - BestJet(Kz.Tuning, Air(22.0));
	KS_LOG("KZ2 best jet 22 C %.1f, 16 C %.1f (+%.2f per 6 C)", BestJet(Kz.Tuning, Air(22.0)), BestJet(Kz.Tuning, Air(16.0)), D);
	KS_EXPECT(D >= 2.0 && D <= 3.0);
	// Auto jet snaps to a step on the rich side.
	EngineTuneParams T = Rx.Tuning;
	AutoJet(T, Air(14.0));
	KS_EXPECT(T.Jet >= J14 && T.Jet - J14 < 1.0 + 1e-9);
}

KS_TEST(Tune_MixturePowerCurve)
{
	KS_EXPECT_NEAR(MixturePowerFactor(1.0), 1.0, 1e-12);
	KS_EXPECT(MixturePowerFactor(0.96) > 1.0);           // a touch lean is fastest...
	KS_EXPECT(MixturePowerFactor(0.85) < 0.97);          // ...then it bogs
	KS_EXPECT(MixturePowerFactor(1.10) < 0.98);          // rich = flat
	KS_EXPECT(MixturePowerFactor(1.10) > MixturePowerFactor(0.86));
}

KS_TEST(Tune_SetupJetAndCoverRoundTrip)
{
	int N = 0;
	const SetupParam* P = GetSetupParams(N);
	int Jet = -1, Needle = -1, Cover = -1;
	for (int I = 0; I < N; ++I)
	{
		const std::string K = P[I].Key;
		if (K == "mainJet") { Jet = I; } else if (K == "carbNeedle") { Needle = I; } else if (K == "radiatorCover") { Cover = I; }
	}
	KS_EXPECT(Jet >= 0 && Needle >= 0 && Cover >= 0);
	KartConfig Rx = kstest::LoadRotax();
	SetSetupValue(Rx, Jet, 126.4);
	SetSetupValue(Rx, Cover, 40.0);
	KS_EXPECT_NEAR(Rx.Tuning.Jet, 126.0, 1e-9);
	KS_EXPECT_NEAR(Rx.Tuning.Cover, 0.40, 1e-9);
	SetSetupValue(Rx, Jet, 300.0);                       // clamped to the class range
	KS_EXPECT_NEAR(Rx.Tuning.Jet, Rx.Tuning.JetMax, 1e-9);
	KartConfig D = kstest::LoadRotax();
	std::string Err;
	KS_EXPECT(ApplySetupJson(SerializeSetup(Rx), D, Err));
	KS_EXPECT_NEAR(D.Tuning.Jet, Rx.Tuning.Jet, 1e-9);
	KS_EXPECT_NEAR(D.Tuning.Cover, 0.40, 1e-9);
	// KT100: needle turns, no radiator.
	KartConfig Kt = kstest::LoadKart("KT100_DirectDrive.json");
	KS_EXPECT(!P[Jet].IsAvailable(Kt) && P[Needle].IsAvailable(Kt) && !P[Cover].IsAvailable(Kt));
	SetSetupValue(Kt, Needle, 2.70);
	KS_EXPECT_NEAR(Kt.Tuning.Jet, 2.6875, 1e-9);         // 1/16-turn steps
}

KS_TEST(Engine_TemperaturesSitInTheirWindows)
{
	for (const char* F : {"Rotax125MaxSenior.json", "KZ2_IAMEScreamer.json", "KT100_DirectDrive.json"})
	{
		const KartConfig C = kstest::LoadKart(F);
		const LapSummary S = Drive(C, 3);
		KS_LOG("%s: water/CHT end %.1f max %.1f C (window %.0f-%.0f), EGT max %.0f C, piston max %.0f / limit %.0f, lap %.3f s, max Fz front %.0f rear %.0f N",
		       F, S.WaterEnd, S.WaterMax, C.Tuning.OptimumLow, C.Tuning.OptimumHigh, S.EgtMax, S.PistonMax, C.Tuning.PistonLimit,
		       S.R.BestLap, S.MaxFzFront, S.MaxFzRear);
		KS_EXPECT(S.R.LapsDone == 3);
		KS_EXPECT(!S.bSeized);
		// At 22 degC with no radiator cover the water runs a little under its window (cover it on cold days).
		KS_EXPECT(S.WaterEnd > C.Tuning.OptimumLow - 15.0 && S.WaterEnd < C.Tuning.OptimumHigh + 5.0);
		KS_EXPECT(S.PistonMax < C.Tuning.PistonLimit);
		KS_EXPECT(S.EgtMax > C.Tuning.EgtFull - 120.0 && S.EgtMax < C.Tuning.EgtFull + 80.0);
	}
}

KS_TEST(Engine_RadiatorCoverRaisesWaterTemp)
{
	KartConfig C = kstest::LoadRotax();
	const LapSummary Open = Drive(C, 3);
	C.Tuning.Cover = 0.2;
	const LapSummary Taped = Drive(C, 3);
	C.Tuning.Cover = 0.9;
	const LapSummary Blanked = Drive(C, 4);
	KS_LOG("water end: open %.1f, 20%% covered %.1f, 90%% covered %.1f C (seized %d at %.1f s)", Open.WaterEnd, Taped.WaterEnd,
	       Blanked.WaterEnd, Blanked.bSeized ? 1 : 0, Blanked.SeizedAt);
	KS_EXPECT(Taped.WaterEnd > Open.WaterEnd + 5.0);
	KS_EXPECT(Taped.WaterEnd >= C.Tuning.OptimumLow - 3.0 && Taped.WaterEnd <= C.Tuning.OptimumHigh + 8.0);
	KS_EXPECT(Blanked.WaterMax > 80.0);                  // blanked off it boils over
}

KS_TEST(Engine_LeanJetIsFastThenSeizesRichIsSafe)
{
	KartConfig C = kstest::LoadRotax();
	const LapSummary Best = Drive(C, 3);
	C.Tuning.Jet = 120.0;                                // 2 points lean (r ~ 0.97): fast, hot
	const LapSummary Edge = Drive(C, 3);
	C.Tuning.Jet = 106.0;                                // ~25 % lean: seizes on the first lap
	const LapSummary Lean = Drive(C, 3);
	C.Tuning.Jet = 130.0;                                // ~14 % rich: safe, slow
	const LapSummary Rich = Drive(C, 3);
	KS_LOG("jet 122 %.3f s (piston %.0f), 120 %.3f s (piston %.0f), 130 %.3f s (piston %.0f), 106 seized %d at %.1f s (piston %.0f)",
	       Best.R.BestLap, Best.PistonMax, Edge.R.BestLap, Edge.PistonMax, Rich.R.BestLap, Rich.PistonMax,
	       Lean.bSeized ? 1 : 0, Lean.SeizedAt, Lean.PistonMax);
	KS_EXPECT(!Best.bSeized && !Edge.bSeized && !Rich.bSeized);
	KS_EXPECT(Edge.R.BestLap <= Best.R.BestLap + 0.005);
	KS_EXPECT(Edge.PistonMax > Best.PistonMax);
	KS_EXPECT(Rich.R.BestLap > Best.R.BestLap + 0.05);
	KS_EXPECT(Lean.bSeized);
	KS_EXPECT(Lean.R.LapsDone < 3);
}

KS_TEST(Engine_SeizureLocksAndRepairRestores)
{
	KartConfig C = kstest::LoadRotax();
	C.Tuning.Jet = 100.0; // set directly (bypasses the setup clamp): hopelessly lean
	KartSimulation Sim;
	Sim.Initialize(C);
	PlaneTrack G;
	Sim.Reset(Vec3(0, 0, 0), 0.0);
	kstest::SetRolling(Sim, 15.0);
	KartInputs In; In.Throttle = 1.0;
	double MinAxle = 1e9;
	bool bSeized = false;
	// Accelerate / brake repeatedly (the engine works through its power band, like laps).
	bool bBraking = false;
	for (int I = 0; I < 240 * 60 && !bSeized; ++I)
	{
		const double V = Sim.ForwardSpeed();
		if (V > 27.0) { bBraking = true; } else if (V < 17.0) { bBraking = false; }
		In.Throttle = bBraking ? 0.0 : 1.0;
		In.Brake = bBraking ? 1.0 : 0.0;
		Sim.Step(kTick, In, G);
		bSeized = Sim.GetEngineCondition().IsSeized();
	}
	In.Brake = 0.0; In.Throttle = 1.0;
	KS_LOG("lean straight-line runs: seized %d after %.1f s, piston %.0f / limit %.0f, water %.1f", bSeized ? 1 : 0, Sim.GetTime(),
	       Sim.GetEngineCondition().GetPistonTemp(), Sim.GetEngineCondition().GetPistonLimit(), Sim.GetEngineCondition().GetNodeTemp());
	KS_EXPECT(bSeized);
	KS_EXPECT(Sim.GetDamage().LastEvent == "ENGINE SEIZED");
	const double V0 = Sim.ForwardSpeed();
	for (int I = 0; I < 120; ++I) { Sim.Step(kTick, In, G); MinAxle = std::fmin(MinAxle, Sim.GetTelemetry().SlipRatioRL); }
	KS_LOG("seized at %.1f m/s; rear slip ratio min %.2f over the next 0.5 s; engine %.0f rpm", V0, MinAxle, Sim.GetEngine().GetRpm());
	KS_EXPECT(MinAxle < -0.1);                           // the locked engine drags the rear tyres (clutch still locked)
	KS_EXPECT(!Sim.GetEngine().IsRunning());
	Sim.Reset(Vec3(0, 0, 0), 0.0);
	KS_EXPECT(Sim.GetEngineCondition().IsSeized());      // a reset does not fix it...
	Sim.Repair();
	Sim.Reset(Vec3(0, 0, 0), 0.0);
	KS_EXPECT(!Sim.GetEngineCondition().IsSeized() && Sim.GetEngine().IsRunning()); // ...a repair does
}

KS_TEST(Damage_CleanLapsDoNoDamage)
{
	for (const char* F : {"Rotax125MaxSenior.json", "KZ2_IAMEScreamer.json", "KT100_DirectDrive.json"})
	{
		const KartConfig C = kstest::LoadKart(F);
		const LapSummary S = Drive(C, 3);
		KS_LOG("%s: max Fz front %.0f N (bend at %.0f), rear %.0f N (bend at %.0f)", F, S.MaxFzFront, C.Damage.WheelLoadFront,
		       S.MaxFzRear, C.Damage.WheelLoadRear);
		KS_EXPECT(!S.bDamaged);
		KS_EXPECT(S.MaxFzFront < 0.8 * C.Damage.WheelLoadFront && S.MaxFzRear < 0.8 * C.Damage.WheelLoadRear);
	}
}

namespace
{
	// A rams B from behind on flat ground at Closing m/s (both on idle, clutch open). Returns B's damage.
	KartDamageState RearHit(double Closing, KartDamageState* OutA = nullptr)
	{
		const KartConfig C = kstest::LoadRotax();
		KartSimulation A, B;
		A.Initialize(C);
		B.Initialize(C);
		PlaneTrack G;
		A.Reset(Vec3(0, 0, 0), 0.0);
		B.Reset(Vec3(1.9, 0, 0), 0.0);
		kstest::Run(A, G, KartInputs(), 0.3);
		kstest::Run(B, G, KartInputs(), 0.3);
		A.MutableChassis().Velocity = Vec3(Closing, 0, 0);
		A.SetFrontWheelOmega(Closing / C.FrontTyre.UnloadedRadius);
		A.SetRearAxleOmega(Closing / C.RearTyre.UnloadedRadius);
		const KartFootprint F = KartFootprint::FromConfig(C);
		const ContactParams P;
		std::vector<KartSimulation*> Sims = {&A, &B};
		const std::vector<KartFootprint> Feet = {F, F};
		for (int I = 0; I < 240; ++I)
		{
			ApplyKartContacts(Sims, Feet, P);
			A.Step(kTick, KartInputs(), G);
			B.Step(kTick, KartInputs(), G);
		}
		if (OutA) { *OutA = A.GetDamage(); }
		return B.GetDamage();
	}
}

KS_TEST(Damage_ContactSeverityScales)
{
	KartDamageState A1, A3, A6;
	const KartDamageState B1 = RearHit(1.0, &A1), B3 = RearHit(3.0, &A3), B6 = RearHit(9.0, &A6);
	KS_LOG("rear-ended at 1 m/s: B axle %.2f deg chain %d | A fairing %d toe %.2f/%.2f", B1.AxleBendDeg, B1.bChainOff ? 1 : 0, A1.bFairingDisplaced ? 1 : 0, A1.ToeDeg[0], A1.ToeDeg[1]);
	KS_LOG("rear-ended at 3 m/s: B axle %.2f deg chain %d | A fairing %d toe %.2f/%.2f", B3.AxleBendDeg, B3.bChainOff ? 1 : 0, A3.bFairingDisplaced ? 1 : 0, A3.ToeDeg[0], A3.ToeDeg[1]);
	KS_LOG("rear-ended at 9 m/s: B axle %.2f deg chain %d | A fairing %d toe %.2f/%.2f", B6.AxleBendDeg, B6.bChainOff ? 1 : 0, A6.bFairingDisplaced ? 1 : 0, A6.ToeDeg[0], A6.ToeDeg[1]);
	// A gentle nudge (bump drafting) does nothing; a push displaces the nose; a big hit bends and throws the chain.
	KS_EXPECT(!B1.Any() || (!B1.bChainOff && B1.AxleBendDeg == 0.0));
	KS_EXPECT(!A1.bFairingDisplaced);
	KS_EXPECT(A3.bFairingDisplaced);
	KS_EXPECT(B6.AxleBendDeg > B3.AxleBendDeg && B6.AxleBendDeg > 0.3);
	// A nose hit releases the fairing but does not bend the hitter's steering (the user: "hit someone slightly
	// and my whole steering is gone").
	KS_EXPECT(A3.ToeDeg[0] == 0.0 && A3.ToeDeg[1] == 0.0);
	KS_EXPECT(B6.bChainOff);
	KS_EXPECT(!B3.bChainOff);
}

KS_TEST(Damage_EffectsOnTheKart)
{
	const KartConfig C = kstest::LoadRotax();
	PlaneTrack G;
	// Bent left steering arm (toe-out 1.5 deg): the kart pulls when coasting straight with the wheel centred.
	auto CoastYaw = [&](const KartDamageState& D)
	{
		KartSimulation S;
		S.Initialize(C);
		S.Reset(Vec3(0, 0, 0), 0.0);
		S.SetDamage(D);
		kstest::SetRolling(S, 20.0);
		kstest::Run(S, G, KartInputs(), 2.0);
		return S.GetChassis().Position.Y;
	};
	KartDamageState Toe; Toe.ToeDeg[0] = -1.5;
	const double Y0 = CoastYaw(KartDamageState()), Y1 = CoastYaw(Toe);
	KS_LOG("lateral drift over 2 s coasting at 20 m/s: straight %.3f m, left arm bent %.3f m", Y0, Y1);
	KS_EXPECT(std::fabs(Y0) < 0.05 && std::fabs(Y1) > 0.2);

	// Thrown chain: full throttle does not drive the kart; a bent axle costs coast-down speed.
	KartSimulation S;
	S.Initialize(C);
	S.Reset(Vec3(0, 0, 0), 0.0);
	KartDamageState Chain; Chain.bChainOff = true;
	S.SetDamage(Chain);
	S.Reset(Vec3(0, 0, 0), 0.0);
	KartInputs Full; Full.Throttle = 1.0;
	kstest::Run(S, G, Full, 3.0);
	KS_LOG("chain off: %.2f m/s after 3 s full throttle, engine %.0f rpm", S.ForwardSpeed(), S.GetEngine().GetRpm());
	KS_EXPECT(S.ForwardSpeed() < 0.2 && S.GetEngine().GetRpm() > 10000.0);
	auto Coast = [&](double Bend)
	{
		KartSimulation K;
		K.Initialize(C);
		K.Reset(Vec3(0, 0, 0), 0.0);
		KartDamageState D; D.AxleBendDeg = Bend;
		K.SetDamage(D);
		kstest::SetRolling(K, 20.0);
		kstest::Run(K, G, KartInputs(), 4.0);
		return K.ForwardSpeed();
	};
	const double V0 = Coast(0.0), V2 = Coast(2.0);
	KS_LOG("coast 20 m/s for 4 s: straight axle %.2f m/s, 2 deg bend %.2f m/s", V0, V2);
	KS_EXPECT(V2 < V0 - 0.5);

	// Holed radiator: the coolant runs out and the engine overheats.
	KartSimulation R;
	R.Initialize(C);
	R.Reset(Vec3(0, 0, 0), 0.0);
	KartDamageState Leak; Leak.RadiatorLeak = 0.02;
	R.SetDamage(Leak);
	kstest::SetRolling(R, 25.0);
	KartInputs Part; Part.Throttle = 0.7;
	double MaxW = 0.0;
	for (int I = 0; I < 240 * 90; ++I) { R.Step(kTick, Part, G); MaxW = std::fmax(MaxW, R.GetTelemetry().WaterTemp); }
	KS_LOG("radiator leaking 2 %%/s: coolant %.0f %% after 90 s, water max %.0f C, seized %d", R.GetEngineCondition().GetCoolant() * 100.0, MaxW,
	       R.GetEngineCondition().IsSeized() ? 1 : 0);
	KS_EXPECT(R.GetEngineCondition().GetCoolant() < 0.05 && MaxW > 75.0);

	// Repair clears everything.
	S.Repair();
	KS_EXPECT(!S.GetDamage().Any() && !S.GetTransmission().IsChainOff());
}

namespace
{
	struct TwoKarts
	{
		KartConfig C = kstest::LoadRotax();
		KartSimulation A, B;
		PlaneTrack G;
		std::vector<KartSimulation*> Sims;
		std::vector<KartFootprint> Feet;
		ContactParams P;
		TwoKarts()
		{
			A.Initialize(C);
			B.Initialize(C);
			Sims = {&A, &B};
			Feet = {KartFootprint::FromConfig(C), KartFootprint::FromConfig(C)};
		}
		void Roll(KartSimulation& K, const Vec3& Pos, double Yaw, double V)
		{
			K.Reset(Pos, Yaw);
			kstest::Run(K, G, KartInputs(), 0.2);
			K.MutableChassis().Velocity = Vec3(std::cos(Yaw) * V, std::sin(Yaw) * V, 0.0);
			K.SetFrontWheelOmega(V / C.FrontTyre.UnloadedRadius);
			K.SetRearAxleOmega(V / C.RearTyre.UnloadedRadius);
			const double EngineW = V / C.RearTyre.UnloadedRadius * K.GetTransmission().GetTotalRatio();
			if (EngineW > C.Engine.IdleRpm * kRpmToRadS) { K.MutableEngine().SetOmega(EngineW); }
		}
		void Step(const KartInputs& InA, const KartInputs& InB)
		{
			ApplyKartContacts(Sims, Feet, P);
			A.Step(kTick, InA, G);
			B.Step(kTick, InB, G);
		}
	};
	double HeightAboveGround(const KartSimulation& K) { return K.GetChassis().Position.Z - K.GetConfig().Chassis.CogHeight; }
	double TiltDeg(const KartSimulation& K) { return std::acos(Clamp(K.GetChassis().Orientation.Rotate(Vec3(0, 0, 1)).Z, -1.0, 1.0)) * kRadToDeg; }
}

KS_TEST(Contact_WheelToWheelLaunchesTheFollower)
{
	// The classic launch: the follower's right-front tyre runs into the outer part of the left-rear tyre of the kart
	// ahead (outside its rear bumper). Closing speeds 3 / 6 / 9 m/s at ~20 m/s.
	double Lift[3] = {};
	const double Closing[3] = {3.0, 6.0, 9.0};
	for (int Case = 0; Case < 3; ++Case)
	{
		TwoKarts K;
		const double FrontX = K.C.Chassis.Wheelbase * (1.0 - K.C.Chassis.FrontWeightFraction);
		const double RearX = K.C.Chassis.Wheelbase * K.C.Chassis.FrontWeightFraction;
		const double Gap = K.C.FrontTyre.UnloadedRadius + K.C.RearTyre.UnloadedRadius + 0.05;
		const double Dy = -0.5 * K.C.Chassis.FrontTrack - 0.5 * K.C.Chassis.RearTrack - 0.06;
		K.Roll(K.A, Vec3(0, 0, 0), 0.0, 17.0 + Closing[Case]);
		K.Roll(K.B, Vec3(FrontX + Gap + RearX, Dy, 0), 0.0, 17.0);
		KartInputs Full; Full.Throttle = 1.0;
		double MaxLift = 0.0, MaxTilt = 0.0, MaxFr = 0.0;
		for (int I = 0; I < 240 * 3; ++I)
		{
			K.Step(Full, Full);
			MaxLift = std::fmax(MaxLift, HeightAboveGround(K.A));
			MaxTilt = std::fmax(MaxTilt, TiltDeg(K.A));
			MaxFr = std::fmax(MaxFr, K.A.GetWheel(WheelFR).CentreWorld.Z - K.C.FrontTyre.UnloadedRadius);
		}
		const Vec3 Pa = K.A.GetChassis().Position;
		KS_LOG("wheel-to-wheel at %.0f m/s closing: follower FR wheel up %.2f m, CoG up %.2f m, max tilt %.0f deg; after 3 s tilt %.0f deg, z %.2f",
		       Closing[Case], MaxFr, MaxLift, MaxTilt, TiltDeg(K.A), Pa.Z);
		KS_EXPECT(std::isfinite(Pa.X) && std::isfinite(Pa.Z) && Pa.Z < 5.0);
		Lift[Case] = MaxFr;
	}
	KS_EXPECT(Lift[2] > Lift[0]);
	KS_EXPECT(Lift[2] > 0.15); // a big closing speed throws the wheel up
}

KS_TEST(Contact_TboneShovesTheVictim)
{
	// T-bone at 9 m/s into a stopped kart's sidepod (a 30 cm wall to a 25 cm wheel: it shoves, it does not climb),
	// then a launch case: the follower lands and the simulation stays sane.
	TwoKarts K;
	K.Roll(K.B, Vec3(3.0, 0, 0), kPi * 0.5, 0.0);
	K.Roll(K.A, Vec3(0, 0, 0), 0.0, 9.0);
	double MaxLift = 0.0, MaxTilt = 0.0;
	for (int I = 0; I < 240 * 3; ++I)
	{
		K.Step(KartInputs(), KartInputs());
		MaxLift = std::fmax(MaxLift, HeightAboveGround(K.A));
		MaxTilt = std::fmax(MaxTilt, TiltDeg(K.A));
	}
	KS_LOG("T-bone at 9 m/s: hitter lifted %.2f m, max tilt %.0f deg, ends at x %.2f tilt %.0f; victim moved %.2f m",
	       MaxLift, MaxTilt, K.A.GetChassis().Position.X, TiltDeg(K.A), (K.B.GetChassis().Position - Vec3(3.0, 0, K.B.GetChassis().Position.Z)).Length());
	KS_EXPECT(std::isfinite(K.A.GetChassis().Position.Z) && MaxTilt < 60.0);
	KS_EXPECT((K.B.GetChassis().Position - Vec3(3.0, 0, K.B.GetChassis().Position.Z)).Length() > 1.0); // the victim is shoved
}

KS_TEST(Rollover_KartOnItsSideComesToRest)
{
	// Dropped on its side and upside down: the body points hold it on the ground, nothing explodes.
	for (double RollDeg : {95.0, 180.0})
	{
		const KartConfig C = kstest::LoadRotax();
		KartSimulation S;
		S.Initialize(C);
		PlaneTrack G;
		S.Reset(Vec3(0, 0, 0), 0.0);
		S.MutableChassis().Position = Vec3(0, 0, 1.3);
		S.MutableChassis().Orientation = Quat::FromAxisAngle(Vec3(1, 0, 0), RollDeg * kDegToRad);
		S.MutableChassis().Velocity = Vec3(4.0, 0, 0);
		double MinZ = 1e9;
		for (int I = 0; I < 240 * 4; ++I) { S.Step(kTick, KartInputs(), G); MinZ = std::fmin(MinZ, S.GetChassis().Position.Z); }
		const ChassisState& St = S.GetChassis();
		KS_LOG("dropped at %.0f deg roll: rests at CoG %.2f m (min %.2f), tilt %.0f deg, speed %.2f m/s", RollDeg, St.Position.Z, MinZ, TiltDeg(S), St.Velocity.Length());
		KS_EXPECT(std::isfinite(St.Position.Z) && St.Position.Z > 0.05 && St.Position.Z < 1.0);
		KS_EXPECT(St.Velocity.Length() < 0.5);
		KS_EXPECT(TiltDeg(S) > 45.0); // still on its side / roof: it does not magically right itself
	}
}

KS_TEST(Damage_RearHitMatrix)
{
	// Diagnostic (user: "bodywork wasn't moving"): a follower hits the kart ahead at several closing speeds and
	// lateral offsets; prints whether the fairing releases and the peak contact forces.
	for (double Off : {0.0, 0.3, 0.6, 0.9, 1.15})
	{
		for (double Closing : {1.5, 3.0, 5.0})
		{
			TwoKarts K;
			const double FrontX = K.C.Chassis.Wheelbase * (1.0 - K.C.Chassis.FrontWeightFraction);
			const double RearX = K.C.Chassis.Wheelbase * K.C.Chassis.FrontWeightFraction;
			K.Roll(K.A, Vec3(0, 0, 0), 0.0, 15.0 + Closing);
			K.Roll(K.B, Vec3(FrontX + 0.45 + RearX + 0.45, -Off, 0), 0.0, 15.0);
			KartInputs Full; Full.Throttle = 1.0;
			double MaxExt = 0.0, MaxLift = 0.0, PeakFz[8] = {};
			for (int I = 0; I < 240 * 2; ++I)
			{
				K.Step(Full, Full);
				MaxExt = std::fmax(MaxExt, K.A.GetExternalForce().Length());
				MaxLift = std::fmax(MaxLift, K.A.GetWheel(WheelFR).CentreWorld.Z - K.C.FrontTyre.UnloadedRadius);
				for (int W = 0; W < 4; ++W) { PeakFz[W] = std::fmax(PeakFz[W], K.A.GetWheel(W).Tyre.Fz); PeakFz[4 + W] = std::fmax(PeakFz[4 + W], K.B.GetWheel(W).Tyre.Fz); }
			}
			KS_LOG("   peak Fz A %.0f %.0f %.0f %.0f | B %.0f %.0f %.0f %.0f; A axle %.2f chain %d toe %.2f/%.2f", PeakFz[0], PeakFz[1], PeakFz[2], PeakFz[3], PeakFz[4], PeakFz[5], PeakFz[6], PeakFz[7],
			       K.A.GetDamage().AxleBendDeg, K.A.GetDamage().bChainOff ? 1 : 0, K.A.GetDamage().ToeDeg[0], K.A.GetDamage().ToeDeg[1]);
			const KartDamageState& D = K.A.GetDamage();
			KS_LOG("offset %.2f m, closing %.1f m/s: follower fairing %d toe %.2f/%.2f, peak contact %.0f N, FR wheel up %.2f m | leader axle %.2f chain %d bumper %d",
			       Off, Closing, D.bFairingDisplaced ? 1 : 0, D.ToeDeg[0], D.ToeDeg[1], MaxExt, MaxLift, K.B.GetDamage().AxleBendDeg, K.B.GetDamage().bChainOff ? 1 : 0,
			       K.B.GetDamage().bBumperKnocked ? 1 : 0);
			if (Closing >= 3.0 && Off < 0.5) { KS_EXPECT(D.bFairingDisplaced && K.B.GetDamage().bBumperKnocked); }
		}
	}
}

KS_TEST(Damage_SideSwipeKnocksSidepod)
{
	// Side by side, one kart steers into the other at 3 m/s lateral: the struck sidepod is knocked loose.
	TwoKarts K;
	K.Roll(K.A, Vec3(0, 0, 0), 0.0, 15.0);
	K.Roll(K.B, Vec3(0, -1.50, 0), 0.0, 15.0);
	K.A.MutableChassis().Velocity = Vec3(15.0, -3.0, 0.0);
	for (int I = 0; I < 240; ++I) { K.Step(KartInputs(), KartInputs()); }
	KS_LOG("side swipe at 3 m/s: A sidepods %d/%d, B sidepods %d/%d", K.A.GetDamage().bSidepodKnocked[0] ? 1 : 0, K.A.GetDamage().bSidepodKnocked[1] ? 1 : 0,
	       K.B.GetDamage().bSidepodKnocked[0] ? 1 : 0, K.B.GetDamage().bSidepodKnocked[1] ? 1 : 0);
	KS_EXPECT(K.A.GetDamage().bSidepodKnocked[1] && K.B.GetDamage().bSidepodKnocked[0]); // A's right touches B's left
}

KS_TEST(Damage_PileUpAtTurnOne)
{
	// User: "chain pops off too much". A five-kart train at 20 m/s, 0.4 m apart nose to tail; the leader brakes hard,
	// each follower 0.25-0.4 s late (late reactions in a pack). Ordinary braking-zone contact: bodywork goes, some
	// bending, but the pack should not all lose their chains.
	const KartConfig C = kstest::LoadRotax();
	const int N = 5;
	std::vector<std::unique_ptr<KartSimulation>> K;
	std::vector<KartSimulation*> Sims;
	std::vector<KartFootprint> Feet;
	PlaneTrack G;
	const double Len = C.Chassis.Wheelbase + C.FrontTyre.UnloadedRadius + C.RearTyre.UnloadedRadius + 0.45;
	const double Lat[N] = {0.0, 0.25, -0.35, 0.5, -0.1};
	for (int I = 0; I < N; ++I)
	{
		K.push_back(std::make_unique<KartSimulation>());
		K[I]->Initialize(C);
		K[I]->Reset(Vec3(-I * Len, Lat[I], 0), 0.0);
		kstest::Run(*K[I], G, KartInputs(), 0.2);
		K[I]->MutableChassis().Velocity = Vec3(20.0, 0, 0);
		K[I]->SetFrontWheelOmega(20.0 / C.FrontTyre.UnloadedRadius);
		K[I]->SetRearAxleOmega(20.0 / C.RearTyre.UnloadedRadius);
		K[I]->MutableEngine().SetOmega(20.0 / C.RearTyre.UnloadedRadius * K[I]->GetTransmission().GetTotalRatio());
		Sims.push_back(K[I].get());
		Feet.push_back(KartFootprint::FromConfig(C));
	}
	const ContactParams P;
	const double React[N] = {0.0, 0.3, 0.25, 0.4, 0.35};
	for (int Step = 0; Step < 240 * 4; ++Step)
	{
		ApplyKartContacts(Sims, Feet, P);
		for (int I = 0; I < N; ++I)
		{
			KartInputs In;
			const double T = Step * kTick;
			if (T >= React[I] * I) { In.Brake = 1.0; } else { In.Throttle = 1.0; }
			K[I]->Step(kTick, In, G);
		}
	}
	int Chains = 0;
	for (int I = 0; I < N; ++I)
	{
		const KartDamageState& D = K[I]->GetDamage();
		Chains += D.bChainOff ? 1 : 0;
		KS_LOG("pile-up kart %d: toe %.2f/%.2f axle %.2f chain %d fairing %d bumper %d sidepods %d/%d", I, D.ToeDeg[0], D.ToeDeg[1], D.AxleBendDeg, D.bChainOff ? 1 : 0,
		       D.bFairingDisplaced ? 1 : 0, D.bBumperKnocked ? 1 : 0, D.bSidepodKnocked[0] ? 1 : 0, D.bSidepodKnocked[1] ? 1 : 0);
		KS_EXPECT(std::fabs(D.ToeDeg[0]) <= C.Damage.MaxBendPerHit * 2.0 + 1e-9 && std::fabs(D.ToeDeg[1]) <= C.Damage.MaxBendPerHit * 2.0 + 1e-9);
	}
	KS_EXPECT(Chains <= 1);
}

KS_TEST(Contact_SideBySideBangingStaysPlanted)
{
	// User: wheel-to-wheel "still a bit iffy"; the game log showed karts with all four tyres off the ground for
	// single frames in contact (hopping). Side by side at 20 m/s, A steers into B at 1 / 2 / 4 m/s lateral, with
	// A's front wheels level with B's front, mid and rear wheels.
	for (double Stagger : {0.0, 0.55, 1.05})
	{
		for (double Vy : {1.0, 2.0, 4.0})
		{
			TwoKarts K;
			K.Roll(K.A, Vec3(Stagger, 0, 0), 0.0, 20.0);
			K.Roll(K.B, Vec3(0, -1.42, 0), 0.0, 20.0);
			K.A.MutableChassis().Velocity = Vec3(20.0, -Vy, 0.0);
			KartInputs Full; Full.Throttle = 1.0;
			int AirA = 0, AirB = 0, Lift4 = 0, Toggles = 0;
			bool bWas = false;
			double MaxVz = 0.0, MaxTilt = 0.0, MaxH = 0.0;
			for (int I = 0; I < 240 * 2; ++I)
			{
				const bool bNow = ApplyKartContacts(K.Sims, K.Feet, K.P, nullptr, kTick) > 0;
				if (bNow != bWas) { ++Toggles; }
				bWas = bNow;
				K.A.Step(kTick, Full, K.G);
				K.B.Step(kTick, Full, K.G);
				for (KartSimulation* S : {&K.A, &K.B})
				{
					int Off = 0;
					for (int W = 0; W < 4; ++W) { Off += S->GetWheel(W).bLifted ? 1 : 0; }
					if (Off == 4) { (S == &K.A ? AirA : AirB)++; }
					if (Off >= 2) { ++Lift4; }
					MaxVz = std::fmax(MaxVz, std::fabs(S->GetChassis().Velocity.Z));
					MaxTilt = std::fmax(MaxTilt, TiltDeg(*S));
					MaxH = std::fmax(MaxH, HeightAboveGround(*S));
				}
			}
			KS_LOG("stagger %.2f m, %.0f m/s in: contact on/off %d, all-4-off steps A %d B %d, >=2 off %d, max |vz| %.2f m/s, max tilt %.1f deg, max CoG rise %.3f m, lateral gap after %.2f m",
			       Stagger, Vy, Toggles, AirA, AirB, Lift4, MaxVz, MaxTilt, MaxH, K.A.GetChassis().Position.Y - K.B.GetChassis().Position.Y);
		}
	}
}

KS_TEST(Diag_SideBangTrace)
{
	if (!std::getenv("KS_SIDETRACE")) { return; }
	const double Stagger = std::atof(std::getenv("KS_SIDETRACE"));
	TwoKarts K;
	K.Roll(K.A, Vec3(Stagger, 0, 0), 0.0, 20.0);
	K.Roll(K.B, Vec3(0, -1.42, 0), 0.0, 20.0);
	K.A.MutableChassis().Velocity = Vec3(20.0, -4.0, 0.0);
	KartInputs Full; Full.Throttle = 1.0;
	for (int I = 0; I < 240 * 2; ++I)
	{
		std::vector<KartContactInfo> Cs;
		ApplyKartContacts(K.Sims, K.Feet, K.P, &Cs, kTick);
		const Vec3 Fa = K.A.GetExternalForce();
		K.A.Step(kTick, Full, K.G);
		K.B.Step(kTick, Full, K.G);
		if (I % 6 == 0 && I < 240)
		{
			const ChassisState& A = K.A.GetChassis();
			const ChassisState& B = K.B.GetChassis();
			int OffA = 0, OffB = 0;
			for (int W = 0; W < 4; ++W) { OffA += K.A.GetWheel(W).bLifted ? 1 : 0; OffB += K.B.GetWheel(W).bLifted ? 1 : 0; }
			double D = 0.0;
			for (const KartContactInfo& C : Cs) { D = std::fmax(D, C.Depth); }
			std::printf("      t %.3f dx %+.2f dy %+.3f zA %.3f zB %.3f tiltA %4.1f tiltB %4.1f vyA %+.2f vyB %+.2f yawA %+5.1f  FA %+6.0f %+6.0f %+6.0f boxes %zu depth %.3f off %d/%d\n",
			            I * kTick, A.Position.X - B.Position.X, A.Position.Y - B.Position.Y, A.Position.Z, B.Position.Z, TiltDeg(K.A), TiltDeg(K.B), A.Velocity.Y, B.Velocity.Y,
			            std::atan2(A.Orientation.Rotate(Vec3(1, 0, 0)).Y, A.Orientation.Rotate(Vec3(1, 0, 0)).X) * kRadToDeg, Fa.X, Fa.Y, Fa.Z, Cs.size(), D, OffA, OffB);
		}
	}
}
