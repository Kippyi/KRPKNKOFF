// Milestone 3: per-class validation (100cc direct drive, 125cc Rotax Max, 125cc KZ2).
#include "DriverBot.h"
#include "TestFramework.h"
#include "TestHelpers.h"

using namespace kartsim;
using kstest::kTick;

KS_TEST(Classes_AllConfigsLoad)
{
	for (const char* F : {"Rotax125MaxSenior.json", "KZ2_IAMEScreamer.json", "KT100_DirectDrive.json"})
	{
		const KartConfig C = kstest::LoadKart(F);
		KS_LOG("%-40s mass %.0f kg, %d gear(s), clutch %d, front brakes %s, onboard starter %s",
		       C.Name.c_str(), C.Chassis.Mass, C.Gearbox.NumGears, static_cast<int>(C.Clutch.Type),
		       C.Brakes.FrontMaxTorque > 0 ? "yes" : "no", C.Engine.StarterTorque > 0 ? "yes" : "no");
		KS_EXPECT(!C.ClassId.empty());
	}
}

KS_TEST(KZ2_GearboxMatchesHomologationForm)
{
	// CIK homologation form 22/M/24 lists, per gear, the secondary-shaft angle after three engine
	// revolutions (1080 deg through primary 19/75 and the gear pair). Recompute from the config.
	const KartConfig C = kstest::LoadKart("KZ2_IAMEScreamer.json");
	const double FormDeg[6] = {111.2, 151.0, 182.4, 222.9, 261.7, 296.4};
	KS_EXPECT(C.Gearbox.NumGears == 6);
	for (int G = 0; G < 6 && G < C.Gearbox.NumGears; ++G)
	{
		const double Deg = 1080.0 / (C.Gearbox.PrimaryRatio * C.Gearbox.Ratios[G]);
		KS_LOG("gear %d: %.1f deg (form %.1f)", G + 1, Deg, FormDeg[G]);
		KS_EXPECT_NEAR(Deg, FormDeg[G], 0.06);
	}
}

KS_TEST(KZ2_StallsWhenClutchDroppedAtStandstill)
{
	const KartConfig C = kstest::LoadKart("KZ2_IAMEScreamer.json");
	KartSimulation Sim;
	Sim.Initialize(C);
	PlaneTrack Ground;
	KartInputs In;
	In.Clutch = 1.0;
	kstest::Run(Sim, Ground, In, 0.5);
	KS_EXPECT(Sim.GetEngine().IsRunning());
	In.ShiftRequest = 1;                       // select 1st with the clutch in
	Sim.Step(kTick, In, Ground);
	In.ShiftRequest = 0;
	kstest::Run(Sim, Ground, In, 0.2);
	KS_EXPECT(Sim.GetTransmission().GetGear() == 1);
	In.Clutch = 0.0;                           // drop the clutch, no throttle
	kstest::Run(Sim, Ground, In, 1.5);
	KS_LOG("after dropping the clutch at idle: running %d, rpm %.0f, speed %.2f m/s",
	       Sim.GetEngine().IsRunning() ? 1 : 0, Sim.GetEngine().GetRpm(), Sim.ForwardSpeed());
	KS_EXPECT(!Sim.GetEngine().IsRunning());
}

KS_TEST(KZ2_LaunchAndShiftThroughGears)
{
	const KartConfig C = kstest::LoadKart("KZ2_IAMEScreamer.json");
	KartSimulation Sim;
	Sim.Initialize(C);
	PlaneTrack Ground;
	kstest::DriverAids Aids;
	Aids.Configure(C);
	KartInputs In;
	kstest::Run(Sim, Ground, In, 0.5);
	double T100 = -1.0, MaxSlip = 0.0;
	int MaxGear = 0;
	for (int I = 0; I < static_cast<int>(20.0 / kTick); ++I)
	{
		In.Throttle = 1.0;
		Aids.Apply(Sim, In);
		Sim.Step(kTick, In, Ground);
		const TelemetryFrame& F = Sim.GetTelemetry();
		MaxGear = static_cast<int>(F.Gear) > MaxGear ? static_cast<int>(F.Gear) : MaxGear;
		MaxSlip = std::fabs(F.SideslipDeg) > MaxSlip ? std::fabs(F.SideslipDeg) : MaxSlip;
		if (T100 < 0.0 && F.SpeedKmh >= 100.0) { T100 = (I + 1) * kTick; }
	}
	KS_LOG("KZ2: 0-100 km/h %.2f s, top after 20 s %.1f km/h, max gear %d, running %d",
	       T100, Sim.GetTelemetry().SpeedKmh, MaxGear, Sim.GetEngine().IsRunning() ? 1 : 0);
	KS_EXPECT(MaxGear == 6);
	KS_EXPECT(T100 > 0.0 && T100 < 10.0);      // must out-accelerate the Rotax (10.0 s in the model)
	KS_EXPECT(Sim.GetEngine().IsRunning());
	KS_EXPECT(MaxSlip < 10.0);
}

KS_TEST(KT100_StallsAtRestAndPushStarts)
{
	const KartConfig C = kstest::LoadKart("KT100_DirectDrive.json");
	KartSimulation Sim;
	Sim.Initialize(C);
	PlaneTrack Ground;
	KartInputs In;
	kstest::Run(Sim, Ground, In, 0.5);
	KS_EXPECT(!Sim.GetEngine().IsRunning());   // direct drive at rest: engine stopped
	In.PushForce = 300.0;                       // helper push
	double FireSpeed = -1.0;
	for (int I = 0; I < static_cast<int>(6.0 / kTick) && FireSpeed < 0.0; ++I)
	{
		Sim.Step(kTick, In, Ground);
		if (Sim.GetEngine().IsRunning()) { FireSpeed = Sim.ForwardSpeed(); }
	}
	KS_LOG("push start: engine fired at %.2f m/s (%.0f rpm)", FireSpeed, Sim.GetEngine().GetRpm());
	KS_EXPECT(FireSpeed > 0.0 && FireSpeed < 4.0);
	// Drive away. A direct-drive engine fires at ~1,600 rpm and must pull through its weak low-rpm
	// torque (ESTIMATE), so assert the physics (pulls through to high rpm) not a guessed time.
	In.PushForce = 0.0;
	In.Throttle = 1.0;
	double MaxRpm = 0.0;
	for (int I = 0; I < static_cast<int>(15.0 / kTick); ++I)
	{
		Sim.Step(kTick, In, Ground);
		MaxRpm = Sim.GetEngine().GetRpm() > MaxRpm ? Sim.GetEngine().GetRpm() : MaxRpm;
	}
	const double DriveSpeed = Sim.GetTelemetry().SpeedKmh;
	In.Throttle = 0.0;
	In.Brake = 1.0;
	kstest::Run(Sim, Ground, In, 6.0);
	KS_LOG("after 15 s WOT: %.1f km/h, peak %.0f rpm; after braking to rest: running %d", DriveSpeed, MaxRpm, Sim.GetEngine().IsRunning() ? 1 : 0);
	KS_EXPECT(MaxRpm > 10000.0);
	KS_EXPECT(Sim.GetEngine().GetRpm() < 1.0);
	KS_EXPECT(!Sim.GetEngine().IsRunning());   // stalls when the kart stops
}

KS_TEST(Classes_DriverBotLapsCleanly)
{
	for (const char* F : {"KZ2_IAMEScreamer.json", "KT100_DirectDrive.json"})
	{
		const KartConfig C = kstest::LoadKart(F);
		const LapResult R = DriveLaps(C, 1.4, 3, nullptr);
		KS_LOG("%-36s laps %d best %.2f s spins %d max ay %.2f g max sideslip %.1f top %.1f km/h",
		       C.Name.c_str(), R.LapsDone, R.BestLap, R.Spins, R.MaxAy, R.MaxSideslip, R.TopSpeed);
		KS_EXPECT(R.LapsDone == 3);
		KS_EXPECT(R.Spins == 0);
		KS_EXPECT(R.MaxCrossTrack < 4.0);
	}
}

KS_TEST(KZ2_AutoShiftNeverOverRevsOnDownshift)
{
	// Regression: during a spin, wheel hop made engine rpm collapse and the auto-shift aid went
	// 4-3-2-1 at 67 km/h. Downshifts must be judged on road speed and never over-rev the engine.
	const KartConfig C = kstest::LoadKart("KZ2_IAMEScreamer.json");
	KartSimulation Sim;
	Sim.Initialize(C);
	PlaneTrack Ground;
	kstest::DriverAids Aids;
	Aids.Configure(C);
	KartInputs In;
	double WorstRoadRpm = 0.0;
	int Downshifts = 0, PrevGear = 0;
	for (int I = 0; I < static_cast<int>(25.0 / kTick); ++I)
	{
		const double T = I * kTick;
		In.Throttle = T < 12.0 ? 1.0 : 0.0;
		In.Brake = T >= 12.0 ? 0.6 : 0.0;
		// Mimic wheel hop at speed: engine rpm collapses momentarily (as in the logged spin).
		if (T > 12.5 && T < 12.6) { Sim.MutableEngine().SetOmega(8000.0 * kRpmToRadS); }
		Aids.Apply(Sim, In);
		Sim.Step(kTick, In, Ground);
		const int G = Sim.GetTransmission().GetGear();
		if (G >= 1 && G < PrevGear)
		{
			++Downshifts;
			const double RoadRpm = std::fabs(Sim.ForwardSpeed()) / C.RearTyre.UnloadedRadius * C.TotalRatio(G - 1) * kRadSToRpm;
			WorstRoadRpm = RoadRpm > WorstRoadRpm ? RoadRpm : WorstRoadRpm;
		}
		PrevGear = G;
	}
	KS_LOG("downshifts %d, worst road-speed rpm right after a downshift %.0f (limit %.0f)", Downshifts, WorstRoadRpm, C.Engine.RevLimitRpm);
	KS_EXPECT(Downshifts >= 3);
	KS_EXPECT(WorstRoadRpm < C.Engine.RevLimitRpm);
}