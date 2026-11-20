// Standing starts (KZ clutch dump). The user saw start-line pile-ups in KZ races: dumping the clutch at revs gave
// ~4.3 g spikes and a 1 s wheelie. These tests reproduce the launch and pin what is physical.
#include "TestFramework.h"
#include "TestHelpers.h"

#include "KartSim/KartPhysicsCore.h"

#include <cmath>

using namespace kartsim;
using kstest::kTick;

namespace
{
	struct FLaunch
	{
		double PeakLongG = 0.0, PeakVertG = 0.0, MinVertG = 10.0, PeakPitchDeg = 0.0, FrontLiftTime = 0.0, PeakFzRear = 0.0, PeakFxRear = 0.0;
		double Speed1s = 0.0, PeakClutchNm = 0.0;
	};

	// KZ on the grid in first gear, engine held at LaunchRpm with the clutch in, then the clutch released over
	// ReleaseTime (0 = dumped) at full throttle; 1.5 s recorded.
	FLaunch KzLaunch(double LaunchRpm, double ReleaseTime, bool bPrint, int PrintEvery = 6)
	{
		const KartConfig C = kstest::LoadKart("KZ2_IAMEScreamer.json");
		KartSimulation Sim;
		Sim.Initialize(C);
		PlaneTrack Ground;
		KartInputs In;
		kstest::Run(Sim, Ground, In, 0.5);
		In.bStarter = true;
		In.Clutch = 1.0;
		kstest::Run(Sim, Ground, In, 1.0);
		In.bStarter = false;
		In.ShiftRequest = 1;
		Sim.Step(kTick, In, Ground);
		In.ShiftRequest = 0;
		// Hold the revs (throttle control) with the clutch in.
		for (int I = 0; I < static_cast<int>(1.5 / kTick); ++I)
		{
			In.Throttle = Clamp((LaunchRpm - Sim.GetEngine().GetRpm()) / 1500.0 + 0.3, 0.0, 1.0);
			Sim.Step(kTick, In, Ground);
		}
		FLaunch R;
		const double T0 = Sim.GetTime();
		In.Throttle = 1.0;
		for (int I = 0; I < static_cast<int>(1.5 / kTick); ++I)
		{
			const double T = Sim.GetTime() - T0;
			In.Clutch = ReleaseTime > 0.0 ? Clamp(1.0 - T / ReleaseTime, 0.0, 1.0) : 0.0;
			Sim.Step(kTick, In, Ground);
			const TelemetryFrame& F = Sim.GetTelemetry();
			R.PeakLongG = std::fmax(R.PeakLongG, F.AccLongG);
			R.PeakVertG = std::fmax(R.PeakVertG, F.AccVertG);
			R.MinVertG = std::fmin(R.MinVertG, F.AccVertG);
			R.PeakPitchDeg = std::fmax(R.PeakPitchDeg, std::fabs(F.PitchDeg));
			if (F.LiftFL > 0.5 && F.LiftFR > 0.5) { R.FrontLiftTime += kTick; }
			R.PeakFzRear = std::fmax(R.PeakFzRear, F.FzRL + F.FzRR);
			R.PeakFxRear = std::fmax(R.PeakFxRear, F.FxRL + F.FxRR);
			R.PeakClutchNm = std::fmax(R.PeakClutchNm, std::fabs(F.ClutchTorque));
			if (T < 1.0) { R.Speed1s = F.SpeedKmh; }
			if (bPrint && I % PrintEvery == 0 && T < 0.5)
			{
				const ChassisState& Ch = Sim.GetChassis();
				KS_LOG("  t=%.3f rpm %5.0f clNm %5.1f lk %d | ax %5.2f az %5.2f pitch %5.1f q %6.2f | FzR %5.0f FxR %5.0f muR %4.2f FzF %4.0f | kRL %5.2f | z %.3f vz %5.2f v %4.1f",
				       T, F.EngineRpm, F.ClutchTorque, static_cast<int>(F.ClutchLocked), F.AccLongG, F.AccVertG, F.PitchDeg, Ch.AngularVelocity.Y,
				       F.FzRL + F.FzRR, F.FxRL + F.FxRR, F.MuRL, F.FzFL + F.FzFR, F.SlipRatioRL, Ch.Position.Z, Ch.Velocity.Z, F.SpeedKmh);
			}
		}
		return R;
	}
}

KS_TEST(Launch_KzClutchDumpProbe)
{
	const FLaunch R = KzLaunch(11000.0, 0.0, true);
	KS_LOG("KZ dump at 11000 rpm: peak ax %.2f g, az %.2f..%.2f g, pitch %.1f deg, front lift %.2f s, peak Fz rear %.0f N, peak Fx rear %.0f N, clutch %.0f Nm, 1 s: %.1f km/h",
	       R.PeakLongG, R.MinVertG, R.PeakVertG, R.PeakPitchDeg, R.FrontLiftTime, R.PeakFzRear, R.PeakFxRear, R.PeakClutchNm, R.Speed1s);
}

KS_TEST(Launch_KzSweep)
{
	for (double Rpm : {8000.0, 11000.0, 13500.0})
	{
		for (double Rel : {0.0, 0.15, 0.4, 0.8})
		{
			const FLaunch R = KzLaunch(Rpm, Rel, false);
			KS_LOG("%5.0f rpm, release %.2f s: ax %.2f g, az %.2f..%.2f g, pitch %.1f deg, front lift %.2f s, Fz rear max %.0f N, 1 s: %.1f km/h",
			       Rpm, Rel, R.PeakLongG, R.MinVertG, R.PeakVertG, R.PeakPitchDeg, R.FrontLiftTime, R.PeakFzRear, R.Speed1s);
		}
	}
}

KS_TEST(Launch_KzWorstCaseTrace)
{
	const FLaunch R = KzLaunch(13500.0, 0.15, true, 3);
	KS_LOG("13500 rpm / 0.15 s: peak ax %.2f g, Fz rear %.0f N, Fx rear %.0f N", R.PeakLongG, R.PeakFzRear, R.PeakFxRear);
}

KS_TEST(Launch_KzLaunchControlIsCleanAndFast)
{
	// The shared driver aid (AI karts and the keyboard auto-clutch): full throttle from rest, gearbox from neutral.
	const KartConfig C = kstest::LoadKart("KZ2_IAMEScreamer.json");
	KartSimulation Sim;
	Sim.Initialize(C);
	PlaneTrack Ground;
	kstest::DriverAids Aids;
	Aids.Configure(C);
	KartInputs In;
	In.bStarter = true;
	kstest::Run(Sim, Ground, In, 1.0);
	In.bStarter = false;
	double LaunchAx = 0.0, LaunchLift = 0.0, ShiftAx = 0.0, ShiftLift = 0.0, T60 = -1.0, T100 = -1.0, MinRearFz = 1e9, MinRpm = 1e9;
	for (int I = 0; I < static_cast<int>(8.0 / kTick); ++I)
	{
		In.Throttle = 1.0;
		Aids.Apply(Sim, In);
		Sim.Step(kTick, In, Ground);
		const TelemetryFrame& F = Sim.GetTelemetry();
		const double T = (I + 1) * kTick;
		const bool bLaunch = F.Gear <= 1.5;
		const bool bLift = F.LiftFL > 0.5 && F.LiftFR > 0.5;
		(bLaunch ? LaunchAx : ShiftAx) = std::fmax(bLaunch ? LaunchAx : ShiftAx, F.AccLongG);
		if (bLift) { (bLaunch ? LaunchLift : ShiftLift) += kTick; }
		if (F.SpeedKmh > 3.0) { MinRearFz = std::fmin(MinRearFz, F.FzRL + F.FzRR); }
		if (F.SpeedKmh > 3.0 && bLaunch) { MinRpm = std::fmin(MinRpm, F.EngineRpm); }
		if (T60 < 0.0 && F.SpeedKmh >= 60.0) { T60 = T; }
		if (T100 < 0.0 && F.SpeedKmh >= 100.0) { T100 = T; }
	}
	KS_LOG("KZ launch control: 0-60 %.2f s, 0-100 %.2f s | first gear: peak ax %.2f g, front lift %.2f s, min rpm %.0f | upshifts: peak ax %.2f g, front lift %.2f s | min rear load %.0f N",
	       T60, T100, LaunchAx, LaunchLift, MinRpm, ShiftAx, ShiftLift, MinRearFz);
	KS_EXPECT(T100 > 0.0 && T100 < 3.97);   // faster than the old wheelie-and-bog start (real KZ: ~3-3.5 s, ESTIMATE)
	KS_EXPECT(LaunchLift < 0.05);           // the nose goes light, not up
	KS_EXPECT(LaunchAx < 1.6);
	KS_EXPECT(MinRpm > 4000.0);             // no bog
	KS_EXPECT(MinRearFz > 300.0);           // no power hop: the rear tyres never leave the ground
	KS_EXPECT(ShiftLift < 0.2);
}
