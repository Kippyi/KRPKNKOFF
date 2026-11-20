// Open-loop handling maneuvers reproducing how a driver (keyboard or wheel) actually uses the kart.
// Run: KartSimTests --maneuvers
#include "TestHelpers.h"
#include "TestTrack.h"
#include "DriverBot.h"
#include "KartSim/Autopilot.h"
#include <chrono>
#include <cstdlib>

#include <cstdio>

using namespace kartsim;
using kstest::kTick;

namespace
{
	struct ManeuverResult
	{
		double MaxSideslip = 0, MaxYawRate = 0, MaxAy = 0, FinalSpeed = 0, MinFzRL = 1e9;
		bool bSpun = false;
		double LamF = 0, LamR = 0;
	};

	// Starts at Speed, then ramps the column to ColumnDeg at RateDegPerSec and holds it.
	// ThrottleMode: <0 = speed controller, otherwise fixed throttle. Brake fixed.
	ManeuverResult StepSteer(const KartConfig& C, double SpeedKmh, double ColumnDeg, double RateDegPerSec,
	                         double ThrottleMode, double Brake, double HoldSeconds, const char* CsvName = nullptr)
	{
		KartSimulation Sim;
		Sim.Initialize(C);
		PlaneTrack Ground;
		kstest::Run(Sim, Ground, KartInputs(), 0.5);
		kstest::SetRolling(Sim, SpeedKmh / 3.6);
		CsvTelemetryWriter Csv;
		if (CsvName) { Csv.Open(std::string("Saved/SimRuns/") + CsvName + ".csv"); }
		kstest::SpeedController Ctl;
		KartInputs In;
		double Col = 0.0;
		ManeuverResult R;
		const int N = static_cast<int>((HoldSeconds + ColumnDeg / RateDegPerSec) / kTick);
		for (int I = 0; I < N; ++I)
		{
			Col = Col + RateDegPerSec * kTick < ColumnDeg ? Col + RateDegPerSec * kTick : ColumnDeg;
			In.SteeringColumnAngle = Col * kDegToRad;
			In.Throttle = ThrottleMode < 0 ? Ctl.Throttle(SpeedKmh / 3.6, Sim.ForwardSpeed(), kTick) : ThrottleMode;
			In.Brake = Brake;
			Sim.Step(kTick, In, Ground);
			const TelemetryFrame& F = Sim.GetTelemetry();
			if (CsvName) { Csv.Write(F); }
			R.MaxSideslip = std::fabs(F.SideslipDeg) > R.MaxSideslip ? std::fabs(F.SideslipDeg) : R.MaxSideslip;
			R.MaxYawRate = std::fabs(F.YawRateDeg) > R.MaxYawRate ? std::fabs(F.YawRateDeg) : R.MaxYawRate;
			R.MaxAy = std::fabs(F.AccLatG) > R.MaxAy ? std::fabs(F.AccLatG) : R.MaxAy;
			R.MinFzRL = F.FzRL < R.MinFzRL ? F.FzRL : R.MinFzRL;
			if (std::fabs(F.SideslipDeg) > 20.0) { R.bSpun = true; }
			R.LamF = 0.5 * (F.CombinedSlipFL + F.CombinedSlipFR);
			R.LamR = 0.5 * (F.CombinedSlipRL + F.CombinedSlipRR);
		}
		R.FinalSpeed = Sim.GetTelemetry().SpeedKmh;
		return R;
	}

	void Print(const char* Label, const ManeuverResult& R)
	{
		std::printf("  %-44s %s  sideslip %5.1f  yaw %6.1f deg/s  ay %4.2f g  v_end %5.1f  minFzRL %4.0f  lamF %4.2f lamR %4.2f\n",
		            Label, R.bSpun ? "SPIN" : "ok  ", R.MaxSideslip, R.MaxYawRate, R.MaxAy, R.FinalSpeed, R.MinFzRL, R.LamF, R.LamR);
	}
}

// ---------------------------------------------------------------- closed-loop driver on the test circuit


// A modulating "competent driver": pure-pursuit steering with a rate limit, speed profile from a
// target lateral acceleration, braking that stays below rear lock. This is NOT the AI of M6; it is a
// repeatable drivability probe.
void BuildTestConditions(TrackConditions& Out)
{
	Out.Build(kstest::BuildTestTrack().P);
}

LapResult DriveLaps(const KartConfig& C, double TargetLatG, int Laps, const char* CsvName, LapTimer* Timer, TrackConditions* Conditions, double InitialWear,
                    const std::function<void(const KartSimulation&, double Time)>& OnStep)
{
	const kstest::TrackLine T = kstest::BuildTestTrack();
	KartSimulation Sim;
	Sim.Initialize(C);
	if (kstest::SubstepOverride() > 0) { Sim.Settings.Substeps = kstest::SubstepOverride(); }
	PlaneTrack Plane;
	TrackConditions NoConditions; // unbuilt: grip multiplier stays 1
	ConditionsTrackQuery CondQuery(Plane, Conditions ? *Conditions : NoConditions);
	const ITrackQuery& Ground = Conditions ? static_cast<const ITrackQuery&>(CondQuery) : static_cast<const ITrackQuery&>(Plane);
	double WeatherClock = 0.0;

	const size_t Start = 6;
	const Vec3 Dir = (T.P[Start + 1] - T.P[Start]).Normalized();
	Sim.Reset(T.P[Start], std::atan2(Dir.Y, Dir.X));
	for (int W = 0; W < NumWheels; ++W) { Sim.SetTyreWear(W, InitialWear); }
	CsvTelemetryWriter Csv;
	if (CsvName) { Csv.Open(std::string("Saved/SimRuns/") + CsvName + ".csv"); }

	// The driver is the shared core Autopilot (identical logic in game: -KartAutopilot / key O).
	Autopilot Pilot;
	Pilot.TargetLatG = TargetLatG;
	Pilot.SetPath(T.P);
	Pilot.Reset(Sim);
	double LapStart = 0.0, Travel = 0.0, LiftTime = 0.0, Time = 0.0;
	double PrevS = T.S[Pilot.PathIndex()];
	bool bInSpin = false;
	char WorstInfo[256] = "";
	kstest::DriverAids Aids;
	Aids.Configure(C);
	LapResult R;
	KartInputs In;
	const int MaxSteps = static_cast<int>(Laps * 120.0 / kTick);
	for (int Step = 0; Step < MaxSteps && R.LapsDone < Laps; ++Step)
	{
		const ChassisState& S = Sim.GetChassis();
		const Vec3 Left = S.Orientation.Rotate(Vec3(0, 1, 0));
		Pilot.Drive(Sim, kTick, In);
		const size_t Idx = Pilot.PathIndex();
		if (Pilot.CrossTrackError() > R.MaxCrossTrack)
		{
			R.MaxCrossTrack = Pilot.CrossTrackError();
			const TelemetryFrame& Fd = Sim.GetTelemetry();
			const Vec3 Off = S.Position - T.P[Idx];
			const double Side = Off.Dot(Left) > 0 ? 1.0 : -1.0; // + = left of the line
			const double Outside = (T.Curv[Idx] > 0 ? -1.0 : 1.0) * Side; // + = toward the outside of the bend
			std::snprintf(WorstInfo, sizeof(WorstInfo), "    off-line %.1f m (%s) at s=%.0f m, R=%.0f m, %.1f km/h, ay %.2f, lamF %.2f lamR %.2f, col %.1f, thr %.2f brk %.2f\n",
			              R.MaxCrossTrack, Outside > 0 ? "OUTSIDE" : "inside", T.S[Idx], 1.0 / (std::fabs(T.Curv[Idx]) + 1e-9), Fd.SpeedKmh, Fd.AccLatG,
			              0.5 * (Fd.CombinedSlipFL + Fd.CombinedSlipFR), 0.5 * (Fd.CombinedSlipRL + Fd.CombinedSlipRR), Fd.SteerColumnDeg, Fd.Throttle, Fd.Brake);
		}
		Aids.Apply(Sim, In);
		const Vec3 PrevPos = Sim.GetChassis().Position;
		if (Conditions) { Sim.Settings.TrackTemp = Conditions->Weather.TrackTemp; Sim.Settings.AmbientTemp = Conditions->Weather.AirTemp; }
		Sim.Step(kTick, In, Ground);
		Time += kTick;
		if (Conditions)
		{
			Conditions->ApplyKart(Sim, kTick);
			WeatherClock += kTick;
			if (WeatherClock >= 0.1) { Conditions->Advance(WeatherClock); WeatherClock = 0.0; }
		}
		if (Timer) { Timer->Update(Time, kTick, PrevPos, Sim.GetChassis().Position, false); }
		if (OnStep) { OnStep(Sim, Time); }

		const TelemetryFrame& F = Sim.GetTelemetry();
		if (CsvName) { Csv.Write(F); }
		R.MaxAy = std::fabs(F.AccLatG) > R.MaxAy ? std::fabs(F.AccLatG) : R.MaxAy;
		R.TopSpeed = F.SpeedKmh > R.TopSpeed ? F.SpeedKmh : R.TopSpeed;
		R.MaxSideslip = std::fabs(F.SideslipDeg) > R.MaxSideslip ? std::fabs(F.SideslipDeg) : R.MaxSideslip;
		const bool bSpinNow = std::fabs(F.SideslipDeg) > 25.0;
		if (bSpinNow && !bInSpin) { ++R.Spins; }
		bInSpin = bSpinNow;
		if (F.LiftRL > 0.5 || F.LiftRR > 0.5) { LiftTime += kTick; }
		for (int W = 0; W < 4; ++W)
		{
			const double Tw = Sim.GetTyreModel(W).GetTemperature();
			R.MeanTemp[W] += Tw * kTick;
			R.MaxTemp[W] = Tw > R.MaxTemp[W] ? Tw : R.MaxTemp[W];
			R.SlideEnergy[W] += Sim.GetWheel(W).Tyre.SlidePower * kTick;
		}

		// Lap counting by arc length wrap.
		const double SNow = T.S[Pilot.PathIndex()];
		Travel += (SNow - PrevS < -0.5 * T.Length) ? SNow - PrevS + T.Length : (SNow - PrevS > 0.5 * T.Length ? 0.0 : SNow - PrevS);
		PrevS = SNow;
		if (Travel >= T.Length)
		{
			Travel -= T.Length;
			const double Lap = Time - LapStart;
			if (R.LapsDone > 0 && (R.BestLap == 0.0 || Lap < R.BestLap)) { R.BestLap = Lap; } // lap 1 = standing start
			LapStart = Time;
			++R.LapsDone;
			R.LapTimes.push_back(Lap);
		}
		if (R.MaxCrossTrack > 30.0) { break; } // left the circuit entirely
	}
	R.LiftFraction = Time > 0 ? LiftTime / Time : 0.0;
	for (int W = 0; W < 4; ++W) { R.Wear[W] = Sim.GetTyreModel(W).GetWear(); R.MeanTemp[W] = Time > 0 ? R.MeanTemp[W] / Time : 0.0; }
	if (CsvName && R.MaxCrossTrack > 3.0) { std::printf("    worst excursion: %s\n", WorstInfo); }
	return R;
}

// Physics-rate sweep: the same autopilot laps at several inner rates. Shows convergence (lap time and
// peak values must stop changing as the rate rises) and the CPU cost per kart.
int RunRateSweep()
{
	std::printf("PHYSICS RATE SWEEP (240 Hz outer tick, autopilot 1.4 g, 3 laps)\n");
	for (const char* File : {"Rotax125MaxSenior.json", "KZ2_IAMEScreamer.json"})
	{
		const KartConfig C = kstest::LoadKart(File);
		std::printf(" %s\n", C.Name.c_str());
		for (int N : {4, 8, 12, 16, 32})
		{
			kstest::SubstepOverride() = N;
			const auto T0 = std::chrono::steady_clock::now();
			const LapResult R = DriveLaps(C, std::getenv("KART_SWEEPG") ? std::atof(std::getenv("KART_SWEEPG")) : 1.4, 3, nullptr);
			const double Wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - T0).count();
			double SimTime = 0.0;
			for (double L : R.LapTimes) { SimTime += L; }
			const double UsPerInner = SimTime > 0 ? Wall / (SimTime * 240.0 * N) * 1e6 : 0.0;
			std::printf("  %5d Hz (x%2d): best %7.3f s  spins %d  max ay %4.2f g  max sideslip %5.1f  rear lift %4.1f%%  | %.2f us/substep, %.3f ms per 1 s of one kart\n",
			            240 * N, N, R.BestLap, R.Spins, R.MaxAy, R.MaxSideslip, R.LiftFraction * 100.0, UsPerInner, UsPerInner * 240.0 * N / 1000.0);
		}
	}
	kstest::SubstepOverride() = 0;
	return 0;
}

int RunDriverLaps()
{
	std::printf("CLOSED-LOOP DRIVER on the test circuit (%.0f m), 3 laps\n", kstest::BuildTestTrack().Length);
	for (const char* File : {"Rotax125MaxSenior.json", "KZ2_IAMEScreamer.json", "KT100_DirectDrive.json"})
	{
	const KartConfig C = kstest::LoadKart(File);
	std::printf(" %s\n", C.Name.c_str());
	for (double Gt : {1.0, 1.4, 1.8})
	{
		char Name[96];
		std::snprintf(Name, sizeof(Name), "laps_%s_%03.0f", C.ClassId.c_str(), Gt * 100.0);
		const LapResult R = DriveLaps(C, Gt, 3, Name);
		std::printf("  target %.1f g: laps %d  best %6.2f s  spins %d  max ay %4.2f g  max sideslip %5.1f  max off-line %5.1f m  top %5.1f km/h  rear lift %4.1f%%\n",
		            Gt, R.LapsDone, R.BestLap, R.Spins, R.MaxAy, R.MaxSideslip, R.MaxCrossTrack, R.TopSpeed, R.LiftFraction * 100.0);
	}
	}
	return 0;
}

// ---------------------------------------------------------------- per-class performance comparison

int RunClassComparison()
{
	std::printf("CLASS COMPARISON (flat asphalt, driver aids for clutch/shift/push)\n");
	std::printf("  %-36s %7s %7s %7s %9s %13s %13s\n", "class", "0-60", "0-100", "top30s", "rpm@top", "brake100 lock", "brake100 best");
	for (const char* File : {"KT100_DirectDrive.json", "Rotax125MaxSenior.json", "KZ2_IAMEScreamer.json"})
	{
		const KartConfig C = kstest::LoadKart(File);
		kstest::DriverAids Aids;
		Aids.Configure(C);

		// Launch / top speed: full throttle 30 s from rest (push start for direct drive).
		KartSimulation Sim;
		Sim.Initialize(C);
		PlaneTrack Ground;
		kstest::Run(Sim, Ground, KartInputs(), 0.5);
		KartInputs In;
		double T60 = -1, T100 = -1, VMax = 0, Rpm = 0, Clock = 0;
		bool bStarted = false;
		for (int I = 0; I < static_cast<int>(40.0 / kTick); ++I)
		{
			In.Throttle = 1.0;
			Aids.Apply(Sim, In);
			Sim.Step(kTick, In, Ground);
			const TelemetryFrame& F = Sim.GetTelemetry();
			if (!bStarted && Sim.GetEngine().IsRunning() && In.PushForce == 0.0) { bStarted = true; Clock = 0.0; }
			if (!bStarted) { continue; }
			Clock += kTick;
			if (T60 < 0 && F.SpeedKmh >= 60) { T60 = Clock; }
			if (T100 < 0 && F.SpeedKmh >= 100) { T100 = Clock; }
			if (F.SpeedKmh > VMax) { VMax = F.SpeedKmh; Rpm = F.EngineRpm; }
			if (Clock >= 30.0) { break; }
		}

		// Braking 100 -> 0 km/h: locked (pedal 1.0) and best threshold pedal (sweep).
		auto StopDistance = [&C](double Pedal)
		{
			KartSimulation B;
			B.Initialize(C);
			PlaneTrack G;
			kstest::Run(B, G, KartInputs(), 0.5);
			kstest::SetRolling(B, 100.0 / 3.6);
			KartInputs Br;
			Br.Brake = Pedal;
			Br.Clutch = 1.0; // manual clutch: declutch while braking
			const Vec3 Start = B.GetChassis().Position;
			for (int I = 0; I < static_cast<int>(15.0 / kTick) && B.ForwardSpeed() > 0.3; ++I) { B.Step(kTick, Br, G); }
			return (B.GetChassis().Position - Start).Length();
		};
		const double DLock = StopDistance(1.0);
		double DBest = 1e9, PBest = 0;
		for (double P = 0.2; P <= 1.0001; P += 0.05)
		{
			const double D = StopDistance(P);
			if (D < DBest) { DBest = D; PBest = P; }
		}
		std::printf("  %-36s %6.2fs %6.2fs %5.1fkmh %8.0f %10.1f m %8.1f m @%.2f\n",
		            C.Name.c_str(), T60, T100, VMax, Rpm, DLock, DBest, PBest);
	}
	return 0;
}

int RunManeuvers()
{
	const KartConfig C = kstest::LoadRotax();
	char Label[128];
	std::printf("STEP STEER at 50 km/h, keyboard-like ramp (250 deg/s column), held 3 s\n");
	for (double Col : {10.0, 20.0, 25.0, 30.0, 35.0, 40.0, 65.0})
	{
		std::snprintf(Label, sizeof(Label), "col %4.0f deg, speed held", Col);
		Print(Label, StepSteer(C, 50, Col, 250, -1, 0, 3));
		std::snprintf(Label, sizeof(Label), "col %4.0f deg, full throttle", Col);
		Print(Label, StepSteer(C, 50, Col, 250, 1.0, 0, 3));
		std::snprintf(Label, sizeof(Label), "col %4.0f deg, throttle off", Col);
		Print(Label, StepSteer(C, 50, Col, 250, 0.0, 0, 3));
	}
	std::printf("BRAKING with small steer at 60 km/h\n");
	for (double Brk : {0.3, 0.5, 1.0})
	{
		std::snprintf(Label, sizeof(Label), "brake %.1f, col 10 deg", Brk);
		Print(Label, StepSteer(C, 60, 10, 250, 0.0, Brk, 2.5));
	}
	std::printf("STRAIGHT full throttle 3..80 km/h with tiny steer 3 deg\n");
	Print("col 3 deg, WOT", StepSteer(C, 3, 3, 250, 1.0, 0, 8, "man_straight"));
	StepSteer(C, 50, 40, 250, 1.0, 0, 3, "man_step40_wot");
	StepSteer(C, 50, 40, 250, -1, 0, 3, "man_step40_held");
	return 0;
}
