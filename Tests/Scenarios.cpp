// Repeatable vehicle-dynamics scenarios. Each writes a telemetry CSV and prints summary metrics.
// These are measurements of the model, not pass/fail tests: compare them against real logged data.
#include "TestHelpers.h"

#include <cstdio>
#include <string>

using namespace kartsim;
using kstest::kTick;

namespace
{
	std::string OutPath(const std::string& Dir, const char* Name) { return Dir + "/" + Name + ".csv"; }

	void ScenarioLaunch(const KartConfig& C, const std::string& Dir)
	{
		KartSimulation Sim;
		Sim.Initialize(C);
		PlaneTrack Ground;
		CsvTelemetryWriter Csv;
		Csv.Open(OutPath(Dir, "launch"));
		KartInputs In;
		kstest::Run(Sim, Ground, In, 1.0);
		In.Throttle = 1.0;
		double T60 = -1, T80 = -1, T100 = -1, VMax = 0, LockTime = -1;
		for (int I = 0; I < static_cast<int>(25.0 / kTick); ++I)
		{
			Sim.Step(kTick, In, Ground);
			const TelemetryFrame& F = Sim.GetTelemetry();
			Csv.Write(F);
			const double T = (I + 1) * kTick;
			if (T60 < 0 && F.SpeedKmh >= 60) { T60 = T; }
			if (T80 < 0 && F.SpeedKmh >= 80) { T80 = T; }
			if (T100 < 0 && F.SpeedKmh >= 100) { T100 = T; }
			if (LockTime < 0 && F.ClutchLocked > 0.5) { LockTime = T; }
			VMax = F.SpeedKmh > VMax ? F.SpeedKmh : VMax;
		}
		const TelemetryFrame& F = Sim.GetTelemetry();
		std::printf("LAUNCH (WOT from rest, flat asphalt)\n");
		std::printf("  clutch lock at %.2f s | 0-60 %.2f s | 0-80 %.2f s | 0-100 %.2f s\n", LockTime, T60, T80, T100);
		std::printf("  top speed %.1f km/h at %.0f rpm (limiter %s)\n", VMax, F.EngineRpm, F.RevLimiter > 0.5 ? "hit" : "not hit");
	}

	void ScenarioBraking(const KartConfig& C, const std::string& Dir)
	{
		KartSimulation Sim;
		Sim.Initialize(C);
		PlaneTrack Ground;
		CsvTelemetryWriter Csv;
		Csv.Open(OutPath(Dir, "braking"));
		KartInputs In;
		kstest::Run(Sim, Ground, In, 1.0);
		kstest::SetRolling(Sim, 90.0 / 3.6);
		kstest::Run(Sim, Ground, In, 0.3);
		for (double Pedal : {0.45, 1.0})
		{
			Sim.Reset(Vec3(), 0.0);
			kstest::Run(Sim, Ground, KartInputs(), 1.0);
			kstest::SetRolling(Sim, 90.0 / 3.6);
			In.Brake = Pedal;
			const Vec3 Start = Sim.GetChassis().Position;
			double PeakG = 0, LockedTime = 0, T = 0;
			while (Sim.ForwardSpeed() > 0.3 && T < 15.0)
			{
				Sim.Step(kTick, In, Ground);
				T += kTick;
				const TelemetryFrame& F = Sim.GetTelemetry();
				if (Pedal == 1.0) { Csv.Write(F); }
				PeakG = -F.AccLongG > PeakG ? -F.AccLongG : PeakG;
				if (F.RearAxleRpm == 0.0) { LockedTime += kTick; }
			}
			const double Dist = (Sim.GetChassis().Position - Start).Length();
			std::printf("BRAKING 90->0 km/h, pedal %.2f: %.1f m in %.2f s, peak %.2f g, mean %.2f g, rear locked %.2f s\n",
			            Pedal, Dist, T, PeakG, (90.0 / 3.6) / T / kGravity, LockedTime);
		}
	}

	void ScenarioSteadyStateRamp(const KartConfig& C, const std::string& Dir)
	{
		// Constant steering, slowly increasing speed: measures lateral g, slip angles, inside-rear lift.
		KartSimulation Sim;
		Sim.Initialize(C);
		PlaneTrack Ground;
		CsvTelemetryWriter Csv;
		Csv.Open(OutPath(Dir, "steady_ramp"));
		KartInputs In;
		kstest::Run(Sim, Ground, In, 1.0);
		kstest::SetRolling(Sim, 8.0);
		In.SteeringColumnAngle = 25.0 * kDegToRad;
		kstest::SpeedController Ctl;
		double Target = 8.0, MaxAy = 0, LiftAy = -1, LiftSpeed = -1;
		double MinRL = 1e9;
		for (int I = 0; I < static_cast<int>(40.0 / kTick); ++I)
		{
			Target += 0.4 * kTick; // 0.4 m/s per second ramp
			In.Throttle = Ctl.Throttle(Target, Sim.ForwardSpeed(), kTick);
			Sim.Step(kTick, In, Ground);
			const TelemetryFrame& F = Sim.GetTelemetry();
			Csv.Write(F);
			MaxAy = F.AccLatG > MaxAy ? F.AccLatG : MaxAy;
			MinRL = F.FzRL < MinRL ? F.FzRL : MinRL;
			if (LiftAy < 0 && F.LiftRL > 0.5) { LiftAy = F.AccLatG; LiftSpeed = F.SpeedKmh; }
			if (F.SideslipDeg > 25.0 || F.SideslipDeg < -25.0) { break; }
		}
		std::printf("STEADY-STATE RAMP (column 25 deg, +0.4 m/s^2)\n");
		std::printf("  max lateral %.2f g | min inside-rear Fz %.0f N | inside rear lift: %s",
		            MaxAy, MinRL, LiftAy > 0 ? "yes" : "no");
		if (LiftAy > 0) { std::printf(" at %.2f g, %.1f km/h", LiftAy, LiftSpeed); }
		std::printf("\n");
	}

	void ScenarioKerb(const KartConfig& C, const std::string& Dir)
	{
		KartSimulation Sim;
		Sim.Initialize(C);
		KerbStripTrack Track;
		Track.KerbStartX = 15.0;
		Track.KerbLength = 1.0;
		Track.KerbHeight = 0.03;
		CsvTelemetryWriter Csv;
		Csv.Open(OutPath(Dir, "kerb"));
		KartInputs In;
		kstest::Run(Sim, Track, In, 1.0);
		kstest::SetRolling(Sim, 15.0);
		double PeakFz = 0, PeakCol = 0;
		int Airborne = 0;
		for (int I = 0; I < static_cast<int>(2.5 / kTick); ++I)
		{
			Sim.Step(kTick, In, Track);
			const TelemetryFrame& F = Sim.GetTelemetry();
			Csv.Write(F);
			PeakFz = F.FzFL > PeakFz ? F.FzFL : PeakFz;
			PeakCol = std::fabs(F.ColumnTorque) > PeakCol ? std::fabs(F.ColumnTorque) : PeakCol;
			if (F.LiftFL > 0.5) { ++Airborne; }
		}
		std::printf("KERB STRIKE (30 mm half-sine x 1 m, 54 km/h, straight): peak FL Fz %.0f N, FL airborne %.3f s, peak |column torque| %.2f Nm\n",
		            PeakFz, Airborne * kTick, PeakCol);
	}
}

namespace
{
	// Rectangular 25 mm block (sharp-edged kerb) across the track between StartX and StartX+Length.
	class StepBlockTrack final : public ITrackQuery
	{
	public:
		double StartX = 15.0, Length = 1.0, Height = 0.025;
		bool QueryGround(const Vec3& O, double MaxUp, double MaxDown, GroundHit& Out) const override
		{
			const double Z = (O.X > StartX && O.X < StartX + Length) ? Height : 0.0;
			if (Z > O.Z + MaxUp || Z < O.Z - MaxDown) { return false; }
			Out.Point = Vec3(O.X, O.Y, Z);
			Out.Normal = Vec3(0, 0, 1);
			Out.Surface = Z > 0 ? ESurfaceType::Kerb : ESurfaceType::Asphalt;
			return true;
		}
	};

	void ScenarioSharpKerb(const KartConfig& C, const std::string& Dir)
	{
		for (int Mode = 0; Mode < 2; ++Mode)
		{
			TyreModel::bEnableEnveloping = Mode == 1;
			KartSimulation Sim;
			Sim.Initialize(C);
			StepBlockTrack Track;
			CsvTelemetryWriter Csv;
			Csv.Open(OutPath(Dir, Mode ? "sharpkerb_envelope" : "sharpkerb_single"));
			kstest::Run(Sim, Track, KartInputs(), 1.0);
			kstest::SetRolling(Sim, 15.0);
			double PeakF = 0, PeakR = 0, PeakVert = 0, MaxZ = 0, Air = 0;
			for (int I = 0; I < static_cast<int>(2.0 / kTick); ++I)
			{
				Sim.Step(kTick, KartInputs(), Track);
				const TelemetryFrame& F = Sim.GetTelemetry();
				Csv.Write(F);
				PeakF = F.FzFL > PeakF ? F.FzFL : PeakF;
				PeakR = F.FzRL > PeakR ? F.FzRL : PeakR;
				PeakVert = std::fabs(F.AccVertG) > PeakVert ? std::fabs(F.AccVertG) : PeakVert;
				MaxZ = F.PosZ > MaxZ ? F.PosZ : MaxZ;
				if (F.LiftFL > 0.5 && F.LiftFR > 0.5 && F.LiftRL > 0.5 && F.LiftRR > 0.5) { Air += kTick; }
			}
			std::printf("SHARP KERB 25 mm x 1 m, 54 km/h [%s]: peak Fz front %.0f N rear %.0f N, peak vert %.2f g, CoG rise %.1f mm, all-wheels airborne %.3f s\n",
			            Mode ? "enveloping" : "single-point", PeakF, PeakR, PeakVert, (MaxZ - C.Chassis.CogHeight) * 1000.0, Air);
		}
		TyreModel::bEnableEnveloping = true;
	}

	void ScenarioCoastDown(const KartConfig& C, const std::string& Dir)
	{
		// Coast-down from 25 m/s in neutral-equivalent (throttle off, clutch disengages by itself).
		// Compare against a real coast-down to separate CdA from rolling resistance.
		KartSimulation Sim;
		Sim.Initialize(C);
		PlaneTrack Ground;
		CsvTelemetryWriter Csv;
		Csv.Open(OutPath(Dir, "coastdown"));
		KartInputs In;
		kstest::Run(Sim, Ground, In, 1.0);
		kstest::SetRolling(Sim, 25.0);
		double T15 = -1, T10 = -1;
		for (int I = 0; I < static_cast<int>(40.0 / kTick); ++I)
		{
			Sim.Step(kTick, In, Ground);
			const TelemetryFrame& F = Sim.GetTelemetry();
			Csv.Write(F);
			const double T = (I + 1) * kTick;
			if (T15 < 0 && F.VelLong < 15.0) { T15 = T; }
			if (T10 < 0 && F.VelLong < 10.0) { T10 = T; }
		}
		std::printf("COAST-DOWN from 90 km/h: below 54 km/h at %.2f s, below 36 km/h at %.2f s, after 40 s %.1f km/h\n",
		            T15, T10, Sim.GetTelemetry().SpeedKmh);
	}

	void ScenarioStaticSettle(const KartConfig& C, const std::string& Dir)
	{
		KartSimulation Sim;
		Sim.Initialize(C);
		PlaneTrack Ground;
		CsvTelemetryWriter Csv;
		Csv.Open(OutPath(Dir, "static"));
		KartInputs In;
		In.Brake = 1.0;
		for (int I = 0; I < static_cast<int>(5.0 / kTick); ++I)
		{
			Sim.Step(kTick, In, Ground);
			Csv.Write(Sim.GetTelemetry());
		}
		std::printf("STATIC SETTLE 5 s: residual |v| %.5f m/s, |w| %.5f rad/s\n",
		            Sim.GetChassis().Velocity.Length(), Sim.GetChassis().AngularVelocity.Length());
	}
}

int RunScenarios(const std::string& OutDir)
{
	const KartConfig C = kstest::LoadRotax();
	std::printf("Kart: %s (outputs in %s)\n\n", C.Name.c_str(), OutDir.c_str());
	ScenarioLaunch(C, OutDir);
	ScenarioBraking(C, OutDir);
	ScenarioSteadyStateRamp(C, OutDir);
	ScenarioKerb(C, OutDir);
	ScenarioSharpKerb(C, OutDir);
	ScenarioCoastDown(C, OutDir);
	ScenarioStaticSettle(C, OutDir);
	return 0;
}
