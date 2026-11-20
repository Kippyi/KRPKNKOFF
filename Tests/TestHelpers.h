// Shared helpers for tests and scenarios: config loading and simple closed-loop driver aids.
#pragma once

#include "KartSim/DriverAids.h"
#include "KartSim/KartPhysicsCore.h"

#include <cstdio>
#include <cstdlib>
#include <string>

namespace kstest
{
	inline std::string& ConfigPath()
	{
		static std::string P = "Content/KartData/Karts/Rotax125MaxSenior.json";
		return P;
	}

	inline kartsim::KartConfig LoadRotax()
	{
		kartsim::KartConfig C;
		std::string Err;
		if (!kartsim::LoadKartConfigFile(ConfigPath(), C, Err))
		{
			std::printf("FATAL: %s\n", Err.c_str());
			std::exit(2);
		}
		return C;
	}

	inline kartsim::KartConfig LoadKart(const char* FileName)
	{
		kartsim::KartConfig C;
		std::string Err;
		const std::string Path = std::string("Content/KartData/Karts/") + FileName;
		if (!kartsim::LoadKartConfigFile(Path, C, Err))
		{
			std::printf("FATAL: %s\n", Err.c_str());
			std::exit(2);
		}
		return C;
	}

	constexpr double kTick = 1.0 / 240.0;

	// Test-runner override of SimulationSettings::Substeps for DriveLaps (0 = the sim default).
	inline int& SubstepOverride() { static int N = 0; return N; }

	// Driver aids for the automated tests: the same core logic as the game (KartSim/DriverAids.h),
	// with every aid enabled.
	struct DriverAids
	{
		kartsim::DriverAidSettings S;
		void Configure(const kartsim::KartConfig&) { S.bAutoClutch = true; S.bAutoShift = true; S.bAutoPush = true; }
		void Apply(const kartsim::KartSimulation& Sim, kartsim::KartInputs& In) const
		{
			In.ShiftRequest = 0;
			In.PushForce = 0.0;
			In.Clutch = 0.0;
			kartsim::ApplyDriverAids(Sim, S, In);
		}
	};

	inline void Run(kartsim::KartSimulation& Sim, const kartsim::ITrackQuery& Track, const kartsim::KartInputs& In, double Seconds)
	{
		const int N = static_cast<int>(Seconds / kTick + 0.5);
		for (int I = 0; I < N; ++I) { Sim.Step(kTick, In, Track); }
	}

	// Starts the kart rolling at Speed (m/s) along +x with wheels spinning to match.
	inline void SetRolling(kartsim::KartSimulation& Sim, double Speed)
	{
		Sim.MutableChassis().Velocity = kartsim::Vec3(Speed, 0.0, 0.0);
		const kartsim::KartConfig& C = Sim.GetConfig();
		Sim.SetFrontWheelOmega(Speed / C.FrontTyre.UnloadedRadius);
		Sim.SetRearAxleOmega(Speed / C.RearTyre.UnloadedRadius);
		const double EngineW = Speed / C.RearTyre.UnloadedRadius * Sim.GetTransmission().GetTotalRatio();
		if (EngineW > C.Engine.IdleRpm * kartsim::kRpmToRadS) { Sim.MutableEngine().SetOmega(EngineW); }
	}

	struct SpeedController
	{
		double Kp = 0.6, Ki = 0.4, Integral = 0.0;
		double Throttle(double Target, double Actual, double Dt)
		{
			const double E = Target - Actual;
			Integral = kartsim::Clamp(Integral + E * Dt, -2.0, 2.0);
			return kartsim::Clamp(Kp * E + Ki * Integral, 0.0, 1.0);
		}
	};
}
