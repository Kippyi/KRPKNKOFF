// Closed-loop "competent driver" on the test circuit: a repeatable drivability probe (Milestone 2).
#pragma once

#include "KartSim/KartConfig.h"
#include "KartSim/LapTiming.h"
#include "KartSim/TrackConditions.h"

#include <functional>
#include <vector>

namespace kartsim { class KartSimulation; }

struct LapResult
{
	double BestLap = 0.0, MaxAy = 0.0, MaxSideslip = 0.0, MaxCrossTrack = 0.0, LiftFraction = 0.0, TopSpeed = 0.0;
	int Spins = 0, LapsDone = 0;
	// Tyres (FL, FR, RL, RR): wear at the end, time-averaged and peak temperature, sliding energy (J).
	double Wear[4] = {}, MeanTemp[4] = {}, MaxTemp[4] = {}, SlideEnergy[4] = {};
	std::vector<double> LapTimes; // every completed lap (lap 1 includes the standing start)
};

// Optional Timer: updated every physics step with the CoG path (lines configured by the caller).
LapResult DriveLaps(const kartsim::KartConfig& C, double TargetLatG, int Laps, const char* CsvName, kartsim::LapTimer* Timer = nullptr,
                    kartsim::TrackConditions* Conditions = nullptr, double InitialWear = 0.0,
                    const std::function<void(const kartsim::KartSimulation&, double Time)>& OnStep = nullptr);
// Builds TrackConditions on the test circuit centre line.
void BuildTestConditions(kartsim::TrackConditions& Out);
