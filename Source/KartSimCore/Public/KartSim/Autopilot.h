// Autopilot: a modulating "competent driver" that follows a closed centre-line path. Pure-pursuit
// steering with yaw-rate feedback and a rate limit, speed profile from a target lateral
// acceleration with a braking envelope, braking kept below rear lock. Used by the automated
// drivability tests AND in game (-KartAutopilot / key O) to verify the full engine pipeline.
// This is a test/demo driver, not the racing AI of Milestone 6.
#pragma once

#include "KartSim/KartPhysicsCore.h"

#include <vector>

namespace kartsim
{
	class KARTSIM_API Autopilot
	{
	public:
		double TargetLatG = 1.4;
		double BrakeDecelG = 0.40;
		double MaxBrakePedal = 0.40;
		double SteerRateDegPerSec = 300.0;
		// Race-craft inputs for the AI driver (inert at their defaults, so the regression laps are
		// unchanged): steer toward the path shifted LateralOffset metres to the left, and never target
		// more than SpeedCap (m/s).
		double LateralOffset = 0.0;
		double SpeedCap = 1e9;

		// Closed loop of centre-line points (sim coordinates), roughly evenly spaced.
		void SetPath(const std::vector<Vec3>& Points);
		bool HasPath() const { return Path.size() >= 3; }

		// Restarts tracking from the path point nearest to the kart.
		void Reset(const KartSimulation& Sim);

		// Writes Throttle, Brake and SteeringColumnAngle.
		void Drive(const KartSimulation& Sim, double Dt, KartInputs& In);

		double CrossTrackError() const { return CrossTrack; }
		size_t PathIndex() const { return Idx; }
		double ArcLengthAt(size_t I) const { return S[I]; }
		double PathLength() const { return Length; }
		double CurvatureAt(size_t I) const { return Curv[I]; }
		const Vec3& PathPoint(size_t I) const { return Path[I % Path.size()]; }
		size_t PathSize() const { return Path.size(); }
		// Unit left normal of the path at point I.
		Vec3 PathNormal(size_t I) const;

	private:
		std::vector<Vec3> Path;
		std::vector<double> Curv;   // signed, smoothed
		std::vector<double> S;      // arc length
		double Length = 0.0, Spacing = 1.0;
		size_t Idx = 0;
		double Column = 0.0;
		double CrossTrack = 0.0;
	};
}
