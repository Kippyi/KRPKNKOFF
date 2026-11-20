// AI racing driver (Milestone 6). Drives the racing line with the shared Autopilot control law (pure
// pursuit + speed profile), adding race craft on top:
//  * per-driver skill: grip target (lateral g), braking, lap-to-lap consistency;
//  * traffic: follows a slower kart in its lane at a speed-dependent gap instead of hitting it;
//  * overtaking: after following closely it moves off-line to the side with more room and commits
//    until clear (or aborts after a timeout);
//  * side-by-side: never steers into a kart alongside (keeps a lateral clearance).
// The AI only sees what a driver could see: positions and velocities of the other karts. Its car is
// the same simulated kart as the player's (no grip or power help).
#pragma once

#include "KartSim/Autopilot.h"
#include "KartSim/RacingLine.h"
#include "KartSim/TrackPath.h"

#include <cstdint>

namespace kartsim
{
	struct AiSkill
	{
		double LatG = 1.40;        // cornering target (the Autopilot's 1.4 g is a clean, fast lap)
		double BrakeG = 0.40;      // braking envelope
		double Consistency = 0.02; // +- fraction of LatG varied lap to lap
		double Aggression = 0.5;   // 0..1: following gap and how soon an overtake is tried
		uint32_t Seed = 1;
	};

	struct AiTraffic
	{
		Vec3 Position;   // CoG, world
		Vec3 Velocity;   // world
	};

	enum class EAiState { Racing, Following, Overtaking, Formation };

	class KARTSIM_API AiDriver
	{
	public:
		AiSkill Skill;
		double ExternalSpeedCap = 1e9; // host-imposed limit (formation lap), m/s

		void Configure(const RacingLine& Line);
		void Reset(const KartSimulation& Sim);
		void Drive(const KartSimulation& Self, const AiTraffic* Others, int NumOthers, double Dt, KartInputs& In);

		EAiState State() const { return StateNow; }
		static const char* StateName(EAiState S);
		double LineOffset() const { return Offset; }
		double CurrentLatG() const { return LatGNow; }

	private:
		double NextRandom(); // uniform [-1, 1]

		Autopilot Pilot;
		RacingLine Line;
		TrackPath Path;
		size_t Hint = 0;
		double Offset = 0.0;        // applied lateral offset from the line (+ left)
		double TargetOffset = 0.0;
		double FollowTime = 0.0;    // s spent close behind a kart in our lane
		double OvertakeTime = 0.0;
		int OvertakeSide = 0;       // +1 left, -1 right
		int OvertakeTarget = -1;    // index (in the traffic list) of the kart being passed
		int YieldTarget = -1;       // kart we backed off from after an aborted pass
		double YieldTime = 0.0;
		double LatGNow = 1.4;
	public:
		// Host-estimated grip relative to a dry track on slicks (wet track, wet tyres): scales the cornering
		// target. 1 = dry slicks. See CompoundGripRatio().
		double GripScale = 1.0;
	private:
		double PrevS = -1.0;
		uint32_t Rng = 1;
		EAiState StateNow = EAiState::Racing;
	};
}
