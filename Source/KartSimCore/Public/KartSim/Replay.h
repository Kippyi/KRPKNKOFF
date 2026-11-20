// Replays and ghost laps (engine-independent). A replay stores, per kart, a compact pose + driver-input
// frame at a fixed rate (30 Hz, the same rate as the multiplayer state stream); playback interpolates and
// poses the kart kinematically (KartSimulation::SetKinematicPose), so no physics is re-run and a replay
// always shows exactly what happened. A ghost lap is the same frame stream for one lap of one kart,
// timed from the start/finish line.
#pragma once

#include "KartSim/Math.h"

#include <cstdint>
#include <string>
#include <vector>

namespace kartsim
{
	class KartSimulation;
	struct LapRecord;

	// 72 bytes; floats are plenty for render poses (sub-mm on a km-scale track).
	struct ReplayFrame
	{
		float Time = 0.0f;              // s (session clock, or lap time for a ghost)
		float Pos[3] = {};              // sim world, m
		float Rot[4] = {1, 0, 0, 0};    // orientation quaternion w, x, y, z
		float Vel[3] = {};              // m/s
		float SteerL = 0.0f, SteerR = 0.0f; // rad about the kingpins
		float OmegaF = 0.0f, OmegaR = 0.0f; // rad/s wheel spin
		float Rpm = 0.0f;
		float Throttle = 0.0f, Brake = 0.0f;
		int32_t Gear = 0;
		uint32_t Flags = 0;             // bit 0: engine running
	};

	KARTSIM_API ReplayFrame MakeReplayFrame(const KartSimulation& Sim, double Time);
	// Interpolated frame between A and B (Alpha 0..1): positions/velocities/inputs linear, rotation nlerp.
	KARTSIM_API ReplayFrame LerpFrame(const ReplayFrame& A, const ReplayFrame& B, double Alpha);
	// Poses Sim kinematically at F (Dt advances the wheel spin angles).
	KARTSIM_API void ApplyReplayFrame(KartSimulation& Sim, const ReplayFrame& F, double Dt);

	struct KARTSIM_API ReplayTrack
	{
		std::string Name;               // driver name
		std::string KartFile;           // class config file (e.g. Rotax125MaxSenior.json)
		uint32_t Color = 0xFFFFFFFFu;   // RGBA8 body colour
		std::vector<ReplayFrame> Frames; // ascending Time

		// Interpolated frame at time T (clamped to the recorded span). False if empty.
		bool Sample(double T, ReplayFrame& Out) const;
		double StartTime() const { return Frames.empty() ? 0.0 : Frames.front().Time; }
		double EndTime() const { return Frames.empty() ? 0.0 : Frames.back().Time; }
	};

	struct KARTSIM_API ReplayRecording
	{
		std::string Circuit;            // track layout file
		std::string Info;               // free text (session, weather)
		double LapTime = 0.0;           // ghost files: the lap time
		std::vector<ReplayTrack> Karts;

		double StartTime() const;
		double EndTime() const;
		size_t FrameCount() const;
		// Drops frames older than EndTime() - MaxSeconds (bounded memory for long sessions).
		void Trim(double MaxSeconds);

		bool Save(const std::string& Path) const;
		bool Load(const std::string& Path, std::string& OutError);
	};

	// Records one kart's laps and keeps the fastest valid one as the ghost. Feed it every physics step
	// (it samples at Rate); call OnLapCompleted when the lap timer records a lap.
	class KARTSIM_API GhostKeeper
	{
	public:
		double Rate = 30.0;             // Hz

		void Step(bool bTiming, double LapElapsed, const KartSimulation& Sim);
		// Returns true if this lap became the new ghost.
		bool OnLapCompleted(double LapTime, bool bValid);
		void AbortLap() { Current.clear(); NextSample = 0.0; }

		bool HasGhost() const { return BestTime > 0.0; }
		double GhostTime() const { return BestTime; }
		const std::vector<ReplayFrame>& Ghost() const { return Best; }
		// Loaded from file (only replaces if faster or none yet).
		void Offer(const std::vector<ReplayFrame>& Frames, double LapTime);
		void Clear() { Best.clear(); BestTime = 0.0; AbortLap(); }

	private:
		std::vector<ReplayFrame> Current, Best;
		double BestTime = 0.0;
		double NextSample = 0.0;
		bool bWasTiming = false;
	};
}
