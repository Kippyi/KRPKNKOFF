// Race director (Milestone 6): practice / qualifying / race for N competitors (player, AI or remote),
// with a live classification. Replaces the single-kart Session of Milestone 5 (same rules, generalised).
// Engine-independent; each competitor is described by its LapTimer and its distance past the
// start/finish line along the track.
//
// Rules (common CIK / national practice, simplified):
//  * The grid is past the start/finish line (front row 24 m after it on the test circuit).
//  * Practice / qualifying: classification by best VALID lap. Qualifying: when time expires the
//    chequered flag is out; each competitor's lap in progress still counts and it is classified when it
//    next crosses the line. Practice may be unlimited (Duration 0).
//  * Race, standing start (gearbox classes): lights, then green for everyone; laps count from the grid
//    (lap 1 is slightly short).
//  * Race, rolling start (single-speed classes): formation lap; green when the first kart reaches the
//    line. Every other kart starts racing at its own first crossing after green (that crossing is the
//    start, not a lap), but all race times run from the common green.
//  * When the leader completes the race distance the chequered flag is out; every other competitor is
//    classified at its next crossing (lapped karts with fewer laps). The session ends when all are
//    classified, or 90 s after the leader (the rest classified in running order).
//  * Laps with track-limit violations are counted (no penalty model).
#pragma once

#include "KartSim/LapTiming.h"

#include <string>
#include <vector>

namespace kartsim
{
	enum class ESessionType { Practice, Qualifying, Race };
	enum class ESessionPhase { Grid, Running, Chequered, Finished };

	struct SessionConfig
	{
		ESessionType Type = ESessionType::Practice;
		double Duration = 0.0;        // s; practice (0 = unlimited) and qualifying
		int RaceLaps = 10;
		double StartLightsTime = 3.0; // s from Start() to green (standing race start)
		bool bRollingStart = false;   // race: formation lap, green at the first start/finish crossing
	};

	struct CompetitorInput
	{
		const LapTimer* Timer = nullptr;
		double LapDistance = 0.0;     // m past the start/finish line along the track, [0, TrackLength)
	};

	struct CompetitorResult
	{
		int Laps = 0;               // completed this session (race: racing laps)
		double BestLap = 0.0;       // valid laps this session, 0 if none
		double LastLap = 0.0;
		int InvalidLaps = 0;
		bool bStarted = false;      // race: has taken the start
		bool bFinished = false;     // classified (took the chequered flag)
		double FinishTime = 0.0;    // race: from green to its finishing crossing
		double Progress = 0.0;      // race: laps * length + distance (m)
		int Position = 0;           // 1-based
		double Gap = 0.0;           // s behind the leader (race: estimated while running), 0 for P1
		int LapsDown = 0;
	};

	class KARTSIM_API RaceDirector
	{
	public:
		// The host resets every kart (and its LapTimer) to its grid slot before calling Start.
		void Start(const SessionConfig& Config, double Now, double TrackLength, const std::vector<CompetitorInput>& In);
		// Every physics step, after all LapTimers are updated.
		void Update(double Now, const std::vector<CompetitorInput>& In);

		bool IsActive() const { return bActive; }
		const SessionConfig& Config() const { return Cfg; }
		ESessionPhase Phase() const { return PhaseNow; }
		bool IsGreen() const { return PhaseNow == ESessionPhase::Running || PhaseNow == ESessionPhase::Chequered; }
		double LightsRemaining(double Now) const;
		double TimeRemaining(double Now) const;   // practice/qualifying, 0 if unlimited/expired
		double Elapsed(double Now) const;         // since green (race: frozen at the leader's finish)
		int LeaderLaps() const;
		const std::vector<CompetitorResult>& Results() const { return Res; }
		const std::vector<int>& Order() const { return OrderNow; } // competitor indices, P1 first
		std::string Summary(int Competitor) const;

		static const char* TypeName(ESessionType T);

	private:
		struct Track { int BaseCrossings = 0; size_t SeenLaps = 0; int FlagCrossings = -1; };
		void Classify();

		SessionConfig Cfg;
		bool bActive = false;
		ESessionPhase PhaseNow = ESessionPhase::Finished;
		double StartTime = 0.0, GreenTime = 0.0, Length = 1.0, LeaderFinishTime = -1.0, EndTime = 0.0;
		std::vector<Track> Tr;
		std::vector<CompetitorResult> Res;
		std::vector<int> OrderNow;
	};
}
