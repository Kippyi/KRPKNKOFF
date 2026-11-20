// Lap timing: start/finish and sector lines as segments across the track, forward-direction crossing
// detection with sub-tick interpolation, sector splits, best lap/sectors and a track-limits rule
// (lap invalid if all four wheels are off the track surface at once). Engine-independent.
#pragma once

#include "KartSim/Math.h"

#include <vector>

namespace kartsim
{
	struct TimingLine
	{
		Vec3 A, B;        // endpoints across the track (z ignored)
		Vec3 Forward;     // racing direction at the line (z ignored)
	};

	struct LapRecord
	{
		static constexpr int MaxSectors = 4;
		double LapTime = 0.0;
		double Sectors[MaxSectors] = {};
		int NumSectors = 0;
		bool bValid = true;
	};

	class KARTSIM_API LapTimer
	{
	public:
		// Lines[0] is start/finish; Lines[1..] are sector boundaries in driving order (max 3).
		void Configure(const TimingLine* Lines, int NumLines);
		void Reset();

		// Call once per physics step with the CoG positions before/after the step and the time at the
		// END of the step. bAllWheelsOff: every wheel on a non-track surface (track-limits rule).
		void Update(double TimeEnd, double Dt, const Vec3& PrevPos, const Vec3& Pos, bool bAllWheelsOff);

		bool IsTiming() const { return bTiming; }
		double CurrentLapTime(double Now) const { return bTiming ? Now - LapStart : 0.0; }
		int CurrentSector() const { return Sector; }
		bool IsCurrentLapValid() const { return bLapValid; }
		const LapRecord& CurrentSplits() const { return Current; }
		const std::vector<LapRecord>& Laps() const { return History; }
		int BestLapIndex() const { return BestIndex; }          // valid laps only, -1 if none
		double BestSector(int S) const { return BestSectors[S]; } // 0 if none
		int NumSectors() const { return NumLines > 0 ? NumLines : 0; }
		// Every forward start/finish crossing (including the one that starts timing). Not cleared by
		// Reset(), only by Configure(), so session logic can count race laps across kart resets.
		int StartFinishCrossings() const { return Crossings; }
		double LastStartFinishTime() const { return LastCrossTime; }

		// Segment crossing test (exposed for unit tests). Returns the fraction along Prev->Pos at which
		// the forward crossing happens, or -1 if there is none.
		static double CrossingFraction(const TimingLine& L, const Vec3& Prev, const Vec3& Pos);

	private:
		TimingLine Lines[LapRecord::MaxSectors];
		int NumLines = 0;
		bool bTiming = false;
		bool bLapValid = true;
		int Sector = 0;
		double LapStart = 0.0;
		double SectorStart = 0.0;
		LapRecord Current;
		std::vector<LapRecord> History;
		int BestIndex = -1;
		double BestSectors[LapRecord::MaxSectors] = {};
		int Crossings = 0;
		double LastCrossTime = 0.0;
	};
}
