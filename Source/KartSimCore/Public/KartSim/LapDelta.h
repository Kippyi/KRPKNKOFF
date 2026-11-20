// Live lap delta and distance-based lap traces (Milestone 5 telemetry analysis). Samples time and speed
// against distance along the track centre line during each lap; the best valid lap becomes the
// reference. Delta = current elapsed time - reference time at the same distance (negative = faster).
// Engine-independent.
#pragma once

#include "KartSim/TrackPath.h"

#include <vector>

namespace kartsim
{
	// Driver/kart channels sampled with each distance station (telemetry viewer).
	struct TraceInputs
	{
		double SpeedKmh = 0.0;
		double Throttle = 0.0, Brake = 0.0;     // 0..1
		double SteerDeg = 0.0;                  // steering column, + left
		double Rpm = 0.0, Gear = 0.0;
		double LatG = 0.0, LongG = 0.0;
	};

	struct LapTrace
	{
		static constexpr double Spacing = 2.0;   // m between samples
		std::vector<double> Time;                // s since lap start, per sample
		std::vector<double> Speed;               // km/h, per sample
		std::vector<double> Throttle, Brake, Steer, Rpm, Gear, LatG, LongG;
		std::vector<double> X, Y;                // CoG position (sim m), for the track map
		double LapTime = 0.0;
		int LapNumber = 0;                       // 1-based, as counted by the lap timer
		bool bValid = false;
		bool IsComplete(size_t N) const { return Time.size() >= N; }
		void Push(double T, const Vec3& P, const TraceInputs& In);
		void RepeatLast(double T);               // pad: time T, other channels as the last sample
	};

	// Time difference Lap - Ref at each station (s, + = Lap slower); empty if either is empty.
	KARTSIM_API std::vector<double> DeltaTrace(const LapTrace& Lap, const LapTrace& Ref);

	class KARTSIM_API LapDelta
	{
	public:
		void Configure(const TrackPath* InPath);

		// Call every physics step while timing: lap time so far, CoG position, channels.
		void Sample(double LapElapsed, const Vec3& Pos, const TraceInputs& In);
		void Sample(double LapElapsed, const Vec3& Pos, double SpeedKmh) { TraceInputs In; In.SpeedKmh = SpeedKmh; Sample(LapElapsed, Pos, In); }
		// Call when a lap completes (bValid: counts toward the reference).
		void OnLapCompleted(double LapTime, bool bValid);
		// Call when timing restarts (out-lap / kart reset): discards the lap in progress.
		void AbortLap();

		bool HasReference() const { return Reference.LapTime > 0.0; }
		bool HasDelta() const { return bDeltaValid; }
		double Delta() const { return CurrentDelta; }       // s, valid when HasDelta()
		double Predicted() const { return HasReference() && bDeltaValid ? Reference.LapTime + CurrentDelta : 0.0; }
		const LapTrace& ReferenceLap() const { return Reference; }
		const LapTrace& LastLap() const { return Last; }
		const LapTrace& CurrentLap() const { return Current; }
		// Every completed whole lap of the run, oldest first (at most MaxHistory).
		static constexpr size_t MaxHistory = 200;
		const std::vector<LapTrace>& History() const { return Laps; }
		size_t SamplesPerLap() const { return NumSamples; }

	private:
		const TrackPath* Path = nullptr;
		size_t NumSamples = 0;
		size_t Hint = 0;
		double StartS = -1.0;   // centre-line distance at lap start (the start/finish line)
		double PrevDist = 0.0;
		LapTrace Current, Last, Reference;
		std::vector<LapTrace> Laps;
		int LapCounter = 0;
		double CurrentDelta = 0.0;
		bool bDeltaValid = false;
	};
}
