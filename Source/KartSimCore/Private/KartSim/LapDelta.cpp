#include "KartSim/LapDelta.h"

#include <cmath>

namespace kartsim
{
	void LapTrace::Push(double T, const Vec3& P, const TraceInputs& In)
	{
		Time.push_back(T); Speed.push_back(In.SpeedKmh);
		Throttle.push_back(In.Throttle); Brake.push_back(In.Brake); Steer.push_back(In.SteerDeg);
		Rpm.push_back(In.Rpm); Gear.push_back(In.Gear); LatG.push_back(In.LatG); LongG.push_back(In.LongG);
		X.push_back(P.X); Y.push_back(P.Y);
	}

	void LapTrace::RepeatLast(double T)
	{
		if (Time.empty()) { return; }
		TraceInputs In;
		In.SpeedKmh = Speed.back(); In.Throttle = Throttle.back(); In.Brake = Brake.back(); In.SteerDeg = Steer.back();
		In.Rpm = Rpm.back(); In.Gear = Gear.back(); In.LatG = LatG.back(); In.LongG = LongG.back();
		Push(T, Vec3(X.back(), Y.back(), 0.0), In);
	}

	std::vector<double> DeltaTrace(const LapTrace& Lap, const LapTrace& Ref)
	{
		std::vector<double> D;
		const size_t N = Lap.Time.size() < Ref.Time.size() ? Lap.Time.size() : Ref.Time.size();
		D.reserve(N);
		for (size_t I = 0; I < N; ++I) { D.push_back(Lap.Time[I] - Ref.Time[I]); }
		return D;
	}

	void LapDelta::Configure(const TrackPath* InPath)
	{
		Path = InPath;
		NumSamples = (Path && Path->IsBuilt()) ? static_cast<size_t>(std::floor(Path->Length() / LapTrace::Spacing)) : 0;
		Current = Last = Reference = LapTrace();
		Laps.clear();
		LapCounter = 0;
		AbortLap();
	}

	void LapDelta::AbortLap()
	{
		Current = LapTrace();
		StartS = -1.0;
		PrevDist = 0.0;
		bDeltaValid = false;
	}

	void LapDelta::Sample(double LapElapsed, const Vec3& Pos, const TraceInputs& In)
	{
		if (NumSamples == 0) { return; }
		double S = 0.0, Lat = 0.0;
		if (!Path->Project(Pos, S, Lat, Hint)) { return; }
		if (StartS < 0.0) { StartS = S; }
		// Distance into the lap in [0, L). Just after the start the projection can sit fractionally
		// behind StartS, which wraps to ~L: treat that as 0. Backward motion is ignored (monotonic).
		const double L = Path->Length();
		double Dist = std::fmod(S - StartS + L, L);
		if (PrevDist < 0.25 * L && Dist > 0.75 * L) { Dist = 0.0; }
		if (Dist < PrevDist) { return; }
		PrevDist = Dist;
		// Fill every sample station passed (linear in time between steps is unnecessary at 240 Hz).
		while (Current.Time.size() < NumSamples && Current.Time.size() * LapTrace::Spacing <= Dist)
		{
			Current.Push(LapElapsed, Pos, In);
		}
		const size_t I = Current.Time.size();
		if (HasReference() && I > 0 && I <= Reference.Time.size())
		{
			CurrentDelta = Current.Time[I - 1] - Reference.Time[I - 1];
			bDeltaValid = true;
		}
	}

	void LapDelta::OnLapCompleted(double LapTime, bool bValid)
	{
		// A full lap reaches the last station or misses only the final metres (the line need not be at
		// the path origin); pad those. Anything shorter was interrupted and is not a reference.
		const bool bFull = NumSamples > 0 && Current.Time.size() + 3 >= NumSamples;
		while (bFull && Current.Time.size() < NumSamples) { Current.RepeatLast(LapTime); }
		Current.LapTime = LapTime;
		Current.LapNumber = ++LapCounter;
		Current.bValid = bValid;
		Last = Current;
		if (bFull)
		{
			Laps.push_back(Current);
			if (Laps.size() > MaxHistory) { Laps.erase(Laps.begin()); }
		}
		if (bValid && bFull && (!HasReference() || LapTime < Reference.LapTime)) { Reference = Current; }
		AbortLap();
	}
}
