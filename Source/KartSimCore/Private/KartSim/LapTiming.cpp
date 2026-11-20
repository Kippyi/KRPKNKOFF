#include "KartSim/LapTiming.h"

namespace kartsim
{
	void LapTimer::Configure(const TimingLine* InLines, int InNumLines)
	{
		NumLines = InNumLines < LapRecord::MaxSectors ? InNumLines : LapRecord::MaxSectors;
		for (int I = 0; I < NumLines; ++I) { Lines[I] = InLines[I]; }
		History.clear();
		BestIndex = -1;
		for (double& B : BestSectors) { B = 0.0; }
		Crossings = 0;
		LastCrossTime = 0.0;
		Reset();
	}

	void LapTimer::Reset()
	{
		bTiming = false;
		bLapValid = true;
		Sector = 0;
		LapStart = SectorStart = 0.0;
		Current = LapRecord();
	}

	double LapTimer::CrossingFraction(const TimingLine& L, const Vec3& Prev, const Vec3& Pos)
	{
		// 2D segment intersection: P(t) = Prev + t*(Pos-Prev), Q(u) = A + u*(B-A), t,u in [0,1].
		const double Rx = Pos.X - Prev.X, Ry = Pos.Y - Prev.Y;
		const double Sx = L.B.X - L.A.X, Sy = L.B.Y - L.A.Y;
		const double Den = Rx * Sy - Ry * Sx;
		if (std::fabs(Den) < 1e-12) { return -1.0; }
		const double Qx = L.A.X - Prev.X, Qy = L.A.Y - Prev.Y;
		const double T = (Qx * Sy - Qy * Sx) / Den;
		const double U = (Qx * Ry - Qy * Rx) / Den;
		if (T < 0.0 || T > 1.0 || U < 0.0 || U > 1.0) { return -1.0; }
		// Forward crossings only (moving with the racing direction).
		if (Rx * L.Forward.X + Ry * L.Forward.Y <= 0.0) { return -1.0; }
		return T;
	}

	void LapTimer::Update(double TimeEnd, double Dt, const Vec3& PrevPos, const Vec3& Pos, bool bAllWheelsOff)
	{
		if (NumLines == 0) { return; }
		if (bTiming && bAllWheelsOff) { bLapValid = false; }

		const int NextLine = (Sector + 1) % NumLines;   // the line that ends the current sector
		const int Check = bTiming ? NextLine : 0;       // before timing starts, wait for start/finish
		const double F = CrossingFraction(Lines[Check], PrevPos, Pos);
		if (F < 0.0) { return; }
		const double TCross = TimeEnd - Dt + F * Dt;    // sub-tick interpolation
		if (Check == 0) { ++Crossings; LastCrossTime = TCross; }

		if (!bTiming)
		{
			bTiming = true;
			bLapValid = true;
			Sector = 0;
			LapStart = SectorStart = TCross;
			Current = LapRecord();
			Current.NumSectors = NumLines;
			return;
		}

		Current.Sectors[Sector] = TCross - SectorStart;
		SectorStart = TCross;
		if (NextLine == 0)
		{
			// Lap complete.
			Current.LapTime = TCross - LapStart;
			Current.bValid = bLapValid;
			History.push_back(Current);
			if (Current.bValid)
			{
				const int Idx = static_cast<int>(History.size()) - 1;
				if (BestIndex < 0 || Current.LapTime < History[BestIndex].LapTime) { BestIndex = Idx; }
				for (int S = 0; S < NumLines; ++S)
				{
					if (BestSectors[S] == 0.0 || Current.Sectors[S] < BestSectors[S]) { BestSectors[S] = Current.Sectors[S]; }
				}
			}
			LapStart = TCross;
			bLapValid = true;
			Current = LapRecord();
			Current.NumSectors = NumLines;
			Sector = 0;
		}
		else
		{
			Sector = NextLine;
		}
	}
}
