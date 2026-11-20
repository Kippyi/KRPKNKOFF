#include "KartSim/TrackConditions.h"

#include "KartSim/KartPhysicsCore.h"

#include <cmath>

namespace kartsim
{
	void TrackConditions::Build(const std::vector<Vec3>& CentreLine)
	{
		Path.Build(CentreLine);
		NumAlong = Path.IsBuilt() ? static_cast<int>(std::ceil(Path.Length() / Params.CellLength)) : 0;
		NumAcross = static_cast<int>(std::ceil(2.0 * Params.HalfWidth / Params.CellWidth));
		Grid.assign(static_cast<size_t>(NumAlong) * static_cast<size_t>(NumAcross), Cell());
		HintIdx = 0;
	}

	bool TrackConditions::Locate(const Vec3& P, int& OutAlong, int& OutAcross) const
	{
		if (Grid.empty()) { return false; }
		double S = 0.0, Lat = 0.0;
		if (!Path.Project(P, S, Lat, HintIdx)) { return false; }
		if (std::fabs(Lat) >= Params.HalfWidth) { return false; }
		OutAlong = static_cast<int>(S / Params.CellLength) % NumAlong;
		OutAcross = static_cast<int>((Lat + Params.HalfWidth) / Params.CellWidth);
		return OutAcross >= 0 && OutAcross < NumAcross;
	}

	void TrackConditions::SetRubberedLine(double Level, double LineHalfWidth)
	{
		for (int X = 0; X < NumAcross; ++X)
		{
			const double Lat = std::fabs(-Params.HalfWidth + (X + 0.5) * Params.CellWidth);
			const double W = Lat <= LineHalfWidth ? 1.0 : Clamp(2.0 - Lat / LineHalfWidth, 0.0, 1.0);
			for (int A = 0; A < NumAlong; ++A) { Grid[static_cast<size_t>(A) * NumAcross + X].Rubber = Level * W; }
		}
	}

	double TrackConditions::GripFromCell(const Cell& C) const
	{
		const TrackConditionParams& P = Params;
		const double WetFrac = WetnessFromWater(C.Water);
		const double Dry = 1.0 + P.RubberGripGain * C.Rubber;
		const double RubberWetPenalty = 1.0 - P.WetRubberPenalty * C.Rubber * WetFrac;
		// Blend: the rubber gain fades as the surface gets wet. The water's own grip loss is the tyre's.
		return (1.0 + (Dry - 1.0) * (1.0 - WetFrac)) * RubberWetPenalty;
	}

	double TrackConditions::WetnessFromWater(double WaterMm) const
	{
		return Clamp(WaterMm / Params.WetSaturationDepth, 0.0, 1.0);
	}

	double TrackConditions::GripAt(const Vec3& P, double* OutRubber, double* OutWater, double* OutWetness) const
	{
		int A = 0, X = 0;
		if (!Locate(P, A, X))
		{
			if (OutRubber) { *OutRubber = 0.0; }
			if (OutWater) { *OutWater = 0.0; }
			if (OutWetness) { *OutWetness = 0.0; }
			return 1.0;
		}
		const Cell& C = Grid[static_cast<size_t>(A) * NumAcross + X];
		if (OutRubber) { *OutRubber = C.Rubber; }
		if (OutWater) { *OutWater = C.Water; }
		if (OutWetness) { *OutWetness = WetnessFromWater(C.Water); }
		return GripFromCell(C);
	}

	void TrackConditions::Advance(double Dt)
	{
		if (!Schedule.empty()) { ScheduleClock += Dt; ApplySchedule(); }
		const TrackConditionParams& P = Params;
		WeatherState& W = Weather;
		// Track temperature relaxes toward air + solar heating (less with cloud, much less when raining).
		const double Solar = P.SolarHeating * (1.0 - W.CloudCover) * (W.RainRate > 0.1 ? 0.1 : 1.0);
		const double Target = W.AirTemp + Solar;
		W.TrackTemp += (Target - W.TrackTemp) * (Dt / (P.TrackTempTimeConstant + Dt));

		const double Hours = Dt / 3600.0;
		const double Evap = P.EvaporationPerHour * std::pow(2.0, (W.TrackTemp - 20.0) / 15.0);
		const double RainMm = W.RainRate * Hours;
		for (Cell& C : Grid)
		{
			C.Water += RainMm;
			C.Water -= (P.DrainPerHour * (C.Water / P.MaxWaterDepth) + Evap) * Hours;
			C.Water = Clamp(C.Water, 0.0, P.MaxWaterDepth);
			C.Rubber -= (P.RubberLossPerHour * Hours + P.RubberWashPerMm * RainMm) * C.Rubber;
			C.Rubber = Clamp(C.Rubber, 0.0, 1.0);
		}
	}

	void TrackConditions::ApplySchedule()
	{
		if (Schedule.empty()) { return; }
		const WeatherKey* A = &Schedule.front();
		const WeatherKey* B = A;
		for (const WeatherKey& K : Schedule)
		{
			if (K.Time <= ScheduleClock) { A = &K; B = &K; }
			else { B = &K; break; }
		}
		const double F = B->Time > A->Time ? Clamp((ScheduleClock - A->Time) / (B->Time - A->Time), 0.0, 1.0) : 0.0;
		Weather.RainRate = A->RainRate + (B->RainRate - A->RainRate) * F;
		Weather.CloudCover = A->CloudCover + (B->CloudCover - A->CloudCover) * F;
	}

	void TrackConditions::ApplyKart(const KartSimulation& Sim, double Dt)
	{
		if (Grid.empty()) { return; }
		for (int I = 0; I < NumWheels; ++I)
		{
			const WheelState& W = Sim.GetWheel(I);
			if (!W.Contact.bInContact) { continue; }
			int A = 0, X = 0;
			if (!Locate(W.Contact.ContactPoint, A, X)) { continue; }
			Cell& C = Grid[static_cast<size_t>(A) * NumAcross + X];
			// Rubber from sliding energy (abrasion deposits rubber where tyres slide: braking zones, corners).
			C.Rubber = Clamp(C.Rubber + Params.RubberPerJoule * W.Tyre.SlidePower * Dt, 0.0, 1.0);
			// Water squeezed off: fraction per tyre pass, i.e. per contact-length of travel.
			const double Speed = std::sqrt(W.Tyre.Vx * W.Tyre.Vx + W.Tyre.Vy * W.Tyre.Vy);
			const double PassFraction = Speed * Dt / Params.CellLength;
			C.Water *= (1.0 - Params.SqueegeePerPass * Clamp(PassFraction, 0.0, 1.0));
		}
	}

	void TrackConditions::SetUniform(double Rubber, double Water)
	{
		for (Cell& C : Grid) { C.Rubber = Rubber; C.Water = Water; }
	}

	double TrackConditions::MeanRubberOnLine() const
	{
		// Mean rubber of the most-rubbered lateral cell at each station (the line wanders across the track).
		if (Grid.empty()) { return 0.0; }
		double Sum = 0.0;
		for (int A = 0; A < NumAlong; ++A)
		{
			double Mx = 0.0;
			for (int X = 0; X < NumAcross; ++X) { Mx = Grid[static_cast<size_t>(A) * NumAcross + X].Rubber > Mx ? Grid[static_cast<size_t>(A) * NumAcross + X].Rubber : Mx; }
			Sum += Mx;
		}
		return Sum / NumAlong;
	}

	double TrackConditions::MeanWater() const
	{
		if (Grid.empty()) { return 0.0; }
		double Sum = 0.0;
		for (const Cell& C : Grid) { Sum += C.Water; }
		return Sum / static_cast<double>(Grid.size());
	}

	double TrackConditions::LineWetness() const
	{
		if (Grid.empty()) { return 0.0; }
		double Sum = 0.0;
		for (int A = 0; A < NumAlong; ++A)
		{
			double Mn = 1e9;
			for (int X = 0; X < NumAcross; ++X) { Mn = std::fmin(Mn, Grid[static_cast<size_t>(A) * NumAcross + X].Water); }
			Sum += WetnessFromWater(Mn);
		}
		return Sum / NumAlong;
	}

	namespace
	{
		double PeakGrip(const TyreParams& T, double Wetness) { return T.Mu0 * (1.0 - (1.0 - T.WetGripFloor) * Clamp(Wetness, 0.0, 1.0)); }
	}

	double CompoundGripRatio(const KartConfig& Config, double Wetness)
	{
		return PeakGrip(Config.RearTyre, Wetness) / PeakGrip(Config.CompoundRear[0], 0.0);
	}

	int ChooseTyreCompound(const KartConfig& Config, double Wetness)
	{
		int Best = 0;
		for (int K = 1; K < Config.NumCompounds; ++K)
		{
			if (PeakGrip(Config.CompoundRear[K], Wetness) > PeakGrip(Config.CompoundRear[Best], Wetness)) { Best = K; }
		}
		return Best;
	}

	bool ConditionsTrackQuery::QueryGround(const Vec3& Origin, double MaxUp, double MaxDown, GroundHit& Out) const
	{
		if (!Base.QueryGround(Origin, MaxUp, MaxDown, Out)) { return false; }
		const ESurfaceType S = Out.Surface;
		const bool bPaved = S == ESurfaceType::Asphalt || S == ESurfaceType::RubberedAsphalt || S == ESurfaceType::DirtyAsphalt ||
		                    S == ESurfaceType::WetAsphalt || S == ESurfaceType::Kerb;
		if (bPaved && Conditions.IsBuilt())
		{
			double Wetness = 0.0;
			Out.GripMultiplier *= Conditions.GripAt(Out.Point, nullptr, nullptr, &Wetness);
			Out.Wetness = Wetness > Out.Wetness ? Wetness : Out.Wetness;
		}
		return true;
	}
}
