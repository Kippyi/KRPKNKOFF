// Dynamic track conditions (Milestone 5): rubber build-up and water film on a grid laid along the
// track centre line, plus weather (air/track temperature, cloud, rain). Grip reaches the tyres through
// GroundHit::GripMultiplier via ConditionsTrackQuery, so the tyre model itself is unchanged.
//
// All coefficients are ASSUMPTIONS (no measured rubber-build / wet-grip data); the structure is:
//  * rubber r in [0,1] per cell: deposited by tyre SLIDING energy at the contact, slow natural loss,
//    washed by rain. Dry grip factor 1 + RubberGripGain * r (RubberGripGain 0.08 = the existing
//    RubberedAsphalt assumption).
//  * water film w (mm) per cell: rain adds, drainage + evaporation (faster on a hot track) remove,
//    each tyre pass squeezes a fraction off (the racing line dries first).
//    Wetness = min(1, w / WetSaturationDepth) goes to the tyre through GroundHit::Wetness; the grip it costs
//    is a property of the tyre COMPOUND (TyreParams::WetGripFloor: slick 0.60, wet tyre much higher).
//    The road factor here: rubber gain fading as it gets wet, and rubber on a wet line is extra slippery:
//    * (1 - WetRubberPenalty * r * wetness).
#pragma once

#include "KartSim/TrackPath.h"
#include "KartSim/TrackSurface.h"

#include <vector>

namespace kartsim
{
	class KartSimulation;

	struct WeatherState
	{
		double AirTemp = 22.0;      // degC
		double CloudCover = 0.3;    // 0 clear .. 1 overcast
		double RainRate = 0.0;      // mm/h
		double TrackTemp = 30.0;    // degC (evolves)
		double PressureKPa = 101.325; // station air pressure (engine power / jetting)
		double Humidity = 0.5;      // relative 0..1
	};

	// Changing weather: rain rate and cloud cover keyframes (piecewise linear) on the conditions clock.
	struct WeatherKey { double Time = 0.0; double RainRate = 0.0; double CloudCover = 0.3; };

	struct TrackConditionParams
	{
		double CellLength = 1.0;            // m along the track
		double CellWidth = 0.5;             // m across the track
		double HalfWidth = 6.0;             // m covered each side of the centre line
		double RubberPerJoule = 2.0e-5;     // rubber level per J of tyre sliding energy deposited in a cell (ASSUMPTION)
		double RubberLossPerHour = 0.02;    // natural loss (ASSUMPTION)
		double RubberWashPerMm = 0.05;      // rubber fraction washed per mm of rain (ASSUMPTION)
		double RubberGripGain = 0.08;       // dry grip gain at full rubber (ASSUMPTION, = RubberedAsphalt 1.08)
		double WetSaturationDepth = 0.5;    // mm of water at which the wet floor is reached (ASSUMPTION)
		double WetRubberPenalty = 0.15;     // extra grip loss on a wet rubbered line (ASSUMPTION)
		double MaxWaterDepth = 2.0;         // mm, drainage limit (ASSUMPTION)
		double DrainPerHour = 1.0;          // mm/h at full depth, proportional to depth (ASSUMPTION)
		double EvaporationPerHour = 0.3;    // mm/h at 20 degC track, x2 per +15 degC (ASSUMPTION)
		double SqueegeePerPass = 0.04;      // fraction of water removed per tyre pass (ASSUMPTION)
		double SolarHeating = 25.0;         // degC above air for a clear-sky dark track (ASSUMPTION)
		double TrackTempTimeConstant = 600.0; // s (ASSUMPTION)
	};

	class KARTSIM_API TrackConditions
	{
	public:
		TrackConditionParams Params;
		WeatherState Weather;
		// Optional weather timeline; empty = constant weather. Time runs with Advance(); reset by SetSchedule().
		void SetSchedule(const std::vector<WeatherKey>& Keys) { Schedule = Keys; ScheduleClock = 0.0; ApplySchedule(); }
		double GetScheduleClock() const { return ScheduleClock; }
		bool HasSchedule() const { return !Schedule.empty(); }

		// Closed centre line (sim coordinates).
		void Build(const std::vector<Vec3>& CentreLine);
		bool IsBuilt() const { return !Grid.empty(); }

		// Road grip multiplier at a world point (1 outside the grid), WITHOUT the tyre's wet loss. Also returns
		// rubber, water (mm) and wetness (0..1) there.
		double GripAt(const Vec3& P, double* OutRubber = nullptr, double* OutWater = nullptr, double* OutWetness = nullptr) const;
		double WetnessFromWater(double WaterMm) const;

		// Weather + water/rubber evolution over Dt (call at a modest rate, e.g. 10-60 Hz).
		void Advance(double Dt);

		// Deposits rubber and squeezes water using the kart's current tyre states (call every physics step).
		void ApplyKart(const KartSimulation& Sim, double Dt);

		// Uniform state for tests / presets.
		void SetUniform(double Rubber, double Water);
		double MeanRubberOnLine() const; // mean rubber over the most-rubbered lateral band
		double MeanWater() const;
		// Wetness (0..1) of the driest lateral cell at each station, averaged: what a kart on the line feels.
		double LineWetness() const;

		// Raw grid access (for visualisation): cells along x across.
		int CellsAlong() const { return NumAlong; }
		int CellsAcross() const { return NumAcross; }
		double RubberCell(int Along, int Across) const { return Grid[Along * NumAcross + Across].Rubber; }
		double WaterCell(int Along, int Across) const { return Grid[Along * NumAcross + Across].Water; }
		// Cell grip for visualisation, for a tyre with the given wet grip floor (slick 0.60).
		double GripCell(int Along, int Across, double TyreWetFloor = 0.60) const
		{
			const Cell& C = Grid[Along * NumAcross + Across];
			return GripFromCell(C) * (1.0 - (1.0 - TyreWetFloor) * WetnessFromWater(C.Water));
		}
		const TrackPath& GetPath() const { return Path; }

		// Pre-rubbered state: Level of rubber in a band of +-LineHalfWidth around the centre line
		// (where the shared Autopilot drives), tapering to zero over another LineHalfWidth. A solo kart
		// deposits ~0.1 % per lap (see Docs/Milestone5Conditions.md), so sessions start from a preset.
		void SetRubberedLine(double Level, double LineHalfWidth);

	private:
		struct Cell { double Rubber = 0.0; double Water = 0.0; };
		bool Locate(const Vec3& P, int& OutAlong, int& OutAcross) const;
		void ApplySchedule();
		std::vector<WeatherKey> Schedule;
		double ScheduleClock = 0.0;
		double GripFromCell(const Cell& C) const;

		TrackPath Path;
		int NumAlong = 0, NumAcross = 0;
		std::vector<Cell> Grid;
		mutable size_t HintIdx = 0;
	};

	struct KartConfig;
	// Peak-grip ratio of the FITTED rear tyre at a wetness vs the dry compound on a dry track (AI pace scaling).
	KARTSIM_API double CompoundGripRatio(const KartConfig& Config, double Wetness);
	// Compound index with the most grip at this wetness (0 = dry set; 1 = wets if the class has them).
	KARTSIM_API int ChooseTyreCompound(const KartConfig& Config, double Wetness);

	// ITrackQuery decorator: forwards to the base query and multiplies GripMultiplier by the track
	// conditions on paved surfaces (asphalt variants and kerbs; grass/gravel are unaffected).
	class KARTSIM_API ConditionsTrackQuery final : public ITrackQuery
	{
	public:
		ConditionsTrackQuery(const ITrackQuery& InBase, const TrackConditions& InConditions) : Base(InBase), Conditions(InConditions) {}
		bool QueryGround(const Vec3& Origin, double MaxUp, double MaxDown, GroundHit& Out) const override;

	private:
		const ITrackQuery& Base;
		const TrackConditions& Conditions;
	};
}
