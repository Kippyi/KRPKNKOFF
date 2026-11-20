// Engine tuning and condition: air density, carburettor mixture, cooling, exhaust gas temperature,
// piston temperature and seizure. Feeds EngineModel a power factor every substep.
//
// Physics (structure) and provenance:
//  * Air: dry-air partial density from temperature, pressure and relative humidity (Buck vapour pressure).
//    The class torque curves are RATED figures (Rotax datasheet: ISO 15550 / ISO 4106 conditions), so the
//    power at the actual air is the ISO 1585 spark-ignition correction inverted:
//    P / P_ref = (p_dry / 99 kPa)^1.2 * (298 K / T)^0.6 (exponents as widely quoted for ISO 1585 6.3.1;
//    ESTIMATE until checked against the standard text).
//  * Mixture: a float carb meters fuel ~ jet area, so the best jet moves with the air density. Model:
//    BestJet = BestJetRef * (rho_dry / rho_ref)^k. k = 0.70 reproduces the IAME X30 Shifter manual rule
//    "+2-3 main-jet points per 6 degC drop" (VERIFIED as published) and RADNE's Rotax EVO steps
//    (122 >= 18 degC, 125 < 18, 128 < 10; third-party table). Richness r = (Jet / BestJet)^FlowExponent.
//    Power vs r: peak slightly lean (r = 0.96), falling faster on the lean side (lean bog) - ASSUMPTION.
//  * Heat: one lumped cooling node (coolant + block, or cylinder head on an air-cooled engine) heated by
//    the combustion power (+ more when lean), cooled by the radiator / fins with airspeed, radiator cover
//    and coolant loss. Typical windows: Rotax water 50-55 degC (AiM sensor guide), IAME KZ 50-58 degC
//    (IAME Shifter manual). Cooling coefficients are ESTIMATES tuned to those windows.
//  * Piston: an abstract crown-temperature index = node + rise x load x (lean heat), lagged. Above the limit
//    the clearance is consumed: seize level grows (power loss = "tightening"), at 1 the engine seizes.
//    Limits and rates are ASSUMPTIONS (gameplay-calibrated: a few jet points lean is fast; ~10 % lean seizes).
#pragma once

#include "KartSim/KartConfig.h"

namespace kartsim
{
	struct AirState
	{
		double TempC = 22.0;
		double PressureKPa = 101.325;
		double Humidity = 0.5; // relative 0..1
	};

	KARTSIM_API double SaturationVapourPressureKPa(double TempC);
	KARTSIM_API double DryAirDensity(const AirState& Air);   // kg/m^3, oxygen-carrying part
	KARTSIM_API double MoistAirDensity(const AirState& Air); // kg/m^3, total (aero)
	KARTSIM_API double IsoPowerCorrection(const AirState& Air); // power at Air / rated power
	KARTSIM_API double BestJet(const EngineTuneParams& P, const AirState& Air);
	KARTSIM_API double MixtureRichness(const EngineTuneParams& P, const AirState& Air); // with the fitted jet
	KARTSIM_API double MixturePowerFactor(double Richness);
	// Snaps the fitted jet to the best setting for Air (AI karts, "auto jet").
	KARTSIM_API void AutoJet(EngineTuneParams& P, const AirState& Air);
	// AutoJet plus a radiator cover for the air temperature (water-cooled): what a mechanic does for the AI.
	KARTSIM_API void AutoTune(EngineTuneParams& P, const AirState& Air);

	class KARTSIM_API EngineCondition
	{
	public:
		void Init(const EngineTuneParams& InTune, const EngineParams& InEngine);
		// Cooling node to WarmTemp (or the air temperature); seizure and leaks are kept (see Repair).
		void Reset(double AirTempC);
		void Repair(); // new piston, full coolant

		// After the engine step: Rpm, combustion (brake) torque produced, kart speed.
		void Update(double Dt, const AirState& Air, double Rpm, double CombustionTorque, bool bRunning, double SpeedMs, double Throttle = 1.0);
		double PowerFactor() const { return bSeized ? 0.0 : Power; }

		double GetNodeTemp() const { return NodeTemp; }  // water (or CHT for air-cooled)
		double GetPistonTemp() const { return PistonTemp; }
		double GetPistonLimit() const { return Limit; }
		double GetEgt() const { return Egt; }
		double GetRichness() const { return Richness; }
		// Wide-band lambda sensor in the header (AiM lambda channel): BestPowerLambda / richness under power, reading
		// lean on a closed throttle (air pumped through, little fuel), lagged 0.15 s.
		double GetLambda() const { return Lambda; }
		static constexpr double BestPowerLambda = 0.86; // ESTIMATE: two-stroke best power ~12.5:1 on a ~14.5:1 stoichiometric premix
		double GetSeizeLevel() const { return Seize; }
		bool IsSeized() const { return bSeized; }
		double GetCoolant() const { return Coolant; }
		void SetCoolantLeak(double FractionPerS) { Leak = FractionPerS; }
		double GetCoolantLeak() const { return Leak; }
		void SetNodeTemp(double T) { NodeTemp = T; PistonTemp = T; }
		double GetMaxPower() const { return MaxPower; }
		bool bAllowSeizure = true; // false = heat and mixture still cost power, but the engine never seizes

	private:
		const EngineTuneParams* P = nullptr;
		const EngineParams* E = nullptr;
		double MaxPower = 1.0;
		double NodeTemp = 50.0, PistonTemp = 50.0, Egt = 30.0, Limit = 290.0;
		double Richness = 1.0, Seize = 0.0, PeakSeize = 0.0, Power = 1.0;
		double Coolant = 1.0, Leak = 0.0;
		double Lambda = 1.0;
		bool bSeized = false;
	};
}
