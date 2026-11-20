#include "KartSim/EngineTune.h"

#include <cmath>

namespace kartsim
{
	namespace
	{
		constexpr double Rdry = 287.05;   // J/(kg K)
		constexpr double Rvap = 461.495;  // J/(kg K)
		double Kelvin(double C) { return C + 273.15; }
		double VapourKPa(const AirState& A) { return Clamp(A.Humidity, 0.0, 1.0) * SaturationVapourPressureKPa(A.TempC); }
		AirState RefAir(const EngineTuneParams& P) { AirState A; A.TempC = P.RefAirTemp; A.PressureKPa = P.RefPressure; A.Humidity = P.RefHumidity; return A; }
	}

	double SaturationVapourPressureKPa(double TempC)
	{
		// Buck (1996), over water.
		return 0.61121 * std::exp((18.678 - TempC / 234.5) * (TempC / (257.14 + TempC)));
	}

	double DryAirDensity(const AirState& Air)
	{
		return (Air.PressureKPa - VapourKPa(Air)) * 1000.0 / (Rdry * Kelvin(Air.TempC));
	}

	double MoistAirDensity(const AirState& Air)
	{
		const double Pv = VapourKPa(Air);
		return ((Air.PressureKPa - Pv) * 1000.0 / Rdry + Pv * 1000.0 / Rvap) / Kelvin(Air.TempC);
	}

	double IsoPowerCorrection(const AirState& Air)
	{
		const double Pd = Air.PressureKPa - VapourKPa(Air);
		return std::pow(Pd / 99.0, 1.2) * std::pow(298.0 / Kelvin(Air.TempC), 0.6);
	}

	double BestJet(const EngineTuneParams& P, const AirState& Air)
	{
		return P.BestJetRef * std::pow(DryAirDensity(Air) / DryAirDensity(RefAir(P)), P.JetDensityExponent);
	}

	double MixtureRichness(const EngineTuneParams& P, const AirState& Air)
	{
		return std::pow(P.Jet / BestJet(P, Air), P.FlowExponent);
	}

	double MixturePowerFactor(double R)
	{
		// Peak at r = 0.96; normalised so the best SAFE setting (r = 1) gives exactly 1.
		constexpr double Peak = 0.96, Rich = 1.5, Lean = 4.0;
		auto Raw = [](double X) { const double D = X - Peak; return 1.0 - (D >= 0.0 ? Rich : Lean) * D * D; };
		return Clamp(Raw(R) / Raw(1.0), 0.0, 1.1);
	}

	void AutoJet(EngineTuneParams& P, const AirState& Air)
	{
		const double B = BestJet(P, Air);
		const double Snapped = P.JetMin + std::round((B - P.JetMin) / P.JetStep) * P.JetStep;
		// Round toward rich when between steps (what a mechanic does).
		P.Jet = Clamp(Snapped < B ? Snapped + P.JetStep : Snapped, P.JetMin, P.JetMax);
	}

	void AutoTune(EngineTuneParams& P, const AirState& Air)
	{
		AutoJet(P, Air);
		// 20 % at 22 degC puts the water in its window (Tests: Engine_RadiatorCoverRaisesWaterTemp); ~3 % more per
		// degC colder, none on a hot day (ESTIMATE from the cooling model).
		P.Cover = P.bWaterCooled ? Clamp(0.20 + 0.03 * (22.0 - Air.TempC), 0.0, 0.7) : 0.0;
		P.Cover = std::round(P.Cover * 20.0) / 20.0;
	}

	void EngineCondition::Init(const EngineTuneParams& InTune, const EngineParams& InEngine)
	{
		P = &InTune;
		E = &InEngine;
		MaxPower = 1.0;
		for (int I = 0; I < E->TorqueWOT.Count; ++I)
		{
			const double W = E->TorqueWOT.Xs[I] * kRpmToRadS * E->TorqueWOT.Ys[I];
			if (W > MaxPower) { MaxPower = W; }
		}
		Repair();
		Reset(22.0);
	}

	void EngineCondition::Reset(double AirTempC)
	{
		NodeTemp = P->WarmTemp > 0.0 ? P->WarmTemp : AirTempC;
		PistonTemp = NodeTemp;
		Egt = AirTempC;
		Power = 1.0;
		Richness = 1.0;
	}

	void EngineCondition::Repair()
	{
		Seize = PeakSeize = 0.0;
		bSeized = false;
		Coolant = 1.0;
		Leak = 0.0;
	}

	void EngineCondition::Update(double Dt, const AirState& Air, double Rpm, double CombustionTorque, bool bRunning, double SpeedMs, double Throttle)
	{
		const EngineTuneParams& T = *P;
		Richness = MixtureRichness(T, Air);
		const double LeanBy = std::fmax(0.0, 1.0 - Richness);
		const double RichBy = std::fmax(0.0, Richness - 1.0);
		const double Pcomb = std::fmax(0.0, CombustionTorque * Rpm * kRpmToRadS);
		// Heat follows the CHARGE burnt (throttle x airflow), not the brake power: a lean bog makes less power
		// but no less heat. Undo the power factor applied to the torque this step.
		const double Load = Clamp(Pcomb / std::fmax(Power, 0.3) / MaxPower, 0.0, 1.2);
		const bool bFiring = bRunning && !bSeized;

		// Cooling node.
		const double Idle = bFiring ? T.IdleHeat * Rpm / std::fmax(E->IdleRpm, 1.0) * 0.5 : 0.0;
		const double Heat = (T.HeatPerPower * Load * MaxPower + Idle) * (1.0 + 0.8 * LeanBy) * (1.0 - 0.3 * std::fmin(RichBy, 1.0));
		// Leak, and boil-off once the (barely pressurised) kart system passes ~105 degC (ASSUMPTION).
		const double Boil = T.bWaterCooled ? 0.002 * std::fmax(0.0, NodeTemp - 105.0) : 0.0;
		if (Leak > 0.0 || Boil > 0.0) { Coolant = std::fmax(0.0, Coolant - (Leak + Boil) * Dt); }
		double Ua = T.CoolingBase + T.CoolingPerSpeed * std::fmax(SpeedMs, 0.0);
		if (T.bWaterCooled) { Ua *= Clamp(1.0 - T.Cover, 0.02, 1.0) * (0.08 + 0.92 * Coolant); }
		const double Cap = T.bWaterCooled ? T.HeatCapacity * (0.55 + 0.45 * Coolant) : T.HeatCapacity;
		NodeTemp += (Heat - Ua * (NodeTemp - Air.TempC)) / Cap * Dt;

		// Piston crown index and seizure.
		const double PistonSS = NodeTemp + (bFiring ? T.PistonRise * Load * (1.0 + T.LeanHeat * LeanBy - 0.5 * std::fmin(RichBy, 0.5)) : 0.0);
		PistonTemp += (PistonSS - PistonTemp) * Clamp(Dt / T.PistonTimeConstant, 0.0, 1.0);
		// Cold: the piston expands faster than the bore. Hot: the oil film breaks down once the cooling node is
		// well past its window (classic overheat seizure, ASSUMPTION 1.5 degC of margin per degC).
		Limit = T.PistonLimit - T.ColdPerDeg * std::fmax(0.0, (T.OptimumLow - 15.0) - NodeTemp)
		        - 1.5 * std::fmax(0.0, NodeTemp - (T.OptimumHigh + 35.0));
		if (bFiring && bAllowSeizure && PistonTemp > Limit)
		{
			Seize += T.SeizeRate * (PistonTemp - Limit) * Dt;
			PeakSeize = std::fmax(PeakSeize, Seize);
			if (Seize >= 1.0) { Seize = 1.0; bSeized = true; }
		}
		else if (!bSeized)
		{
			// The tightening eases as it cools; a scuffed piston keeps part of the loss until it is replaced.
			Seize = std::fmax(0.3 * PeakSeize, Seize - 0.02 * Dt);
		}

		// Exhaust gas temperature at the probe.
		const double RpmFrac = Clamp(Rpm / std::fmax(E->RevLimitRpm, 1.0), 0.0, 1.0);
		const double EgtSS = bFiring
			? T.EgtIdle + (T.EgtFull - T.EgtIdle) * Clamp(0.35 * RpmFrac + 0.65 * Load, 0.0, 1.1) + T.EgtPerLean * LeanBy - T.EgtPerRich * RichBy
			: NodeTemp;
		Egt += (EgtSS - Egt) * Clamp(Dt / T.EgtTimeConstant, 0.0, 1.0);
		{
			const double Under = BestPowerLambda / std::fmax(Richness, 0.2);
			const double Off = Clamp((0.15 - Throttle) / 0.15, 0.0, 1.0); // overrun: reads lean
			const double LambdaSS = bFiring ? Under + (1.6 - Under) * Off : 1.6;
			Lambda += (LambdaSS - Lambda) * Clamp(Dt / 0.15, 0.0, 1.0);
		}

		// Power for the next step.
		double Thermal = 1.0;
		if (NodeTemp < T.OptimumLow) { Thermal -= T.PowerPerDegCold * (T.OptimumLow - NodeTemp); }
		if (NodeTemp > T.OptimumHigh) { Thermal -= T.PowerPerDegHot * (NodeTemp - T.OptimumHigh); }
		Power = IsoPowerCorrection(Air) * MixturePowerFactor(Richness) * Clamp(Thermal, 0.5, 1.0) * (1.0 - 0.35 * Seize);
	}
}
