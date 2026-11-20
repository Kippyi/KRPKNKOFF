#include "KartSim/BrakeModel.h"

namespace kartsim
{
	void BrakeModel::Init(const BrakeParams& InParams)
	{
		P = &InParams;
		Reset();
	}

	void BrakeModel::Reset()
	{
		DiscTemp = P->InitialTemp;
	}

	double BrakeModel::FadeFactor() const
	{
		const double Over = DiscTemp - P->FadeStartTemp;
		return Over > 0.0 ? Clamp(1.0 - P->FadePerDeg * Over, 0.3, 1.0) : 1.0;
	}

	double BrakeModel::RearTorque(double Pedal) const
	{
		return Clamp(Pedal, 0.0, 1.0) * P->RearMaxTorque * FadeFactor();
	}

	double BrakeModel::FrontTorque(double Pedal) const
	{
		return Clamp(Pedal, 0.0, 1.0) * P->FrontMaxTorque * FadeFactor();
	}

	void BrakeModel::UpdateThermal(double AbsorbedPower, double KartSpeed, double AirTemp, double Dt)
	{
		const double H = P->DiscCoolingBase + P->DiscCoolingPerSpeed * KartSpeed;
		DiscTemp += Dt * (AbsorbedPower - H * (DiscTemp - AirTemp)) / P->DiscHeatCapacity;
	}
}
