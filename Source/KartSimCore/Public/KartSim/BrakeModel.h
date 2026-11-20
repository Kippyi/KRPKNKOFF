// Brakes: pedal -> torque (rear axle disc, optional front discs), disc temperature and fade.
// Lock-up handling (Coulomb clamping) is applied where the torque acts. Docs/PhysicsModel.md section 6.
#pragma once

#include "KartSim/KartConfig.h"

namespace kartsim
{
	class KARTSIM_API BrakeModel
	{
	public:
		void Init(const BrakeParams& InParams);
		void Reset();

		double RearTorque(double Pedal) const;
		double FrontTorque(double Pedal) const; // per wheel
		bool HasFrontBrakes() const { return P->FrontMaxTorque > 0.0; }

		// AbsorbedPower: braking power dissipated in the rear disc [W].
		void UpdateThermal(double AbsorbedPower, double KartSpeed, double AirTemp, double Dt);

		double GetDiscTemp() const { return DiscTemp; }
		double FadeFactor() const;

	private:
		const BrakeParams* P = nullptr;
		double DiscTemp = 30.0;
	};
}
