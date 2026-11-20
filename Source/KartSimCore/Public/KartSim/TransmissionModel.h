// Transmission: clutch (centrifugal / manual / direct drive) with stick-slip, gearbox, chain final
// drive, and the coupled engine <-> solid rear axle integration. Docs/PhysicsModel.md section 5.
#pragma once

#include "KartSim/EngineModel.h"
#include "KartSim/KartConfig.h"

namespace kartsim
{
	// Coulomb friction torque applied to a spinning body over Dt: decelerates toward zero, never reverses.
	inline void ApplyCoulombTorque(double& Omega, double Torque, double Inertia, double Dt)
	{
		const double DW = Torque * Dt / Inertia;
		if (Omega <= DW && Omega >= -DW) { Omega = 0.0; }
		else { Omega -= (Omega > 0.0 ? DW : -DW); }
	}

	struct DrivetrainStepResult
	{
		double ClutchTorque = 0.0;      // Nm transmitted at the crank side
		double ClutchCapacity = 0.0;    // Nm
		double SlipRpm = 0.0;           // crank rpm - clutch drum rpm
		bool bLocked = false;
	};

	class KARTSIM_API TransmissionModel
	{
	public:
		void Init(const KartConfig& InConfig);
		void Reset();

		// +1 upshift, -1 downshift. Single-speed karts ignore shifts.
		void RequestShift(int Direction);
		void Update(double Dt); // shift timers

		// 0 = neutral (multi-speed only), 1..N = gears. During a shift this is the TARGET gear.
		int GetGear() const { return Gear; }
		double GetTotalRatio() const; // engine rad per axle rad; 0 in neutral or while shifting
		// True while the dogs are between gears (gearbox.shiftCutTime, transmits no torque).
		bool IsShifting() const { return ShiftCutTimer > 0.0; }
		bool IsClutchLocked() const { return bLocked; }
		// Damage: a thrown chain disconnects the engine from the axle (survives Reset; cleared by the host).
		void SetChainOff(bool bOff) { bChainOff = bOff; if (bOff) { bLocked = false; } }
		bool IsChainOff() const { return bChainOff; }
		double ClutchCapacity(double EngineOmega, double ClutchPedal) const;

		// Integrates engine speed and rear-axle speed over Dt, including clutch stick/slip and
		// the rear brake (Coulomb). AxleExternalTorque: tyre reaction + rolling resistance at the axle.
		// AxleImplicitDamping [Nm s/rad]: -d(external torque)/d(omega), treated implicitly
		// (linearised backward Euler) so stiff low-speed tyre damping cannot destabilise the axle.
		void Step(EngineModel& Engine, double EngineTorque, double ClutchPedal,
		          double& AxleOmega, double AxleExternalTorque, double AxleInertia, double AxleImplicitDamping,
		          double BrakeTorque, double Dt, DrivetrainStepResult& Out);

	private:
		const KartConfig* Config = nullptr;
		int Gear = 1;
		bool bLocked = false;
		bool bChainOff = false;
		double ShiftCutTimer = 0.0;
	};
}
