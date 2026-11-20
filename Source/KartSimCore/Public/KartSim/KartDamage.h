// Mechanical damage state of one kart (Milestone 8). Accumulated by KartSimulation from kart-to-kart
// contact forces (KartContact -> KartSimulation::AddContactForce) and from tyre load spikes (kerb strikes,
// landings). Effects:
//  * bent steering arm / track rod: per-side toe change (SteeringModel::SetDamageToe)
//  * bent rear axle: axle friction torque (scrubbing bearings / wobbling wheels)
//  * thrown chain: engine disconnected from the axle (TransmissionModel::SetChainOff)
//  * radiator holed (side impact on the radiator side): coolant leak -> overheating -> seizure
//  * front fairing pushed back (CIK nose release kit): flag only (penalty in the race rules)
// Engine failures (lean / overheat seizure) live in EngineCondition. Thresholds: DamageParams (ASSUMPTIONS).
#pragma once

#include <string>

namespace kartsim
{
	struct KartDamageState
	{
		double ToeDeg[2] = {0.0, 0.0};   // toe-in change, [0] = left front, [1] = right front
		double AxleBendDeg = 0.0;
		bool bChainOff = false;
		bool bFairingDisplaced = false;
		bool bSidepodKnocked[2] = {false, false}; // [0] left, [1] right: knocked loose by a side impact (visual)
		bool bBumperKnocked = false;              // rear bumper pushed in by a rear impact (visual)
		double RadiatorLeak = 0.0;       // coolant fraction lost per second
		// Impulse (N s above the threshold) of the contact event in progress, per zone: front, rear, left, right.
		double EventImpulse[4] = {0.0, 0.0, 0.0, 0.0};
		int NumEvents = 0;               // increments on every new damage (HUD message)
		std::string LastEvent;
		double LastEventTime = -1.0;

		bool Any() const
		{
			return ToeDeg[0] != 0.0 || ToeDeg[1] != 0.0 || AxleBendDeg != 0.0 || bChainOff || bFairingDisplaced || RadiatorLeak > 0.0 ||
			       bSidepodKnocked[0] || bSidepodKnocked[1] || bBumperKnocked;
		}
	};
}
