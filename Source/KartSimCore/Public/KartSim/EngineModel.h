// Engine: WOT torque map, closed-throttle drag (engine braking), throttle lag, idle control,
// ignition-cut rev limiter, running/stalled state and starter. Docs/PhysicsModel.md section 5.1.
#pragma once

#include "KartSim/KartConfig.h"

namespace kartsim
{
	class KARTSIM_API EngineModel
	{
	public:
		void Init(const EngineParams& InParams);
		// bStartRunning = false leaves the engine stopped (e.g. a direct-drive kart at rest).
		void Reset(double Rpm, bool bStartRunning = true);

		// Updates throttle lag / limiter / running state and returns net crank torque [Nm] at the
		// current speed. bIgnitionCut forces zero combustion torque (gearshift cut, kill switch).
		// bStarter engages the onboard starter (no effect when StarterTorque = 0).
		double UpdateTorque(double ThrottlePedal, bool bIgnitionCut, double Dt, bool bStarter = false);

		double GetOmega() const { return Omega; }
		void SetOmega(double W) { Omega = W > 0.0 ? W : 0.0; }
		double GetRpm() const { return Omega * kRadSToRpm; }
		double GetInertia() const { return P->Inertia; }
		double GetEffectiveThrottle() const { return EffectiveThrottle; }
		bool IsLimiterActive() const { return bLimiterCut; }
		bool IsRunning() const { return bRunning; }
		bool HasStarter() const { return P->StarterTorque > 0.0; }
		double GetLastTorque() const { return LastTorque; }
		double GetLastCombustionTorque() const { return LastCombustion; }

		// Tuning / condition (EngineCondition): multiplies the combustion torque (air, mixture, heat).
		void SetPowerFactor(double F) { PowerFactor = F; }
		double GetPowerFactor() const { return PowerFactor; }
		// A seized engine stops firing and adds DragTorque at the crank (it cannot be restarted until repaired).
		void SetSeized(bool bSeize, double DragTorque) { bSeized = bSeize; SeizedDrag = DragTorque; if (bSeize) { bRunning = false; } }
		bool IsSeized() const { return bSeized; }

		// Pure functions (for tests and tooling).
		double WotTorque(double Rpm) const { return P->TorqueWOT.Eval(Rpm); }
		double DragTorque(double W) const { return P->DragTorqueConst + P->DragTorquePerRadS * W; }

	private:
		const EngineParams* P = nullptr;
		double Omega = 0.0;
		double LaggedPedal = 0.0;
		double EffectiveThrottle = 0.0;
		double LastTorque = 0.0;
		double LastCombustion = 0.0;
		double PowerFactor = 1.0;
		double SeizedDrag = 0.0;
		bool bSeized = false;
		bool bLimiterCut = false;
		bool bRunning = true;
	};
}
