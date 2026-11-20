#include "KartSim/EngineModel.h"

namespace kartsim
{
	void EngineModel::Init(const EngineParams& InParams)
	{
		P = &InParams;
		Reset(P->IdleRpm);
	}

	void EngineModel::Reset(double Rpm, bool bStartRunning)
	{
		Omega = Rpm * kRpmToRadS;
		LaggedPedal = 0.0;
		EffectiveThrottle = 0.0;
		LastTorque = 0.0;
		bLimiterCut = false;
		bRunning = bStartRunning;
	}

	double EngineModel::UpdateTorque(double ThrottlePedal, bool bIgnitionCut, double Dt, bool bStarter)
	{
		const double Pedal = Clamp(ThrottlePedal, 0.0, 1.0);
		const double Alpha = Dt / (P->ThrottleTimeConstant + Dt);
		LaggedPedal += (Pedal - LaggedPedal) * Alpha;

		const double Rpm = GetRpm();

		// Running state: stall below StallRpm, fire when turned above FireRpm with ignition on.
		if (bRunning && Rpm < P->StallRpm) { bRunning = false; }
		else if (!bRunning && !bIgnitionCut && !bSeized && Rpm > P->FireRpm) { bRunning = true; }
		if (bSeized) { bRunning = false; }

		double Throttle = 0.0;
		if (bRunning)
		{
			// Idle control: opens the throttle as rpm falls below idle (idle screw + carb mixture).
			const double Deficit = (P->IdleRpm - Rpm) / P->IdleRpm;
			const double IdleThrottle = Clamp(P->IdleControlGain * Deficit + 0.5 * P->IdleMaxThrottle, 0.0, P->IdleMaxThrottle);
			Throttle = LaggedPedal > IdleThrottle ? LaggedPedal : IdleThrottle;

			// Rev limiter: ignition cut with hysteresis.
			if (Rpm > P->RevLimitRpm) { bLimiterCut = true; }
			else if (Rpm < P->RevLimitRpm - P->RevLimitHysteresisRpm) { bLimiterCut = false; }
			if (bLimiterCut || bIgnitionCut) { Throttle = 0.0; }
		}

		EffectiveThrottle = Throttle;
		const double Combustion = Throttle * WotTorque(Rpm) * PowerFactor;
		LastCombustion = Combustion;
		// The WOT curve is brake torque (already net of friction). Closed-throttle drag blends in as
		// the throttle closes; a stopped engine is pure drag (pumping + friction).
		const double Drag = (1.0 - Throttle) * DragTorque(Omega);
		const double Starter = (bStarter && !bRunning) ? P->StarterTorque : 0.0;
		LastTorque = Combustion + Starter - (Omega > 0.0 ? Drag + (bSeized ? SeizedDrag : 0.0) : 0.0);
		return LastTorque;
	}
}
