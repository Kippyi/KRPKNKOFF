// Optional driving aids that act ONLY on the driver inputs (never on the physics): automatic
// clutch for launches / anti-stall, rpm-based automatic shifting with a throttle lift, and a helper
// push for direct-drive karts whose engine is stopped. Shared by the Unreal input layer and the
// automated driver tests, so both use identical logic.
#pragma once

#include "KartSim/KartPhysicsCore.h"

#include <cmath>

namespace kartsim
{
	struct DriverAidSettings
	{
		bool bAutoClutch = true;   // manual-clutch karts: slip the clutch on launch, disengage near stall
		bool bAutoShift = false;   // gearbox karts: shift up/down by rpm
		bool bAutoPush = false;    // direct-drive karts: push until the engine fires
		double LaunchClutchStartRpm = 6000.0;
		double LaunchClutchFullRpm = 9000.0;
		// Launch control (manual-clutch karts, first gear): slip the clutch to hold LaunchHoldFraction of the rev
		// limit, lock it once road speed has caught up with those revs, and ease the throttle when the front goes
		// light (anti-wheelie). This is how a KZ driver starts; dumping the clutch at revs wheelies, hops and bogs.
		double LaunchHoldFraction = 0.75;
		double LaunchHoldGainRpm = 2500.0;   // rpm error for a full clutch-engagement swing
		double AntiWheelieFrontLoad = 0.25;  // front load (fraction of static) below which throttle is eased
		double AntiWheelieMinThrottle = 0.45;
		double LaunchLiftMargin = 0.8;       // launch drive force as a fraction of the front-lift force
		double ShiftClutchDip = 0.6;         // clutch pedal during an automatic upshift
		double AntiStallRpm = 4000.0;
		double LaunchSpeed = 8.0;          // m/s, below this the launch clutch logic is active
		double UpshiftFraction = 0.93;     // of rev limit
		double DownshiftFraction = 0.62;
		double PushForce = 300.0;          // N, roughly one helper pushing
	};

	inline void ApplyDriverAids(const KartSimulation& Sim, const DriverAidSettings& S, KartInputs& In)
	{
		const KartConfig& C = Sim.GetConfig();
		const TransmissionModel& Tr = Sim.GetTransmission();
		const double Rpm = Sim.GetEngine().GetRpm();
		const double V = Sim.ForwardSpeed();

		if (S.bAutoPush && C.Clutch.Type == EClutchType::DirectDrive && !Sim.GetEngine().IsRunning())
		{
			In.PushForce = S.PushForce;
			In.Throttle = 0.0;
			return;
		}
		if (S.bAutoClutch && C.Clutch.Type == EClutchType::Manual)
		{
			double Pedal = 0.0;
			const double Hold = S.LaunchHoldFraction * C.Engine.RevLimitRpm;
			const double RoadRpm = std::fabs(V) / C.RearTyre.UnloadedRadius * C.TotalRatio(0) * kRadSToRpm;
			if (Tr.GetGear() == 1 && RoadRpm < 1.1 * Hold)
			{
				// Anti-wheelie in first gear: ease the throttle (and, while slipping, the clutch) as the front unloads.
				const double StaticFront = C.Chassis.Mass * kGravity * C.Chassis.FrontWeightFraction;
				const double FrontLoad = (Sim.GetWheel(WheelFL).Tyre.Fz + Sim.GetWheel(WheelFR).Tyre.Fz) / StaticFront;
				const double Ease = Clamp(FrontLoad / S.AntiWheelieFrontLoad, 0.0, 1.0);
				In.Throttle = std::fmin(In.Throttle, S.AntiWheelieMinThrottle + (1.0 - S.AntiWheelieMinThrottle) * Ease);
				// Keep slipping (capped) until the slip has nearly closed, so the clutch locks without a jolt.
				if (RoadRpm < 0.97 * Hold || Rpm - RoadRpm > 300.0)
				{
					// Launch: slip the clutch, feed-forward to the engine torque at the hold rpm, plus rpm feedback; no
					// speed cap, the clutch locks once road speed has caught up with the hold rpm.
					// A slipping clutch transmits exactly its capacity, so capping it caps the drive force: keep the drive
					// below the force that lifts the front (m g * front fraction * wheelbase / CoG height), with margin.
					const ChassisParams& Ch = C.Chassis;
					const double LiftForce = Ch.Mass * kGravity * Ch.FrontWeightFraction * Ch.Wheelbase / Ch.CogHeight;
					const double G1 = C.TotalRatio(0) * C.FinalDrive.ChainEfficiency;
					const double CapMax = G1 > 0.0 ? S.LaunchLiftMargin * LiftForce * C.RearTyre.UnloadedRadius / G1 / C.Clutch.MaxTorque : 1.0;
					const double Feed = In.Throttle * C.Engine.TorqueWOT.Eval(Hold) / C.Clutch.MaxTorque;
					const double Engage = std::fmin(Clamp(Feed + (Rpm - Hold) / S.LaunchHoldGainRpm, 0.0, 1.0), CapMax);
					Pedal = 1.0 - Engage;
				}
			}
			else if (Tr.GetGear() <= 1 && V < S.LaunchSpeed)
			{
				const double Engage = Clamp((Rpm - S.LaunchClutchStartRpm) / (S.LaunchClutchFullRpm - S.LaunchClutchStartRpm), 0.0, 1.0);
				Pedal = 1.0 - Engage;
			}
			if (Rpm < S.AntiStallRpm && Tr.GetGear() >= 1) { Pedal = 1.0; }
			if (Tr.GetGear() == 0) { Pedal = 1.0; }
			In.Clutch = In.Clutch > Pedal ? In.Clutch : Pedal; // the driver's own clutch input still wins
		}
		if (S.bAutoShift && C.Gearbox.NumGears > 1 && In.ShiftRequest == 0)
		{
			const double Limit = C.Engine.RevLimitRpm;
			if (Tr.GetGear() == 0) { In.ShiftRequest = 1; }
			else if (!Tr.IsShifting())
			{
				// Downshift decisions use ROAD speed (engine rpm is distorted by wheelspin, hops and
				// clutch slip), and a downshift is only allowed if the lower gear stays below the upshift
				// point (no over-rev, no "1st gear at 70 km/h").
				const int G = Tr.GetGear();
				const double RoadOmega = std::fabs(V) / C.RearTyre.UnloadedRadius;
				const double RoadRpmLower = G > 1 ? RoadOmega * C.TotalRatio(G - 2) * kRadSToRpm : 1e9;
				const double RoadRpmNow = RoadOmega * C.TotalRatio(G - 1) * kRadSToRpm;
				if (Rpm > S.UpshiftFraction * Limit && RoadRpmNow > 0.8 * S.UpshiftFraction * Limit && G < C.Gearbox.NumGears)
				{
					In.ShiftRequest = 1;
				}
				else if (G > 1 && RoadRpmNow < S.DownshiftFraction * Limit && RoadRpmLower < 0.95 * S.UpshiftFraction * Limit)
				{
					In.ShiftRequest = -1;
				}
			}
		}
		if (C.Gearbox.NumGears > 1 && (Tr.IsShifting() || In.ShiftRequest != 0) && (S.bAutoShift))
		{
			In.Throttle = 0.0; // lift through the shift
			// Half-dip the clutch while the dogs engage: the engine is pulled down to the new gear's speed through
			// slip instead of a hard kick (with a manual clutch; the driver's own pedal still wins).
			if (S.bAutoClutch && C.Clutch.Type == EClutchType::Manual && Tr.GetGear() > 1) { In.Clutch = std::fmax(In.Clutch, S.ShiftClutchDip); }
		}
		// After an automatic upshift the dogs are in but the engine is still above the new gear's speed: keep the dip
		// until the slip has nearly closed (then the clutch locks smoothly).
		if (S.bAutoShift && S.bAutoClutch && C.Clutch.Type == EClutchType::Manual && Tr.GetGear() > 1 && !Tr.IsClutchLocked() && V > S.LaunchSpeed)
		{
			const double GearRoadRpm = std::fabs(V) / C.RearTyre.UnloadedRadius * C.TotalRatio(Tr.GetGear() - 1) * kRadSToRpm;
			if (Rpm - GearRoadRpm > 300.0) { In.Clutch = std::fmax(In.Clutch, S.ShiftClutchDip); }
		}
	}
}
