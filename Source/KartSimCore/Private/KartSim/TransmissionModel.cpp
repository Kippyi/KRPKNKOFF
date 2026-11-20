#include "KartSim/TransmissionModel.h"

#include <cmath>

namespace kartsim
{
	void TransmissionModel::Init(const KartConfig& InConfig)
	{
		Config = &InConfig;
		Reset();
	}

	void TransmissionModel::Reset()
	{
		// Single-speed karts sit permanently in gear 1; gearboxes start in neutral.
		Gear = Config->Gearbox.NumGears > 1 ? 0 : 1;
		bLocked = Config->Clutch.Type == EClutchType::DirectDrive && !bChainOff;
		ShiftCutTimer = 0.0;
	}

	void TransmissionModel::RequestShift(int Direction)
	{
		const int N = Config->Gearbox.NumGears;
		if (N <= 1 || Direction == 0)
		{
			return;
		}
		const int NewGear = static_cast<int>(Clamp(static_cast<double>(Gear + (Direction > 0 ? 1 : -1)), 0.0, static_cast<double>(N)));
		if (NewGear == Gear)
		{
			return;
		}
		// Sequential dog box: the old gear disengages, the box is in neutral for ShiftCutTime,
		// then the new gear engages (any speed mismatch is absorbed through clutch slip).
		ShiftCutTimer = Config->Gearbox.ShiftCutTime;
		Gear = NewGear;
		// A gear change breaks the rigid coupling; it re-locks once speeds match.
		if (Config->Clutch.Type != EClutchType::DirectDrive)
		{
			bLocked = false;
		}
	}

	void TransmissionModel::Update(double Dt)
	{
		if (ShiftCutTimer > 0.0) { ShiftCutTimer -= Dt; }
	}

	double TransmissionModel::GetTotalRatio() const
	{
		return (Gear >= 1 && ShiftCutTimer <= 0.0 && !bChainOff) ? Config->TotalRatio(Gear - 1) : 0.0;
	}

	double TransmissionModel::ClutchCapacity(double EngineOmega, double ClutchPedal) const
	{
		const ClutchParams& C = Config->Clutch;
		switch (C.Type)
		{
		case EClutchType::Centrifugal:
		{
			// Shoe normal force = m r w^2 - F_spring  =>  capacity ~ (w^2 - w0^2), reaching MaxTorque
			// at FullLockRpm (shoe/drum contact fully developed; capped beyond).
			const double W0 = C.EngageStartRpm * kRpmToRadS;
			const double Wf = C.FullLockRpm * kRpmToRadS;
			return C.MaxTorque * Clamp((EngineOmega * EngineOmega - W0 * W0) / (Wf * Wf - W0 * W0), 0.0, 1.0);
		}
		case EClutchType::Manual:
			return C.MaxTorque * (1.0 - Clamp(ClutchPedal, 0.0, 1.0));
		case EClutchType::DirectDrive:
		default:
			return 1e9;
		}
	}

	void TransmissionModel::Step(EngineModel& Engine, double EngineTorque, double ClutchPedal,
	                             double& AxleOmega, double AxleExternalTorque, double AxleInertia, double AxleImplicitDamping,
	                             double BrakeTorque, double Dt, DrivetrainStepResult& Out)
	{
		Out = DrivetrainStepResult();
		const double G = GetTotalRatio();
		const double Ie = Engine.GetInertia();
		const double Eta = Config->FinalDrive.ChainEfficiency;
		const bool bDirect = Config->Clutch.Type == EClutchType::DirectDrive;
		double We = Engine.GetOmega();
		const double ImplicitInertia = AxleImplicitDamping * Dt; // added to inertia in velocity updates

		const double Cap = (G > 0.0) ? ClutchCapacity(We, ClutchPedal) : 0.0;
		Out.ClutchCapacity = Cap;

		if (G <= 0.0 || (Cap <= 0.0 && !bDirect))
		{
			// Neutral or fully disengaged: engine and axle are independent.
			bLocked = false;
			We += EngineTorque / Ie * Dt;
			AxleOmega += AxleExternalTorque / (AxleInertia + ImplicitInertia) * Dt;
			ApplyCoulombTorque(AxleOmega, BrakeTorque, AxleInertia + ImplicitInertia, Dt);
			Engine.SetOmega(We);
			Out.SlipRpm = (Engine.GetOmega() - G * AxleOmega) * kRadSToRpm;
			return;
		}

		if (bLocked || bDirect)
		{
			// Rigid coupling: one inertia reflected to the axle.
			const double Drive = EngineTorque * G * (EngineTorque >= 0.0 ? Eta : 1.0 / Eta);
			const double Itot = Ie * G * G + AxleInertia + ImplicitInertia;
			const double W0 = AxleOmega;
			AxleOmega += (Drive + AxleExternalTorque) / Itot * Dt;
			ApplyCoulombTorque(AxleOmega, BrakeTorque, Itot, Dt);
			const double Alpha = (AxleOmega - W0) / Dt;
			const double ClutchTorqueReq = EngineTorque - Ie * G * Alpha;
			We = G * AxleOmega;
			Out.ClutchTorque = ClutchTorqueReq;
			if (We < 0.0)
			{
				// The engine cannot run backwards: decouple.
				We = 0.0;
				bLocked = bDirect;
			}
			else if (!bDirect && std::fabs(ClutchTorqueReq) > Cap)
			{
				bLocked = false;
			}
			Engine.SetOmega(We);
			Out.bLocked = bLocked || bDirect;
			Out.SlipRpm = 0.0;
			return;
		}

		// Slipping: kinetic clutch torque in the direction of slip.
		const double Slip = We - G * AxleOmega;
		const double Tc = Cap * Sign(Slip);
		const double WeNew = We + (EngineTorque - Tc) / Ie * Dt;
		const double Taxle = G * Tc * (Tc >= 0.0 ? Eta : 1.0 / Eta);
		double WaNew = AxleOmega + (Taxle + AxleExternalTorque) / (AxleInertia + ImplicitInertia) * Dt;
		ApplyCoulombTorque(WaNew, BrakeTorque, AxleInertia + ImplicitInertia, Dt);
		const double SlipNew = WeNew - G * WaNew;

		Out.ClutchTorque = Tc;
		if (Slip != 0.0 && (Sign(SlipNew) != Sign(Slip)))
		{
			// Slip reversed inside the step: lock at the momentum-conserving common speed.
			const double W = (Ie * G * WeNew + AxleInertia * WaNew) / (Ie * G * G + AxleInertia);
			AxleOmega = W;
			Engine.SetOmega(G * W);
			bLocked = true;
			Out.bLocked = true;
			Out.SlipRpm = 0.0;
			return;
		}
		AxleOmega = WaNew;
		Engine.SetOmega(WeNew);
		Out.SlipRpm = (Engine.GetOmega() - G * AxleOmega) * kRadSToRpm;
	}
}
