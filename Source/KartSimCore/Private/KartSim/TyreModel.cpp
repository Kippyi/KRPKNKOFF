#include "KartSim/TyreModel.h"

#include <cmath>

namespace kartsim
{
	namespace
	{
		constexpr double kLowSpeedDampingSpeed = 1.5; // m/s, below this the viscous low-speed term fades in
		constexpr double kReverseThreshold = -0.3;   // m/s, below this the slip frame is mirrored
	}

	bool TyreModel::bEnableEnveloping = true;

	void TyreModel::Init(const TyreParams& InParams)
	{
		P = &InParams;
		Wear = 0.0;
		Reset();
	}

	void TyreModel::Reset()
	{
		SlipX = 0.0;
		SlipY = 0.0;
		SigScale = 1.0;
		Temperature = P ? P->InitialTemp : 30.0;
		GasTemp = ColdGasTemp = Temperature;
		PrevPenetration = 0.0;
		bHasPrevPenetration = false;
		FilteredRoadRate = 0.0;
	}

	bool TyreModel::ComputeContact(const TyreParams& Params, const Vec3& WheelCentre, const Vec3& Axle,
	                               const ITrackQuery& Track, TyreContact& Out)
	{
		Out = TyreContact();
		Out.WheelCentre = WheelCentre;
		Out.Axle = Axle;

		const double R0 = Params.UnloadedRadius;
		GroundHit Hit;
		if (!Track.QueryGround(WheelCentre, R0, 3.0 * R0, Hit))
		{
			return false;
		}

		// --- Tyre enveloping: effective road plane ---
		// A tyre straddles short obstacles over its contact zone instead of following the road at one
		// point. Sample the ground at 5 points along the heading (+/-0.5 R) and 2 across the width,
		// fit a weighted plane (height + slopes) and contact the disc against it. On a plane this is
		// exact (identical to the single-point query); on steps/kerb edges it gives a progressive ramp.
		// Ref: effective-road-plane enveloping (Schmeisser; Zegelaar; Pacejka "Tyre and Vehicle Dynamics" ch.10).
		Vec3 N = Hit.Normal;
		Vec3 PlanePoint = Hit.Point;
		bool bEnveloped = false;
		{
			const Vec3 Up(0.0, 0.0, 1.0);
			const Vec3 FwdH = Axle.Cross(Up).Normalized();
			const Vec3 LatH = Up.Cross(FwdH);
			const double Dx = 0.25 * R0, Dy = 0.35 * Params.Width;
			const double Xs[7] = {-2 * Dx, -Dx, 0.0, Dx, 2 * Dx, 0.0, 0.0};
			const double Ys[7] = {0.0, 0.0, 0.0, 0.0, 0.0, -Dy, Dy};
			const double Ws[7] = {0.4, 0.8, 1.0, 0.8, 0.4, 0.8, 0.8};
			double Zs[7];
			bool bAll = bEnableEnveloping && FwdH.LengthSq() > 0.5;
			for (int K = 0; K < 7 && bAll; ++K)
			{
				if (K == 2) { Zs[K] = Hit.Point.Z; continue; }
				GroundHit H;
				const Vec3 P = WheelCentre + FwdH * Xs[K] + LatH * Ys[K];
				if (!Track.QueryGround(P, R0, 3.0 * R0, H)) { bAll = false; break; }
				Zs[K] = H.Point.Z;
			}
			if (bAll)
			{
				// Symmetric sample pattern: the weighted normal equations decouple.
				double Sw = 0, Swz = 0, Swxx = 0, Swxz = 0, Swyy = 0, Swyz = 0;
				for (int K = 0; K < 7; ++K)
				{
					Sw += Ws[K]; Swz += Ws[K] * Zs[K];
					Swxx += Ws[K] * Xs[K] * Xs[K]; Swxz += Ws[K] * Xs[K] * Zs[K];
					Swyy += Ws[K] * Ys[K] * Ys[K]; Swyz += Ws[K] * Ys[K] * Zs[K];
				}
				const double H0 = Swz / Sw, Gx = Swxz / Swxx, Gy = Swyz / Swyy;
				N = (Up - FwdH * Gx - LatH * Gy).Normalized();
				PlanePoint = Vec3(WheelCentre.X, WheelCentre.Y, H0);
				bEnveloped = true;
			}
		}
		const double NA = N.Dot(Axle);
		const Vec3 Radial = N - Axle * NA;                // in the wheel plane, toward +normal
		const double RadialLen = Radial.Length();
		if (RadialLen < 1e-6)
		{
			return false; // wheel lying flat on its side
		}
		const Vec3 U = Radial / RadialLen;
		// Enveloped: penetration from the effective HEIGHT (vertical), slope only tilts the force
		// direction; measuring along the tilted normal would add spurious penetration at edges.
		const double CentreHeight = bEnveloped ? (WheelCentre.Z - PlanePoint.Z) : (WheelCentre - PlanePoint).Dot(N);
		const double Reach = bEnveloped ? R0 * std::sqrt(1.0 - Axle.Z * Axle.Z) : R0 * RadialLen;
		const double Pen = Reach - CentreHeight;

		Out.Normal = N;
		Out.ContactPoint = WheelCentre - U * R0;
		Out.Penetration = Pen;
		Out.Forward = Axle.Cross(N).Normalized();
		Out.Lateral = N.Cross(Out.Forward);
		Out.Surface = Hit.Surface;
		const SurfaceProperties& SP = GetSurfaceProperties(Hit.Surface);
		Out.GripFactor = SP.GripFactor * Hit.GripMultiplier;
		Out.RollingResFactor = SP.RollingResistanceFactor;
		Out.Wetness = Hit.Surface == ESurfaceType::WetAsphalt ? 1.0 : Clamp(Hit.Wetness, 0.0, 1.0);

		const double D = Pen > 0.0 ? Pen : 0.0;
		Out.LoadedRadius = R0 - D / 3.0;
		const double Chord = 2.0 * R0 * D - D * D;
		Out.HalfLength = Chord > 0.0 ? std::sqrt(Chord) : 0.0;
		if (Out.HalfLength < 0.005) { Out.HalfLength = 0.005; }
		Out.bInContact = Pen > 0.0;
		return true;
	}

	double TyreModel::GetPressure() const
	{
		if (!P || P->ColdPressure <= 0.0) { return 0.0; }
		constexpr double Atm = 1.01325; // bar
		return (P->ColdPressure + Atm) * (GasTemp + 273.15) / (ColdGasTemp + 273.15) - Atm;
	}

	double TyreModel::TemperatureGripFactor(double T) const
	{
		const double X = (T - P->OptimumTemp) / P->TempWindow;
		const double Q = X * X < 1.0 ? X * X : 1.0;
		return 1.0 - (1.0 - P->MinTempGripFactor) * Q;
	}

	double TyreModel::WearGripFactor() const
	{
		return 1.0 - Wear * Wear * (1.0 - P->MinWearGripFactor);
	}

	double TyreModel::WetGripFactor(double Wetness) const
	{
		return 1.0 - (1.0 - P->WetGripFloor) * Clamp(Wetness, 0.0, 1.0);
	}

	double TyreModel::WearTempFactor(double T) const
	{
		return Clamp(std::pow(2.0, (T - P->OptimumTemp) / P->WearTempDoubling), 0.25, 16.0);
	}

	void TyreModel::BrushForces(double Fz, double KappaEff, double SlipLat, double Mu, double HalfLength,
	                            double& OutFx, double& OutFy, double& OutMz, double& OutLambda) const
	{
		OutFx = OutFy = OutMz = OutLambda = 0.0;
		if (Fz <= 1e-3 || Mu <= 1e-6)
		{
			return;
		}
		const double LoadRatio = Fz / P->RefLoad;
		const double StiffScale = std::pow(LoadRatio, P->StiffnessLoadExponent);
		const double Cx = P->CxCoeff * P->RefLoad * StiffScale;
		const double Cy = P->CyCoeff * P->RefLoad * StiffScale;

		const double Denom = (1.0 + KappaEff) > 0.05 ? (1.0 + KappaEff) : 0.05;
		const double SigX = KappaEff / Denom;
		const double SigY = SlipLat / Denom;

		const double MuFz = Mu * Fz;
		const double LamX = Cx / (3.0 * MuFz) * SigX;
		const double LamY = Cy / (3.0 * MuFz) * SigY;
		const double Lam = std::sqrt(LamX * LamX + LamY * LamY);
		OutLambda = Lam;
		if (Lam < 1e-9)
		{
			return;
		}
		double F;
		if (Lam < 1.0)
		{
			const double OneMinus = 1.0 - Lam;
			const double OneMinus3 = OneMinus * OneMinus * OneMinus;
			F = MuFz * (1.0 - OneMinus3);
			OutMz = MuFz * HalfLength * LamY * OneMinus3;
		}
		else
		{
			F = MuFz;
		}
		OutFx = F * LamX / Lam;
		OutFy = -F * LamY / Lam;
	}

	void TyreModel::SteadyStateForces(double Fz, double Kappa, double InSlipY, double Mu, double& OutFx, double& OutFy) const
	{
		double Mz = 0.0, Lam = 0.0;
		BrushForces(Fz, Kappa, InSlipY, Mu, 0.02, OutFx, OutFy, Mz, Lam);
	}

	void TyreModel::Evaluate(const TyreContact& C, const Vec3& ContactVel, const Vec3& CentreVel,
	                         double WheelOmega, double Dt, TyreOutput& Out, const Vec3* MountVel)
	{
		Out = TyreOutput();
		// Penetration rate = wheel motion (exact) + road rising/falling under the wheel. The road part is
		// the change in penetration not explained by the wheel's own motion; a real tyre feels a road
		// profile spread over its contact patch, so that part is low-passed with the time the patch takes
		// to roll over it (contact length / speed). Without the filter a ground STEP (track edge lip, kerb
		// end, mesh seam) made the damper see metres per second for one step: load spikes of 1-4 kN and
		// the "all four tyres spazzing" reported in game (Docs/Milestone7MenusSetupSound.md).
		const double WheelRate = -CentreVel.Dot(C.Normal);
		double PenRate = WheelRate;
		if (bHasPrevPenetration && Dt > 0.0 && C.bInContact)
		{
			const double RoadRate = (C.Penetration - PrevPenetration) / Dt - WheelRate;
			const double Speed = std::sqrt(ContactVel.X * ContactVel.X + ContactVel.Y * ContactVel.Y);
			const double PatchLength = std::fmax(2.0 * C.HalfLength, 0.04);
			const double Tau = PatchLength / std::fmax(Speed, 0.5);
			FilteredRoadRate += (RoadRate - FilteredRoadRate) * (Dt / (Tau + Dt));
			PenRate = WheelRate + FilteredRoadRate;
		}
		else
		{
			FilteredRoadRate = 0.0;
		}
		PrevPenetration = C.Penetration;
		bHasPrevPenetration = C.bInContact;

		const double Vx = ContactVel.Dot(C.Forward);
		const double Vy = ContactVel.Dot(C.Lateral);
		const double Re = C.LoadedRadius > 0.0 ? C.LoadedRadius : P->UnloadedRadius;
		const double AbsVx = std::fabs(Vx);
		Out.Vx = Vx;
		Out.Vy = Vy;
		Out.SlipRatio = SlipX;
		Out.SlipAngle = std::atan(-SlipY);

		Cache = FCache();
		Cache.bInContact = C.bInContact;
		Cache.Vx = Vx;
		Cache.Vy = Vy;
		Cache.Re = Re;
		Cache.Speed = C.bInContact ? std::sqrt(Vx * Vx + Vy * Vy) : CentreVel.Length();

		if (!C.bInContact)
		{
			return; // airborne: no forces
		}

		// --- Vertical force (radial spring-damper, unilateral) ---
		// Carcass damping acts on the whole penetration rate; the seated driver's (lumped) damping only on
		// the wheel's own vertical motion, so it calms chassis bounce without making road bumps harsher.
		const double BodyRate = MountVel ? -MountVel->Dot(C.Normal) : WheelRate;
		const double Kf = P->StiffnessFactor(GetPressure());
		Cache.StiffFactor = Kf;
		double Fz = P->RadialStiffness * Kf * C.Penetration + P->RadialDamping * PenRate + P->BodyDamping * BodyRate;
		if (Fz < 0.0) { Fz = 0.0; }

		// --- Relaxation length at a constant carcass stiffness (Pacejka, Tyre and Vehicle Dynamics 7.2) ---
		// sigma = C(Fz) / k_carcass: a tyre with more load (larger patch, stiffer brush) relaxes over a longer distance.
		// With sigma held constant instead, a load change scaled the force at an unchanged carcass deflection - a spring
		// whose stiffness changes under load, which adds energy (found by the energy audit, Milestone 9). The slip
		// states are rescaled so the deflection sigma * slip carries over. Lower inflation = softer carcass: longer sigma.
		{
			const double SigNew = std::pow(Clamp(Fz / P->RefLoad, 0.2, 3.0), P->StiffnessLoadExponent) / std::sqrt(Kf);
			if (C.bInContact && Fz > 1.0)
			{
				SlipX *= SigScale / SigNew;
				SlipY *= SigScale / SigNew;
				SigScale = SigNew;
			}
			Cache.SigScale = SigScale;
		}

		// --- Friction coefficient ---
		const double SlipVelX = Vx - WheelOmega * Re;
		const double SlipVelY = Vy;
		const double SlideSpeed = std::sqrt(SlipVelX * SlipVelX + SlipVelY * SlipVelY);
		// Friction falls with the contact pressure. Patch area ~ deflection^0.5 at a given load, so the contact pressure
		// goes with sqrt(Fz * k): a stiffer (harder pumped) tyre sees the same as a higher load.
		const double LoadRatio = Fz / P->RefLoad * Kf;
		const double LoadFactor = Clamp(1.0 - P->MuLoadSensitivity * (LoadRatio - 1.0), 0.5, 1.3);
		const double SlideFactor = 1.0 - P->SlideMuDrop * (1.0 - std::exp(-SlideSpeed / P->SlideRefSpeed));
		const double WetFactor = WetGripFactor(C.Wetness);
		const double Mu = P->Mu0 * C.GripFactor * WetFactor * LoadFactor * TemperatureGripFactor(Temperature) * WearGripFactor() * SlideFactor;
		Out.WetFactor = WetFactor;
		Cache.Wetness = C.Wetness;

		// --- Brush forces from the current lagged slip state ---
		const double Dir = Vx < kReverseThreshold ? -1.0 : 1.0;
		double Fx = 0.0, Fy = 0.0, Mz = 0.0, Lam = 0.0;
		// Camber thrust: a tyre leaning by gamma pushes toward the side it leans, modelled as an
		// equivalent lateral slip C_gamma*gamma / C_y = ratio*gamma (linear, combined via the brush).
		// For a wheel leaning left the axle dips on the left: gamma = asin(-axle . n).
		const double Camber = std::asin(Clamp(-C.Axle.Dot(C.Normal), -1.0, 1.0));
		Out.Camber = Camber;
		const double SlipLatEff = SlipY - P->CamberStiffnessRatio * Camber;
		BrushForces(Fz, Dir * SlipX, SlipLatEff, Mu, C.HalfLength, Fx, Fy, Mz, Lam);
		if (Fz > 1e-3 && Mu > 1e-6)
		{
			const double Scale = std::pow(Fz / P->RefLoad, P->StiffnessLoadExponent) * P->RefLoad;
			Cache.ThetaX = P->CxCoeff * Scale / (3.0 * Mu * Fz);
			Cache.ThetaY = P->CyCoeff * Scale / (3.0 * Mu * Fz);
			Cache.Dir = Dir;
			Cache.CamberSlip = P->CamberStiffnessRatio * Camber;
		}
		Fx *= Dir;

		// --- Low-speed viscous damping of the slip-state spring (fades out above 1.5 m/s) ---
		const double LowW = Clamp(1.0 - AbsVx / kLowSpeedDampingSpeed, 0.0, 1.0);
		if (LowW > 0.0 && Fz > 1.0)
		{
			const double CornerMass = Fz / kGravity;
			// Carcass stiffness C(Fz)/sigma(Fz) = C_ref * sqrt(Kf) / sigma_ref (load-independent).
			const double Kx = P->CxCoeff * P->RefLoad / P->RelaxLengthX * std::sqrt(Kf);
			const double Ky = P->CyCoeff * P->RefLoad / P->RelaxLengthY * std::sqrt(Kf);
			const double Dx = 2.0 * 0.5 * std::sqrt(Kx * CornerMass);
			const double Dy = 2.0 * 0.5 * std::sqrt(Ky * CornerMass);
			Fx += -Dx * SlipVelX * LowW;
			Fy += -Dy * SlipVelY * LowW;
			Out.LongitudinalDamping = Dx * LowW;
			const double FMag = std::sqrt(Fx * Fx + Fy * Fy);
			const double FMax = Mu * Fz;
			if (FMag > FMax && FMag > 1e-9)
			{
				Fx *= FMax / FMag;
				Fy *= FMax / FMag;
				Out.LongitudinalDamping *= FMax / FMag;
			}
		}

		// --- Rolling resistance: a moment on the wheel, not a force (it reaches the kart via slip) ---
		const double Crr = P->RollingResistance * C.RollingResFactor / Kf; // hysteresis ~ deflection
		Out.RollingResistanceTorque = Crr * Fz * Re * std::tanh(WheelOmega * Re / 0.5);

		Out.Fx = Fx;
		Out.Fy = Fy;
		Out.Fz = Fz;
		Out.Mz = Mz;
		Out.SlipVelX = SlipVelX;
		Out.SlipVelY = SlipVelY;
		Out.Mu = Mu;
		Out.CombinedSlip = Lam;
		Out.ForceWorld = C.Forward * Fx + C.Lateral * Fy + C.Normal * Fz;

		Cache.SlidePower = std::fabs(Fx * SlipVelX) + std::fabs(Fy * SlipVelY);
		Cache.RollPower = Crr * Fz * AbsVx;
		Out.SlidePower = Cache.SlidePower;
	}

	void TyreModel::Integrate(double WheelOmegaNew, double AmbientTemp, double TrackTemp, double Dt)
	{
		if (!Cache.bInContact)
		{
			// Airborne: slip states relax quickly, tyre only cools by convection.
			const double Decay = 1.0 / (1.0 + Dt * 30.0);
			SlipX *= Decay;
			SlipY *= Decay;
			const double H = P->ConvectionBase + P->ConvectionPerSpeed * Cache.Speed;
			Temperature += Dt * (-H * (Temperature - AmbientTemp)) / P->HeatCapacity;
			GasTemp += (Temperature - GasTemp) * (Dt / (P->GasTimeConstant + Dt));
			return;
		}

		// --- Transient slip states (implicit in the |Vx| relaxation term: unconditionally stable) ---
		const double AbsVx = std::fabs(Cache.Vx);
		const double SigX = P->RelaxLengthX * Cache.SigScale, SigY = P->RelaxLengthY * Cache.SigScale;
		SlipX = (SlipX + Dt / SigX * (WheelOmegaNew * Cache.Re - Cache.Vx)) / (1.0 + Dt * AbsVx / SigX);
		SlipY = (SlipY + Dt / SigY * Cache.Vy) / (1.0 + Dt * AbsVx / SigY);
		SlipX = Clamp(SlipX, -1.0, 50.0);
		SlipY = Clamp(SlipY, -5.0, 5.0);

		// Carcass deflection cannot exceed the sliding boundary (brush lambda <= 1): beyond it the tread
		// slides. Without this cap the lagged states "remember" slip at low speed and push the kart
		// after the sliding direction reverses (energy injection, found by the energy audit test).
		// At lambda = 1 the force is mu*Fz either way, so saturated forces are unchanged.
		if (Cache.ThetaX > 0.0)
		{
			const double KEff = Cache.Dir * SlipX;
			const double Denom = (1.0 + KEff) > 0.05 ? (1.0 + KEff) : 0.05;
			double Sx = KEff / Denom;
			double Sy = (SlipY - Cache.CamberSlip) / Denom;
			const double Lam = std::sqrt(Sq(Cache.ThetaX * Sx) + Sq(Cache.ThetaY * Sy));
			if (Lam > 1.0)
			{
				Sx /= Lam;
				Sy /= Lam;
				const double KNew = Sx < 0.95 ? Sx / (1.0 - Sx) : 19.0;
				SlipX = Cache.Dir * KNew;
				SlipY = Sy * (1.0 + KNew) + Cache.CamberSlip;
			}
		}

		// --- Thermal & wear ---
		const double H = P->ConvectionBase + P->ConvectionPerSpeed * Cache.Speed;
		const double Q = P->SlideHeatFraction * Cache.SlidePower + Cache.RollPower
			- (H + P->WetCooling * Cache.Wetness) * (Temperature - AmbientTemp) - P->TrackConduction * (Temperature - TrackTemp);
		Temperature += Dt * Q / P->HeatCapacity;
		GasTemp += (Temperature - GasTemp) * (Dt / (P->GasTimeConstant + Dt));
		Wear = Clamp(Wear + P->WearRate * WearTempFactor(Temperature) * Cache.SlidePower * Dt, 0.0, 1.0);
	}
}
