// Energy audit: with no driver inputs, friction and damping can only remove energy. Any growth in
// total mechanical energy exposes numerical energy injection (a classic source of "hopping").
#include "TestFramework.h"
#include "TestHelpers.h"

#include <cstdlib>

using namespace kartsim;
using kstest::kTick;

namespace
{
	double TotalEnergy(const KartSimulation& Sim)
	{
		const KartConfig& C = Sim.GetConfig();
		const ChassisState& S = Sim.GetChassis();
		const Vec3& I = C.Chassis.Inertia;
		const Vec3& W = S.AngularVelocity;
		double E = 0.5 * C.Chassis.Mass * S.Velocity.LengthSq()
			+ 0.5 * (I.X * W.X * W.X + I.Y * W.Y * W.Y + I.Z * W.Z * W.Z)
			+ C.Chassis.Mass * kGravity * S.Position.Z;
		// Frame torsion springs and twist DOF kinetic energy.
		(void)S;
		E += Sim.GetChassisModel().FrameElasticEnergy() + Sim.GetChassisModel().FrameTwistKineticEnergy();
		// Milestone 9 flex: frame bending and the wheel mounts.
		E += Sim.GetChassisModel().BendingElasticEnergy() + Sim.GetChassisModel().BendingKineticEnergy() + Sim.MountEnergy();
		// Tyre radial springs and wheel/axle/engine spin.
		for (int K = 0; K < NumWheels; ++K)
		{
			const WheelState& Wh = Sim.GetWheel(K);
			const TyreParams& T = K < 2 ? C.FrontTyre : C.RearTyre;
			const double D = Wh.Contact.Penetration > 0.0 ? Wh.Contact.Penetration : 0.0;
			E += 0.5 * T.RadialStiffness * T.StiffnessFactor(Sim.GetTyreModel(K).GetPressure()) * D * D;
			// Tyre carcass deflection energy, exact for the brush model: along the deflection y the force is
			// mu Fz (1 - (1 - lambda)^3) with lambda proportional to |y|, so E = mu Fz |y| / lambda * g(lambda),
			// g = lambda - (1 - (1 - lambda)^4) / 4. (The linear F^2/2k used before undercounts a saturated tyre ~4.5x.)
			if (Wh.Tyre.Fz > 1.0 && Wh.Tyre.CombinedSlip > 1e-9)
			{
				const TyreModel& Tm = Sim.GetTyreModel(K);
				const double Sx = T.RelaxLengthX * Tm.GetRelaxScale() * Tm.GetSlipStateX();
				const double Sy = T.RelaxLengthY * Tm.GetRelaxScale() * (Tm.GetSlipStateY() - T.CamberStiffnessRatio * Wh.Tyre.Camber);
				const double Lam = std::fmin(Wh.Tyre.CombinedSlip, 1.0);
				const double G = Lam - (1.0 - std::pow(1.0 - Lam, 4.0)) / 4.0;
				E += Wh.Tyre.Mu * Wh.Tyre.Fz * std::sqrt(Sx * Sx + Sy * Sy) / Wh.Tyre.CombinedSlip * G;
			}
			if (K < 2) { E += 0.5 * T.SpinInertia * Wh.Omega * Wh.Omega; }
		}
		E += 0.5 * C.FinalDrive.RearAxleInertia * Sim.GetWheel(WheelRL).Omega * Sim.GetWheel(WheelRL).Omega;
		return E;
	}

	struct SlideResult { double MaxRise = 0.0, MaxIncreaseJ = 0.0, MaxHeight = 0.0, AirTime = 0.0; };

	// Kart released sliding at Speed with the given sideslip, wheels locked to the rolling speed
	// of the velocity's forward component, engine off, no inputs.
	SlideResult Slide(const KartConfig& InC, double Speed, double SideslipDeg)
	{
		// Feature A/B switches for diagnosis (Milestone 9 flex / pressure).
		KartConfig C = InC;
		if (std::getenv("KS_NOPRESSURE")) { for (int K = 0; K < C.NumCompounds; ++K) { C.CompoundFront[K].ColdPressure = C.CompoundRear[K].ColdPressure = 0.0; } C.FitTyres(C.Compound); }
		if (std::getenv("KS_PRESSREFCOLD")) { for (int K = 0; K < C.NumCompounds; ++K) { C.CompoundFront[K].RefPressure = C.CompoundFront[K].ColdPressure; C.CompoundRear[K].RefPressure = C.CompoundRear[K].ColdPressure; } C.FitTyres(C.Compound); }
		if (const char* M = std::getenv("KS_MUSCALE")) { for (int K = 0; K < C.NumCompounds; ++K) { C.CompoundFront[K].Mu0 *= std::atof(M); C.CompoundRear[K].Mu0 *= std::atof(M); } C.FitTyres(C.Compound); }
		if (const char* M = std::getenv("KS_LOADSENS")) { for (int K = 0; K < C.NumCompounds; ++K) { C.CompoundFront[K].MuLoadSensitivity *= std::atof(M); C.CompoundRear[K].MuLoadSensitivity *= std::atof(M); } C.FitTyres(C.Compound); }
		if (std::getenv("KS_NOMOUNTS")) { C.Chassis.FrontMount.UnsprungMass = C.Chassis.RearMount.UnsprungMass = 0.0; }
		if (std::getenv("KS_NOBEND")) { C.Chassis.FrameBendingEI = 0.0; }
		KartSimulation Sim;
		Sim.Initialize(C);
		PlaneTrack Ground;
		KartInputs In;
		In.bIgnitionKill = true;
		kstest::Run(Sim, Ground, In, 0.5);
		const double B = SideslipDeg * kDegToRad;
		Sim.MutableChassis().Velocity = Vec3(Speed * std::cos(B), Speed * std::sin(B), 0.0);
		Sim.SetFrontWheelOmega(Speed * std::cos(B) / C.FrontTyre.UnloadedRadius);
		Sim.SetRearAxleOmega(Speed * std::cos(B) / C.RearTyre.UnloadedRadius);
		SlideResult R;
		double Prev = TotalEnergy(Sim);
		double Peak = Prev;
		for (int I = 0; I < static_cast<int>(4.0 / kTick); ++I)
		{
			Sim.Step(kTick, In, Ground);
			const double E = TotalEnergy(Sim);
			if (E - Prev > 1.0 && std::getenv("KS_ENERGY_TRACE"))
			{
				const ChassisState& Sx = Sim.GetChassis();
				const Vec3& Ix = C.Chassis.Inertia;
				const Vec3& Wx = Sx.AngularVelocity;
				std::printf("      parts: trans %.1f rotX %.1f rotY %.1f rotZ %.1f pot %.1f frameEl %.2f frameKE %.2f axle %.1f\n",
				            0.5 * C.Chassis.Mass * Sx.Velocity.LengthSq(), 0.5 * Ix.X * Wx.X * Wx.X, 0.5 * Ix.Y * Wx.Y * Wx.Y, 0.5 * Ix.Z * Wx.Z * Wx.Z,
				            C.Chassis.Mass * kGravity * Sx.Position.Z, Sim.GetChassisModel().FrameElasticEnergy(), Sim.GetChassisModel().FrameTwistKineticEnergy(),
				            0.5 * C.FinalDrive.RearAxleInertia * Sim.GetWheel(WheelRL).Omega * Sim.GetWheel(WheelRL).Omega);
				std::printf("      flex: mount %.2f bendEl %.2f bendKE %.2f  press %.3f %.3f %.3f %.3f\n", Sim.MountEnergy(), Sim.GetChassisModel().BendingElasticEnergy(),
				            Sim.GetChassisModel().BendingKineticEnergy(), Sim.GetTyreModel(0).GetPressure(), Sim.GetTyreModel(1).GetPressure(), Sim.GetTyreModel(2).GetPressure(), Sim.GetTyreModel(3).GetPressure());
				const TelemetryFrame& Ft = Sim.GetTelemetry();
				std::printf("      +%.1f J at t=%.4f v=%.2f yaw=%.1f fz %.0f %.0f %.0f %.0f fy %.0f %.0f %.0f %.0f fx %.0f %.0f %.0f %.0f rearW %.1f frontW %.1f\n",
				            E - Prev, Ft.Time, Sim.GetChassis().Velocity.Length(), Ft.YawRateDeg, Ft.FzFL, Ft.FzFR, Ft.FzRL, Ft.FzRR,
				            Ft.FyFL, Ft.FyFR, Ft.FyRL, Ft.FyRR, Ft.FxFL, Ft.FxFR, Ft.FxRL, Ft.FxRR,
				            Sim.GetWheel(WheelRL).Omega, Sim.GetWheel(WheelFL).Omega);
			}
			R.MaxIncreaseJ = (E - Prev) > R.MaxIncreaseJ ? (E - Prev) : R.MaxIncreaseJ;
			if (E > Peak) { R.MaxRise = E - Peak > R.MaxRise ? E - Peak : R.MaxRise; Peak = E; }
			Prev = E;
			const TelemetryFrame& F = Sim.GetTelemetry();
			R.MaxHeight = F.PosZ > R.MaxHeight ? F.PosZ : R.MaxHeight;
			if (F.LiftFL > 0.5 && F.LiftFR > 0.5 && F.LiftRL > 0.5 && F.LiftRR > 0.5) { R.AirTime += kTick; }
			if (Sim.GetChassis().Velocity.Length() < 0.2) { break; }
		}
		return R;
	}
}

KS_TEST(Energy_SidewaysSlideNeverGainsEnergy)
{
	for (const char* File : {"Rotax125MaxSenior.json", "KZ2_IAMEScreamer.json"})
	{
		const KartConfig C = kstest::LoadKart(File);
		for (double Beta : {30.0, 60.0, 90.0})
		{
			const SlideResult R = Slide(C, 18.0, Beta);
			KS_LOG("%-34s 65 km/h sideslip %2.0f deg: max per-step energy gain %.2f J, CoG max %.3f m (static %.2f), all-wheels airborne %.3f s",
			       C.ClassId.c_str(), Beta, R.MaxIncreaseJ, R.MaxHeight, C.Chassis.CogHeight, R.AirTime);
			// Kinetic energy at 18 m/s is ~26-28 kJ; allow 1 J/step (carcass-energy term is the
			// linear-range formula F^2*sigma/(2C), approximate once the tyre saturates).
			KS_EXPECT(R.MaxIncreaseJ < 1.0);
		}
	}
}
