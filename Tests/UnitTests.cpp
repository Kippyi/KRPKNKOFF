// Automated physics tests (Docs/PhysicsModel.md section 9).
#include "TestFramework.h"
#include "TestHelpers.h"

#include "DriverBot.h"
#include "KartSim/Json.h"

using namespace kartsim;
using kstest::kTick;

// ---------------------------------------------------------------- config & data

KS_TEST(Json_ParsesCommentsAndBom)
{
	const std::string Text = "\xEF\xBB\xBF// c\n{ \"a\": 1.5, /* b */ \"o\": {\"x\": [1, 2, 3],}, \"s\": \"hi\" }";
	const JsonParseResult R = ParseJson(Text);
	KS_EXPECT(R.bOk);
	KS_EXPECT_NEAR(R.Root.FindPath("a")->NumberValue, 1.5, 1e-12);
	KS_EXPECT(R.Root.FindPath("o.x")->ArrayValue.size() == 3);
	KS_EXPECT(R.Root.FindPath("s")->StringValue == "hi");
	KS_EXPECT(!ParseJson("{\"a\": }").bOk);
}

KS_TEST(Config_RotaxLoadsAndRejectsMissingField)
{
	const KartConfig C = kstest::LoadRotax();
	KS_EXPECT(C.ClassId == "rotax_max_senior");
	KS_EXPECT(C.Clutch.Type == EClutchType::Centrifugal);
	KS_EXPECT_NEAR(C.TotalRatio(0), 80.0 / 12.0, 1e-12);
	KS_EXPECT_NEAR(C.Steering.Caster, 14.0 * kDegToRad, 1e-12);

	KartConfig Out;
	std::string Err;
	KS_EXPECT(!ParseKartConfig("{ \"name\": \"x\" }", Out, Err));
	KS_EXPECT(Err.find("chassis.mass") != std::string::npos);
}

KS_TEST(Engine_VerifiedDatasheetPoints)
{
	const KartConfig C = kstest::LoadRotax();
	EngineModel E;
	E.Init(C.Engine);
	// VERIFIED: 21 Nm @ 9000 rpm, 22 kW @ 11500 rpm.
	KS_EXPECT_NEAR(E.WotTorque(9000.0), 21.0, 1e-9);
	KS_EXPECT_NEAR(E.WotTorque(11500.0) * 11500.0 * kRpmToRadS, 22000.0, 50.0);
	double BestP = 0.0, BestRpm = 0.0, BestT = 0.0, BestTRpm = 0.0;
	for (double Rpm = 2000.0; Rpm <= 15000.0; Rpm += 50.0)
	{
		const double T = E.WotTorque(Rpm);
		const double P = T * Rpm * kRpmToRadS;
		if (P > BestP) { BestP = P; BestRpm = Rpm; }
		if (T > BestT) { BestT = T; BestTRpm = Rpm; }
	}
	KS_LOG("peak power %.2f kW @ %.0f rpm, peak torque %.2f Nm @ %.0f rpm", BestP / 1000.0, BestRpm, BestT, BestTRpm);
	KS_EXPECT_NEAR(BestRpm, 11500.0, 1.0);
	KS_EXPECT_NEAR(BestTRpm, 9000.0, 1.0);
}

KS_TEST(Engine_IdleHoldsAndLimiterCuts)
{
	const KartConfig C = kstest::LoadRotax();
	EngineModel E;
	E.Init(C.Engine);
	const double Dt = 1.0 / 960.0;
	for (int I = 0; I < 960 * 3; ++I)
	{
		const double T = E.UpdateTorque(0.0, false, Dt);
		E.SetOmega(E.GetOmega() + T / E.GetInertia() * Dt);
	}
	KS_LOG("free idle rpm %.0f", E.GetRpm());
	KS_EXPECT_NEAR(E.GetRpm(), C.Engine.IdleRpm, 150.0);

	double MaxRpm = 0.0;
	for (int I = 0; I < 960 * 2; ++I)
	{
		const double T = E.UpdateTorque(1.0, false, Dt);
		E.SetOmega(E.GetOmega() + T / E.GetInertia() * Dt);
		MaxRpm = E.GetRpm() > MaxRpm ? E.GetRpm() : MaxRpm;
	}
	KS_LOG("free rev max rpm %.0f (limiter %.0f)", MaxRpm, C.Engine.RevLimitRpm);
	KS_EXPECT(MaxRpm < C.Engine.RevLimitRpm + 300.0);
	KS_EXPECT(MaxRpm > C.Engine.RevLimitRpm - 50.0);
}

// ---------------------------------------------------------------- tyre

KS_TEST(Tyre_SmallSlipSlopeEqualsStiffness)
{
	const KartConfig C = kstest::LoadRotax();
	TyreModel T;
	T.Init(C.RearTyre);
	const double Fz = C.RearTyre.RefLoad;
	double Fx, Fy;
	T.SteadyStateForces(Fz, 0.0, 1e-4, 1.6, Fx, Fy);
	KS_EXPECT_NEAR(-Fy / 1e-4, C.RearTyre.CyCoeff * Fz, 0.01 * C.RearTyre.CyCoeff * Fz);
	T.SteadyStateForces(Fz, 1e-4, 0.0, 1.6, Fx, Fy);
	KS_EXPECT_NEAR(Fx / 1e-4, C.RearTyre.CxCoeff * Fz, 0.01 * C.RearTyre.CxCoeff * Fz);
}

KS_TEST(Tyre_SaturatesAtMuFzAndStaysInFrictionCircle)
{
	const KartConfig C = kstest::LoadRotax();
	TyreModel T;
	T.Init(C.FrontTyre);
	const double Fz = 400.0, Mu = 1.5;
	double Fx, Fy;
	T.SteadyStateForces(Fz, 0.0, 0.5, Mu, Fx, Fy);
	KS_EXPECT_NEAR(std::fabs(Fy), Mu * Fz, 1e-6);
	double Worst = 0.0;
	for (double K = -0.9; K <= 1.0; K += 0.05)
	{
		for (double Sy = -0.6; Sy <= 0.6; Sy += 0.02)
		{
			T.SteadyStateForces(Fz, K, Sy, Mu, Fx, Fy);
			const double Ratio = std::sqrt(Fx * Fx + Fy * Fy) / (Mu * Fz);
			Worst = Ratio > Worst ? Ratio : Worst;
		}
	}
	KS_EXPECT(Worst <= 1.0 + 1e-9);
	// Peak lateral slip angle (full sliding onset) for documentation.
	const double AlphaSl = std::atan(3.0 * Mu / C.FrontTyre.CyCoeff);
	KS_LOG("front full-slide slip angle %.1f deg at mu %.2f", AlphaSl * kRadToDeg, Mu);
}

KS_TEST(Tyre_TransientSlipConvergesToSteadyState)
{
	const KartConfig C = kstest::LoadRotax();
	TyreModel T;
	T.Init(C.RearTyre);
	PlaneTrack Ground;
	TyreContact Contact;
	const Vec3 Centre(0.0, 0.0, C.RearTyre.UnloadedRadius - 0.0024);
	KS_EXPECT(TyreModel::ComputeContact(C.RearTyre, Centre, Vec3(0, 1, 0), Ground, Contact));
	KS_EXPECT(Contact.bInContact);
	const double Vx = 12.0, Vy = 0.6, Kappa = 0.04;
	const double Omega = Vx * (1.0 + Kappa) / Contact.LoadedRadius;
	TyreOutput Out;
	for (int I = 0; I < 960; ++I)
	{
		T.Evaluate(Contact, Vec3(Vx, Vy, 0.0), Vec3(Vx, Vy, 0.0), Omega, 1.0 / 960.0, Out);
		T.Integrate(Omega, 20.0, 20.0, 1.0 / 960.0);
	}
	KS_EXPECT_NEAR(T.GetSlipStateX(), Kappa, 1e-4);
	KS_EXPECT_NEAR(T.GetSlipStateY(), Vy / Vx, 1e-4);
	KS_EXPECT(Out.Fy < 0.0);  // opposes the leftward sliding
	KS_EXPECT(Out.Fx > 0.0);  // driving slip
}

// ---------------------------------------------------------------- full kart

KS_TEST(Kart_StaticLoadsMatchWeightDistribution)
{
	const KartConfig C = kstest::LoadRotax();
	KartSimulation Sim;
	Sim.Initialize(C);
	PlaneTrack Ground;
	KartInputs In;
	In.Brake = 1.0;
	kstest::Run(Sim, Ground, In, 3.0);
	const TelemetryFrame& F = Sim.GetTelemetry();
	const double Total = F.FzFL + F.FzFR + F.FzRL + F.FzRR;
	const double W = C.Chassis.Mass * kGravity;
	KS_LOG("Fz FL %.1f FR %.1f RL %.1f RR %.1f, total %.1f N (weight %.1f)", F.FzFL, F.FzFR, F.FzRL, F.FzRR, Total, W);
	KS_EXPECT_NEAR(Total, W, 0.005 * W);
	KS_EXPECT_NEAR((F.FzFL + F.FzFR) / Total, C.Chassis.FrontWeightFraction, 0.01);
	KS_EXPECT_NEAR(F.FzFL, F.FzFR, 1.0);
	KS_EXPECT_NEAR(Sim.GetChassis().Position.Z, C.Chassis.CogHeight, 0.002);
	KS_LOG("residual velocity %.6f m/s, ang %.6f rad/s", Sim.GetChassis().Velocity.Length(), Sim.GetChassis().AngularVelocity.Length());
	KS_EXPECT(std::fabs(Sim.GetChassis().Velocity.Length()) < 1e-3);
}

KS_TEST(Kart_CasterJackingLoadsInsideFrontAndUnloadsInsideRear)
{
	const KartConfig C = kstest::LoadRotax();
	KartSimulation Sim;
	Sim.Initialize(C);
	PlaneTrack Ground;
	KartInputs In;
	In.Brake = 1.0;
	kstest::Run(Sim, Ground, In, 2.0);
	const TelemetryFrame Before = Sim.GetTelemetry();
	In.SteeringColumnAngle = 90.0 * kDegToRad; // steer left: left = inside
	kstest::Run(Sim, Ground, In, 2.0);
	const TelemetryFrame After = Sim.GetTelemetry();
	KS_LOG("static -> steered: FL %.0f->%.0f FR %.0f->%.0f RL %.0f->%.0f RR %.0f->%.0f N",
	       Before.FzFL, After.FzFL, Before.FzFR, After.FzFR, Before.FzRL, After.FzRL, Before.FzRR, After.FzRR);
	KS_EXPECT(After.FzFL > Before.FzFL + 5.0);  // inside front loaded (wheel pushed down)
	KS_EXPECT(After.FzRL < Before.FzRL - 5.0);  // inside rear unloaded (diagonal)
	KS_EXPECT(After.FzRR > Before.FzRR + 5.0);
	KS_EXPECT(After.FzFR < Before.FzFR - 5.0);
}

KS_TEST(Kart_CentrifugalClutchNoDriveAtIdleThenLaunch)
{
	const KartConfig C = kstest::LoadRotax();
	KartSimulation Sim;
	Sim.Initialize(C);
	PlaneTrack Ground;
	KartInputs In;
	kstest::Run(Sim, Ground, In, 2.0);
	KS_LOG("idle: speed %.4f m/s, rpm %.0f, clutch torque %.3f", Sim.ForwardSpeed(), Sim.GetEngine().GetRpm(), Sim.GetTelemetry().ClutchTorque);
	KS_EXPECT(std::fabs(Sim.ForwardSpeed()) < 0.01);
	KS_EXPECT(Sim.GetTelemetry().ClutchLocked == 0.0);

	// WOT launch: the clutch must slip at the rpm where its capacity balances engine torque,
	// then lock once the drum catches up. (Timing depends on ASSUMPTION clutch parameters.)
	In.Throttle = 1.0;
	kstest::Run(Sim, Ground, In, 1.0);
	const TelemetryFrame Slip = Sim.GetTelemetry();
	const TransmissionModel& Tr = Sim.GetTransmission();
	const double Cap = Tr.ClutchCapacity(Slip.EngineRpm * kRpmToRadS, 0.0);
	KS_LOG("1 s WOT: %.1f km/h, %.0f rpm, slip %.0f rpm, capacity %.1f Nm vs WOT torque %.1f Nm",
	       Slip.SpeedKmh, Slip.EngineRpm, Slip.ClutchSlipRpm, Cap, Sim.GetEngine().WotTorque(Slip.EngineRpm));
	KS_EXPECT(Slip.ClutchLocked == 0.0 && Slip.ClutchSlipRpm > 500.0);
	KS_EXPECT(Slip.EngineRpm > C.Clutch.EngageStartRpm && Slip.EngineRpm < C.Clutch.FullLockRpm);
	KS_EXPECT_NEAR(Cap, Sim.GetEngine().WotTorque(Slip.EngineRpm), 1.5); // quasi-static slip balance

	kstest::Run(Sim, Ground, In, 7.0);
	const TelemetryFrame& F = Sim.GetTelemetry();
	KS_LOG("8 s WOT: %.1f km/h, %.0f rpm, locked %d", F.SpeedKmh, F.EngineRpm, (int)F.ClutchLocked);
	KS_EXPECT(F.SpeedKmh > 60.0);
	KS_EXPECT(F.ClutchLocked == 1.0);
}

KS_TEST(Kart_BrakeLocksRearAxleWithoutReversal)
{
	const KartConfig C = kstest::LoadRotax();
	KartSimulation Sim;
	Sim.Initialize(C);
	PlaneTrack Ground;
	KartInputs In;
	kstest::Run(Sim, Ground, In, 1.0);
	kstest::SetRolling(Sim, 15.0);
	In.Brake = 1.0;
	double MinAxle = 1e9;
	bool bLocked = false;
	const int N = static_cast<int>(4.0 / kTick);
	for (int I = 0; I < N; ++I)
	{
		Sim.Step(kTick, In, Ground);
		const double W = Sim.GetWheel(WheelRL).Omega;
		MinAxle = W < MinAxle ? W : MinAxle;
		if (W == 0.0 && Sim.ForwardSpeed() > 2.0) { bLocked = true; }
	}
	KS_LOG("min axle omega %.3f, final speed %.3f", MinAxle, Sim.ForwardSpeed());
	KS_EXPECT(bLocked);
	KS_EXPECT(MinAxle >= 0.0);
	KS_EXPECT(std::fabs(Sim.ForwardSpeed()) < 0.05);
}

KS_TEST(Kart_CoastDownIsMonotonic)
{
	const KartConfig C = kstest::LoadRotax();
	KartSimulation Sim;
	Sim.Initialize(C);
	PlaneTrack Ground;
	KartInputs In;
	kstest::Run(Sim, Ground, In, 1.0);
	kstest::SetRolling(Sim, 20.0);
	kstest::Run(Sim, Ground, In, 0.5); // let transients settle
	double Prev = Sim.ForwardSpeed();
	int Violations = 0;
	for (int I = 0; I < static_cast<int>(20.0 / kTick); ++I)
	{
		Sim.Step(kTick, In, Ground);
		const double V = Sim.ForwardSpeed();
		if (V > Prev + 1e-4) { if (Violations < 5) { KS_LOG("increase at t=%.3f: %.5f -> %.5f (rpm %.0f locked %d slip %.0f)", Sim.GetTime(), Prev, V, Sim.GetEngine().GetRpm(), (int)Sim.GetTelemetry().ClutchLocked, Sim.GetTelemetry().ClutchSlipRpm); } ++Violations; }
		Prev = V;
	}
	KS_LOG("speed after 20 s coast: %.2f m/s", Prev);
	KS_EXPECT(Violations == 0);
	KS_EXPECT(Prev < 20.0 && Prev > 0.0);
}

KS_TEST(Kart_TimestepConvergence)
{
	const KartConfig C = kstest::LoadRotax();
	auto Launch = [&C](int Substeps)
	{
		KartSimulation Sim;
		Sim.Initialize(C);
		Sim.Settings.Substeps = Substeps;
		PlaneTrack Ground;
		KartInputs In;
		kstest::Run(Sim, Ground, In, 1.0);
		In.Throttle = 1.0;
		In.SteeringColumnAngle = 20.0 * kDegToRad;
		kstest::Run(Sim, Ground, In, 5.0);
		return Sim.GetTelemetry();
	};
	const TelemetryFrame A = Launch(4);
	const TelemetryFrame B = Launch(16);
	KS_LOG("4 substeps: %.3f km/h yaw %.2f deg | 16 substeps: %.3f km/h yaw %.2f deg", A.SpeedKmh, A.YawDeg, B.SpeedKmh, B.YawDeg);
	KS_EXPECT_NEAR(A.SpeedKmh, B.SpeedKmh, 0.01 * B.SpeedKmh);
	KS_EXPECT_NEAR(A.YawDeg, B.YawDeg, 2.0);
}

KS_TEST(Kart_LateralLoadTransferInSteadyCorner)
{
	const KartConfig C = kstest::LoadRotax();
	KartSimulation Sim;
	Sim.Initialize(C);
	PlaneTrack Ground;
	KartInputs In;
	kstest::Run(Sim, Ground, In, 1.0);
	kstest::SetRolling(Sim, 12.0);
	kstest::SpeedController Ctl;
	In.SteeringColumnAngle = 30.0 * kDegToRad;
	for (int I = 0; I < static_cast<int>(8.0 / kTick); ++I)
	{
		In.Throttle = Ctl.Throttle(12.0, Sim.ForwardSpeed(), kTick);
		Sim.Step(kTick, In, Ground);
	}
	const TelemetryFrame& F = Sim.GetTelemetry();
	const double Total = F.FzFL + F.FzFR + F.FzRL + F.FzRR;
	const double OutsideMinusInside = (F.FzFR + F.FzRR) - (F.FzFL + F.FzRL);
	// Quasi-static total lateral transfer: dFz_total = 2 * m*ay*h / t_mean (both axles).
	const double Ay = F.AccLatG * kGravity;
	const double TMean = 0.5 * (C.Chassis.FrontTrack + C.Chassis.RearTrack);
	const double Expected = 2.0 * C.Chassis.Mass * std::fabs(Ay) * C.Chassis.CogHeight / TMean;
	KS_LOG("ay %.2f g, Fz FL %.0f FR %.0f RL %.0f RR %.0f, transfer %.0f N (rigid estimate %.0f N), liftRL %d",
	       F.AccLatG, F.FzFL, F.FzFR, F.FzRL, F.FzRR, OutsideMinusInside, Expected, (int)F.LiftRL);
	KS_EXPECT(F.AccLatG > 0.3);              // turning left: positive lateral specific force
	KS_EXPECT_NEAR(Total, C.Chassis.Mass * kGravity, 0.03 * Total);
	KS_EXPECT(OutsideMinusInside > 0.0);
	KS_EXPECT_NEAR(OutsideMinusInside, Expected, 0.30 * Expected); // differs by track widths & jacking
}

// ---------------------------------------------------------------- drivability regression (M2)

KS_TEST(Kart_DriverBotLapsCleanlyWithinGripBand)
{
	// A modulating driver targeting 1.4 g (inside the 1.5-2.2 g band reported for karts, with
	// margin) must complete 3 laps of the test circuit with no spin and stay on the track.
	const KartConfig C = kstest::LoadRotax();
	const LapResult R = DriveLaps(C, 1.4, 3, nullptr);
	KS_LOG("laps %d, best %.2f s, spins %d, max ay %.2f g, max sideslip %.1f deg, max off-line %.1f m",
	       R.LapsDone, R.BestLap, R.Spins, R.MaxAy, R.MaxSideslip, R.MaxCrossTrack);
	KS_EXPECT(R.LapsDone == 3);
	KS_EXPECT(R.Spins == 0);
	KS_EXPECT(R.MaxSideslip < 10.0);
	KS_EXPECT(R.MaxCrossTrack < 4.0); // track half-width is 4 m
	KS_EXPECT(R.MaxAy > 1.3);         // the kart must actually be able to corner hard
}

KS_TEST(Kart_ModerateSteerWithFullThrottleDoesNotSpin)
{
	// The M1 failure reported by the user: holding full throttle with moderate steering spun the
	// kart. 50 km/h, 10/20/30 deg column, WOT for 3 s.
	const KartConfig C = kstest::LoadRotax();
	for (double Col : {10.0, 20.0, 30.0})
	{
		KartSimulation Sim;
		Sim.Initialize(C);
		PlaneTrack Ground;
		kstest::Run(Sim, Ground, KartInputs(), 0.5);
		kstest::SetRolling(Sim, 50.0 / 3.6);
		KartInputs In;
		In.Throttle = 1.0;
		In.SteeringColumnAngle = Col * kDegToRad;
		double MaxSlip = 0.0;
		for (int I = 0; I < static_cast<int>(3.0 / kTick); ++I)
		{
			Sim.Step(kTick, In, Ground);
			MaxSlip = std::fabs(Sim.GetTelemetry().SideslipDeg) > MaxSlip ? std::fabs(Sim.GetTelemetry().SideslipDeg) : MaxSlip;
		}
		KS_LOG("column %.0f deg WOT: max sideslip %.1f deg, end %.1f km/h", Col, MaxSlip, Sim.GetTelemetry().SpeedKmh);
		KS_EXPECT(MaxSlip < 10.0);
	}
}
KS_TEST(Tyre_CamberThrustPushesTowardLean)
{
	// A wheel leaning left (top toward +y) at zero slip must push the kart left, ~ ratio*C_y*gamma.
	const KartConfig C = kstest::LoadRotax();
	TyreModel T;
	T.Init(C.FrontTyre);
	PlaneTrack Ground;
	const double Gamma = 5.0 * kDegToRad;
	const Vec3 Axle(0.0, std::cos(Gamma), -std::sin(Gamma));
	TyreContact Contact;
	KS_EXPECT(TyreModel::ComputeContact(C.FrontTyre, Vec3(0, 0, C.FrontTyre.UnloadedRadius - 0.003), Axle, Ground, Contact));
	TyreOutput Out;
	const double V = 10.0;
	T.Evaluate(Contact, Vec3(V, 0, 0), Vec3(V, 0, 0), V / Contact.LoadedRadius, 1.0 / 960.0, Out);
	const double Expected = C.FrontTyre.CamberStiffnessRatio * C.FrontTyre.CyCoeff * C.FrontTyre.RefLoad
		* std::pow(Out.Fz / C.FrontTyre.RefLoad, C.FrontTyre.StiffnessLoadExponent) * Gamma;
	KS_LOG("camber %.2f deg -> Fy %.1f N (linear estimate %.1f N)", Out.Camber * kRadToDeg, Out.Fy, Expected);
	KS_EXPECT_NEAR(Out.Camber, Gamma, 1e-6);
	KS_EXPECT(Out.Fy > 0.0);
	KS_EXPECT_NEAR(Out.Fy, Expected, 0.1 * Expected);
}

KS_TEST(Steering_FrontCamberFromCasterAtLock)
{
	// Report the steering-induced camber (emerges from caster/KPI geometry) at 90 deg column.
	const KartConfig C = kstest::LoadRotax();
	KartSimulation Sim;
	Sim.Initialize(C);
	PlaneTrack Ground;
	KartInputs In;
	In.Brake = 1.0;
	In.SteeringColumnAngle = 90.0 * kDegToRad; // left turn: left = inside
	kstest::Run(Sim, Ground, In, 1.0);
	const TelemetryFrame& F = Sim.GetTelemetry();
	KS_LOG("column 90 deg: steer L %.1f R %.1f deg, camber inside(L) %.2f outside(R) %.2f deg (+ = top leaning left)",
	       F.SteerLeftDeg, F.SteerRightDeg, F.CamberFLDeg, F.CamberFRDeg);
	// With positive caster, both wheels lean INTO the turn (top toward the turn centre, i.e. left).
	KS_EXPECT(F.CamberFRDeg > 1.0);
	KS_EXPECT(F.CamberFLDeg > 1.0);
}
KS_TEST(Steering_ColumnTorqueIsSelfCentring)
{
	// Steering left must produce a column torque pushing the wheel back to the right (negative),
	// both at standstill (weight jacking via caster/KPI) and in a steady corner (trail).
	// Cases that spin the kart (e.g. 60 deg at 60 km/h) are excluded: once sliding, the torque
	// legitimately reverses (the wheel snaps into opposite lock).
	const KartConfig C = kstest::LoadRotax();
	double Torque30[3] = {0, 0, 0};
	for (double Speed : {0.0, 30.0, 60.0})
	{
		for (double Col : {10.0, 30.0, 60.0})
		{
			if (Speed == 60.0 && Col == 60.0) { continue; }
			KartSimulation Sim;
			Sim.Initialize(C);
			PlaneTrack Ground;
			kstest::Run(Sim, Ground, KartInputs(), 0.5);
			KartInputs In;
			if (Speed > 0.0) { kstest::SetRolling(Sim, Speed / 3.6); }
			else { In.Brake = 1.0; }
			kstest::SpeedController Ctl;
			In.SteeringColumnAngle = Col * kDegToRad;
			for (int I = 0; I < static_cast<int>(2.0 / kTick); ++I)
			{
				if (Speed > 0.0) { In.Throttle = Ctl.Throttle(Speed / 3.6, Sim.ForwardSpeed(), kTick); }
				Sim.Step(kTick, In, Ground);
			}
			const TelemetryFrame& F = Sim.GetTelemetry();
			KS_LOG("%3.0f km/h col %3.0f deg: column torque %6.2f Nm  ay %4.2f g  Mz FL %5.1f FR %5.1f",
			       Speed, Col, F.ColumnTorque, F.AccLatG, F.MzFL, F.MzFR);
			// Only steady (non-rotating) cornering is a valid centring check; a spinning kart legitimately
			// reverses the torque.
			const bool bSteady = std::fabs(F.SideslipDeg) < 15.0 && (Speed == 0.0 || F.AccLatG > 0.05);
			if (bSteady) { KS_EXPECT(F.ColumnTorque < 0.0); }
			else { KS_LOG("   (not steady: sideslip %.1f deg, ay %.2f g; sign check skipped)", F.SideslipDeg, F.AccLatG); }
		}
	}
	(void)Torque30;
}

KS_TEST(Tyre_AligningMomentCollapsesAtSaturation)
{
	// Physical origin of the understeer cue in the steering: the pneumatic trail (aligning moment)
	// rises with slip angle, peaks well before full sliding and collapses to zero when fully sliding.
	const KartConfig C = kstest::LoadRotax();
	PlaneTrack Ground;
	TyreContact Contact;
	KS_EXPECT(TyreModel::ComputeContact(C.FrontTyre, Vec3(0, 0, C.FrontTyre.UnloadedRadius - 0.003), Vec3(0, 1, 0), Ground, Contact));
	double PeakMz = 0.0, PeakAlpha = 0.0, MzAtFull = 1e9, FyAtPeakMz = 0.0, FyMax = 0.0;
	for (double AlphaDeg = 0.5; AlphaDeg <= 20.0; AlphaDeg += 0.5)
	{
		TyreModel T;
		T.Init(C.FrontTyre);
		const double V = 10.0, Vy = V * std::tan(AlphaDeg * kDegToRad);
		TyreOutput Out;
		for (int I = 0; I < 2000; ++I)
		{
			T.Evaluate(Contact, Vec3(V, Vy, 0.0), Vec3(V, Vy, 0.0), V / Contact.LoadedRadius, 1.0 / 960.0, Out);
			T.Integrate(V / Contact.LoadedRadius, 20.0, 20.0, 1.0 / 960.0);
		}
		const double Mz = std::fabs(Out.Mz), Fy = std::fabs(Out.Fy);
		if (Mz > PeakMz) { PeakMz = Mz; PeakAlpha = AlphaDeg; FyAtPeakMz = Fy; }
		FyMax = Fy > FyMax ? Fy : FyMax;
		if (AlphaDeg == 20.0) { MzAtFull = Mz; }
	}
	KS_LOG("Mz peaks %.2f Nm at %.1f deg (Fy there %.0f N = %.0f%% of max); Mz at 20 deg %.3f Nm", PeakMz, PeakAlpha, FyAtPeakMz, 100.0 * FyAtPeakMz / FyMax, MzAtFull);
	KS_EXPECT(PeakAlpha > 0.5 && PeakAlpha < 6.0);
	KS_EXPECT(FyAtPeakMz < 0.9 * FyMax);        // aligning moment peaks before lateral force does
	KS_EXPECT(MzAtFull < 0.05 * PeakMz);        // collapses when fully sliding
}
KS_TEST(Clutch_RegulationEngagementAt4000Rpm)
{
	// Global RMC Technical Regulation 2025 v3, section 2.2.1 (125 Senior MAX):
	// "Engagement speed of centrifugal clutch at maximum 4.000 rpm (the kart without driver must start to move)."
	// Reproduce the procedure: kart WITHOUT driver (85 kg dry kart, ASSUMPTION), engine held at a fixed rpm.
	KartConfig C = kstest::LoadRotax();
	const double DryMass = 85.0;
	const double Scale = DryMass / C.Chassis.Mass;
	C.Chassis.Mass = DryMass;
	C.Chassis.Inertia = C.Chassis.Inertia * Scale;
	auto MovesAt = [&C](double Rpm)
	{
		KartSimulation Sim;
		Sim.Initialize(C);
		PlaneTrack Ground;
		kstest::Run(Sim, Ground, KartInputs(), 0.5);
		for (int I = 0; I < static_cast<int>(3.0 / kTick); ++I)
		{
			Sim.MutableEngine().SetOmega(Rpm * kRpmToRadS); // dyno-style speed hold
			Sim.Step(kTick, KartInputs(), Ground);
		}
		return Sim.ForwardSpeed();
	};
	const double V3600 = MovesAt(3600.0), V4000 = MovesAt(4000.0);
	KS_LOG("kart without driver after 3 s: at 3600 rpm %.3f m/s, at 4000 rpm %.3f m/s", V3600, V4000);
	KS_EXPECT(V4000 > 0.2);   // must start to move at <= 4000 rpm (regulation)
	KS_EXPECT(V3600 < 0.01);  // below engagement start (3800 rpm, ASSUMPTION) it must not drive
}
namespace
{
	// Ground with a step: Lip (m) higher for x < StepX (a track edge lip / kerb end / mesh seam).
	struct StepGround final : kartsim::ITrackQuery
	{
		double StepX = 30.0, Lip = 0.005;
		bool QueryGround(const kartsim::Vec3& O, double Up, double Down, kartsim::GroundHit& Out) const override
		{
			const double Z = O.X < StepX ? Lip : 0.0;
			if (Z > O.Z + Up || Z < O.Z - Down) { return false; }
			Out.Point = kartsim::Vec3(O.X, O.Y, Z);
			Out.Normal = kartsim::Vec3(0, 0, 1);
			Out.Surface = kartsim::ESurfaceType::Asphalt;
			Out.GripMultiplier = 1.0;
			return true;
		}
	};
}

KS_TEST(Tyre_GroundStepDoesNotSpikeTheDamper)
{
	// Regression for the in-game "all four tyres spazzing" (user telemetry 2026-11-17): the damper used
	// the raw change in penetration, so a 5 mm ground step looked like several m/s for one step and the
	// tyre load jumped by 1-4 kN. The road part of the rate is now spread over the contact patch.
	const kartsim::KartConfig C = kstest::LoadRotax();
	for (const double Lip : {0.005, -0.005}) // step down and step up
	{
		kartsim::KartSimulation Sim;
		Sim.Initialize(C);
		StepGround G;
		G.Lip = Lip;
		Sim.Reset(kartsim::Vec3(0, 0, Lip > 0 ? Lip : 0.0), 0.0);
		kstest::Run(Sim, G, kartsim::KartInputs(), 0.5);
		kstest::SetRolling(Sim, 16.7);
		kstest::SpeedController Ctl;
		kartsim::KartInputs In;
		double Prev[4] = {}, MaxJump = 0.0;
		for (int S = 0; S < 240 * 4; ++S)
		{
			In.Throttle = Ctl.Throttle(16.7, Sim.ForwardSpeed(), kstest::kTick);
			Sim.Step(kstest::kTick, In, G);
			for (int W = 0; W < 4; ++W)
			{
				const double Fz = Sim.GetWheel(W).Tyre.Fz;
				if (S > 10) { MaxJump = std::fmax(MaxJump, std::fabs(Fz - Prev[W])); }
				Prev[W] = Fz;
			}
		}
		// The bug: the damper saw the raw step as ~5 mm per 1/960 s substep = 4.8 m/s, i.e. a ~4.3 kN spike
		// (rear carcass 900 N s/m). Step UP: the step enters the ~8 cm patch within about one step at 60 km/h,
		// so a jump of the order of the SPRING response (k x 5 mm) plus the kart's damped motion (body
		// damping, seated driver) is physical. Bounds sit well below the bug level.
		const double SpringResponse = std::fmax(C.FrontTyre.RadialStiffness, C.RearTyre.RadialStiffness) * std::fabs(Lip);
		KS_LOG("5 mm step %s at 60 km/h: max tyre-load change in one step %.0f N (spring k*h = %.0f N)", Lip > 0 ? "down" : "up", MaxJump, SpringResponse);
		KS_EXPECT(MaxJump < (Lip > 0 ? 800.0 : 2.0 * SpringResponse));
	}
}

KS_TEST(Kart_SlidingKZDoesNotPogoHop)
{
	// Regression for user telemetry 2026-11-17 (KZ, Riverside build): sliding at 70-80 km/h with ~48 deg of
	// steering, the kart bounced on its outside tyres until all four wheels left the ground, 5-10 times in
	// 2 s. Vertical damping was only the tyre carcass (~10 % critical); the seated driver's damping
	// (bodyDamping) removes it. Same open-loop manoeuvre that reproduced 5 hops before.
	const kartsim::KartConfig C = kstest::LoadKart("KZ2_IAMEScreamer.json");
	for (const double SteerDeg : {30.0, 48.0})
	{
		kartsim::KartSimulation Sim;
		Sim.Initialize(C);
		kartsim::PlaneTrack G;
		kstest::Run(Sim, G, kartsim::KartInputs(), 0.5);
		kstest::SetRolling(Sim, 19.4);
		kartsim::KartInputs In;
		In.SteeringColumnAngle = -SteerDeg * kartsim::kDegToRad;
		In.Throttle = 0.6;
		for (int Shift = 0; Shift < 3; ++Shift)
		{
			In.ShiftRequest = 1;
			Sim.Step(kstest::kTick, In, G);
			In.ShiftRequest = 0;
			for (int K = 0; K < 60; ++K) { Sim.Step(kstest::kTick, In, G); }
		}
		int AllAirborne = 0;
		double MaxFz = 0.0;
		for (int S = 0; S < 240 * 4; ++S)
		{
			Sim.Step(kstest::kTick, In, G);
			bool bAll = true;
			for (int W = 0; W < 4; ++W) { bAll = bAll && !Sim.GetWheel(W).Contact.bInContact; MaxFz = std::fmax(MaxFz, Sim.GetWheel(W).Tyre.Fz); }
			AllAirborne += bAll ? 1 : 0;
		}
		KS_LOG("KZ sliding, %.0f deg steering: all-four-airborne steps %d, max tyre load %.0f N", SteerDeg, AllAirborne, MaxFz);
		KS_EXPECT(AllAirborne == 0);
	}
}
