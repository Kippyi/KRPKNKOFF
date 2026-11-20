#include "KartSim/KartPhysicsCore.h"

#include <cmath>
#include <cstdlib>

namespace kartsim
{
	bool WheelMountCompliance(const WheelMountParams& P, double WheelY, double& Cff, double& Cfm, double& Cmm)
	{
		Cff = Cfm = Cmm = 0.0;
		if (P.UnsprungMass <= 0.0 || P.TubeOuterDiameter <= 0.0 || P.YoungsModulus <= 0.0) { return false; }
		// Free span L (cantilever from the bearing / stub root) and rigid lever A (hub + wheel) to the wheel centre.
		double L = P.FreeSpan, A = 0.0;
		if (P.BearingY > 0.0)
		{
			const double HubInner = WheelY - P.HubToWheelCentre - P.HubLength;
			L = HubInner - P.BearingY;
			A = P.HubLength + P.HubToWheelCentre;
		}
		L = std::fmax(L, 1e-3); // hub up against the bearing: practically rigid
		const double D = P.TubeOuterDiameter;
		const double Di = P.TubeWall > 0.0 ? std::fmax(0.0, D - 2.0 * P.TubeWall) : 0.0;
		const double EI = P.YoungsModulus * kPi / 64.0 * (D * D * D * D - Di * Di * Di * Di);
		// Euler-Bernoulli cantilever, tip shear F and moment M at the end of the free span, rigid lever A beyond it:
		// end deflection F L^3/3EI + M L^2/2EI, end slope F L^2/2EI + M L/EI, with M = F A for a force at the centre.
		Cff = (L * L * L / 3.0 + A * L * L + A * A * L) / EI + P.ExtraCompliance;
		Cfm = (0.5 * L * L + A * L) / EI;
		Cmm = L / EI;
		return true;
	}

	void KartSimulation::InitMounts()
	{
		const ChassisParams& Ch = Config.Chassis;
		for (int I = 0; I < NumWheels; ++I)
		{
			FMount& Mt = Mounts[I];
			Mt = FMount();
			const bool bFront = I < 2;
			const WheelMountParams& P = bFront ? Ch.FrontMount : Ch.RearMount;
			const double WheelY = std::fabs(Chassis.StaticWheelCentre(I).Y);
			double Cff, Cfm, Cmm;
			if (!WheelMountCompliance(P, WheelY, Cff, Cfm, Cmm)) { continue; }
			// Left wheels (y > 0): an upward tip force rotates the outer end up = + about x. Right wheels mirror.
			const double Side = Chassis.StaticWheelCentre(I).Y > 0.0 ? 1.0 : -1.0;
			const double C01 = Side * Cfm;
			const double Det = Cff * Cmm - C01 * C01;
			if (Det <= 0.0) { continue; }
			Mt.bActive = true;
			Mt.M = P.UnsprungMass;
			const double R0 = bFront ? Config.FrontTyre.UnloadedRadius : Config.RearTyre.UnloadedRadius;
			Mt.J = P.CamberInertia > 0.0 ? P.CamberInertia : 0.25 * P.UnsprungMass * R0 * R0;
			Mt.K[0][0] = Cmm / Det; Mt.K[1][1] = Cff / Det; Mt.K[0][1] = Mt.K[1][0] = -C01 / Det;
			Mt.C[0][0] = 2.0 * P.DampingRatio * std::sqrt(Mt.K[0][0] * Mt.M);
			Mt.C[1][1] = 2.0 * P.DampingRatio * std::sqrt(Mt.K[1][1] * Mt.J);
			// Static deflection under the corner load minus the assembly's own weight (static geometry = loaded).
			const double Corner = 0.5 * Ch.Mass * kGravity * (bFront ? Ch.FrontWeightFraction : 1.0 - Ch.FrontWeightFraction);
			const double Fm = Corner - P.UnsprungMass * kGravity;
			Mt.Qs[0] = Cff * Fm;
			Mt.Qs[1] = C01 * Fm;
		}
	}

	double KartSimulation::MountEnergy() const
	{
		double E = 0.0;
		for (int I = 0; I < NumWheels; ++I)
		{
			const FMount& Mt = Mounts[I];
			if (!Mt.bActive) { continue; }
			const WheelState& W = Wheels[I];
			const double Q[2] = {W.MountZ + Mt.Qs[0], W.MountPsi + Mt.Qs[1]};
			double El = 0.0, El0 = 0.0;
			for (int A = 0; A < 2; ++A) { for (int B = 0; B < 2; ++B) { El += 0.5 * Q[A] * Mt.K[A][B] * Q[B]; El0 += 0.5 * Mt.Qs[A] * Mt.K[A][B] * Mt.Qs[B]; } }
			E += El - El0 + 0.5 * Mt.M * W.MountZRate * W.MountZRate + 0.5 * Mt.J * W.MountPsiRate * W.MountPsiRate + Mt.M * kGravity * W.MountZ;
		}
		return E;
	}

	void KartSimulation::Initialize(const KartConfig& InConfig)
	{
		Config = InConfig;
		Chassis.Init(Config);
		const Vec3 FrontCentres[2] = {Chassis.StaticWheelCentre(WheelFL), Chassis.StaticWheelCentre(WheelFR)};
		Steering.Init(Config, FrontCentres);
		InitMounts();
		Tyres[WheelFL].Init(Config.FrontTyre);
		Tyres[WheelFR].Init(Config.FrontTyre);
		Tyres[WheelRL].Init(Config.RearTyre);
		Tyres[WheelRR].Init(Config.RearTyre);
		Engine.Init(Config.Engine);
		Transmission.Init(Config);
		Brakes.Init(Config.Brakes);
		Cond.Init(Config.Tuning, Config.Engine);
		Damage = KartDamageState();
		ApplyDamageEffects();
		bInitialized = true;
		Reset(Vec3(0.0, 0.0, 0.0), 0.0);
	}

	void KartSimulation::Reset(const Vec3& GroundPoint, double Yaw)
	{
		Chassis.State = ChassisState();
		ClearExternalForce();
		Chassis.State.Position = GroundPoint + Vec3(0.0, 0.0, Config.Chassis.CogHeight);
		Chassis.State.Orientation = Quat::FromAxisAngle(Vec3(0.0, 0.0, 1.0), Yaw);
		for (int I = 0; I < NumWheels; ++I)
		{
			Tyres[I].Reset();
			Wheels[I] = WheelState();
		}
		RearAxleOmega = 0.0;
		LastSpecificForceBody = Vec3(0.0, 0.0, kGravity);
		for (double& A : BendAccPrev) { A = 0.0; }
		// A kart with a clutch can idle at rest; a direct-drive engine is necessarily stopped.
		const bool bCanIdle = Config.Clutch.Type != EClutchType::DirectDrive;
		Cond.Reset(Settings.AmbientTemp);
		const bool bSeized = Cond.IsSeized();
		Engine.Reset(bCanIdle && !bSeized ? Config.Engine.IdleRpm : 0.0, bCanIdle && !bSeized);
		Engine.SetSeized(bSeized, Config.Tuning.SeizedDragTorque);
		Transmission.Reset();
		NumContacts = 0;
		for (double& Fz : WheelPeakFz) { Fz = 0.0; }
		for (double& Fz : WheelFzFilt) { Fz = 0.0; }
		for (double& D : WheelStrikeDeg) { D = 0.0; }
		for (double& F : ContactFilt) { F = 0.0; }
		for (double& D : EventDeg) { D = 0.0; }
		for (double& J : Damage.EventImpulse) { J = 0.0; }
		ApplyDamageEffects();
		Brakes.Reset();
		Ffb.Reset();
		LastDrivetrain = DrivetrainStepResult();
		LastBrakeTorque = 0.0;
		ColumnTorqueAvg = 0.0;
		Time = 0.0;
		Frame = TelemetryFrame();
	}

	double KartSimulation::ForwardSpeed() const
	{
		return Chassis.WorldToBodyDir(Chassis.State.Velocity).X;
	}

	void KartSimulation::Step(double Dt, const KartInputs& Inputs, const ITrackQuery& Track)
	{
		if (!bInitialized || Dt <= 0.0)
		{
			return;
		}
		if (Inputs.ShiftRequest != 0)
		{
			Transmission.RequestShift(Inputs.ShiftRequest);
		}
		const int N = Settings.Substeps > 0 ? Settings.Substeps : 1;
		const double H = Dt / N;
		double ColumnAccum = 0.0;
		Vec3 SpecificForceAccum;
		for (int I = 0; I < N; ++I)
		{
			Substep(H, Inputs, Track, ColumnAccum, SpecificForceAccum);
		}
		ColumnTorqueAvg = ColumnAccum / N;
		Time += Dt;
		if (Settings.bDamage) { UpdateContactDamage(Dt); }
		NumContacts = 0;
		Ffb.Process(ColumnTorqueAvg, Dt);
		FillTelemetry(Inputs, SpecificForceAccum / static_cast<double>(N));
	}

	void KartSimulation::Substep(double Dt, const KartInputs& Inputs, const ITrackQuery& Track,
	                             double& ColumnTorqueAccum, Vec3& SpecificForceAccum)
	{
		ChassisState& S = Chassis.State;
		const ChassisParams& CP = Config.Chassis;

		// 1. Steering angles about the kingpins.
		double DeltaL = 0.0, DeltaR = 0.0;
		Steering.ComputeAngles(Inputs.SteeringColumnAngle, DeltaL, DeltaR);
		Wheels[WheelFL].SteerAngle = DeltaL;
		Wheels[WheelFR].SteerAngle = DeltaR;

		// 2. Wheel poses in the body frame (front: kingpin rotation then frame twist).
		Vec3 CentreB[NumWheels], AxleB[NumWheels];
		for (int Side = 0; Side < 2; ++Side)
		{
			Vec3 C, A;
			Steering.WheelPose(Side, Side == 0 ? DeltaL : DeltaR, C, A);
			CentreB[Side] = Chassis.TwistPointBody(C, Chassis.FrontNode());
			AxleB[Side] = Chassis.TwistDirBody(A, Chassis.FrontNode());
		}
		// Rear: solid axle, rotated with the rear-axle frame node's twist.
		const int RearNode = Chassis.RearNode();
		CentreB[WheelRL] = Chassis.TwistPointBody(Chassis.StaticWheelCentre(WheelRL), RearNode);
		CentreB[WheelRR] = Chassis.TwistPointBody(Chassis.StaticWheelCentre(WheelRR), RearNode);
		AxleB[WheelRL] = AxleB[WheelRR] = Chassis.TwistDirBody(Vec3(0.0, 1.0, 0.0), RearNode);
		// Frame bending moves the axle nodes; each wheel then sits on its own elastic mount (z, camber).
		Vec3 MountRelVelB[NumWheels];
		for (int I = 0; I < NumWheels; ++I)
		{
			const int Node = I < 2 ? Chassis.FrontNode() : RearNode;
			const WheelState& W = Wheels[I];
			CentreB[I].Z += Chassis.NodeBendOffset(Node) + W.MountZ;
			if (W.MountPsi != 0.0) { AxleB[I] = RotateAboutAxis(AxleB[I], Vec3(1.0, 0.0, 0.0), W.MountPsi); }
			MountRelVelB[I] = Vec3(0.0, 0.0, Chassis.NodeBendVelocity(Node) + W.MountZRate);
		}

		// 3. Contacts and tyre forces.
		Vec3 ForceW(0.0, 0.0, -CP.Mass * kGravity);
		const double Speed = S.Velocity.Length();
		const Vec3 Aero = S.Velocity * (-0.5 * CP.AirDensity * CP.CdA * Settings.DragScale * Speed);
		ForceW += Aero;
		if (Inputs.PushForce != 0.0)
		{
			// Helper pushing on the rear bumper, along the kart heading (push start).
			Vec3 Heading = Chassis.BodyToWorldDir(Vec3(1.0, 0.0, 0.0));
			Heading.Z = 0.0;
			ForceW += Heading.Normalized() * Inputs.PushForce;
		}
		Vec3 TorqueB;
		if (ExtForceW.X != 0.0 || ExtForceW.Y != 0.0 || ExtForceW.Z != 0.0 || ExtTorqueB.X != 0.0 || ExtTorqueB.Y != 0.0 || ExtTorqueB.Z != 0.0)
		{
			ForceW += ExtForceW;
			TorqueB += ExtTorqueB;
		}
		double NodeMoments[ChassisParams::MaxFrameNodes] = {};
		double NodeVertical[ChassisParams::MaxFrameNodes] = {};
		double KingpinMoments[2] = {0.0, 0.0};
		BodyGroundContacts(Track, ForceW, TorqueB);

		for (int I = 0; I < NumWheels; ++I)
		{
			WheelState& W = Wheels[I];
			const bool bFront = I < 2;
			const TyreParams& TP = bFront ? Config.FrontTyre : Config.RearTyre;

			W.CentreWorld = Chassis.BodyToWorldPoint(CentreB[I]);
			W.AxleWorld = Chassis.BodyToWorldDir(AxleB[I]);
			TyreModel::ComputeContact(TP, W.CentreWorld, W.AxleWorld, Track, W.Contact);

			const Vec3 ContactB = S.Orientation.Unrotate(W.Contact.ContactPoint - S.Position);
			Vec3 ContactVelW = Chassis.PointVelocityWorld(ContactB);
			Vec3 CentreVelW = Chassis.PointVelocityWorld(CentreB[I]);
			const int Section = bFront ? Chassis.FrontNode() : RearNode;
			ContactVelW += Chassis.BodyToWorldDir(Chassis.TwistVelocityBody(ContactB, Section));
			CentreVelW += Chassis.BodyToWorldDir(Chassis.TwistVelocityBody(CentreB[I], Section));
			// The seated driver's damping (tyre BodyDamping) acts on the frame's motion at the mount, not on the
			// wheel assembly vibrating on its mount.
			const Vec3 MountVelW = CentreVelW + Chassis.BodyToWorldDir(Vec3(0.0, 0.0, Chassis.NodeBendVelocity(Section)));
			const Vec3 RelB = MountRelVelB[I] + Vec3(W.MountPsiRate, 0.0, 0.0).Cross(ContactB - CentreB[I]);
			ContactVelW += Chassis.BodyToWorldDir(RelB);
			CentreVelW += Chassis.BodyToWorldDir(MountRelVelB[I]);
			const double Omega = bFront ? W.Omega : RearAxleOmega;
			Tyres[I].Evaluate(W.Contact, ContactVelW, CentreVelW, Omega, Dt, W.Tyre, &MountVelW);
			W.bLifted = W.Tyre.Fz < 1.0;
			if (Settings.bDamage) { WheelLoadDamage(I, W.Tyre.Fz, Dt); }

			ForceW += W.Tyre.ForceWorld;
			const Vec3 ForceB = Chassis.WorldToBodyDir(W.Tyre.ForceWorld);
			const Vec3 NormalB = Chassis.WorldToBodyDir(W.Contact.Normal);
			const Vec3 MzB = NormalB * W.Tyre.Mz;
			TorqueB += ContactB.Cross(ForceB) + MzB;

			NodeMoments[Section] += (ContactB - Chassis.TwistAxisPoint(Section)).Cross(ForceB).X + MzB.X;

			// Wheel mount: the assembly (mass M, camber inertia J) moves on the mount's stiffness relative to its
			// frame node. Implicit (backward Euler) in the mount's own stiffness and damping - the rear-axle and stub
			// modes are far above anything else in the model - with the tyre force explicit. The rigid body gets the
			// tyre force in full plus the reaction of the assembly's relative acceleration, so momentum is exact.
			FMount& Mt = Mounts[I];
			if (Mt.bActive)
			{
				const Vec3 Arm = ContactB - CentreB[I];
				const double G0 = ForceB.Z, G1 = Arm.Cross(ForceB).X + MzB.X;
				const double Fatt = LastSpecificForceBody.Z + BendAccPrev[Section]; // attachment's specific force
				const double Q0 = W.MountZ + Mt.Qs[0], Q1 = W.MountPsi + Mt.Qs[1];
				const double A00 = Mt.M + Dt * Mt.C[0][0] + Dt * Dt * Mt.K[0][0];
				const double A11 = Mt.J + Dt * Mt.C[1][1] + Dt * Dt * Mt.K[1][1];
				const double A01 = Dt * Mt.C[0][1] + Dt * Dt * Mt.K[0][1];
				const double R0 = Mt.M * W.MountZRate + Dt * (G0 - Mt.K[0][0] * Q0 - Mt.K[0][1] * Q1 - Mt.M * Fatt);
				const double R1 = Mt.J * W.MountPsiRate + Dt * (G1 - Mt.K[1][0] * Q0 - Mt.K[1][1] * Q1);
				const double Det = A00 * A11 - A01 * A01;
				const double Vz = (R0 * A11 - A01 * R1) / Det;
				const double Vp = (A00 * R1 - A01 * R0) / Det;
				const double Az = (Vz - W.MountZRate) / Dt, Ap = (Vp - W.MountPsiRate) / Dt;
				W.MountZRate = Vz; W.MountPsiRate = Vp;
				W.MountZ += Vz * Dt; W.MountPsi += Vp * Dt;
				W.MountForceZ = Mt.K[0][0] * (W.MountZ + Mt.Qs[0]) + Mt.K[0][1] * (W.MountPsi + Mt.Qs[1]) + Mt.C[0][0] * Vz;
				// Reaction of the assembly's acceleration relative to the rigid body (mount + frame bending).
				const Vec3 Rz(0.0, 0.0, -Mt.M * (Az + BendAccPrev[Section]));
				const double Rp = -Mt.J * Ap;
				ForceW += Chassis.BodyToWorldDir(Rz);
				TorqueB += CentreB[I].Cross(Rz) + Vec3(Rp, 0.0, 0.0);
				NodeMoments[Section] += (CentreB[I] - Chassis.TwistAxisPoint(Section)).Cross(Rz).X + Rp;
				NodeVertical[Section] += W.MountForceZ;
			}
			else
			{
				W.MountForceZ = ForceB.Z;
				NodeVertical[Section] += ForceB.Z;
			}
			if (bFront)
			{
				// Twist (~1 deg) is neglected for the kingpin moment arm.
				KingpinMoments[I] = Steering.KingpinMoment(I, ContactB, ForceB, W.Tyre.Mz, NormalB);
			}
		}

		// TorqueB is the full rigid-body moment; the frame chain splits its roll part into node twists
		// (tyre moments act on the axle nodes, see ChassisModel::Integrate).

		LastSpecificForceBody = Chassis.WorldToBodyDir(ForceW + Vec3(0.0, 0.0, CP.Mass * kGravity)) / CP.Mass;
		SpecificForceAccum += LastSpecificForceBody;

		// 4. Chassis integration.
		double BendRateBefore[ChassisParams::MaxFrameNodes];
		for (int K = 0; K < Chassis.NumNodes(); ++K) { BendRateBefore[K] = Chassis.NodeBendVelocity(K); }
		Chassis.Integrate(ForceW, TorqueB, NodeMoments, NodeVertical, Dt);
		for (int K = 0; K < Chassis.NumNodes(); ++K) { BendAccPrev[K] = (Chassis.NodeBendVelocity(K) - BendRateBefore[K]) / Dt; }

		// 5. Front wheel spin (independent wheels on stub axles).
		for (int I = 0; I < 2; ++I)
		{
			WheelState& W = Wheels[I];
			const double Re = W.Contact.bInContact ? W.Contact.LoadedRadius : Config.FrontTyre.UnloadedRadius;
			const double Torque = -W.Tyre.Fx * Re - W.Tyre.RollingResistanceTorque + ExtWheelTorque[I];
			// Linearised backward Euler for the low-speed tyre damping (dT/dw = -D*Re^2).
			const double Ieff = Config.FrontTyre.SpinInertia + Dt * W.Tyre.LongitudinalDamping * Re * Re;
			W.Omega += Torque / Ieff * Dt;
			if (Brakes.HasFrontBrakes())
			{
				ApplyCoulombTorque(W.Omega, Brakes.FrontTorque(Inputs.Brake), Ieff, Dt);
			}
		}

		// 6. Engine, clutch, gearbox, chain and the solid rear axle (with the rear brake).
		Transmission.Update(Dt);
		Cond.bAllowSeizure = Settings.bEngineFailures;
		Engine.SetPowerFactor(Cond.PowerFactor());
		const double EngineTorque = Engine.UpdateTorque(Inputs.Throttle, Inputs.bIgnitionKill, Dt, Inputs.bStarter);
		double AxleExternal = ExtWheelTorque[WheelRL] + ExtWheelTorque[WheelRR];
		double AxleDamping = 0.0;
		for (int I = WheelRL; I <= WheelRR; ++I)
		{
			const WheelState& W = Wheels[I];
			const double Re = W.Contact.bInContact ? W.Contact.LoadedRadius : Config.RearTyre.UnloadedRadius;
			AxleExternal += -W.Tyre.Fx * Re - W.Tyre.RollingResistanceTorque;
			AxleDamping += W.Tyre.LongitudinalDamping * Re * Re;
		}
		LastBrakeTorque = Brakes.RearTorque(Inputs.Brake);
		const double AxleBefore = RearAxleOmega;
		// A bent axle scrubs (bearings, wobbling wheels): Coulomb friction on the axle, like the brake.
		const double BendDrag = Config.Damage.AxleDragPerDeg * Damage.AxleBendDeg;
		Transmission.Step(Engine, EngineTorque, Inputs.Clutch, RearAxleOmega, AxleExternal,
		                  Config.FinalDrive.RearAxleInertia, AxleDamping, LastBrakeTorque + BendDrag, Dt, LastDrivetrain);
		Cond.Update(Dt, Settings.Air(), Engine.GetRpm(), Engine.GetLastCombustionTorque(), Engine.IsRunning(), Speed, Engine.GetEffectiveThrottle());
		if (Cond.IsSeized() && !Engine.IsSeized())
		{
			Engine.SetSeized(true, Config.Tuning.SeizedDragTorque);
			AddDamageEvent("ENGINE SEIZED");
		}
		const double BrakePower = LastBrakeTorque * std::fabs(0.5 * (AxleBefore + RearAxleOmega));
		Brakes.UpdateThermal(BrakePower, Speed, Settings.AmbientTemp, Dt);
		Wheels[WheelRL].Omega = Wheels[WheelRR].Omega = RearAxleOmega;

		// 7. Tyre state (slip, thermal, wear) advanced with the NEW wheel speeds; visual spin angles.
		for (int I = 0; I < NumWheels; ++I)
		{
			Tyres[I].Integrate(Wheels[I].Omega, Settings.AmbientTemp, Settings.TrackTemp, Dt);
			Wheels[I].SpinAngle = std::fmod(Wheels[I].SpinAngle + Wheels[I].Omega * Dt, 2.0 * kPi);
		}

		// 8. Steering column torque from the kingpin moments.
		ColumnTorqueAccum += Steering.ColumnTorque(KingpinMoments[0], KingpinMoments[1], Inputs.SteeringColumnAngle);
	}

	void KartSimulation::FillTelemetry(const KartInputs& Inputs, const Vec3& SpecificForceBody)
	{
		const ChassisState& S = Chassis.State;
		TelemetryFrame& F = Frame;
		F.Time = Time;
		F.PosX = S.Position.X;
		F.PosY = S.Position.Y;
		F.PosZ = S.Position.Z;
		const Vec3 VB = Chassis.WorldToBodyDir(S.Velocity);
		F.SpeedKmh = S.Velocity.Length() * 3.6;
		F.VelLong = VB.X;
		F.VelLat = VB.Y;
		F.AccLongG = SpecificForceBody.X / kGravity;
		F.AccLatG = SpecificForceBody.Y / kGravity;
		F.AccVertG = SpecificForceBody.Z / kGravity;
		F.YawRateDeg = S.AngularVelocity.Z * kRadToDeg;
		double Roll, Pitch, Yaw;
		S.Orientation.ToEuler(Roll, Pitch, Yaw);
		F.RollDeg = Roll * kRadToDeg;
		F.PitchDeg = Pitch * kRadToDeg;
		F.YawDeg = Yaw * kRadToDeg;
		F.ChassisTwistDeg = S.Twist * kRadToDeg;
		F.RearTwistDeg = S.RearTwist * kRadToDeg;
		F.SideslipDeg = std::fabs(VB.X) > 1.0 ? std::atan2(VB.Y, std::fabs(VB.X)) * kRadToDeg : 0.0;

		F.EngineRpm = Engine.GetRpm();
		F.EngineTorque = Engine.GetLastTorque();
		F.Throttle = Inputs.Throttle;
		F.ThrottleEff = Engine.GetEffectiveThrottle();
		F.Brake = Inputs.Brake;
		F.ClutchPedal = Inputs.Clutch;
		F.Gear = Transmission.GetGear();
		F.ClutchSlipRpm = LastDrivetrain.SlipRpm;
		F.ClutchLocked = LastDrivetrain.bLocked ? 1.0 : 0.0;
		F.ClutchTorque = LastDrivetrain.ClutchTorque;
		F.RevLimiter = Engine.IsLimiterActive() ? 1.0 : 0.0;
		F.EngineRunning = Engine.IsRunning() ? 1.0 : 0.0;
		F.WaterTemp = Cond.GetNodeTemp();
		F.PistonTemp = Cond.GetPistonTemp();
		F.Egt = Cond.GetEgt();
		F.Richness = Cond.GetRichness();
		F.Lambda = Cond.GetLambda();
		F.SeizeLevel = Cond.GetSeizeLevel();
		F.PowerFactor = Engine.GetPowerFactor();
		F.Coolant = Cond.GetCoolant();
		F.AirDensity = MoistAirDensity(Settings.Air());
		F.DamageToeL = Damage.ToeDeg[0];
		F.DamageToeR = Damage.ToeDeg[1];
		F.AxleBend = Damage.AxleBendDeg;
		F.ChainOff = Damage.bChainOff ? 1.0 : 0.0;
		F.FairingOff = Damage.bFairingDisplaced ? 1.0 : 0.0;

		F.SteerColumnDeg = Inputs.SteeringColumnAngle * kRadToDeg;
		F.SteerLeftDeg = Wheels[WheelFL].SteerAngle * kRadToDeg;
		F.SteerRightDeg = Wheels[WheelFR].SteerAngle * kRadToDeg;
		F.ColumnTorque = ColumnTorqueAvg;
		F.FfbOutput = Ffb.GetOutput();
		F.FfbClipping = Ffb.GetClipRatio();
		F.RearAxleRpm = RearAxleOmega * kRadSToRpm;
		F.BrakeTorque = LastBrakeTorque;
		F.BrakeDiscTemp = Brakes.GetDiscTemp();

		const WheelState& FL = Wheels[WheelFL];
		const WheelState& FR = Wheels[WheelFR];
		const WheelState& RL = Wheels[WheelRL];
		const WheelState& RR = Wheels[WheelRR];
		F.FzFL = FL.Tyre.Fz; F.FzFR = FR.Tyre.Fz; F.FzRL = RL.Tyre.Fz; F.FzRR = RR.Tyre.Fz;
		F.FxFL = FL.Tyre.Fx; F.FxFR = FR.Tyre.Fx; F.FxRL = RL.Tyre.Fx; F.FxRR = RR.Tyre.Fx;
		F.FyFL = FL.Tyre.Fy; F.FyFR = FR.Tyre.Fy; F.FyRL = RL.Tyre.Fy; F.FyRR = RR.Tyre.Fy;
		F.MzFL = FL.Tyre.Mz; F.MzFR = FR.Tyre.Mz;
		F.CamberFLDeg = FL.Tyre.Camber * kRadToDeg; F.CamberFRDeg = FR.Tyre.Camber * kRadToDeg;
		F.CamberRLDeg = RL.Tyre.Camber * kRadToDeg; F.CamberRRDeg = RR.Tyre.Camber * kRadToDeg;
		// Mount deflection (static included) and the camber the rear axle's bending adds.
		F.MountDzFLmm = (FL.MountZ + Mounts[WheelFL].Qs[0]) * 1000.0; F.MountDzFRmm = (FR.MountZ + Mounts[WheelFR].Qs[0]) * 1000.0;
		F.MountDzRLmm = (RL.MountZ + Mounts[WheelRL].Qs[0]) * 1000.0; F.MountDzRRmm = (RR.MountZ + Mounts[WheelRR].Qs[0]) * 1000.0;
		F.MountCamberRLDeg = (RL.MountPsi + Mounts[WheelRL].Qs[1]) * kRadToDeg;
		F.MountCamberRRDeg = -(RR.MountPsi + Mounts[WheelRR].Qs[1]) * kRadToDeg;
		// Frame sag of the heaviest interior node relative to the axle line (static included), mm, + = down.
		{
			const ChassisModel& CM = Chassis;
			double Sag = 0.0;
			if (CM.HasBending())
			{
				const int N = CM.NumNodes();
				int Mid = 1;
				for (int K = 1; K + 1 < N; ++K) { if (CM.BendingMass(K) > CM.BendingMass(Mid)) { Mid = K; } }
				const double X0 = Config.Chassis.FrameNodePos[0], X1 = Config.Chassis.FrameNodePos[N - 1], Xm = Config.Chassis.FrameNodePos[Mid];
				auto Wt = [&](int K) { return CM.NodeStaticBend(K) + CM.NodeBendOffset(K); };
				const double Line = Wt(0) + (Wt(N - 1) - Wt(0)) * (Xm - X0) / (X1 - X0);
				Sag = (Line - Wt(Mid)) * 1000.0;
			}
			F.FrameBendMm = Sag;
		}
		F.SlipRatioFL = FL.Tyre.SlipRatio; F.SlipRatioFR = FR.Tyre.SlipRatio;
		F.SlipRatioRL = RL.Tyre.SlipRatio; F.SlipRatioRR = RR.Tyre.SlipRatio;
		F.SlipAngleFLDeg = FL.Tyre.SlipAngle * kRadToDeg; F.SlipAngleFRDeg = FR.Tyre.SlipAngle * kRadToDeg;
		F.SlipAngleRLDeg = RL.Tyre.SlipAngle * kRadToDeg; F.SlipAngleRRDeg = RR.Tyre.SlipAngle * kRadToDeg;
		F.CombinedSlipFL = FL.Tyre.CombinedSlip; F.CombinedSlipFR = FR.Tyre.CombinedSlip;
		F.CombinedSlipRL = RL.Tyre.CombinedSlip; F.CombinedSlipRR = RR.Tyre.CombinedSlip;
		F.MuFL = FL.Tyre.Mu; F.MuFR = FR.Tyre.Mu; F.MuRL = RL.Tyre.Mu; F.MuRR = RR.Tyre.Mu;
		F.TempFL = Tyres[WheelFL].GetTemperature(); F.TempFR = Tyres[WheelFR].GetTemperature();
		F.TempRL = Tyres[WheelRL].GetTemperature(); F.TempRR = Tyres[WheelRR].GetTemperature();
		F.WearFL = Tyres[WheelFL].GetWear(); F.WearFR = Tyres[WheelFR].GetWear();
		F.WearRL = Tyres[WheelRL].GetWear(); F.WearRR = Tyres[WheelRR].GetWear();
		F.PressFL = Tyres[WheelFL].GetPressure(); F.PressFR = Tyres[WheelFR].GetPressure();
		F.PressRL = Tyres[WheelRL].GetPressure(); F.PressRR = Tyres[WheelRR].GetPressure();
		F.LiftFL = FL.bLifted ? 1.0 : 0.0; F.LiftFR = FR.bLifted ? 1.0 : 0.0;
		F.LiftRL = RL.bLifted ? 1.0 : 0.0; F.LiftRR = RR.bLifted ? 1.0 : 0.0;
		F.SurfaceFL = static_cast<double>(FL.Contact.Surface); F.SurfaceFR = static_cast<double>(FR.Contact.Surface);
		F.SurfaceRL = static_cast<double>(RL.Contact.Surface); F.SurfaceRR = static_cast<double>(RR.Contact.Surface);
	}

	void KartSimulation::BodyGroundContacts(const ITrackQuery& Track, Vec3& ForceW, Vec3& TorqueB)
	{
		// Body points (kart frame: x from the CoG, y left, z ABOVE THE GROUND at rest) - ESTIMATES from the model.
		const ChassisParams& Ch = Config.Chassis;
		const double RearX = -Ch.FrontWeightFraction * Ch.Wheelbase, FrontX = RearX + Ch.Wheelbase;
		struct Pt { double X, Y, Z; };
		const Pt Pts[] = {
			{RearX + 0.33, 0.0, 1.08},                                          // helmet top
			{RearX + 0.33, 0.24, 0.80}, {RearX + 0.33, -0.24, 0.80},            // shoulders
			{FrontX - 0.25, 0.17, 0.60}, {FrontX - 0.25, -0.17, 0.60},          // steering wheel rim
			{FrontX + 0.40, 0.30, 0.10}, {FrontX + 0.40, -0.30, 0.10},          // nose
			{FrontX - 0.20, 0.68, 0.10}, {FrontX - 0.20, -0.68, 0.10},          // sidepod front
			{RearX + 0.25, 0.68, 0.10}, {RearX + 0.25, -0.68, 0.10},            // sidepod rear
			{RearX - 0.38, 0.68, 0.18}, {RearX - 0.38, -0.68, 0.18},            // rear bumper ends
			{RearX + 0.40, 0.0, 0.015},                                         // seat bottom (scrapes on hard landings)
			{FrontX - 0.10, 0.25, 0.025}, {FrontX - 0.10, -0.25, 0.025},        // front frame rails
		};
		// Ground estimate from the tyre contacts: only points close to it (or any point when the kart is not on its
		// wheels) are ray-cast, which keeps this cheap on a kart that is just driving.
		Vec3 Pg, Ng;
		int InContact = 0;
		for (int I = 0; I < NumWheels; ++I)
		{
			if (Wheels[I].Contact.bInContact) { Pg += Wheels[I].Contact.ContactPoint; Ng += Wheels[I].Contact.Normal; ++InContact; }
		}
		const bool bOnWheels = InContact >= 3;
		if (bOnWheels) { Pg = Pg / static_cast<double>(InContact); Ng = Ng.Normalized(); }
		constexpr double K = 2.0e5, C = 3000.0, Mu = 0.55; // ASSUMPTIONS: plastic / steel / race suit on asphalt
		for (const Pt& P : Pts)
		{
			const Vec3 Pb(P.X, P.Y, P.Z - Ch.CogHeight);
			const Vec3 Pw = Chassis.BodyToWorldPoint(Pb);
			if (bOnWheels && (Pw - Pg).Dot(Ng) > 0.06) { continue; }
			GroundHit Hit;
			if (!Track.QueryGround(Pw, 1.5, 0.5, Hit)) { continue; }
			const double Pen = (Hit.Point - Pw).Dot(Hit.Normal);
			if (Pen <= 0.0) { continue; }
			const Vec3 V = Chassis.PointVelocityWorld(Pb);
			const double Vn = V.Dot(Hit.Normal);
			const double Fn = Clamp(K * std::fmin(Pen, 0.1) - C * Vn, 0.0, 60000.0);
			if (Fn <= 0.0) { continue; }
			const Vec3 Vt = V - Hit.Normal * Vn;
			const double VtLen = Vt.Length();
			Vec3 F = Hit.Normal * Fn;
			if (VtLen > 1e-6) { F += Vt * (-Mu * Fn * std::tanh(VtLen / 0.2) / VtLen); }
			ForceW += F;
			TorqueB += Pb.Cross(Chassis.WorldToBodyDir(F));
		}
	}

	void KartSimulation::Repair()
	{
		Cond.Repair();
		Engine.SetSeized(false, 0.0);
		Damage = KartDamageState();
		ApplyDamageEffects();
	}

	void KartSimulation::SetDamage(const KartDamageState& D)
	{
		Damage = D;
		ApplyDamageEffects();
	}

	void KartSimulation::ApplyDamageEffects()
	{
		Steering.SetDamageToe(0, Damage.ToeDeg[0] * kDegToRad);
		Steering.SetDamageToe(1, Damage.ToeDeg[1] * kDegToRad);
		Transmission.SetChainOff(Damage.bChainOff);
		Cond.SetCoolantLeak(Damage.RadiatorLeak);
	}

	void KartSimulation::AddDamageEvent(const char* What)
	{
		++Damage.NumEvents;
		Damage.LastEvent = What;
		Damage.LastEventTime = Time;
	}

	void KartSimulation::AddContactForce(const Vec3& PointWorld, const Vec3& ForceWorld)
	{
		if (NumContacts < MaxContacts)
		{
			ContactPoint[NumContacts] = PointWorld;
			ContactForce[NumContacts] = ForceWorld;
			++NumContacts;
		}
	}

	void KartSimulation::WheelLoadDamage(int Wheel, double Fz, double Dt)
	{
		// Damage grows with the PEAK (20 ms filtered) load of each strike (kerb hit, landing); the event ends when the
		// load falls back under the threshold. One strike adds at most MaxBendPerHit.
		const DamageParams& P = Config.Damage;
		double& Ff = WheelFzFilt[Wheel];
		Ff += (Fz - Ff) * std::fmin(1.0, Dt / 0.02);
		Fz = Ff;
		const bool bFront = Wheel < 2;
		const double Threshold = bFront ? P.WheelLoadFront : P.WheelLoadRear;
		if (Fz <= Threshold) { WheelPeakFz[Wheel] = 0.0; WheelStrikeDeg[Wheel] = 0.0; return; }
		const double Prev = std::fmax(WheelPeakFz[Wheel], Threshold);
		if (Fz <= Prev) { return; }
		WheelPeakFz[Wheel] = Fz;
		const double Want = (bFront ? P.LoadToePerKn : P.LoadBendPerKn) * (Fz - Threshold) / 1000.0;
		const double Add = std::fmin(Want, P.MaxBendPerHit) - WheelStrikeDeg[Wheel];
		if (Add <= 0.0) { return; }
		WheelStrikeDeg[Wheel] += Add;
		if (bFront)
		{
			// A vertical strike bends the steering arm: toe-out on that side.
			double& Toe = Damage.ToeDeg[Wheel];
			if (Toe == 0.0) { AddDamageEvent(Wheel == 0 ? "Bent left steering arm" : "Bent right steering arm"); }
			Toe = Clamp(Toe - Add, -P.MaxToe, P.MaxToe);
		}
		else
		{
			if (Damage.AxleBendDeg == 0.0) { AddDamageEvent("Bent rear axle"); }
			Damage.AxleBendDeg = std::fmin(P.MaxAxleBend, Damage.AxleBendDeg + Add);
			if (!Damage.bChainOff && Damage.AxleBendDeg >= P.ChainOffBend) { Damage.bChainOff = true; AddDamageEvent("CHAIN OFF"); }
		}
		ApplyDamageEffects();
	}

	void KartSimulation::UpdateContactDamage(double Dt)
	{
		const DamageParams& P = Config.Damage;
		const ChassisState& S = Chassis.State;
		const double FrontX = Chassis.StaticWheelCentre(WheelFL).X;
		const double RearX = Chassis.StaticWheelCentre(WheelRL).X;
		// Sum this step's kart-kart forces per channel: 0 nose (rearward push on the fairing), 1 rear, 2 left side,
		// 3 right side, 4 / 5 left / right front corner (steering arm side).
		double Raw[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		double CornerDir[2] = {0.0, 0.0};
		for (int I = 0; I < NumContacts; ++I)
		{
			const Vec3 Pb = S.Orientation.Unrotate(ContactPoint[I] - S.Position);
			const Vec3 Fb = Chassis.WorldToBodyDir(ContactForce[I]);
			const double F = std::sqrt(Fb.X * Fb.X + Fb.Y * Fb.Y);
			if (Pb.X > FrontX - 0.2)
			{
				if (std::fabs(Pb.Y) < 0.45) { Raw[0] += std::fmax(0.0, -Fb.X); continue; }
				const int Side = Pb.Y > 0.0 ? 0 : 1;
				Raw[4 + Side] += F;
				CornerDir[Side] += F * (std::fabs(Fb.X) > std::fabs(Fb.Y) ? (Fb.X < 0.0 ? -1.0 : 1.0) : 1.0); // rearward push = toe-out
			}
			else { Raw[Pb.X < RearX + 0.15 ? 1 : (Pb.Y > 0.0 ? 2 : 3)] += F; }
		}
		// 15 ms low-pass: one-substep overlap spikes in a pile-up do not count as a hit.
		const double A = std::fmin(1.0, Dt / 0.015);
		for (int C = 0; C < 6; ++C) { ContactFilt[C] += (Raw[C] - ContactFilt[C]) * A; }
		const double* Fz = ContactFilt;
		bool bChanged = false;
		// Bodywork (visual + race rules): CIK front fairing kit, sidepods, rear bumper.
		if (Fz[0] > P.FairingForce && !Damage.bFairingDisplaced) { Damage.bFairingDisplaced = true; AddDamageEvent("FRONT FAIRING DISPLACED"); bChanged = true; }
		for (int Sd = 0; Sd < 2; ++Sd)
		{
			if (Fz[2 + Sd] > P.SidepodForce && !Damage.bSidepodKnocked[Sd])
			{
				Damage.bSidepodKnocked[Sd] = true;
				AddDamageEvent(Sd == 0 ? "Left sidepod knocked loose" : "Right sidepod knocked loose");
				bChanged = true;
			}
		}
		if (Fz[1] > P.BumperForce && !Damage.bBumperKnocked) { Damage.bBumperKnocked = true; AddDamageEvent("Rear bumper pushed in"); bChanged = true; }
		// Bent parts: impulse above ContactForce, at most MaxBendPerHit per hit (EventImpulse zones: front corners, rear,
		// left, right).
		const double Over[4] = {std::fmax(0.0, std::fmax(Fz[4], Fz[5]) - P.ContactForce) * Dt, std::fmax(0.0, Fz[1] - P.ContactForce) * Dt,
		                        std::fmax(0.0, Fz[2] - P.ContactForce) * Dt, std::fmax(0.0, Fz[3] - P.ContactForce) * Dt};
		for (int Z = 0; Z < 4; ++Z) { Damage.EventImpulse[Z] = Over[Z] > 0.0 ? Damage.EventImpulse[Z] + Over[Z] : 0.0; }
		for (int Sd = 0; Sd < 2; ++Sd)
		{
			const double O = std::fmax(0.0, Fz[4 + Sd] - P.ContactForce) * Dt;
			if (O <= 0.0) { EventDeg[Sd] = 0.0; continue; }
			const double Add = std::fmin(P.ToePerImpulse * O, P.MaxBendPerHit - EventDeg[Sd]);
			if (Add <= 0.0) { continue; }
			EventDeg[Sd] += Add;
			double& Toe = Damage.ToeDeg[Sd];
			if (Toe == 0.0) { AddDamageEvent(Sd == 0 ? "Bent left steering" : "Bent right steering"); }
			Toe = Clamp(Toe + (CornerDir[Sd] < 0.0 ? -1.0 : 1.0) * Add, -P.MaxToe, P.MaxToe);
			bChanged = true;
		}
		if (Over[1] <= 0.0) { EventDeg[2] = 0.0; }
		else
		{
			const double Add = std::fmin(P.AxleBendPerImpulse * Over[1], P.MaxBendPerHit - EventDeg[2]);
			if (Add > 0.0)
			{
				EventDeg[2] += Add;
				if (Damage.AxleBendDeg == 0.0) { AddDamageEvent("Bent rear axle"); }
				Damage.AxleBendDeg = std::fmin(P.MaxAxleBend, Damage.AxleBendDeg + Add);
				bChanged = true;
			}
			if (!Damage.bChainOff && (Damage.EventImpulse[1] >= P.ChainOffImpulse || Damage.AxleBendDeg >= P.ChainOffBend))
			{
				Damage.bChainOff = true;
				AddDamageEvent("CHAIN OFF");
				bChanged = true;
			}
		}
		for (int Sd = 0; Sd < 2; ++Sd)
		{
			if (Over[2 + Sd] <= 0.0 || P.RadiatorSide != (Sd == 0 ? 1.0 : -1.0) || !Config.Tuning.bWaterCooled) { continue; }
			if (Damage.RadiatorLeak == 0.0) { AddDamageEvent("RADIATOR LEAKING"); }
			Damage.RadiatorLeak += P.RadiatorLeakPerImpulse * Over[2 + Sd];
			bChanged = true;
		}
		if (bChanged) { ApplyDamageEffects(); }
	}

	void KartSimulation::SetExternalForce(const Vec3& ForceWorld, const Vec3& PointWorld)
	{
		SetExternalLoad(ForceWorld, (PointWorld - Chassis.State.Position).Cross(ForceWorld));
	}

	void KartSimulation::SetExternalLoad(const Vec3& ForceWorld, const Vec3& TorqueAboutCogWorld)
	{
		// Held over the next Step(): the moment is frozen in the body frame (contacts are recomputed
		// every outer step, 1/240 s, so the rotation within a step is negligible).
		ExtForceW = ForceWorld;
		ExtTorqueB = Chassis.WorldToBodyDir(TorqueAboutCogWorld);
	}

	void KartSimulation::SetKinematicPose(const Vec3& Position, const Quat& Orientation, const Vec3& Velocity, const Vec3& AngularVelocityBody,
	                                      double SteerLeft, double SteerRight, double OmegaFront, double OmegaRear, double Dt)
	{
		ChassisState& S = Chassis.State;
		S.Position = Position;
		S.Orientation = Orientation;
		S.Velocity = Velocity;
		S.AngularVelocity = AngularVelocityBody;
		for (int I = 0; I < NumWheels; ++I)
		{
			WheelState& W = Wheels[I];
			Vec3 CentreB = Chassis.StaticWheelCentre(I), AxleB(0.0, 1.0, 0.0);
			if (I < 2)
			{
				W.SteerAngle = I == WheelFL ? SteerLeft : SteerRight;
				Steering.WheelPose(I, W.SteerAngle, CentreB, AxleB);
			}
			W.Omega = I < 2 ? OmegaFront : OmegaRear;
			W.SpinAngle += W.Omega * Dt;
			W.CentreWorld = Chassis.BodyToWorldPoint(CentreB);
			W.AxleWorld = Chassis.BodyToWorldDir(AxleB);
			W.Contact.bInContact = false;
		}
	}
}
