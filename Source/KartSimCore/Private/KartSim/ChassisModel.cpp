#include "KartSim/ChassisModel.h"

#include <cmath>

namespace kartsim
{
	void ChassisModel::Init(const KartConfig& InConfig)
	{
		Config = &InConfig;
		const ChassisParams& C = InConfig.Chassis;
		A = C.Wheelbase * (1.0 - C.FrontWeightFraction); // CoG -> front axle
		B = C.Wheelbase * C.FrontWeightFraction;         // CoG -> rear axle

		// Wheel centres are placed so that, under static load, the CoG sits at CogHeight.
		const double W = C.Mass * kGravity;
		const double FzFront = 0.5 * W * C.FrontWeightFraction;
		const double FzRear = 0.5 * W * (1.0 - C.FrontWeightFraction);
		// Tyre deflection at the cold pressure (the kart is set down with cold tyres).
		const double DefF = FzFront / (InConfig.FrontTyre.RadialStiffness * InConfig.FrontTyre.StiffnessFactor(InConfig.FrontTyre.ColdPressure));
		const double DefR = FzRear / (InConfig.RearTyre.RadialStiffness * InConfig.RearTyre.StiffnessFactor(InConfig.RearTyre.ColdPressure));
		const double Zf = -C.CogHeight + InConfig.FrontTyre.UnloadedRadius - DefF;
		const double Zr = -C.CogHeight + InConfig.RearTyre.UnloadedRadius - DefR;

		WheelCentres[WheelFL] = Vec3(A, 0.5 * C.FrontTrack, Zf);
		WheelCentres[WheelFR] = Vec3(A, -0.5 * C.FrontTrack, Zf);
		WheelCentres[WheelRL] = Vec3(-B, 0.5 * C.RearTrack, Zr);
		WheelCentres[WheelRR] = Vec3(-B, -0.5 * C.RearTrack, Zr);

		// Frame chain: uniform GJ, segment stiffness/damping inversely proportional to length.
		const double GJ = C.FrameTorsionalStiffness * C.Wheelbase;
		const double CJ = C.FrameTorsionalDamping * C.Wheelbase;
		for (int I = 0; I < C.NumFrameNodes; ++I)
		{
			// Twist axes at frame-rail height (the frame's shear-centre axis): a ground-level tyre force
			// twists an axle section with lever (TwistAxisHeight), the rest passes into the body as shear.
			// (A CoG-height axis was tried and rejected: it exaggerated twist excitation 2.7x and broke the
			// drivability regressions. The energy gain that motivated it was the tyre deflection memory,
			// fixed in TyreModel::Integrate.)
			NodePoints[I] = Vec3(A - C.FrameNodePos[I], 0.0, -C.CogHeight + C.TwistAxisHeight);
			if (I + 1 < C.NumFrameNodes)
			{
				const double Len = C.FrameNodePos[I + 1] - C.FrameNodePos[I];
				SegK[I] = C.FrameSegmentStiffness[I] > 0.0 ? C.FrameSegmentStiffness[I] : GJ / Len;
				SegC[I] = C.FrameSegmentDamping[I] > 0.0 ? C.FrameSegmentDamping[I] : CJ / Len;
			}
		}
		State = ChassisState();

		// --- Vertical bending chain (Milestone 9) ---
		// The wheel assemblies at the axle nodes are their own masses on the wheel mounts (KartPhysicsCore), so they
		// do not take part in bending. Static: the mounts carry the axle loads minus the wheel assemblies' weight.
		const int N = C.NumFrameNodes;
		const double Mf = C.FrontMount.UnsprungMass > 0.0 ? 2.0 * C.FrontMount.UnsprungMass : 0.0;
		const double Mr = C.RearMount.UnsprungMass > 0.0 ? 2.0 * C.RearMount.UnsprungMass : 0.0;
		bBending = C.FrameBendingEI > 0.0 && N >= 3;
		for (int I = 0; I < N; ++I)
		{
			BendX[I] = A - C.FrameNodePos[I];
			BendMass[I] = C.FrameNodeMass[I] - (I == 0 ? Mf : 0.0) - (I == N - 1 ? Mr : 0.0);
			if (BendMass[I] < 0.5) { bBending = false; }
			BendStatic[I] = 0.0;
			HingeK[I] = HingeC[I] = 0.0;
		}
		if (bBending)
		{
			const double L = C.FrameNodePos[N - 1] - C.FrameNodePos[0];
			for (int J = 1; J + 1 < N; ++J)
			{
				const double L0 = C.FrameNodePos[J] - C.FrameNodePos[J - 1], L1 = C.FrameNodePos[J + 1] - C.FrameNodePos[J];
				// Three nodes: 3 EI / L reproduces the exact simply supported beam stiffness under a point load at the
				// middle node (3 EI L / (a^2 b^2)). More nodes: the standard discrete hinge EI / mean segment length.
				HingeK[J] = N == 3 ? 3.0 * C.FrameBendingEI / L : C.FrameBendingEI / (0.5 * (L0 + L1));
				const double G = 1.0 / L0 + 1.0 / L1;
				const double Omega = std::sqrt(HingeK[J] * G * G / BendMass[J]);
				HingeC[J] = 2.0 * C.FrameBendingDampingRatio * HingeK[J] / Omega;
			}
			double Fs[ChassisParams::MaxFrameNodes] = {};
			Fs[0] = C.Mass * kGravity * C.FrontWeightFraction - Mf * kGravity;
			Fs[N - 1] = C.Mass * kGravity * (1.0 - C.FrontWeightFraction) - Mr * kGravity;
			SolveStaticBending(Fs);
		}
	}

	void ChassisModel::BendingForces(const double W[], const double Wd[], double F[]) const
	{
		const ChassisParams& C = Config->Chassis;
		const int N = C.NumFrameNodes;
		for (int I = 0; I < N; ++I) { F[I] = 0.0; }
		for (int J = 1; J + 1 < N; ++J)
		{
			const double L0 = C.FrameNodePos[J] - C.FrameNodePos[J - 1], L1 = C.FrameNodePos[J + 1] - C.FrameNodePos[J];
			const double Th = (W[J + 1] - W[J]) / L1 - (W[J] - W[J - 1]) / L0;
			const double Thd = (Wd[J + 1] - Wd[J]) / L1 - (Wd[J] - Wd[J - 1]) / L0;
			const double M = HingeK[J] * Th + HingeC[J] * Thd;
			F[J + 1] -= M / L1;
			F[J] += M * (1.0 / L1 + 1.0 / L0);
			F[J - 1] -= M / L0;
		}
	}

	void ChassisModel::RemoveRigidModes(double U[]) const
	{
		// Mass-weighted least-squares heave + pitch component, removed: the rigid body carries those.
		const int N = Config->Chassis.NumFrameNodes;
		double Sm = 0, Smx = 0, Smxx = 0, Smu = 0, Smxu = 0;
		for (int I = 0; I < N; ++I)
		{
			const double M = BendMass[I], X = BendX[I];
			Sm += M; Smx += M * X; Smxx += M * X * X; Smu += M * U[I]; Smxu += M * X * U[I];
		}
		const double Det = Sm * Smxx - Smx * Smx;
		if (std::fabs(Det) < 1e-12) { return; }
		const double C0 = (Smu * Smxx - Smx * Smxu) / Det;
		const double C1 = (Sm * Smxu - Smx * Smu) / Det;
		for (int I = 0; I < N; ++I) { U[I] -= C0 + C1 * BendX[I]; }
	}

	void ChassisModel::SolveStaticBending(const double Fs[])
	{
		// Heavily damped pseudo-dynamics to the static bent shape (N <= 8, runs once per Init).
		const int N = Config->Chassis.NumFrameNodes;
		double W[ChassisParams::MaxFrameNodes] = {}, V[ChassisParams::MaxFrameNodes] = {}, Zero[ChassisParams::MaxFrameNodes] = {};
		double OmegaMax = 0.0;
		for (int J = 1; J + 1 < N; ++J)
		{
			const double L0 = Config->Chassis.FrameNodePos[J] - Config->Chassis.FrameNodePos[J - 1];
			const double L1 = Config->Chassis.FrameNodePos[J + 1] - Config->Chassis.FrameNodePos[J];
			const double G = 1.0 / L0 + 1.0 / L1;
			for (int I = J - 1; I <= J + 1; ++I) { OmegaMax = std::fmax(OmegaMax, std::sqrt(HingeK[J] * G * G * 2.0 / BendMass[I])); }
		}
		if (OmegaMax <= 0.0) { return; }
		const double H = 0.3 / OmegaMax;
		for (int It = 0; It < 400000; ++It)
		{
			double F[ChassisParams::MaxFrameNodes];
			BendingForces(W, Zero, F);
			double U[ChassisParams::MaxFrameNodes];
			for (int I = 0; I < N; ++I) { U[I] = (F[I] + Fs[I]) / BendMass[I]; }
			RemoveRigidModes(U);
			double Res = 0.0;
			for (int I = 0; I < N; ++I)
			{
				V[I] = V[I] * (1.0 - 0.2) + U[I] * H;
				W[I] += V[I] * H;
				Res = std::fmax(Res, std::fabs(U[I]));
			}
			if (Res < 1e-7 && It > 100) { break; }
		}
		RemoveRigidModes(W);
		for (int I = 0; I < N; ++I) { BendStatic[I] = W[I]; }
	}

	double ChassisModel::BendingElasticEnergy() const
	{
		if (!bBending) { return 0.0; }
		const ChassisParams& C = Config->Chassis;
		double E = 0.0;
		for (int J = 1; J + 1 < C.NumFrameNodes; ++J)
		{
			const double L0 = C.FrameNodePos[J] - C.FrameNodePos[J - 1], L1 = C.FrameNodePos[J + 1] - C.FrameNodePos[J];
			auto Th = [&](const double* W) { return (W[J + 1] - W[J]) / L1 - (W[J] - W[J - 1]) / L0; };
			double Wt[ChassisParams::MaxFrameNodes];
			for (int I = 0; I < C.NumFrameNodes; ++I) { Wt[I] = BendStatic[I] + State.NodeBend[I]; }
			// Energy relative to the static shape (the static strain energy is part of the rest state).
			E += 0.5 * HingeK[J] * (Sq(Th(Wt)) - Sq(Th(BendStatic)));
		}
		return E;
	}

	double ChassisModel::BendingKineticEnergy() const
	{
		double E = 0.0;
		for (int I = 0; bBending && I < Config->Chassis.NumFrameNodes; ++I) { E += 0.5 * BendMass[I] * Sq(State.NodeBendRate[I]); }
		return E;
	}

	Vec3 ChassisModel::TwistPointBody(const Vec3& P, int Node) const
	{
		return NodePoints[Node] + RotateAboutAxis(P - NodePoints[Node], Vec3(1.0, 0.0, 0.0), State.NodeTwist[Node]);
	}

	Vec3 ChassisModel::TwistDirBody(const Vec3& D, int Node) const
	{
		return RotateAboutAxis(D, Vec3(1.0, 0.0, 0.0), State.NodeTwist[Node]);
	}

	Vec3 ChassisModel::TwistVelocityBody(const Vec3& TwistedPoint, int Node) const
	{
		return Vec3(State.NodeTwistRate[Node], 0.0, 0.0).Cross(TwistedPoint - NodePoints[Node]);
	}

	double ChassisModel::FrameElasticEnergy() const
	{
		double E = 0.0;
		for (int I = 0; I + 1 < NumNodes(); ++I)
		{
			const double D = State.NodeTwist[I + 1] - State.NodeTwist[I];
			E += 0.5 * SegK[I] * D * D;
		}
		return E;
	}

	double ChassisModel::FrameTwistKineticEnergy() const
	{
		double E = 0.0;
		for (int I = 0; I < NumNodes(); ++I)
		{
			E += 0.5 * Config->Chassis.FrameNodeInertia[I] * State.NodeTwistRate[I] * State.NodeTwistRate[I];
		}
		return E;
	}

	void ChassisModel::Integrate(const Vec3& ForceWorld, const Vec3& TorqueBody, const double NodeMoments[], const double NodeVertical[], double Dt)
	{
		const ChassisParams& C = Config->Chassis;
		const int N = C.NumFrameNodes;

		// Linear
		State.Velocity += ForceWorld / C.Mass * Dt;
		State.Position += State.Velocity * Dt;

		// Angular (Euler's equations, body frame, diagonal inertia). Roll inertia = sum of node inertias.
		const Vec3& I = C.Inertia;
		const Vec3& W = State.AngularVelocity;
		const Vec3 Gyro = W.Cross(I.Mul(W));
		const Vec3 Net = TorqueBody - Gyro;
		const Vec3 AngAcc(Net.X / I.X, Net.Y / I.Y, Net.Z / I.Z);
		State.AngularVelocity += AngAcc * Dt;
		State.Orientation = State.Orientation.IntegrateBody(State.AngularVelocity, Dt);

		// Twist chain (relative to the rigid roll). Node external moment = attached tyre moment about
		// the node axis + inertia share of the rest of the total roll moment (keeps sum(J*phi'') = 0).
		double SumNode = 0.0, SumJ = 0.0;
		for (int K = 0; K < N; ++K) { SumNode += NodeMoments[K]; SumJ += C.FrameNodeInertia[K]; }
		const double Remainder = Net.X - SumNode;
		// Inertial (d'Alembert) roll load of each node's mass: the non-gravitational specific force
		// f = (F_total - m g)/m acts on node mass m_i at height z_i relative to the CoG axis, giving
		// roll moment m_i * z_i * f_y. These sum to zero over the nodes (sum m_i z_i = 0), so they only
		// redistribute twist load along the frame: a high, heavy driver twists the seat region.
		const Vec3 SpecificBody = State.Orientation.Unrotate(ForceWorld / C.Mass + Vec3(0.0, 0.0, kGravity));
		double DAlembert[ChassisParams::MaxFrameNodes];
		for (int K = 0; K < N; ++K)
		{
			DAlembert[K] = C.FrameNodeMass[K] * (C.FrameNodeHeight[K] - C.CogHeight) * SpecificBody.Y;
		}
		double Acc[ChassisParams::MaxFrameNodes];
		for (int K = 0; K < N; ++K)
		{
			double M = NodeMoments[K] + DAlembert[K] + Remainder * C.FrameNodeInertia[K] / SumJ;
			if (K > 0)
			{
				M += SegK[K - 1] * (State.NodeTwist[K - 1] - State.NodeTwist[K]) + SegC[K - 1] * (State.NodeTwistRate[K - 1] - State.NodeTwistRate[K]);
			}
			if (K + 1 < N)
			{
				M += SegK[K] * (State.NodeTwist[K + 1] - State.NodeTwist[K]) + SegC[K] * (State.NodeTwistRate[K + 1] - State.NodeTwistRate[K]);
			}
			// Relative to the chain's mean roll acceleration Net.X/SumJ: sum(J*Acc) = 0 exactly.
			Acc[K] = M / C.FrameNodeInertia[K] - Net.X / SumJ;
		}
		for (int K = 0; K < N; ++K)
		{
			State.NodeTwistRate[K] += Acc[K] * Dt;
			State.NodeTwist[K] += State.NodeTwistRate[K] * Dt;
		}
		State.Twist = State.NodeTwist[0];
		State.RearTwist = State.NodeTwist[N - 1];

		// Vertical bending: node loads (wheel-mount forces at the axle nodes) and hinge springs, rigid heave/pitch
		// removed (the rigid body above already carries them). Gravity is uniform per unit mass, so it drops out:
		// the frame sags only because the mounts carry it at the axles (static shape from SolveStaticBending).
		if (bBending && NodeVertical)
		{
			double Wt[ChassisParams::MaxFrameNodes], F[ChassisParams::MaxFrameNodes], U[ChassisParams::MaxFrameNodes];
			for (int K = 0; K < N; ++K) { Wt[K] = BendStatic[K] + State.NodeBend[K]; }
			BendingForces(Wt, State.NodeBendRate, F);
			for (int K = 0; K < N; ++K) { U[K] = (F[K] + NodeVertical[K]) / BendMass[K]; }
			RemoveRigidModes(U);
			for (int K = 0; K < N; ++K)
			{
				State.NodeBendRate[K] += U[K] * Dt;
				State.NodeBend[K] += State.NodeBendRate[K] * Dt;
			}
		}
	}
}
