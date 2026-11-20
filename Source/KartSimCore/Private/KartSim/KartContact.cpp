#include "KartSim/KartContact.h"

#include <cmath>

namespace kartsim
{
	KartFootprint KartFootprint::FromConfig(const KartConfig& C)
	{
		const double L = C.Chassis.Wheelbase;
		const double CogFromRearAxle = C.Chassis.FrontWeightFraction * L;
		const double Overhang = 0.5 * (1.82 - L);
		KartFootprint F;
		F.Rear = CogFromRearAxle + Overhang;
		F.Front = (L - CogFromRearAxle) + Overhang;
		F.HalfWidth = 0.70;
		F.NoseStart = (L - CogFromRearAxle) - 0.20; // the sidepods end here; ahead of it only the bumper bars and nose
		F.RearTyreBack = -CogFromRearAxle - C.RearTyre.UnloadedRadius;
		return F;
	}

	namespace
	{
		struct Box2
		{
			Vec3 C;          // centre (z ignored)
			Vec3 Ax, Ay;     // unit horizontal axes (forward, left)
			double Hx = 0.0, Hy = 0.0;
			Vec3 Corner(int I) const
			{
				const double Sx = (I == 0 || I == 3) ? 1.0 : -1.0;
				const double Sy = (I < 2) ? 1.0 : -1.0;
				return C + Ax * (Sx * Hx) + Ay * (Sy * Hy);
			}
			bool Contains(const Vec3& P) const
			{
				Vec3 D = P - C;
				D.Z = 0.0;
				return std::fabs(D.Dot(Ax)) <= Hx && std::fabs(D.Dot(Ay)) <= Hy;
			}
			double Radius(const Vec3& Axis) const { return Hx * std::fabs(Ax.Dot(Axis)) + Hy * std::fabs(Ay.Dot(Axis)); }
		};

		Box2 MakeBox(const KartSimulation& K, const KartFootprint& F, int Part)
		{
			const ChassisState& S = K.GetChassis();
			Vec3 Fwd = S.Orientation.Rotate(Vec3(1, 0, 0));
			Fwd.Z = 0.0;
			Fwd = Fwd.Normalized();
			Box2 B;
			B.Ax = Fwd;
			B.Ay = Vec3(-Fwd.Y, Fwd.X, 0.0);
			// Part 0 main (full width), 1 nose, 2 rear bumper.
			const double X0 = Part == 0 ? F.RearTyreBack : (Part == 1 ? F.NoseStart : -F.Rear);
			const double X1 = Part == 0 ? F.NoseStart : (Part == 1 ? F.Front : F.RearTyreBack);
			B.Hx = 0.5 * (X1 - X0);
			B.Hy = Part == 0 ? F.HalfWidth : (Part == 1 ? F.NoseHalfWidth : F.BumperHalfWidth);
			B.C = S.Position + Fwd * (0.5 * (X0 + X1));
			B.C.Z = 0.0;
			return B;
		}
	}

	namespace
	{
		KartContactInfo BoxContact(const KartSimulation& A, const KartFootprint& FA, int PartA, const KartSimulation& B, const KartFootprint& FB, int PartB,
		                           const ContactParams& P);
	}

	KartContactInfo ComputeKartContact(const KartSimulation& A, const KartFootprint& FA, const KartSimulation& B, const KartFootprint& FB,
	                                   const ContactParams& P)
	{
		KartContactInfo Best;
		for (int Pa = 0; Pa < 3; ++Pa)
		{
			for (int Pb = 0; Pb < 3; ++Pb)
			{
				const KartContactInfo C = BoxContact(A, FA, Pa, B, FB, Pb, P);
				if (C.bContact && (!Best.bContact || C.Depth > Best.Depth)) { Best = C; }
			}
		}
		return Best;
	}

	namespace
	{
	KartContactInfo BoxContact(const KartSimulation& A, const KartFootprint& FA, int PartA, const KartSimulation& B, const KartFootprint& FB, int PartB,
	                           const ContactParams& P)
	{
		KartContactInfo Out;
		const ChassisState& SA = A.GetChassis();
		const ChassisState& SB = B.GetChassis();
		// Vertical overlap of the two bodies (ground under each = CoG height below the CoG).
		const double GroundA = SA.Position.Z - A.GetConfig().Chassis.CogHeight;
		const double GroundB = SB.Position.Z - B.GetConfig().Chassis.CogHeight;
		if (GroundA + FA.TopHeight < GroundB || GroundB + FB.TopHeight < GroundA) { return Out; }

		const Box2 BA = MakeBox(A, FA, PartA), BB = MakeBox(B, FB, PartB);
		Vec3 D = BB.C - BA.C;
		D.Z = 0.0;
		if (D.Length() > BA.Hx + BA.Hy + BB.Hx + BB.Hy) { return Out; }

		// Separating axes: the two boxes' local axes. Keep the axis of minimum overlap.
		const Vec3 Axes[4] = {BA.Ax, BA.Ay, BB.Ax, BB.Ay};
		double MinOverlap = 1e18;
		Vec3 N;
		for (const Vec3& Ax : Axes)
		{
			const double Dist = D.Dot(Ax);
			const double Overlap = BA.Radius(Ax) + BB.Radius(Ax) - std::fabs(Dist);
			if (Overlap <= 0.0) { return Out; }
			if (Overlap < MinOverlap) { MinOverlap = Overlap; N = Dist >= 0.0 ? Ax : -Ax; }
		}

		// Contact point: mean of the corners of each box inside the other (edge-edge crossing without
		// a contained corner: midpoint between the boxes along the normal).
		Vec3 Sum;
		int Count = 0;
		for (int I = 0; I < 4; ++I)
		{
			const Vec3 Ca = BA.Corner(I), Cb = BB.Corner(I);
			if (BB.Contains(Ca)) { Sum += Ca; ++Count; }
			if (BA.Contains(Cb)) { Sum += Cb; ++Count; }
		}
		Vec3 Pt = Count > 0 ? Sum / static_cast<double>(Count) : BA.C + D * 0.5;
		Pt.Z = 0.5 * (GroundA + GroundB) + 0.5 * (FA.BumperHeight + FB.BumperHeight);

		// Relative velocity of B w.r.t. A at the contact point.
		const Vec3 VA = A.GetChassisModel().PointVelocityWorld(SA.Orientation.Unrotate(Pt - SA.Position));
		const Vec3 VB = B.GetChassisModel().PointVelocityWorld(SB.Orientation.Unrotate(Pt - SB.Position));
		Vec3 Vrel = VB - VA;
		Vrel.Z = 0.0;
		const double Vn = Vrel.Dot(N); // > 0 = separating
		double Fn = P.Stiffness * MinOverlap - P.Damping * Clamp(MinOverlap / P.DampingRamp, 0.0, 1.0) * Vn;
		Fn = Clamp(Fn, 0.0, P.MaxForce); // no adhesion
		// The bumpers / bodywork are a ~0.3 m slab: a kart riding up on another loses this push as the slabs
		// separate vertically (the 3-D wheel / body contacts carry it instead).
		const double SlabOverlap = 0.29 - std::fabs(GroundA - GroundB);
		Fn *= Clamp(SlabOverlap / 0.15, 0.0, 1.0);
		if (Fn <= 0.0) { return KartContactInfo(); }
		const Vec3 Vt = Vrel - N * Vn;
		const double VtLen = Vt.Length();
		Vec3 Ft;
		if (VtLen > 1e-9)
		{
			Ft = Vt * (-P.Friction * Fn * std::tanh(VtLen / P.FrictionVelocity) / VtLen);
		}
		Out.bContact = true;
		Out.Point = Pt;
		Out.Normal = N;
		Out.Depth = MinOverlap;
		Out.ForceOnB = N * Fn + Ft;
		return Out;
	}
	}

	int KartBodyBoxes(const KartConfig& C, KartBodyBox* Out, int Max)
	{
		// Body frame: origin at the CoG. Extents from the CIK envelope and the imported model (ESTIMATES).
		const double L = C.Chassis.Wheelbase;
		const double RearX = -C.Chassis.FrontWeightFraction * L;
		const double FrontX = RearX + L;
		const double G = -C.Chassis.CogHeight;
		auto Box = [G](double X0, double X1, double Y0, double Y1, double Z0, double Z1)
		{
			KartBodyBox B;
			B.Centre = Vec3(0.5 * (X0 + X1), 0.5 * (Y0 + Y1), G + 0.5 * (Z0 + Z1));
			B.Half = Vec3(0.5 * (X1 - X0), 0.5 * (Y1 - Y0), 0.5 * (Z1 - Z0));
			return B;
		};
		const KartBodyBox All[] = {
			Box(RearX + 0.10, FrontX - 0.15, -0.42, 0.42, 0.03, 0.30),   // floor tray, seat, engine
			Box(RearX + 0.25, FrontX - 0.20, 0.45, 0.68, 0.06, 0.30),    // left sidepod
			Box(RearX + 0.25, FrontX - 0.20, -0.68, -0.45, 0.06, 0.30),  // right sidepod
			// CIK front fairing, tapering (measured on the imported model, Tools/SplitNose.py; x ahead of the front axle)
			Box(FrontX + 0.25, FrontX + 0.37, -0.56, 0.56, 0.03, 0.20),
			Box(FrontX + 0.37, FrontX + 0.47, -0.46, 0.46, 0.03, 0.22),
			Box(FrontX + 0.47, FrontX + 0.60, -0.30, 0.30, 0.03, 0.20),
			Box(FrontX - 0.05, FrontX + 0.25, -0.30, 0.30, 0.05, 0.20),  // front bumper bars / nose mounts between the wheels
			Box(FrontX - 0.40, FrontX - 0.05, -0.20, 0.20, 0.28, 0.62),  // front panel, steering wheel
			Box(RearX - 0.38, RearX - 0.18, -0.60, 0.60, 0.08, 0.24),    // rear bumper (narrower than the rear tyres)
			Box(RearX + 0.10, RearX + 0.55, -0.24, 0.24, 0.30, 0.85),    // driver torso
			Box(RearX + 0.18, RearX + 0.48, -0.14, 0.14, 0.85, 1.10),    // helmet
		};
		const int N = static_cast<int>(sizeof(All) / sizeof(All[0]));
		for (int I = 0; I < N && I < Max; ++I) { Out[I] = All[I]; }
		return N < Max ? N : Max;
	}

	namespace
	{
		struct ContactAccum
		{
			std::vector<Vec3> F, T;
			std::vector<double> W; // 4 per kart: torque about each wheel's axle
			explicit ContactAccum(size_t N) : F(N), T(N), W(N * 4, 0.0) {}
		};

		// Velocity of a point on wheel I of K (chassis motion + spin about the axle).
		Vec3 WheelSurfaceVelocity(const KartSimulation& K, int I, const Vec3& P)
		{
			const ChassisState& S = K.GetChassis();
			const WheelState& W = K.GetWheel(I);
			return K.GetChassisModel().PointVelocityWorld(S.Orientation.Unrotate(P - S.Position)) + (W.AxleWorld * W.Omega).Cross(P - W.CentreWorld);
		}

		// Spin "mass" of a wheel at its rim (kg): what the friction has to accelerate.
		double WheelRimMass(const KartSimulation& K, int I)
		{
			const KartConfig& C = K.GetConfig();
			if (I < 2) { return C.FrontTyre.SpinInertia / (C.FrontTyre.UnloadedRadius * C.FrontTyre.UnloadedRadius); }
			const double G = C.TotalRatio(K.GetTransmission().GetGear() > 0 ? K.GetTransmission().GetGear() - 1 : 0);
			const double I2 = C.FinalDrive.RearAxleInertia + (K.GetTransmission().IsClutchLocked() ? C.Engine.Inertia * G * G : 0.0);
			return I2 / (C.RearTyre.UnloadedRadius * C.RearTyre.UnloadedRadius);
		}

		// Applies ForceOnA at P to A (and the reaction to B), with the spin torques on the wheels involved.
		void AddPair(ContactAccum& Acc, size_t Ia, const KartSimulation& A, int WheelA, size_t Ib, const KartSimulation& B, int WheelB, const Vec3& P, const Vec3& ForceOnA)
		{
			Acc.F[Ia] += ForceOnA;
			Acc.T[Ia] += (P - A.GetChassis().Position).Cross(ForceOnA);
			Acc.F[Ib] -= ForceOnA;
			Acc.T[Ib] -= (P - B.GetChassis().Position).Cross(ForceOnA);
			if (WheelA >= 0) { const WheelState& W = A.GetWheel(WheelA); Acc.W[Ia * 4 + WheelA] += (P - W.CentreWorld).Cross(ForceOnA).Dot(W.AxleWorld); }
			if (WheelB >= 0) { const WheelState& W = B.GetWheel(WheelB); Acc.W[Ib * 4 + WheelB] += (P - W.CentreWorld).Cross(-ForceOnA).Dot(W.AxleWorld); }
		}

		// Spring-damper along N (from A toward B) + friction on the relative surface velocity, capped so it cannot
		// reverse the slip within one outer step (MassEff = what the friction accelerates). Returns the force on A.
		Vec3 ContactForce(double Pen, const Vec3& N, const Vec3& VA, const Vec3& VB, double K, double C, double Mu, double MassEff, double Dt, double MaxF)
		{
			const Vec3 Vrel = VB - VA;
			const double Vn = Vrel.Dot(N);
			const double Fn = Clamp(K * Pen - C * Vn, 0.0, MaxF);
			if (Fn <= 0.0) { return Vec3(); }
			const Vec3 Vt = Vrel - N * Vn;
			const double VtLen = Vt.Length();
			Vec3 Ft;
			if (VtLen > 1e-6)
			{
				const double Cap = 0.5 * VtLen * MassEff / Dt;
				Ft = Vt * (std::fmin(Mu * Fn, Cap) / VtLen); // B's surface drags A along
			}
			return N * (-Fn) + Ft;
		}

		void WheelContacts(ContactAccum& Acc, size_t Ia, const KartSimulation& A, size_t Ib, const KartSimulation& B, const ContactParams& P, double Dt,
		                   bool& bTouch)
		{
			const KartConfig& CA = A.GetConfig();
			const KartConfig& CB = B.GetConfig();
			// Tyre to tyre (once per pair: the caller runs this for (A, B) and (B, A)).
			for (int I = 0; I < NumWheels && Ia < Ib; ++I)
			{
				const WheelState& Wa = A.GetWheel(I);
				const TyreParams& Ta = I < 2 ? CA.FrontTyre : CA.RearTyre;
				for (int J = 0; J < NumWheels; ++J)
				{
					const WheelState& Wb = B.GetWheel(J);
					const TyreParams& Tb = J < 2 ? CB.FrontTyre : CB.RearTyre;
					const Vec3 D = Wb.CentreWorld - Wa.CentreWorld;
					if (D.LengthSq() > 0.36) { continue; }
					const double La = D.Dot(Wa.AxleWorld);
					const double Hw = 0.5 * (Ta.Width + Tb.Width);
					if (std::fabs(La) >= Hw) { continue; }
					const Vec3 Dp = D - Wa.AxleWorld * La;
					const double Dist = Dp.Length();
					const double PenRad = Ta.UnloadedRadius + Tb.UnloadedRadius - Dist;
					if (PenRad <= 0.0) { continue; }
					const double PenLat = Hw - std::fabs(La);
					// Push out along the axis of least penetration: tread to tread (in the wheel plane), or sidewall to
					// sidewall (along the axle) when the wheels sit side by side.
					const bool bSide = PenLat < PenRad || Dist < 1e-6;
					const double Pen = bSide ? PenLat : PenRad;
					const Vec3 N = bSide ? Wa.AxleWorld * (La >= 0.0 ? 1.0 : -1.0) : Dp / Dist;
					const Vec3 Pt = bSide ? Wa.CentreWorld + Dp * 0.5 + Wa.AxleWorld * (La >= 0.0 ? 0.5 * Ta.Width : -0.5 * Ta.Width)
					                      : Wa.CentreWorld + N * (Ta.UnloadedRadius - 0.5 * Pen) + Wa.AxleWorld * (0.5 * La);
					const double WidthShare = bSide ? Clamp(PenRad / (Ta.UnloadedRadius + Tb.UnloadedRadius), 0.2, 1.0)
					                                : Clamp((Hw - std::fabs(La)) / std::fmin(Ta.Width, Tb.Width), 0.2, 1.0);
					const double Ma = WheelRimMass(A, I), Mb = WheelRimMass(B, J);
					const Vec3 Fa = ContactForce(Pen, N, WheelSurfaceVelocity(A, I, Pt), WheelSurfaceVelocity(B, J, Pt), P.TyreStiffness * WidthShare,
					                             P.TyreDamping * WidthShare, P.TyreFriction, Ma * Mb / (Ma + Mb) + P.TyreEngageMass, Dt, P.MaxForce);
					if (Fa.LengthSq() > 0.0)
					{
						// Chassis loads in full; the spin torques limited so the slip cannot reverse within the step
						// (the tread-engagement share of the friction goes into the chassis, not the light wheel).
						AddPair(Acc, Ia, A, -1, Ib, B, -1, Pt, Fa);
						const Vec3 Vs = WheelSurfaceVelocity(B, J, Pt) - WheelSurfaceVelocity(A, I, Pt);
						const double Slip = (Vs - N * Vs.Dot(N)).Length();
						auto SpinTorque = [&](const WheelState& W, double Mrim, double R, const Vec3& F)
						{
							const double Tq = (Pt - W.CentreWorld).Cross(F).Dot(W.AxleWorld);
							const double Max = 0.5 * Slip * Mrim * R / Dt;
							return Clamp(Tq, -Max, Max);
						};
						Acc.W[Ia * 4 + I] += SpinTorque(Wa, Ma, Ta.UnloadedRadius, Fa);
						Acc.W[Ib * 4 + J] += SpinTorque(Wb, Mb, Tb.UnloadedRadius, Fa * -1.0);
						bTouch = true;
					}
				}
			}
			// Tyres of A against B's body (and the reverse, with the roles swapped by the caller).
			KartBodyBox Boxes[12];
			const int Nb = KartBodyBoxes(CB, Boxes, 12);
			const ChassisState& Sb = B.GetChassis();
			for (int I = 0; I < NumWheels; ++I)
			{
				const WheelState& Wa = A.GetWheel(I);
				const double R = (I < 2 ? CA.FrontTyre : CA.RearTyre).UnloadedRadius;
				const Vec3 Pl = Sb.Orientation.Unrotate(Wa.CentreWorld - Sb.Position);
				// The tyre as an ellipsoid: its radius in the wheel plane, half its width along the axle.
				const Vec3 Al = Sb.Orientation.Unrotate(Wa.AxleWorld);
				const double Hw = 0.5 * (I < 2 ? CA.FrontTyre : CA.RearTyre).Width;
				const double Ks = R / Hw;
				for (int K = 0; K < Nb; ++K)
				{
					const KartBodyBox& Bx = Boxes[K];
					const Vec3 Q = Pl - Bx.Centre;
					const Vec3 Cl(Clamp(Q.X, -Bx.Half.X, Bx.Half.X), Clamp(Q.Y, -Bx.Half.Y, Bx.Half.Y), Clamp(Q.Z, -Bx.Half.Z, Bx.Half.Z));
					Vec3 Dl = Q - Cl;
					const double Dist = Dl.Length();
					const double Da = Dl.Dot(Al);
					const Vec3 Dp = Dl - Al * Da;
					const double Eff = std::sqrt(Dp.LengthSq() + Da * Da * Ks * Ks); // ellipsoid "radius" metric
					if (Eff >= R) { continue; }
					double Pen;
					Vec3 Nl; // from the box toward the tyre centre (body frame of B)
					if (Dist > 1e-6) { Nl = (Dp + Al * (Da * Ks * Ks)).Normalized(); Pen = (R - Eff) * Dist / Eff; }
					else
					{
						// Centre inside the box: push out through the nearest face.
						const double Ex = Bx.Half.X - std::fabs(Q.X), Ey = Bx.Half.Y - std::fabs(Q.Y), Ez = Bx.Half.Z - std::fabs(Q.Z);
						if (Ez <= Ex && Ez <= Ey) { Nl = Vec3(0, 0, Q.Z >= 0 ? 1.0 : -1.0); Pen = R + Ez; }
						else if (Ey <= Ex) { Nl = Vec3(0, Q.Y >= 0 ? 1.0 : -1.0, 0); Pen = R + Ey; }
						else { Nl = Vec3(Q.X >= 0 ? 1.0 : -1.0, 0, 0); Pen = R + Ex; }
					}
					const Vec3 Pt = Sb.Position + Sb.Orientation.Rotate(Bx.Centre + Cl);
					const Vec3 N = Sb.Orientation.Rotate(Nl) * -1.0; // from A (tyre) toward B (box)
					const Vec3 VB = B.GetChassisModel().PointVelocityWorld(Bx.Centre + Cl);
					const Vec3 Fa = ContactForce(std::fmin(Pen, 0.15), N, WheelSurfaceVelocity(A, I, Pt), VB, P.BodyStiffness, P.BodyDamping, P.BodyFriction,
					                             WheelRimMass(A, I), Dt, P.MaxForce);
					if (Fa.LengthSq() > 0.0) { AddPair(Acc, Ia, A, I, Ib, B, -1, Pt, Fa); bTouch = true; }
				}
			}
		}
	}

	int ApplyKartContacts(const std::vector<KartSimulation*>& Karts, const std::vector<KartFootprint>& Feet, const ContactParams& P,
	                      std::vector<KartContactInfo>* OutContacts, double Dt)
	{
		const size_t N = Karts.size();
		ContactAccum Acc(N);
		std::vector<Vec3>& F = Acc.F;
		std::vector<Vec3>& T = Acc.T;
		if (OutContacts) { OutContacts->clear(); }
		int Pairs = 0;
		for (size_t I = 0; I < N; ++I)
		{
			for (size_t J = I + 1; J < N; ++J)
			{
				bool bTouch = false;
				if (P.bWheelContacts && (Karts[I]->GetChassis().Position - Karts[J]->GetChassis().Position).LengthSq() < 9.0)
				{
					// Wheel / body contacts: forces are recorded for damage too.
					const size_t Before = 0;
					(void)Before;
					const std::vector<Vec3> F0 = {F[I], F[J]};
					WheelContacts(Acc, I, *Karts[I], J, *Karts[J], P, Dt, bTouch);
					WheelContacts(Acc, J, *Karts[J], I, *Karts[I], P, Dt, bTouch);
					if (bTouch)
					{
						const Vec3 Fi = F[I] - F0[0];
						const Vec3 Mid = (Karts[I]->GetChassis().Position + Karts[J]->GetChassis().Position) * 0.5;
						Karts[I]->AddContactForce(Mid, Fi);
						Karts[J]->AddContactForce(Mid, Fi * -1.0);
					}
				}
				bool bBox = false;
				for (int Pa = 0; Pa < 3; ++Pa)
				{
					for (int Pb = 0; Pb < 3; ++Pb)
					{
						const KartContactInfo C = BoxContact(*Karts[I], Feet[I], Pa, *Karts[J], Feet[J], Pb, P);
						if (!C.bContact) { continue; }
						bBox = true;
						F[J] += C.ForceOnB;
						T[J] += (C.Point - Karts[J]->GetChassis().Position).Cross(C.ForceOnB);
						F[I] -= C.ForceOnB;
						T[I] -= (C.Point - Karts[I]->GetChassis().Position).Cross(C.ForceOnB);
						Karts[J]->AddContactForce(C.Point, C.ForceOnB);
						Karts[I]->AddContactForce(C.Point, C.ForceOnB * -1.0);
						if (OutContacts) { OutContacts->push_back(C); }
					}
				}
				if (bBox || bTouch) { ++Pairs; }
			}
		}
		for (size_t I = 0; I < N; ++I)
		{
			Karts[I]->SetExternalLoad(F[I], T[I]);
			Karts[I]->SetWheelTorques(&Acc.W[I * 4]);
		}
		return Pairs;
	}
}
