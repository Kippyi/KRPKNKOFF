#include "KartSim/Autopilot.h"

#include <cmath>

namespace kartsim
{
	void Autopilot::SetPath(const std::vector<Vec3>& Points)
	{
		Path = Points;
		const size_t M = Path.size();
		Curv.assign(M, 0.0);
		S.assign(M, 0.0);
		Length = 0.0;
		if (M < 3) { return; }
		std::vector<double> Raw(M);
		for (size_t I = 0; I < M; ++I)
		{
			const Vec3& A = Path[(I + M - 1) % M];
			const Vec3& B = Path[I];
			const Vec3& C = Path[(I + 1) % M];
			const Vec3 T0 = (B - A).Normalized(), T1 = (C - B).Normalized();
			const double Ds = 0.5 * ((B - A).Length() + (C - B).Length());
			Raw[I] = (T0.X * T1.Y - T0.Y * T1.X) / (Ds > 1e-3 ? Ds : 1e-3);
			S[I] = Length;
			Length += (C - B).Length();
		}
		for (size_t I = 0; I < M; ++I)
		{
			double Sum = 0.0;
			for (int K = -4; K <= 4; ++K) { Sum += Raw[(I + M + static_cast<size_t>(K + static_cast<int>(M))) % M]; }
			Curv[I] = Sum / 9.0;
		}
		Spacing = Length / static_cast<double>(M);
	}

	void Autopilot::Reset(const KartSimulation& Sim)
	{
		Column = 0.0;
		double Best = 1e18;
		for (size_t I = 0; I < Path.size(); ++I)
		{
			const double D = (Path[I] - Sim.GetChassis().Position).LengthSq();
			if (D < Best) { Best = D; Idx = I; }
		}
	}

	void Autopilot::Drive(const KartSimulation& Sim, double Dt, KartInputs& In)
	{
		const size_t M = Path.size();
		if (M < 3) { return; }
		const KartConfig& C = Sim.GetConfig();
		const ChassisState& St = Sim.GetChassis();
		const Vec3 Fwd = St.Orientation.Rotate(Vec3(1, 0, 0));
		const Vec3 Left = St.Orientation.Rotate(Vec3(0, 1, 0));
		const double V = Sim.ForwardSpeed();

		// Nearest centre-line point (forward search window).
		double Best = 1e18;
		size_t BestI = Idx;
		const size_t Window = static_cast<size_t>(45.0 / Spacing) + 1;
		for (size_t K = 0; K < Window; ++K)
		{
			const size_t J = (Idx + K) % M;
			Vec3 D = Path[J] - St.Position;
			D.Z = 0.0;
			if (D.LengthSq() < Best) { Best = D.LengthSq(); BestI = J; }
		}
		Idx = BestI;
		CrossTrack = std::sqrt(Best);

		// Pure pursuit + yaw-rate feedback (adds steering when the kart rotates less than intended).
		const double Ld = Clamp(3.0 + 0.35 * V, 4.0, 14.0);
		const size_t TgtI = (Idx + static_cast<size_t>(Ld / Spacing)) % M;
		Vec3 Tgt = Path[TgtI] - St.Position;
		if (LateralOffset != 0.0) { Tgt += PathNormal(TgtI) * LateralOffset; }
		Tgt.Z = 0.0;
		const double Alpha = std::atan2(Tgt.Dot(Left), Tgt.Dot(Fwd));
		const double KappaCmd = 2.0 * std::sin(Alpha) / Ld;
		const double Delta = std::atan(KappaCmd * C.Chassis.Wheelbase) + 0.35 * (V * KappaCmd - St.AngularVelocity.Z);
		const double ColTarget = Clamp(Delta * C.Steering.Ratio, -C.Steering.MaxColumnAngle, C.Steering.MaxColumnAngle);
		const double MaxRate = SteerRateDegPerSec * kDegToRad * Dt;
		Column += Clamp(ColTarget - Column, -MaxRate, MaxRate);

		// Speed profile: corner speed from TargetLatG, braking envelope ~80 m ahead.
		const double G = kGravity;
		double VTarget = 40.0;
		const size_t Ahead = static_cast<size_t>(82.0 / Spacing);
		for (size_t K = 0; K < Ahead; ++K)
		{
			const size_t J = (Idx + K) % M;
			// Curvature of the path actually driven: offset o to the left of a path with curvature k
			// (+ = left turn) has k / (1 - k o) (identical to k when o = 0).
			const double Den = 1.0 - Curv[J] * LateralOffset;
			const double Kc = std::fabs(Den > 0.2 ? Curv[J] / Den : Curv[J] / 0.2);
			const double VCorner = Kc > 1e-4 ? std::sqrt(TargetLatG * G / Kc) : 40.0;
			const double VAllowed = std::sqrt(VCorner * VCorner + 2.0 * BrakeDecelG * G * K * Spacing);
			VTarget = VAllowed < VTarget ? VAllowed : VTarget;
		}
		VTarget = VTarget < SpeedCap ? VTarget : SpeedCap;
		const double E = VTarget - V;
		In.Throttle = E > 0.0 ? Clamp(0.3 + 0.6 * E, 0.0, 1.0) : 0.0;
		In.Brake = E < -0.4 ? Clamp(-0.12 * E, 0.0, MaxBrakePedal) : 0.0;
		In.SteeringColumnAngle = Column;
	}

	Vec3 Autopilot::PathNormal(size_t I) const
	{
		const size_t M = Path.size();
		Vec3 T = Path[(I + 1) % M] - Path[(I + M - 1) % M];
		T.Z = 0.0;
		T = T.Normalized();
		return Vec3(-T.Y, T.X, 0.0);
	}
}
