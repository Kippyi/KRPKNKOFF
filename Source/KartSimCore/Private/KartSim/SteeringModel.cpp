#include "KartSim/SteeringModel.h"

#include <cmath>

namespace kartsim
{
	void SteeringModel::Init(const KartConfig& InConfig, const Vec3 FrontCentres[2])
	{
		Config = &InConfig;
		const SteeringParams& S = InConfig.Steering;
		for (int Side = 0; Side < 2; ++Side)
		{
			const double SideSign = Side == 0 ? 1.0 : -1.0; // left wheel sits at +y
			KingpinGeometry& G = Geo[Side];
			G.CentreStatic = FrontCentres[Side];
			G.Pivot = FrontCentres[Side] - Vec3(0.0, SideSign * S.SpindleOffset, 0.0);
			// Caster tilts the top of the kingpin rearward; KPI tilts it inboard.
			G.Axis = Vec3(-std::tan(S.Caster), -SideSign * std::tan(S.KingpinInclination), 1.0).Normalized();
			// Static camber (negative = top inward) rotates the axle about the forward axis.
			const double Theta = -SideSign * S.StaticCamber;
			G.AxleStatic = Vec3(0.0, std::cos(Theta), std::sin(Theta));
		}
		KingpinSpacing = (Geo[0].Pivot - Geo[1].Pivot).Length();
	}

	void SteeringModel::ComputeAngles(double ColumnAngle, double& OutLeft, double& OutRight) const
	{
		const SteeringParams& S = Config->Steering;
		const double Column = Clamp(ColumnAngle, -S.MaxColumnAngle, S.MaxColumnAngle);
		const double Ref = Column / S.Ratio;
		double Inner = Ref, Outer = Ref;
		const double AbsRef = std::fabs(Ref);
		if (AbsRef > 1e-6)
		{
			const double L = Config->Chassis.Wheelbase;
			const double R = L / std::tan(AbsRef);
			const double IdealInner = std::atan(L / (R - 0.5 * KingpinSpacing));
			const double IdealOuter = std::atan(L / (R + 0.5 * KingpinSpacing));
			const double A = S.AckermannFactor;
			Inner = Sign(Ref) * (AbsRef + A * (IdealInner - AbsRef));
			Outer = Sign(Ref) * (AbsRef + A * (IdealOuter - AbsRef));
		}
		// Turning left (Ref > 0): left wheel is the inner wheel.
		OutLeft = (Ref >= 0.0 ? Inner : Outer) - S.StaticToe - DamageToe[0];
		OutRight = (Ref >= 0.0 ? Outer : Inner) + S.StaticToe + DamageToe[1];
	}

	void SteeringModel::WheelPose(int Side, double Delta, Vec3& OutCentre, Vec3& OutAxle) const
	{
		const KingpinGeometry& G = Geo[Side];
		OutCentre = G.Pivot + RotateAboutAxis(G.CentreStatic - G.Pivot, G.Axis, Delta);
		OutAxle = RotateAboutAxis(G.AxleStatic, G.Axis, Delta).Normalized();
	}

	double SteeringModel::KingpinMoment(int Side, const Vec3& ContactPoint, const Vec3& Force, double Mz, const Vec3& Normal) const
	{
		const KingpinGeometry& G = Geo[Side];
		return (ContactPoint - G.Pivot).Cross(Force).Dot(G.Axis) + Mz * Normal.Dot(G.Axis);
	}

	double SteeringModel::ColumnTorque(double KingpinMomentLeft, double KingpinMomentRight, double ColumnAngle) const
	{
		// Virtual work: column torque = sum(M_k * d(delta)/d(column)). Derivatives by central difference.
		const double H = 1e-4;
		double L1, R1, L2, R2;
		ComputeAngles(ColumnAngle + H, L1, R1);
		ComputeAngles(ColumnAngle - H, L2, R2);
		const double DL = (L1 - L2) / (2.0 * H);
		const double DR = (R1 - R2) / (2.0 * H);
		return KingpinMomentLeft * DL + KingpinMomentRight * DR;
	}
}
