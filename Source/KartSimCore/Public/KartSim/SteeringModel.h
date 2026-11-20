// Steering: column -> road-wheel angles (Ackermann blend, toe), front wheel pose from rotation
// about the inclined kingpin axis (caster, KPI, spindle offset) and kingpin / column torque.
// All vectors are in the kart body frame. Docs/PhysicsModel.md section 4.
#pragma once

#include "KartSim/KartConfig.h"

namespace kartsim
{
	enum EWheel { WheelFL = 0, WheelFR = 1, WheelRL = 2, WheelRR = 3, NumWheels = 4 };

	struct KingpinGeometry
	{
		Vec3 Pivot;        // body frame, on the kingpin axis at wheel-centre height
		Vec3 Axis;         // body frame, unit, pointing up
		Vec3 CentreStatic; // wheel centre at zero steer
		Vec3 AxleStatic;   // wheel axle at zero steer (points left), includes static camber
	};

	class KARTSIM_API SteeringModel
	{
	public:
		// FrontCentres: body-frame wheel centres at zero steer, [0] = left, [1] = right.
		void Init(const KartConfig& InConfig, const Vec3 FrontCentres[2]);

		// Rotation angles about the kingpin axes for a column angle (positive = steer left).
		void ComputeAngles(double ColumnAngle, double& OutLeft, double& OutRight) const;

		// Wheel pose (body frame) after rotating by Delta about the kingpin. Side: 0 = left, 1 = right.
		void WheelPose(int Side, double Delta, Vec3& OutCentre, Vec3& OutAxle) const;

		// Moment of a tyre contact force about the kingpin axis (body frame inputs).
		double KingpinMoment(int Side, const Vec3& ContactPoint, const Vec3& Force, double Mz, const Vec3& Normal) const;

		// Torque the tyres apply to the steering column [Nm], positive = turning the column left.
		double ColumnTorque(double KingpinMomentLeft, double KingpinMomentRight, double ColumnAngle) const;

		const KingpinGeometry& GetGeometry(int Side) const { return Geo[Side]; }

		// Damage: extra toe-in [rad] of one side (bent steering arm / track rod); negative = toe-out.
		void SetDamageToe(int Side, double ToeRad) { DamageToe[Side] = ToeRad; }
		double GetDamageToe(int Side) const { return DamageToe[Side]; }

	private:
		const KartConfig* Config = nullptr;
		KingpinGeometry Geo[2];
		double KingpinSpacing = 0.9;
		double DamageToe[2] = {0.0, 0.0};
	};
}
