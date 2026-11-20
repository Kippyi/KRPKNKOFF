// The ONLY place where simulation space (SI metres, right-handed ISO 8855: x fwd, y left, z up)
// is converted to Unreal space (centimetres, left-handed: x fwd, y right, z up).
// Mirroring y: p_ue = 100 * (x, -y, z); a rotation R maps to S R S, i.e. quaternion (w, -x, y, -z).
#pragma once

#include "CoreMinimal.h"
#include "KartSim/Math.h"

namespace KartConv
{
	constexpr double MetresToCm = 100.0;

	inline FVector ToUEPos(const kartsim::Vec3& P) { return FVector(P.X * MetresToCm, -P.Y * MetresToCm, P.Z * MetresToCm); }
	inline FVector ToUEDir(const kartsim::Vec3& D) { return FVector(D.X, -D.Y, D.Z); }
	inline kartsim::Vec3 FromUEPos(const FVector& P) { return kartsim::Vec3(P.X / MetresToCm, -P.Y / MetresToCm, P.Z / MetresToCm); }
	inline kartsim::Vec3 FromUEDir(const FVector& D) { return kartsim::Vec3(D.X, -D.Y, D.Z); }

	inline FQuat ToUEQuat(const kartsim::Quat& Q) { return FQuat(-Q.X, Q.Y, -Q.Z, Q.W); }
	inline kartsim::Quat FromUEQuat(const FQuat& Q) { return kartsim::Quat(Q.W, -Q.X, Q.Y, -Q.Z); }

	// Sim yaw (counter-clockwise from above, radians) from a UE yaw in degrees (clockwise from above).
	inline double YawFromUE(double UEYawDeg) { return -UEYawDeg * kartsim::kDegToRad; }
}
