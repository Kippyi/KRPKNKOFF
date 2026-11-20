#include "KartSim/TrackSurface.h"

namespace kartsim
{
	namespace
	{
		// ASSUMPTION values; see Docs/PhysicsModel.md section 7.
		const SurfaceProperties GSurfaceTable[static_cast<int>(ESurfaceType::Count)] = {
			{"Asphalt",         1.00, 1.0},
			{"RubberedAsphalt", 1.08, 1.0},
			{"DirtyAsphalt",    0.85, 1.1},
			{"WetAsphalt",      1.00, 1.0}, // wetness 1: grip loss is the tyre compound's wet.gripFloor
			{"Kerb",            0.90, 1.0},
			{"Grass",           0.45, 4.0},
			{"Gravel",          0.35, 12.0},
		};
	}

	const SurfaceProperties& GetSurfaceProperties(ESurfaceType Type)
	{
		const int Index = static_cast<int>(Type);
		return GSurfaceTable[(Index >= 0 && Index < static_cast<int>(ESurfaceType::Count)) ? Index : 0];
	}

	const char* SurfaceName(ESurfaceType Type)
	{
		return GetSurfaceProperties(Type).Name;
	}

	bool PlaneTrack::QueryGround(const Vec3& Origin, double MaxUp, double MaxDown, GroundHit& Out) const
	{
		// Plane: n . p = HeightAtOrigin * n.z. Intersect vertical line through Origin.
		if (Normal.Z < 1e-6)
		{
			return false;
		}
		const double D = HeightAtOrigin * Normal.Z;
		const double Z = (D - Normal.X * Origin.X - Normal.Y * Origin.Y) / Normal.Z;
		if (Z > Origin.Z + MaxUp || Z < Origin.Z - MaxDown)
		{
			return false;
		}
		Out.Point = Vec3(Origin.X, Origin.Y, Z);
		Out.Normal = Normal;
		Out.Surface = Surface;
		Out.GripMultiplier = GripMultiplier;
		return true;
	}

	bool KerbStripTrack::QueryGround(const Vec3& Origin, double MaxUp, double MaxDown, GroundHit& Out) const
	{
		double Z = 0.0;
		Vec3 N(0.0, 0.0, 1.0);
		ESurfaceType S = BaseSurface;
		const double U = (Origin.X - KerbStartX) / KerbLength;
		if (U > 0.0 && U < 1.0)
		{
			Z = KerbHeight * std::sin(kPi * U);
			const double Slope = KerbHeight * kPi / KerbLength * std::cos(kPi * U);
			N = Vec3(-Slope, 0.0, 1.0).Normalized();
			S = ESurfaceType::Kerb;
		}
		if (Z > Origin.Z + MaxUp || Z < Origin.Z - MaxDown)
		{
			return false;
		}
		Out.Point = Vec3(Origin.X, Origin.Y, Z);
		Out.Normal = N;
		Out.Surface = S;
		Out.GripMultiplier = 1.0;
		return true;
	}
}
