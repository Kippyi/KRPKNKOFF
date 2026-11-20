// Track surface catalogue and the ground-query interface the simulation uses.
// The host (Unreal or a test harness) implements ITrackQuery.
#pragma once

#include "KartSim/Math.h"

#include <cstdint>

namespace kartsim
{
	enum class ESurfaceType : uint8_t
	{
		Asphalt = 0,
		RubberedAsphalt,
		DirtyAsphalt,
		WetAsphalt,
		Kerb,
		Grass,
		Gravel,
		Count
	};

	struct SurfaceProperties
	{
		const char* Name;
		double GripFactor;               // multiplies tyre mu
		double RollingResistanceFactor;  // multiplies tyre C_rr
	};

	// Grip factors are ASSUMPTIONS (Docs/PhysicsModel.md section 7).
	KARTSIM_API const SurfaceProperties& GetSurfaceProperties(ESurfaceType Type);
	KARTSIM_API const char* SurfaceName(ESurfaceType Type);

	struct GroundHit
	{
		Vec3 Point;                      // world, on the surface
		Vec3 Normal = {0.0, 0.0, 1.0};   // world, unit
		ESurfaceType Surface = ESurfaceType::Asphalt;
		double GripMultiplier = 1.0;     // dynamic local road factor (rubber build-up), 1 = nominal
		double Wetness = 0.0;            // 0 dry .. 1 saturated water film; the TYRE decides what that costs
	};

	class ITrackQuery
	{
	public:
		virtual ~ITrackQuery() = default;

		// Find the ground below Origin (searching straight down along world -Z, starting up to
		// MaxUp above Origin and ending MaxDown below it). Returns false if there is no ground.
		virtual bool QueryGround(const Vec3& Origin, double MaxUp, double MaxDown, GroundHit& Out) const = 0;
	};

	// Infinite plane, optionally inclined. Used by tests and the scenario harness.
	class KARTSIM_API PlaneTrack final : public ITrackQuery
	{
	public:
		PlaneTrack() = default;
		PlaneTrack(const Vec3& InNormal, double InHeightAtOrigin, ESurfaceType InSurface)
			: Normal(InNormal.Normalized()), HeightAtOrigin(InHeightAtOrigin), Surface(InSurface) {}

		bool QueryGround(const Vec3& Origin, double MaxUp, double MaxDown, GroundHit& Out) const override;

		Vec3 Normal = {0.0, 0.0, 1.0};
		double HeightAtOrigin = 0.0;
		ESurfaceType Surface = ESurfaceType::Asphalt;
		double GripMultiplier = 1.0;
	};

	// Flat plane with a raised kerb strip between two x positions (perpendicular to x).
	// Kerb cross-section is a half-sine bump of the given height. Used for kerb-response tests.
	class KARTSIM_API KerbStripTrack final : public ITrackQuery
	{
	public:
		bool QueryGround(const Vec3& Origin, double MaxUp, double MaxDown, GroundHit& Out) const override;

		double KerbStartX = 10.0;
		double KerbLength = 1.0;
		double KerbHeight = 0.03;
		ESurfaceType BaseSurface = ESurfaceType::Asphalt;
	};
}
