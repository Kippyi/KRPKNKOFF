// Closed track centre line with arc length: projects a world point to (distance along, lateral
// offset). Used by the track-conditions grid and the live lap delta. Engine-independent.
#pragma once

#include "KartSim/Math.h"

#include <cstddef>
#include <vector>

namespace kartsim
{
	class KARTSIM_API TrackPath
	{
	public:
		void Build(const std::vector<Vec3>& Points);
		bool IsBuilt() const { return P.size() >= 3; }
		double Length() const { return Total; }
		size_t NumPoints() const { return P.size(); }
		const Vec3& Point(size_t I) const { return P[I]; }

		// Distance along [0, Length) and lateral offset (+ = left of the driving direction). Uses a local
		// search around Hint (updated) and falls back to a full search when the point is far from it.
		// Returns false only if the path is not built.
		bool Project(const Vec3& Pos, double& OutS, double& OutLateral, size_t& Hint) const;

	private:
		std::vector<Vec3> P;
		std::vector<double> S;
		double Total = 0.0;
	};
}
