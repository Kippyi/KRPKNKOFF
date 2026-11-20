// Racing line (Milestone 6): minimum-curvature path inside the track edges. The line is the centre line
// plus a lateral offset per point; offsets are relaxed by Gauss-Seidel on the discrete curvature
// (biharmonic stencil) and clamped to +-(HalfWidth - Margin). Minimum curvature is the usual racing-AI
// baseline: it is close to, but not exactly, the minimum-time line (which also weighs path length and
// the kart's acceleration limits). Engine-independent.
#pragma once

#include "KartSim/Math.h"

#include <vector>

namespace kartsim
{
	struct RacingLine
	{
		std::vector<Vec3> Points;         // the line
		std::vector<double> Offset;       // lateral offset from the centre line (+ = left)
		std::vector<double> RoomLeft;     // how far the kart CoG may move left of the line (m)
		std::vector<double> RoomRight;    // ... and right (m, positive)
	};

	// Centre: closed loop of evenly spaced points. HalfWidth: half the paved width. Margin: distance
	// kept between the kart CoG and the edge (half the kart width plus clearance).
	KARTSIM_API RacingLine ComputeRacingLine(const std::vector<Vec3>& Centre, double HalfWidth, double Margin, int Sweeps = 6000);
}
