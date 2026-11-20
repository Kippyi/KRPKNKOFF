// Centre line of the Milestone-1 test circuit, generated exactly like AKartTestTrackActor
// (same control points, closed Catmull-Rom, ~0.75 m spacing). Units: metres, UE plane coords
// converted to sim coords (y mirrored). Used by the closed-loop driver tests.
#pragma once

#include "KartSim/Math.h"

#include <cmath>
#include <vector>

namespace kstest
{
	struct TrackLine
	{
		std::vector<kartsim::Vec3> P;   // sim coordinates, z = 0
		std::vector<double> Curv;       // signed curvature (1/m), + = left turn in sim frame
		std::vector<double> S;          // arc length
		double Length = 0.0;
	};

	inline TrackLine BuildTestTrack()
	{
		const double Ctl[][2] = {{0, 0}, {60, 0}, {95, 8}, {110, 30}, {105, 55}, {80, 65}, {55, 58}, {40, 40},
		                         {20, 35}, {0, 45}, {-15, 70}, {-40, 78}, {-65, 65}, {-70, 40}, {-55, 20}, {-30, 5}};
		const int N = static_cast<int>(sizeof(Ctl) / sizeof(Ctl[0]));
		auto At = [&](int I, int C) { return Ctl[(I % N + N) % N][C]; };
		std::vector<kartsim::Vec3> Dense;
		for (int I = 0; I < N; ++I)
		{
			for (int K = 0; K < 200; ++K)
			{
				const double T = K / 200.0, T2 = T * T, T3 = T2 * T;
				double Out[2];
				for (int C = 0; C < 2; ++C)
				{
					const double P0 = At(I - 1, C), P1 = At(I, C), P2 = At(I + 1, C), P3 = At(I + 2, C);
					Out[C] = 0.5 * (2 * P1 + (-P0 + P2) * T + (2 * P0 - 5 * P1 + 4 * P2 - P3) * T2 + (-P0 + 3 * P1 - 3 * P2 + P3) * T3);
				}
				Dense.push_back(kartsim::Vec3(Out[0], -Out[1], 0.0)); // UE y -> sim -y
			}
		}
		TrackLine L;
		L.P.push_back(Dense[0]);
		double Acc = 0.0;
		for (size_t I = 1; I < Dense.size(); ++I)
		{
			const double D = (Dense[I] - Dense[I - 1]).Length();
			Acc += D;
			L.Length += D;
			if (Acc >= 0.75) { L.P.push_back(Dense[I]); Acc = 0.0; }
		}
		L.Length += (Dense.front() - Dense.back()).Length();
		const size_t M = L.P.size();
		L.Curv.resize(M);
		L.S.resize(M);
		double S = 0.0;
		for (size_t I = 0; I < M; ++I)
		{
			const kartsim::Vec3& A = L.P[(I + M - 1) % M];
			const kartsim::Vec3& B = L.P[I];
			const kartsim::Vec3& C = L.P[(I + 1) % M];
			const kartsim::Vec3 T0 = (B - A).Normalized(), T1 = (C - B).Normalized();
			const double Ds = 0.5 * ((B - A).Length() + (C - B).Length());
			L.Curv[I] = (T0.X * T1.Y - T0.Y * T1.X) / (Ds > 1e-3 ? Ds : 1e-3);
			L.S[I] = S;
			S += (C - B).Length();
		}
		// Smooth curvature (spline sampling noise).
		std::vector<double> Sm(M);
		for (size_t I = 0; I < M; ++I)
		{
			double Sum = 0.0;
			for (int K = -4; K <= 4; ++K) { Sum += L.Curv[(I + M + K) % M]; }
			Sm[I] = Sum / 9.0;
		}
		L.Curv = Sm;
		return L;
	}
}
