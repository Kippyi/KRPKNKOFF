#include "KartSim/RacingLine.h"

#include <cmath>

namespace kartsim
{
	RacingLine ComputeRacingLine(const std::vector<Vec3>& Centre, double HalfWidth, double Margin, int Sweeps)
	{
		RacingLine R;
		const size_t M = Centre.size();
		if (M < 5) { R.Points = Centre; R.Offset.assign(M, 0.0); R.RoomLeft.assign(M, 0.0); R.RoomRight.assign(M, 0.0); return R; }
		const double W = HalfWidth - Margin > 0.0 ? HalfWidth - Margin : 0.0;
		std::vector<Vec3> N(M);
		for (size_t I = 0; I < M; ++I)
		{
			Vec3 T = Centre[(I + 1) % M] - Centre[(I + M - 1) % M];
			T.Z = 0.0;
			T = T.Normalized();
			N[I] = Vec3(-T.Y, T.X, 0.0);
		}
		std::vector<double> O(M, 0.0);
		auto P = [&](size_t I) { const size_t J = I % M; return Centre[J] + N[J] * O[J]; };
		// Gauss-Seidel on min sum |p[i-h] - 2 p[i] + p[i+h]|^2 with box constraints: the unconstrained
		// optimum for p[i] given its neighbours is (4 (p[i-h] + p[i+h]) - (p[i-2h] + p[i+2h])) / 6, then
		// projected onto the point's normal line and clamped to the track. Plain sweeps converge as
		// (spacing / corner length)^4, hopeless at 0.75 m spacing, so it runs coarse to fine: stride h
		// from 32 points down to 1, linearly interpolating the offsets between the stride points.
		const int SweepsPerLevel = Sweeps / 6 > 50 ? Sweeps / 6 : 50;
		for (size_t H = 32; H >= 1; H /= 2)
		{
			if (H * 6 > M) { continue; }
			for (int Sweep = 0; Sweep < SweepsPerLevel; ++Sweep)
			{
				for (size_t I = 0; I < M; I += H)
				{
					const Vec3 Q = (P(I + M - H) + P(I + H)) * (4.0 / 6.0) - (P(I + M - 2 * H) + P(I + 2 * H)) * (1.0 / 6.0);
					O[I] = Clamp((Q - Centre[I]).Dot(N[I]), -W, W);
				}
			}
			if (H > 1)
			{
				for (size_t I = 0; I < M; I += H)
				{
					const size_t J = (I + H < M) ? I + H : 0; // last block wraps to point 0
					const size_t End = (I + H < M) ? I + H : M;
					for (size_t K = I + 1; K < End; ++K)
					{
						const double F = static_cast<double>(K - I) / static_cast<double>(End - I);
						O[K] = O[I] + (O[J] - O[I]) * F;
					}
				}
			}
		}
		R.Points.resize(M);
		R.Offset = O;
		R.RoomLeft.resize(M);
		R.RoomRight.resize(M);
		for (size_t I = 0; I < M; ++I)
		{
			R.Points[I] = P(I);
			R.Points[I].Z = Centre[I].Z;
			R.RoomLeft[I] = W - O[I];
			R.RoomRight[I] = W + O[I];
		}
		return R;
	}
}
