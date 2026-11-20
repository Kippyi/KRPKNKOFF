#include "KartSim/TrackPath.h"

#include <cmath>

namespace kartsim
{
	void TrackPath::Build(const std::vector<Vec3>& Points)
	{
		P = Points;
		const size_t M = P.size();
		S.assign(M, 0.0);
		Total = 0.0;
		for (size_t I = 0; I < M; ++I)
		{
			S[I] = Total;
			Vec3 D = P[(I + 1) % M] - P[I];
			D.Z = 0.0;
			Total += D.Length();
		}
	}

	bool TrackPath::Project(const Vec3& Pos, double& OutS, double& OutLateral, size_t& Hint) const
	{
		const size_t M = P.size();
		if (M < 3) { return false; }
		auto Dist2 = [&](size_t I) { const double Dx = P[I].X - Pos.X, Dy = P[I].Y - Pos.Y; return Dx * Dx + Dy * Dy; };
		size_t Best = Hint % M;
		double BestD = Dist2(Best);
		for (size_t K = 1; K <= 40; ++K)
		{
			const size_t Fwd = (Hint + K) % M, Back = (Hint + M - (K % M)) % M;
			const double Df = Dist2(Fwd), Db = Dist2(Back);
			if (Df < BestD) { BestD = Df; Best = Fwd; }
			if (Db < BestD) { BestD = Db; Best = Back; }
		}
		if (BestD > 15.0 * 15.0)
		{
			for (size_t J = 0; J < M; ++J) { const double D = Dist2(J); if (D < BestD) { BestD = D; Best = J; } }
		}
		Hint = Best;
		// Project onto whichever adjacent segment contains the point.
		size_t A = Best;
		{
			const Vec3 Next = P[(Best + 1) % M] - P[Best];
			const Vec3 Rel = Pos - P[Best];
			if (Rel.X * Next.X + Rel.Y * Next.Y < 0.0) { A = (Best + M - 1) % M; }
		}
		Vec3 T = P[(A + 1) % M] - P[A];
		T.Z = 0.0;
		const double Seg = T.Length();
		if (Seg < 1e-9) { OutS = S[A]; OutLateral = 0.0; return true; }
		T = T / Seg;
		Vec3 D = Pos - P[A];
		D.Z = 0.0;
		double Along = S[A] + D.Dot(T);
		Along = std::fmod(Along, Total);
		if (Along < 0.0) { Along += Total; }
		OutS = Along;
		OutLateral = D.X * -T.Y + D.Y * T.X;
		return true;
	}
}
