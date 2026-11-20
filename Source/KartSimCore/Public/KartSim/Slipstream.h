// Slipstream (Milestone 6): a kart close behind another and roughly in line sits in its wake and has
// less aerodynamic drag. Returned as a drag multiplier for KartSimulation::Settings.DragScale.
//
// Magnitudes are ASSUMPTIONS (no kart wake measurements): up to 45 % less drag right behind the kart
// ahead (gap ~0), decaying with a 4 m length scale and fading linearly to nothing at 1.0 m lateral
// offset; no effect beyond 15 m. Karting drafting is known to be strong (bluff bodies, open driver), but
// the numbers need validation against straight-line speed data in a draft.
#pragma once

#include "KartSim/KartPhysicsCore.h"

#include <cmath>
#include <vector>

namespace kartsim
{
	struct SlipstreamParams
	{
		double MaxReduction = 0.45;
		double DecayLength = 4.0;     // m
		double MaxLateral = 1.0;      // m
		double MaxGap = 15.0;         // m
		double KartLength = 1.82;     // m (CIK maximum)
	};

	inline double DraftDragScale(const KartSimulation& Self, const std::vector<KartSimulation*>& All, const SlipstreamParams& P = SlipstreamParams())
	{
		const ChassisState& S = Self.GetChassis();
		Vec3 Dir = S.Velocity;
		Dir.Z = 0.0;
		const double V = Dir.Length();
		if (V < 5.0) { return 1.0; }
		Dir = Dir / V;
		double Best = 0.0;
		for (const KartSimulation* O : All)
		{
			if (O == &Self) { continue; }
			Vec3 D = O->GetChassis().Position - S.Position;
			D.Z = 0.0;
			const double Along = D.Dot(Dir);
			const double Gap = Along - P.KartLength;
			if (Along <= 0.0 || Gap > P.MaxGap) { continue; }
			const double Lateral = std::fabs(D.X * -Dir.Y + D.Y * Dir.X);
			if (Lateral >= P.MaxLateral) { continue; }
			const double R = P.MaxReduction * std::exp(-(Gap > 0.0 ? Gap : 0.0) / P.DecayLength) * (1.0 - Lateral / P.MaxLateral);
			Best = R > Best ? R : Best;
		}
		return 1.0 - Best;
	}
}
