// FKartLapAnalysis: per-kart live lap delta and distance-based speed traces (kartsim::LapDelta) fed
// from the pawn's physics step. Sessions and classification live in AKartRaceWorld (Milestone 6).
#pragma once

#include "CoreMinimal.h"
#include "KartSim/LapDelta.h"
#include "KartSim/TrackPath.h"

namespace kartsim { class KartSimulation; class LapTimer; }

class KRPKNKOFF_API FKartLapAnalysis
{
public:
	// Centre line in UE world coordinates (cm).
	void Configure(const TArray<FVector>& CentreLineUE);
	// Every physics step, after LapTimer::Update.
	void Step(double Now, const kartsim::LapTimer& Timer, const kartsim::KartSimulation& Sim);
	const kartsim::LapDelta& GetDelta() const { return Delta; }

private:
	kartsim::TrackPath Path;
	kartsim::LapDelta Delta;
	bool bWasTiming = false;
	size_t SeenLaps = 0;
};
