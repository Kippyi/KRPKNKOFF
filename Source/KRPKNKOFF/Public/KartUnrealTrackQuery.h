// ITrackQuery implemented with Unreal line traces. Surface type: component tag "KartSurface.<Name>"
// (e.g. KartSurface.Grass), else a physical material named "PM_Kart_<Name>" (e.g. PM_Kart_Grass,
// the convention for imported track art), else Asphalt.
#pragma once

#include "CoreMinimal.h"
#include "KartSim/TrackSurface.h"

class UWorld;
class AActor;

class KRPKNKOFF_API FKartUnrealTrackQuery final : public kartsim::ITrackQuery
{
public:
	FKartUnrealTrackQuery() = default;
	void Setup(UWorld* InWorld, const AActor* InIgnoreActor);

	virtual bool QueryGround(const kartsim::Vec3& Origin, double MaxUp, double MaxDown, kartsim::GroundHit& Out) const override;

	static bool SurfaceFromTags(const TArray<FName>& Tags, kartsim::ESurfaceType& Out);
	static bool SurfaceFromName(const FString& Text, const TCHAR* Prefix, kartsim::ESurfaceType& Out);

	// Number of traces issued since the last reset (for profiling).
	mutable int32 TraceCount = 0;

private:
	TWeakObjectPtr<UWorld> World;
	TWeakObjectPtr<const AActor> IgnoreActor;
};
