// AKartConditionsActor (Milestone 5): owns the world's dynamic track conditions (kartsim::TrackConditions:
// rubber + water grid, weather) and their visual response. Karts query grip through
// kartsim::ConditionsTrackQuery and deposit rubber / squeeze water via ApplyKart every physics step.
//
// Presets (scenario choices, the numbers are ASSUMPTIONS - see Docs/Milestone5Conditions.md):
//   weather  -KartWeather=Dry|Hot|Overcast|Rain|Drying|Shower  (key N cycles)
//   track    -KartTrack=Green|Rubbered                  (key M toggles)
//   -KartWeatherTimeScale=<x>  speeds up weather/drying evolution (default 1 = real time)
//   -KartConditions=0          disables the system (grip multiplier exactly 1; physics regression runs)
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "KartSim/TrackConditions.h"
#include "KartConditionsActor.generated.h"

UCLASS()
class KRPKNKOFF_API AKartConditionsActor : public AActor
{
	GENERATED_BODY()

public:
	AKartConditionsActor();
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;

	static AKartConditionsActor* Find(const UWorld* World);

	bool IsActive() const { return bEnabled && Conditions.IsBuilt(); }
	kartsim::TrackConditions& Get() { return Conditions; }
	const kartsim::TrackConditions& Get() const { return Conditions; }

	void CycleWeather();
	void ToggleRubber();
	const FString& GetWeatherName() const { return WeatherName; }
	const FString& GetTrackName() const { return TrackName; }
	int32 GetWeatherIndex() const { return WeatherIndex; }
	bool IsRubbered() const { return bRubbered; }
	// Multiplayer clients follow the host's presets (each machine then evolves the conditions locally).
	void ApplyPresets(int32 InWeather, bool bInRubbered);

private:
	void ApplyWeatherPreset(int32 Index);
	void ApplyTrackPreset(bool bRubbered);
	void UpdateVisuals();

	kartsim::TrackConditions Conditions;
	bool bEnabled = true;
	int32 WeatherIndex = 0;
	bool bRubbered = false;
	FString WeatherName;
	FString TrackName;
	float TimeScale = 1.0f;
	float VisualTimer = 0.0f;
	float BaseSunIntensity = -1.0f;
	float BaseFogDensity = -1.0f;
};
