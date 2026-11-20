#include "KartConditionsActor.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMeshActor.h"

#include "Components/DirectionalLightComponent.h"
#include "Components/ExponentialHeightFogComponent.h"
#include "Engine/DirectionalLight.h"
#include "Engine/ExponentialHeightFog.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "KartGameInstance.h"
#include "KartTestTrackActor.h"
#include "KartUnrealConversions.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

DEFINE_LOG_CATEGORY_STATIC(LogKartConditions, Log, All);

namespace
{
	// Weather scenarios. Air temperature / cloud / rain are scenario inputs; the starting track
	// temperature is the model's equilibrium for them, and the starting water depth matches the scenario.
	// Pressure (kPa, station) and relative humidity set the air density: engine power and the right main jet.
	struct FWeatherPreset { const TCHAR* Name; double Air, Cloud, Rain, StartWater, Pressure, Humidity; };
	const FWeatherPreset GWeather[] = {
		{TEXT("Dry"), 22.0, 0.3, 0.0, 0.0, 101.325, 0.50},
		{TEXT("Hot"), 32.0, 0.0, 0.0, 0.0, 100.8, 0.40},
		{TEXT("Overcast"), 12.0, 1.0, 0.0, 0.0, 101.6, 0.75},
		{TEXT("Rain"), 15.0, 1.0, 6.0, 1.0, 100.6, 0.95},
		{TEXT("Drying"), 18.0, 0.4, 0.0, 0.6, 101.1, 0.80},
		{TEXT("Shower"), 17.0, 0.6, 0.0, 0.0, 101.0, 0.70}, // changing weather: GShowerSchedule
		{TEXT("Cold"), 6.0, 0.1, 0.0, 0.0, 102.4, 0.60},    // winter morning: dense air (bigger jet, tape the radiator)
	};
	// Shower scenario (minutes of the conditions clock; -KartWeatherTimeScale speeds it up): dry and clouding over
	// for 6 min, rain 8 mm/h from 7 to 15 min, then it clears and the line dries. A scenario, not a forecast model.
	const std::vector<kartsim::WeatherKey> GShowerSchedule = {
		{0.0, 0.0, 0.6}, {360.0, 0.0, 1.0}, {420.0, 8.0, 1.0}, {900.0, 8.0, 1.0}, {960.0, 0.0, 0.8}, {1500.0, 0.0, 0.3}};
	constexpr int32 GNumWeather = UE_ARRAY_COUNT(GWeather);
	// Rubbered line: level and half-width of the band (the shared Autopilot drives the centre line).
	constexpr double GRubberLevel = 0.7;
	constexpr double GRubberHalfWidth = 1.0;
}

AKartConditionsActor::AKartConditionsActor()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PrePhysics;
}

AKartConditionsActor* AKartConditionsActor::Find(const UWorld* World)
{
	if (!World) { return nullptr; }
	for (TActorIterator<AKartConditionsActor> It(World); It; ++It) { return *It; }
	return nullptr;
}

void AKartConditionsActor::BeginPlay()
{
	Super::BeginPlay();
	int32 Enabled = 1;
	FParse::Value(FCommandLine::Get(), TEXT("KartConditions="), Enabled);
	bEnabled = Enabled != 0;
	FParse::Value(FCommandLine::Get(), TEXT("KartWeatherTimeScale="), TimeScale);
	TimeScale = FMath::Clamp(TimeScale, 0.0f, 600.0f);

	AKartTestTrackActor::OnLayoutChanged.AddWeakLambda(this, [this]()
	{
		// New layout (client following the host): rebuild the grid on the new centre line, same presets.
		for (TActorIterator<AKartTestTrackActor> It(GetWorld()); It; ++It)
		{
			TArray<FVector> C;
			It->GetCentreLine(C);
			std::vector<kartsim::Vec3> P;
			for (const FVector& V : C) { P.push_back(KartConv::FromUEPos(V)); }
			Conditions.Build(P);
			break;
		}
		const int32 W = WeatherIndex;
		WeatherIndex = -1;
		ApplyPresets(W, bRubbered);
	});
	// The grid follows the track centre line.
	for (TActorIterator<AKartTestTrackActor> It(GetWorld()); It; ++It)
	{
		TArray<FVector> Centre;
		It->GetCentreLine(Centre);
		std::vector<kartsim::Vec3> Path;
		Path.reserve(Centre.Num());
		for (const FVector& P : Centre) { Path.push_back(KartConv::FromUEPos(P)); }
		Conditions.Build(Path);
		break;
	}
	if (!Conditions.IsBuilt())
	{
		UE_LOG(LogKartConditions, Warning, TEXT("No track centre line: track conditions inactive"));
		return;
	}

	FString Weather, Track;
	int32 WIndex = 0;
	if (FParse::Value(FCommandLine::Get(), TEXT("KartWeather="), Weather))
	{
		for (int32 I = 0; I < GNumWeather; ++I) { if (Weather.Equals(GWeather[I].Name, ESearchCase::IgnoreCase)) { WIndex = I; } }
	}
	bool bRubberStart = FParse::Value(FCommandLine::Get(), TEXT("KartTrack="), Track) && Track.Equals(TEXT("Rubbered"), ESearchCase::IgnoreCase);
	if (const UKartGameInstance* GI = UKartGameInstance::Get(this); GI && GI->bLaunchedFromMenu && Weather.IsEmpty())
	{
		WIndex = FMath::Clamp(GI->Launch.Weather, 0, GNumWeather - 1);
		bRubberStart = bRubberStart || GI->Launch.bRubbered;
	}
	ApplyWeatherPreset(WIndex);
	ApplyTrackPreset(bRubberStart);
	UE_LOG(LogKartConditions, Log, TEXT("Track conditions %s: %d x %d cells, weather %s, track %s, time scale %.1f"),
	       bEnabled ? TEXT("ACTIVE") : TEXT("DISABLED (-KartConditions=0)"), Conditions.CellsAlong(), Conditions.CellsAcross(), *WeatherName, *TrackName, TimeScale);
	UpdateVisuals();
}

void AKartConditionsActor::ApplyWeatherPreset(int32 Index)
{
	WeatherIndex = (Index % GNumWeather + GNumWeather) % GNumWeather;
	const FWeatherPreset& P = GWeather[WeatherIndex];
	kartsim::WeatherState& W = Conditions.Weather;
	W.AirTemp = P.Air;
	W.PressureKPa = P.Pressure;
	W.Humidity = P.Humidity;
	// -KartAirTemp= / -KartPressure= (kPa) / -KartHumidity= (0..1): exact air for jetting tests.
	FParse::Value(FCommandLine::Get(), TEXT("KartAirTemp="), W.AirTemp);
	FParse::Value(FCommandLine::Get(), TEXT("KartPressure="), W.PressureKPa);
	FParse::Value(FCommandLine::Get(), TEXT("KartHumidity="), W.Humidity);
	W.Humidity = FMath::Clamp(W.Humidity, 0.0, 1.0);
	W.CloudCover = P.Cloud;
	W.RainRate = P.Rain;
	// Start at the equilibrium track temperature for this weather (same law as TrackConditions::Advance).
	W.TrackTemp = W.AirTemp + Conditions.Params.SolarHeating * (1.0 - P.Cloud) * (P.Rain > 0.1 ? 0.1 : 1.0);
	// Scenario start: water from the preset, rubber from the track preset (any rubber laid while
	// driving is reset too - a preset is a fresh scenario).
	Conditions.SetUniform(0.0, P.StartWater);
	Conditions.SetSchedule(FCString::Strcmp(P.Name, TEXT("Shower")) == 0 ? GShowerSchedule : std::vector<kartsim::WeatherKey>());
	if (bRubbered) { Conditions.SetRubberedLine(GRubberLevel, GRubberHalfWidth); }
	WeatherName = P.Name;
}

void AKartConditionsActor::ApplyTrackPreset(bool bInRubbered)
{
	bRubbered = bInRubbered;
	// Re-applying the weather preset resets the water to the scenario start, then lays the rubber.
	ApplyWeatherPreset(WeatherIndex);
	TrackName = bRubbered ? TEXT("Rubbered") : TEXT("Green");
}

void AKartConditionsActor::CycleWeather()
{
	ApplyWeatherPreset(WeatherIndex + 1);
	UE_LOG(LogKartConditions, Log, TEXT("Weather -> %s (air %.0f C, track %.0f C, rain %.1f mm/h)"), *WeatherName,
	       Conditions.Weather.AirTemp, Conditions.Weather.TrackTemp, Conditions.Weather.RainRate);
	UpdateVisuals();
}

void AKartConditionsActor::ToggleRubber()
{
	ApplyTrackPreset(!bRubbered);
	UE_LOG(LogKartConditions, Log, TEXT("Track -> %s"), *TrackName);
	UpdateVisuals();
}

void AKartConditionsActor::ApplyPresets(int32 InWeather, bool bInRubbered)
{
	if (InWeather == WeatherIndex && bInRubbered == bRubbered) { return; }
	WeatherIndex = InWeather;
	ApplyTrackPreset(bInRubbered); // re-applies the weather preset, then the rubber
	UE_LOG(LogKartConditions, Log, TEXT("Conditions from host: %s, %s track"), *WeatherName, *TrackName);
	UpdateVisuals();
}

void AKartConditionsActor::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (!Conditions.IsBuilt()) { return; }
	Conditions.Advance(static_cast<double>(DeltaSeconds) * TimeScale);
	VisualTimer += DeltaSeconds;
	if (VisualTimer >= 0.5f)
	{
		VisualTimer = 0.0f;
		UpdateVisuals();
	}
}

void AKartConditionsActor::UpdateVisuals()
{
	const kartsim::TrackConditions& C = Conditions;
	const float Wet = bEnabled ? static_cast<float>(FMath::Clamp(C.MeanWater() / C.Params.WetSaturationDepth, 0.0, 1.0)) : 0.0f;
	for (TActorIterator<AKartTestTrackActor> It(GetWorld()); It; ++It) { It->SetWetness(Wet); }
	// Cloud dims the sun (overcast: ~25 % direct light). Visual only.
	for (TActorIterator<ADirectionalLight> It(GetWorld()); It; ++It)
	{
		if (UDirectionalLightComponent* L = Cast<UDirectionalLightComponent>(It->GetLightComponent()))
		{
			if (BaseSunIntensity < 0.0f) { BaseSunIntensity = L->Intensity; }
			L->SetIntensity(BaseSunIntensity * (1.0f - 0.75f * static_cast<float>(C.Weather.CloudCover)));
		}
		break;
	}
	// Sky photo: clear -> overcast with the cloud cover (M_KartSky "Overcast").
	for (TActorIterator<AStaticMeshActor> It(GetWorld()); It; ++It)
	{
		if (!It->Tags.Contains(TEXT("KartSky"))) { continue; }
		if (UMaterialInstanceDynamic* Sky = Cast<UMaterialInstanceDynamic>(It->GetStaticMeshComponent()->GetMaterial(0)))
		{
			Sky->SetScalarParameterValue(TEXT("Overcast"), FMath::SmoothStep(0.45f, 0.95f, static_cast<float>(C.Weather.CloudCover)));
		}
		break;
	}
	// Overcast / rain: thicker haze so the sky reads grey (visual only; clouds themselves are static).
	for (TActorIterator<AExponentialHeightFog> It(GetWorld()); It; ++It)
	{
		if (UExponentialHeightFogComponent* F = It->GetComponent())
		{
			if (BaseFogDensity < 0.0f) { BaseFogDensity = F->FogDensity; }
			const float Gloom = static_cast<float>(C.Weather.CloudCover) * (C.Weather.RainRate > 0.1 ? 1.0f : 0.5f);
			F->SetFogDensity(BaseFogDensity * (1.0f + 12.0f * Gloom));
		}
		break;
	}
}
