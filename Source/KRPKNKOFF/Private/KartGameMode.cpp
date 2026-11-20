#include "KartGameMode.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Components/ExponentialHeightFogComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMeshActor.h"

#include "KartConditionsActor.h"
#include "KartGraphicsSettings.h"
#include "KartGameInstance.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "KartRaceWorld.h"

#include "Components/DirectionalLightComponent.h"
#include "Components/SkyLightComponent.h"
#include "Engine/DirectionalLight.h"
#include "Engine/ExponentialHeightFog.h"
#include "Engine/PostProcessVolume.h"
#include "Engine/SkyLight.h"
#include "ComponentRecreateRenderStateContext.h"
#include "TimerManager.h"
#include "Components/VolumetricCloudComponent.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/PlayerStart.h"
#include "Components/SkyAtmosphereComponent.h"
#include "KartHUD.h"
#include "KartTestTrackActor.h"
#include "KartVehiclePawn.h"

AKartGameMode::AKartGameMode()
{
	DefaultPawnClass = AKartVehiclePawn::StaticClass();
	HUDClass = AKartHUD::StaticClass();
}

void AKartGameMode::InitGame(const FString& MapName, const FString& Options, FString& ErrorMessage)
{
	Super::InitGame(MapName, Options, ErrorMessage);
	// Re-applied per level: UE's startup re-applies the saved scalability after UKartGameInstance::Init.
	FKartGraphicsSettings::Get().Apply();
	EnsureEnvironment(GetWorld(), true);
}

void AKartGameMode::EnsureEnvironment(UWorld* World, bool bServer)
{
	if (!World)
	{
		return;
	}
	FActorSpawnParameters Params;
	Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

	AKartTestTrackActor* Track = nullptr;
	for (TActorIterator<AKartTestTrackActor> It(World); It; ++It) { Track = *It; break; }
	if (!Track)
	{
		Track = World->SpawnActor<AKartTestTrackActor>(AKartTestTrackActor::StaticClass(), FTransform::Identity, Params);
	}
	if (bServer && Track)
	{
		// Layout: -KartCircuit=<file> (tests), else the main menu's choice, else the class-file default.
		FString Circuit;
		if (!FParse::Value(FCommandLine::Get(), TEXT("KartCircuit="), Circuit))
		{
			if (const UKartGameInstance* GI = UKartGameInstance::Get(World); GI && (GI->bLaunchedFromMenu || UKartGameInstance::IsFrontEnd(World))) { Circuit = GI->Launch.Circuit; }
		}
		if (!Circuit.IsEmpty() && Circuit != Track->GetLayoutFile()) { Track->LoadLayout(Circuit); }
	}

	// Track conditions (rubber / water / weather) follow the track's centre line.
	if (!AKartConditionsActor::Find(World))
	{
		World->SpawnActor<AKartConditionsActor>(AKartConditionsActor::StaticClass(), FTransform::Identity, Params);
	}

	// Race world (replicated, server-spawned): lockstep stepping of all karts, contacts, AI, sessions.
	if (bServer && !AKartRaceWorld::Find(World))
	{
		World->SpawnActor<AKartRaceWorld>(AKartRaceWorld::StaticClass(), FTransform::Identity, Params);
	}

	bool bHasStart = false;
	for (TActorIterator<APlayerStart> It(World); It; ++It) { bHasStart = true; break; }
	if (bServer && !bHasStart && Track)
	{
		World->SpawnActor<APlayerStart>(APlayerStart::StaticClass(), Track->GetStartTransform(), Params);
	}

	bool bHasSun = false;
	for (TActorIterator<ADirectionalLight> It(World); It; ++It) { bHasSun = true; break; }
	if (!bHasSun)
	{
		ADirectionalLight* Sun = World->SpawnActor<ADirectionalLight>(ADirectionalLight::StaticClass(),
			FTransform(FRotator(-47.9f, 34.2f, 0.0f), FVector(0, 0, 1000)), Params); // matches the sun in the sky photo (Tools/CreateSky.py)
		if (Sun)
		{
			UDirectionalLightComponent* L = Cast<UDirectionalLightComponent>(Sun->GetLightComponent());
			if (L)
			{
				L->SetMobility(EComponentMobility::Movable);
				L->SetIntensity(90000.0f); // lux: clear-sky midday sun (physical units; exposure set below)
				L->SetAtmosphereSunLight(true);
				// Realism pass: the real sun disc (0.53 deg) gives soft, distance-correct penumbrae; contact shadows
				// ground small detail (tyre on asphalt, driver in seat) that virtual shadow maps miss.
				L->SetLightSourceAngle(0.53f);
				L->ContactShadowLength = 0.04f;
				L->MarkRenderStateDirty();
			}
		}
		// Photographed sky (CC0 Poly Haven HDRI) on a 5 km "IsSky" sphere: M_KartSky (Tools/CreateSky.py). The sky
		// light's real-time capture picks it up, so ambient light and reflections come from the real sky photo.
		// Weather blends it to the overcast photo (AKartConditionsActor, parameter "Overcast"). Falls back to the
		// physical SkyAtmosphere when the material is missing.
		UMaterialInterface* SkyMat = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/KartVisuals/M_KartSky.M_KartSky"));
		UStaticMesh* SphereMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Sphere.Sphere"));
		if (SkyMat && SphereMesh)
		{
			if (AStaticMeshActor* SkyDome = World->SpawnActor<AStaticMeshActor>(AStaticMeshActor::StaticClass(), FTransform(FRotator::ZeroRotator, FVector::ZeroVector, FVector(10000.0f)), Params))
			{
				SkyDome->Tags.Add(TEXT("KartSky"));
				UStaticMeshComponent* C = SkyDome->GetStaticMeshComponent();
				C->SetMobility(EComponentMobility::Movable);
				C->SetStaticMesh(SphereMesh);
				C->SetMaterial(0, UMaterialInstanceDynamic::Create(SkyMat, SkyDome));
				C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
				C->SetCastShadow(false);
				C->bAffectDistanceFieldLighting = false;
				C->bAffectDynamicIndirectLighting = false;
			}
		}
		else if (AActor* Sky = World->SpawnActor<AActor>(AActor::StaticClass(), FTransform::Identity, Params))
		{
			USkyAtmosphereComponent* Atmos = NewObject<USkyAtmosphereComponent>(Sky, TEXT("SkyAtmosphere"));
			Sky->SetRootComponent(Atmos);
			Atmos->RegisterComponent();
		}
		// Deferred spawn: real-time capture and mobility must be set BEFORE the component registers. Setting
		// bRealTimeCapture on a live component left the render proxy without it, and the sky light contributed
		// nothing (verified: ShowFlag.SkyLighting 0/1 gave identical frames; shaded faces were black).
		ASkyLight* SkyLight = World->SpawnActorDeferred<ASkyLight>(ASkyLight::StaticClass(), FTransform::Identity, nullptr, nullptr, ESpawnActorCollisionHandlingMethod::AlwaysSpawn);
		if (SkyLight)
		{
			if (USkyLightComponent* SL = SkyLight->GetLightComponent())
			{
				SL->Mobility = EComponentMobility::Movable;
				SL->bRealTimeCapture = true;
				SL->SourceType = ESkyLightSourceType::SLS_CapturedScene;
				SL->Intensity = 1.0f;          // the sky sphere already emits physical nits (SkyIntensity)
			}
			SkyLight->FinishSpawning(FTransform::Identity);
		}
		// Lumen (hardware RT) and the real-time sky capture start in a stale state with the runtime-built scene:
		// shaded faces stayed black until any render-state rebuild (verified: toggling r.Lumen.HardwareRayTracing
		// at runtime lit them; ShowFlag.SkyLighting 0/1 made no difference). One rebuild after the level has
		// settled fixes it. -KartNoLightRefresh skips it (A/B check).
		if (!FParse::Param(FCommandLine::Get(), TEXT("KartNoLightRefresh")))
		{
			FTimerHandle Handle;
			World->GetTimerManager().SetTimer(Handle, FTimerDelegate::CreateLambda([]()
			{
				FGlobalComponentRecreateRenderStateContext Refresh;
			}), 1.5f, false);
		}
		// Light aerial haze only (the photo already has its own horizon haze); no volumetric clouds (the photo has them).
		if (AExponentialHeightFog* Fog = World->SpawnActor<AExponentialHeightFog>(AExponentialHeightFog::StaticClass(), FTransform::Identity, Params))
		{
			UExponentialHeightFogComponent* F = Fog->GetComponent();
			F->SetFogDensity(0.0035f);
			F->SetFogHeightFalloff(0.15f);
			F->SetStartDistance(4000.0f);
			// Physical luminance (nits) matched to the sky photo's horizon (~0.33 HDR x 6000): the old 0-1 colour
			// rendered as a near-black haze band at daylight exposure.
			F->SetFogInscatteringColor(FLinearColor(0.42f, 0.52f, 0.66f) * 3600.0f);
		}
		// Unbound post-process: auto exposure over-brightened the (physically dark) asphalt and grass.
		if (APostProcessVolume* PP = World->SpawnActor<APostProcessVolume>(APostProcessVolume::StaticClass(), FTransform::Identity, Params))
		{
			PP->bUnbound = true;
			// Physical daylight (90 klx sun, ~6 knit sky): auto exposure limited to a daylight EV100 range so the
			// camera behaves like a real one (no pumping when looking at dark asphalt or the bright sky).
			PP->Settings.bOverride_AutoExposureBias = true;
			PP->Settings.AutoExposureBias = 0.0f;
			PP->Settings.bOverride_AutoExposureMinBrightness = true;
			PP->Settings.AutoExposureMinBrightness = 12.0f;
			PP->Settings.bOverride_AutoExposureMaxBrightness = true;
			PP->Settings.AutoExposureMaxBrightness = 14.6f;
			// Realism pass (camera-like, kept subtle): Lumen quality, bloom, vignette, grain, motion blur, AO.
			FPostProcessSettings& S = PP->Settings;
			const float Lq = FKartGraphicsSettings::LumenPostQuality(FKartGraphicsSettings::Get().Preset); // 2 on Epic, 1 below
			S.bOverride_LumenFinalGatherQuality = true;      S.LumenFinalGatherQuality = Lq;
			S.bOverride_LumenReflectionQuality = true;       S.LumenReflectionQuality = Lq;
			S.bOverride_LumenSceneDetail = true;             S.LumenSceneDetail = Lq;
			S.bOverride_BloomIntensity = true;               S.BloomIntensity = 0.45f;
			S.bOverride_VignetteIntensity = true;            S.VignetteIntensity = 0.28f;
			S.bOverride_FilmGrainIntensity = true;           S.FilmGrainIntensity = 0.04f;
			S.bOverride_MotionBlurAmount = true;             S.MotionBlurAmount = 0.0f; // off: per-object blur smeared the spinning wheels (user feedback)
			S.bOverride_MotionBlurMax = true;                S.MotionBlurMax = 3.0f;
			S.bOverride_AmbientOcclusionIntensity = true;    S.AmbientOcclusionIntensity = 0.6f;
			S.bOverride_AmbientOcclusionRadius = true;       S.AmbientOcclusionRadius = 120.0f;
			S.bOverride_SceneFringeIntensity = true;         S.SceneFringeIntensity = 0.15f;
			// Realism pass 4 grading (camera-like, subtle): a touch warmer white balance for midday sun, a little
			// more contrast / saturation to cut the washed-out look of the hazy distance, slightly richer shadows.
			S.bOverride_WhiteTemp = true;                    S.WhiteTemp = 6150.0f;
			S.bOverride_ColorContrast = true;                S.ColorContrast = FVector4(1.06f, 1.06f, 1.06f, 1.0f);
			S.bOverride_ColorSaturation = true;              S.ColorSaturation = FVector4(1.07f, 1.07f, 1.07f, 1.0f);
			S.bOverride_ColorGammaShadows = true;            S.ColorGammaShadows = FVector4(1.0f, 1.0f, 1.0f, 0.96f);
			S.bOverride_FilmToe = true;                      S.FilmToe = 0.58f;
		}
	}
}
