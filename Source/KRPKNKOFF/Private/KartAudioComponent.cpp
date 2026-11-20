#include "KartAudioComponent.h"
#include "Kismet/GameplayStatics.h"
#include "KartGameInstance.h"

#include "Audio.h"
#include "Components/AudioComponent.h"
#include "HAL/FileManager.h"
#include "KartSim/KartPhysicsCore.h"
#include "KartSoundSettings.h"
#include "KartSynthWave.h"
#include "KartVehiclePawn.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"

DEFINE_LOG_CATEGORY_STATIC(LogKartAudio, Log, All);

namespace
{
	// Loads Content/KartData/Sounds/<ClassId>/{on,off}_<rpm>.wav (16-bit PCM, any rate, mixed to mono).
	TArray<FKartEngineLayer> LoadLayers(const FString& ClassId)
	{
		TArray<FKartEngineLayer> Out;
		const FString Dir = FPaths::ProjectContentDir() / TEXT("KartData/Sounds") / ClassId;
		TArray<FString> Files;
		IFileManager::Get().FindFiles(Files, *(Dir / TEXT("*.wav")), true, false);
		for (const FString& F : Files)
		{
			FString Base = FPaths::GetBaseFilename(F).ToLower(), Kind, RpmStr;
			if (!Base.Split(TEXT("_"), &Kind, &RpmStr) || (Kind != TEXT("on") && Kind != TEXT("off"))) { continue; }
			TArray<uint8> Bytes;
			FWaveModInfo Info;
			if (!FFileHelper::LoadFileToArray(Bytes, *(Dir / F)) || !Info.ReadWaveInfo(Bytes.GetData(), Bytes.Num()) || *Info.pBitsPerSample != 16)
			{
				UE_LOG(LogKartAudio, Warning, TEXT("Skipped %s (needs 16-bit PCM WAV)"), *F);
				continue;
			}
			FKartEngineLayer L;
			L.bOnThrottle = Kind == TEXT("on");
			L.RecordedRpm = FCString::Atof(*RpmStr);
			L.SampleRate = static_cast<float>(*Info.pSamplesPerSec);
			const int32 Ch = FMath::Max<int32>(*Info.pChannels, 1);
			const int16* Pcm = reinterpret_cast<const int16*>(Info.SampleDataStart);
			const int32 Frames = static_cast<int32>(Info.SampleDataSize / (2 * Ch));
			L.Samples.SetNumUninitialized(Frames);
			for (int32 I = 0; I < Frames; ++I)
			{
				float Sum = 0.0f;
				for (int32 C = 0; C < Ch; ++C) { Sum += Pcm[I * Ch + C] / 32768.0f; }
				L.Samples[I] = Sum / Ch;
			}
			if (L.RecordedRpm > 500.0f && Frames > 100) { Out.Add(MoveTemp(L)); }
		}
		if (Out.Num() > 0) { UE_LOG(LogKartAudio, Log, TEXT("%s: %d recorded engine layer(s) loaded"), *ClassId, Out.Num()); }
		return Out;
	}
}

UKartAudioComponent::UKartAudioComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PostPhysics;
	PrimaryComponentTick.bTickEvenWhenPaused = true; // must run while paused to silence the karts behind a menu
}

void UKartAudioComponent::Start(bool bOwnKart, const FString& ClassId)
{
	bStarted = true;
	bOwn = bOwnKart;
	ActiveClass = ClassId;
	if (FParse::Param(FCommandLine::Get(), TEXT("KartMute")) || IsRunningDedicatedServer()) { return; }
	AActor* Owner = GetOwner();
	Wave = NewObject<UKartSynthWave>(this);
	Wave->SynthProfile = kartsim::EngineSoundProfile::ForClass(TCHAR_TO_UTF8(*ClassId.ToLower()));
	// Engine sound v4 (physical model). -KartOldEngineSound = the v3 pulse synth (A/B).
	Wave->SynthProfile.bPhysical = !FParse::Param(FCommandLine::Get(), TEXT("KartOldEngineSound"));
	if (!bOwn)
	{
		// Other karts are heard from outside: the exhaust dominates (the airbox is next to THEIR driver's ear), and
		// half the cylinder sub-steps keep six karts cheap on the audio thread.
		Wave->SynthProfile.IntakeMix *= 0.45f;
		Wave->SynthProfile.ExhaustMix *= 1.1f;
		Wave->SynthProfile.Geo.Substeps = 2;
	}
	Wave->Layers = LoadLayers(ClassId);
	Audio = NewObject<UAudioComponent>(Owner);
	Audio->SetupAttachment(Owner->GetRootComponent());
	Audio->bAutoActivate = false;
	Audio->bAllowSpatialization = !bOwn;
	if (!bOwn)
	{
		Audio->bOverrideAttenuation = true;
		FSoundAttenuationSettings& A = Audio->AttenuationOverrides;
		A.bAttenuate = true;
		A.bSpatialize = true;
		A.AttenuationShape = EAttenuationShape::Sphere;
		// Other karts: full volume only within 2 m, then a natural (logarithmic) fall-off to silence at ~70 m.
		// The old linear 5 m / 65 m curve kept a grid of AI karts next to you near full volume.
		A.AttenuationShapeExtents = FVector(200.0f, 0.0f, 0.0f);
		A.FalloffDistance = 7000.0f;
		A.DistanceAlgorithm = EAttenuationDistanceModel::NaturalSound;
		A.dBAttenuationAtMax = -50.0f;
	}
	Audio->RegisterComponent();
	Audio->SetSound(Wave);
	Audio->Play();
}

void UKartAudioComponent::StopSound()
{
	if (Audio) { Audio->Stop(); Audio->DestroyComponent(); }
	Audio = nullptr;
	Wave = nullptr;
}

void UKartAudioComponent::EndPlay(const EEndPlayReason::Type Reason)
{
	StopSound();
	Super::EndPlay(Reason);
}

void UKartAudioComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	const AKartVehiclePawn* K = Cast<AKartVehiclePawn>(GetOwner());
	if (!K || !K->GetSim()) { return; }
	const kartsim::KartSimulation& S = *K->GetSim();
	const FString ClassId = UTF8_TO_TCHAR(S.GetConfig().ClassId.c_str());
	if (!bStarted)
	{
		StartDelay -= DeltaTime; // wait for possession so we know whether this is our own kart
		if (StartDelay > 0.0f) { return; }
		Start(K->IsLocallyControlled() && !K->bIsAI, ClassId);
	}
	else if (ClassId != ActiveClass)
	{
		StopSound(); // class switched: new character / recordings
		Start(bOwn, ClassId);
	}
	if (!Wave) { return; }

	const FKartSoundSettings& Vol = FKartSoundSettings::Get();
	const float Master = Vol.bMute ? 0.0f : Vol.Master;
	Wave->EngineGain = Master * (bOwn ? Vol.Engine : Vol.OtherKarts);
	Wave->ToneScale = Vol.EngineTone;
	Wave->FxGain = Master * Vol.Effects * (bOwn ? 1.0f : 0.6f);
	Wave->WindGain = bOwn ? Master * Vol.Wind : 0.0f;
	// Silent: hidden live karts during a replay, the ghost, the main-menu showroom and a paused game (menus).
	if (K->IsHidden() || K->bPuppetSilent || UKartGameInstance::IsFrontEnd(K) || UGameplayStatics::IsGamePaused(K))
	{
		// Live karts hidden during a replay, and the ghost: silent.
		Wave->EngineGain = 0.0f;
		Wave->FxGain = 0.0f;
		Wave->WindGain = 0.0f;
	}

	const kartsim::TelemetryFrame& F = S.GetTelemetry();
	const float Speed = static_cast<float>(S.GetChassis().Velocity.Length());
	float PRpm = 0.0f, PThr = 0.0f;
	if (K->GetPuppetSound(PRpm, PThr))
	{
		// Replay puppet: engine from the recorded frame.
		Wave->Rpm = PRpm;
		Wave->Throttle = PThr;
		Wave->Load = PThr;
		Wave->bRunning = PRpm > 500.0f;
		Wave->bLimiter = false;
		Wave->bClutchLocked = PRpm > 5000.0f;
		Wave->Scrub = 0.0f;
	}
	else if (K->IsSimulatedLocally())
	{
		Wave->Rpm = static_cast<float>(F.EngineRpm);
		Wave->Throttle = static_cast<float>(F.ThrottleEff);
		Wave->Load = FMath::Clamp(static_cast<float>(F.EngineTorque) / 25.0f, 0.0f, 1.0f);
		Wave->bRunning = F.EngineRunning > 0.5;
		Wave->bLimiter = F.RevLimiter > 0.5;
		Wave->bClutchLocked = F.ClutchLocked > 0.5;
		double Slide = 0.0;
		for (int32 I = 0; I < kartsim::NumWheels; ++I) { Slide += S.GetWheel(I).Tyre.SlidePower; }
		Wave->Scrub = FMath::Clamp(static_cast<float>(Slide / 8000.0), 0.0f, 1.0f);
		const float Ext = static_cast<float>(S.GetExternalForce().Length());
		if (Ext > 800.0f && PrevExtForce < 300.0f) { Wave->HitEnergy = FMath::Clamp(Ext / 6000.0f, 0.2f, 1.0f); }
		PrevExtForce = Ext;
	}
	else
	{
		// Remote kart: no engine data on the network; estimate from the wheels and gearing.
		const kartsim::KartConfig& C = S.GetConfig();
		const double Omega = S.GetWheel(kartsim::WheelRL).Omega;
		double Rpm = 0.0;
		for (int32 G = C.Gearbox.NumGears - 1; G >= 0; --G)
		{
			Rpm = Omega * C.TotalRatio(G) * 60.0 / (2.0 * PI);
			if (Rpm > 0.55 * C.Engine.RevLimitRpm || G == 0) { break; }
		}
		Wave->Rpm = FMath::Max(static_cast<float>(Rpm), static_cast<float>(C.Engine.IdleRpm));
		const float Accel = (Speed - PrevSpeed) / FMath::Max(DeltaTime, 1e-3f);
		Wave->Throttle = Accel > 0.3f ? 1.0f : 0.05f;
		Wave->Load = Wave->Throttle.load();
		Wave->bLimiter = false;
		Wave->bClutchLocked = Rpm > 5000.0;
		Wave->Scrub = 0.0f;
	}
	Wave->AxleHz = static_cast<float>(FMath::Abs(S.GetWheel(kartsim::WheelRL).Omega) / (2.0 * PI)); // chain whine
	Wave->Wind = bOwn ? FMath::Clamp(Speed * Speed / 900.0f, 0.0f, 1.0f) : 0.0f;
	PrevSpeed = Speed;
}
