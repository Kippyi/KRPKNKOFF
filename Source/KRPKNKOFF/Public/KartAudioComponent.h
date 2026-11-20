// UKartAudioComponent: feeds a kart's UKartSynthWave from its simulation every frame.
//  * Own kart (locally controlled): non-spatialised, with wind. Other karts: 3-D, 5 m inner / 60 m falloff.
//  * Volumes from FKartSoundSettings (menu: Settings).
//  * RECORDED ENGINE SOUND: put 16-bit PCM WAV loops in Content/KartData/Sounds/<classId>/ named
//      on_<rpm>.wav   (full throttle / under load, recorded at a steady <rpm>)
//      off_<rpm>.wav  (off throttle / overrun at <rpm>)
//    e.g. rotax_max_senior/on_6000.wav, on_9000.wav, on_12000.wav, off_8000.wav. Loops should be
//    seamless (cut at zero crossings). With no files the per-class synthesis is used.
//  * Remote multiplayer karts carry no engine data: rpm is estimated from wheel speed and gearing.
#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "KartAudioComponent.generated.h"

class UAudioComponent;
class UKartSynthWave;

UCLASS()
class KRPKNKOFF_API UKartAudioComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UKartAudioComponent();
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;

private:
	void Start(bool bOwnKart, const FString& ClassId);
	void StopSound();

	UPROPERTY(Transient) TObjectPtr<UKartSynthWave> Wave;
	UPROPERTY(Transient) TObjectPtr<UAudioComponent> Audio;
	bool bStarted = false;
	bool bOwn = false;
	FString ActiveClass;
	float StartDelay = 0.5f;
	float PrevExtForce = 0.0f;
	float PrevSpeed = 0.0f;
};
