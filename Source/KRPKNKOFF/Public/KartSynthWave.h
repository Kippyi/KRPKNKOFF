// UKartSynthWave: a kart's sound, generated on the audio render thread from values the game thread sets
// (atomics). Two engine sources:
//  1. RECORDED LAYERS (preferred, realistic): WAV loops recorded at known rpm, on and off throttle,
//     from Content/KartData/Sounds/<classId>/ (see KartAudioComponent.h). Each is pitch-shifted by
//     rpm / recorded rpm; neighbouring rpm layers and the on/off sets are crossfaded.
//  2. SYNTHESIS (fallback): kartsim::EngineSoundSynth (KartSim/EngineSound.h) - firing pulses at the exhaust port
//     through a tuned-pipe waveguide and the silencer, intake through the airbox, exhaust valve, chain whine,
//     clutch rattle, overrun misfires. Can be rendered offline: KartSimTests --sound.
// Plus tyre scrub (noise band scaled by sliding power), wind and contact thumps.
#pragma once

#include "CoreMinimal.h"
#include "Sound/SoundWaveProcedural.h"
#include "KartSim/EngineSound.h"

#include <atomic>

#include "KartSynthWave.generated.h"

struct FKartEngineLayer
{
	float RecordedRpm = 0.0f;
	bool bOnThrottle = true;
	float SampleRate = 48000.0f;
	TArray<float> Samples;       // mono, -1..1, loops
	double Pos = 0.0;            // audio thread
};

UCLASS()
class KRPKNKOFF_API UKartSynthWave : public USoundWaveProcedural
{
	GENERATED_BODY()

public:
	UKartSynthWave(const FObjectInitializer& ObjectInitializer);
	virtual int32 OnGeneratePCMAudio(TArray<uint8>& OutAudio, int32 NumSamples) override;

	// Set BEFORE playback starts (not thread-safe afterwards).
	kartsim::EngineSoundProfile SynthProfile;
	TArray<FKartEngineLayer> Layers;

	// Game thread -> audio thread.
	std::atomic<float> Rpm{2500.0f};
	std::atomic<float> Throttle{0.0f};    // 0..1 effective throttle
	std::atomic<float> Load{0.0f};        // 0..1 engine torque fraction
	std::atomic<float> Scrub{0.0f};       // 0..1 tyre sliding level
	std::atomic<float> Wind{0.0f};        // 0..1
	std::atomic<float> EngineGain{0.6f};
	std::atomic<float> FxGain{0.5f};
	std::atomic<float> WindGain{0.2f};
	std::atomic<bool> bRunning{true};
	std::atomic<bool> bLimiter{false};
	std::atomic<bool> bClutchLocked{true};
	std::atomic<float> AxleHz{0.0f};      // rear axle rev/s (chain whine)
	std::atomic<float> ToneScale{1.0f};   // player's Engine tone setting (physical synth)
	std::atomic<float> HitEnergy{0.0f};   // set > 0 to trigger a contact sound (0..1)

	static constexpr int32 SampleRateHz = 48000;

private:
	float SampleLayers(float RpmNow, float Thr);
	float NextNoise();

	kartsim::EngineSoundSynth Synth;
	bool bSynthInit = false;
	TArray<float> EngineBuf;
	float SmoothRpm = 2500.0f, SmoothThr = 0.0f, SmoothScrub = 0.0f, SmoothWind = 0.0f, SmoothLoad = 0.0f;
	float ScrubLp = 0.0f, ScrubHp = 0.0f, WindLp1 = 0.0f, WindLp2 = 0.0f, Dc = 0.0f;
	float HitEnv = 0.0f, HitPhase = 0.0f;
	uint32 Noise = 22222u;
};
