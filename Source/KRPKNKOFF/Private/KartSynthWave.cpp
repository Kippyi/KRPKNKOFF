#include "KartSynthWave.h"

UKartSynthWave::UKartSynthWave(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
	SetSampleRate(SampleRateHz);
	NumChannels = 1;
	Duration = INDEFINITELY_LOOPING_DURATION;
	SoundGroup = SOUNDGROUP_Default;
	bLooping = false;
	bProcedural = true;
}

float UKartSynthWave::NextNoise()
{
	Noise ^= Noise << 13;
	Noise ^= Noise >> 17;
	Noise ^= Noise << 5;
	return static_cast<float>(Noise) / 2147483648.0f - 1.0f;
}

float UKartSynthWave::SampleLayers(float RpmNow, float Thr)
{
	// Weighted sum of the two nearest recorded rpms in each throttle set, sets crossfaded by throttle.
	float Out = 0.0f;
	for (int32 Set = 0; Set < 2; ++Set)
	{
		const bool bOn = Set == 0;
		const float SetW = bOn ? Thr : 1.0f - Thr;
		int32 Lo = -1, Hi = -1;
		for (int32 I = 0; I < Layers.Num(); ++I)
		{
			const FKartEngineLayer& L = Layers[I];
			if (L.bOnThrottle != bOn) { continue; }
			if (L.RecordedRpm <= RpmNow && (Lo < 0 || L.RecordedRpm > Layers[Lo].RecordedRpm)) { Lo = I; }
			if (L.RecordedRpm >= RpmNow && (Hi < 0 || L.RecordedRpm < Layers[Hi].RecordedRpm)) { Hi = I; }
		}
		if (Lo < 0) { Lo = Hi; }
		if (Hi < 0) { Hi = Lo; }
		if (Lo < 0) { continue; }
		const float A = Layers[Lo].RecordedRpm, B = Layers[Hi].RecordedRpm;
		const float W = (Hi == Lo || B <= A) ? 0.0f : (RpmNow - A) / (B - A);
		for (int32 Pick = 0; Pick < 2; ++Pick)
		{
			FKartEngineLayer& L = Layers[Pick == 0 ? Lo : Hi];
			const float Wl = SetW * (Pick == 0 ? 1.0f - W : W);
			const int32 N = L.Samples.Num();
			if (N < 2 || Wl <= 0.0f) { continue; }
			const int32 I0 = static_cast<int32>(L.Pos);
			const float Fr = static_cast<float>(L.Pos - I0);
			Out += Wl * FMath::Lerp(L.Samples[I0 % N], L.Samples[(I0 + 1) % N], Fr);
		}
	}
	// Advance every layer (keeps crossfades seamless).
	for (FKartEngineLayer& L : Layers)
	{
		L.Pos += (RpmNow / FMath::Max(L.RecordedRpm, 1.0f)) * (L.SampleRate / SampleRateHz);
		const int32 N = L.Samples.Num();
		if (N > 0 && L.Pos >= N) { L.Pos = FMath::Fmod(L.Pos, static_cast<double>(N)); }
	}
	return Out;
}

int32 UKartSynthWave::OnGeneratePCMAudio(TArray<uint8>& OutAudio, int32 NumSamples)
{
	OutAudio.SetNumUninitialized(NumSamples * sizeof(int16));
	int16* Out = reinterpret_cast<int16*>(OutAudio.GetData());
	const float Fs = static_cast<float>(SampleRateHz);
	const float TargetRpm = bRunning.load() ? Rpm.load() : 0.0f;
	const float TargetThr = Throttle.load();
	const float TargetLoad = Load.load();
	const float TargetScrub = Scrub.load();
	const float TargetWind = Wind.load();
	const float Ge = EngineGain.load(), Gf = FxGain.load(), Gw = WindGain.load();
	const float Hit = HitEnergy.exchange(0.0f);
	if (Hit > 0.0f) { HitEnv = FMath::Max(HitEnv, FMath::Clamp(Hit, 0.0f, 1.0f)); HitPhase = 0.0f; }
	const bool bRecorded = Layers.Num() > 0;
	// Synthesised engine (no recorded layers): render the whole block with the core synth.
	if (!bRecorded)
	{
		if (!bSynthInit) { Synth.SetProfile(SynthProfile, static_cast<float>(SampleRateHz)); bSynthInit = true; }
		kartsim::EngineSoundInput In;
		In.Rpm = TargetRpm;
		In.Throttle = TargetThr;
		In.Load = TargetLoad;
		In.bRunning = bRunning.load();
		In.bLimiter = bLimiter.load();
		In.bClutchLocked = bClutchLocked.load();
		In.AxleHz = AxleHz.load();
		Synth.SetToneScale(ToneScale.load());
		EngineBuf.SetNumUninitialized(NumSamples);
		Synth.Render(EngineBuf.GetData(), NumSamples, In);
	}

	for (int32 I = 0; I < NumSamples; ++I)
	{
		SmoothRpm += (TargetRpm - SmoothRpm) * 0.002f;
		SmoothThr += (TargetThr - SmoothThr) * 0.002f;
		SmoothLoad += (TargetLoad - SmoothLoad) * 0.002f;
		SmoothScrub += (TargetScrub - SmoothScrub) * 0.001f;
		SmoothWind += (TargetWind - SmoothWind) * 0.0005f;

		float Engine = 0.0f;
		if (bRecorded)
		{
			if (SmoothRpm > 300.0f) { Engine = 0.9f * SampleLayers(SmoothRpm, SmoothThr); }
		}
		else
		{
			Engine = EngineBuf[I];
		}

		const float N = NextNoise();
		// Tyre scrub: noise band ~0.5 - 2 kHz.
		ScrubLp += 0.2f * (N - ScrubLp);
		ScrubHp += 0.06f * (ScrubLp - ScrubHp);
		const float Tyre = 0.6f * SmoothScrub * (ScrubLp - ScrubHp);
		// Wind: low rumble (two one-pole low-passes ~150 Hz).
		WindLp1 += 0.02f * (N - WindLp1);
		WindLp2 += 0.02f * (WindLp1 - WindLp2);
		const float WindS = 2.5f * SmoothWind * WindLp2;
		// Contact: 70 Hz thump + noise, ~120 ms.
		float HitS = 0.0f;
		if (HitEnv > 0.001f)
		{
			HitPhase += 70.0f / Fs;
			HitS = HitEnv * (0.8f * FMath::Sin(2.0f * PI * HitPhase) + 0.3f * N);
			HitEnv *= 0.9995f;
		}

		float S = Ge * Engine + Gf * (Tyre + HitS) + Gw * WindS;
		Dc += 0.0005f * (S - Dc); // remove any DC drift
		S = FMath::Clamp(S - Dc, -1.0f, 1.0f);
		Out[I] = static_cast<int16>(S * 32000.0f);
	}
	return NumSamples;
}
