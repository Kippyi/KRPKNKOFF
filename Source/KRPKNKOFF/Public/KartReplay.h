// FKartReplay: session recording and replay playback, owned by AKartRaceWorld.
//  * Recording: every kart in the race world at 30 Hz (kartsim::ReplayRecording, last 30 minutes kept).
//  * Playback (single player): the live karts are hidden and frozen (the race world stops stepping), and
//    display-only puppet karts are posed from the recording. Cameras: chase / cockpit of the focused kart,
//    or trackside TV cameras that follow it.
//  * Files: F7 saves Saved/Replays/<circuit>_<date-time>.krpl; -KartReplay=<file> opens one at start.
// Keys in replay: F5 exit, Space pause, Left/Right -/+5 s (Shift: 30 s), Up/Down speed, V next kart,
// X camera (chase / cockpit / TV).
#pragma once

#include "CoreMinimal.h"
#include "KartSim/Replay.h"

class AKartVehiclePawn;
class ACameraActor;
class APlayerController;

class KRPKNKOFF_API FKartReplay
{
public:
	// --- Recording ---
	void Reset(const FString& Circuit);
	void Record(double Clock, const TArray<AKartVehiclePawn*>& Karts);
	bool HasRecording() const { return Rec.FrameCount() > 0; }
	bool SaveToFile(FString& OutPath) const;
	bool SaveAs(const FString& Path) const;
	bool LoadFromFile(const FString& Path, FString& OutError);
	double RecordedSeconds() const { return Rec.EndTime() - Rec.StartTime(); }

	// --- Playback ---
	bool IsPlaying() const { return bPlaying; }
	// Live karts are hidden while playing; TvLine = track centre line (UE cm) for the TV cameras.
	void Begin(UWorld* World, const TArray<AKartVehiclePawn*>& LiveKarts, APlayerController* PC, const TArray<FVector>& TvLine, double StartAt = -1.0);
	void End(const TArray<AKartVehiclePawn*>& LiveKarts, APlayerController* PC);
	void Tick(float DeltaSeconds, APlayerController* PC);

	// --- HUD ---
	double GetTime() const { return Time; }
	double GetStart() const { return Rec.StartTime(); }
	double GetEnd() const { return Rec.EndTime(); }
	float GetSpeed() const { return Speed; }
	bool IsPaused() const { return bPaused; }
	FString GetFocusName() const;
	const TCHAR* GetCameraName() const;
	int32 NumKarts() const { return static_cast<int32>(Rec.Karts.size()); }
	const FString& GetCircuit() const { return Circuit; }

private:
	void UpdateCamera(APlayerController* PC, float DeltaSeconds);
	void SetFocus(int32 Index, APlayerController* PC);

	kartsim::ReplayRecording Rec;
	FString Circuit;
	TArray<TPair<TWeakObjectPtr<AKartVehiclePawn>, int32>> TrackOf; // live kart -> recorded track
	double NextSample = -1.0;
	double TrimTimer = 0.0;

	bool bPlaying = false;
	double Time = 0.0;
	float Speed = 1.0f;
	bool bPaused = false;
	int32 Focus = 0;
	int32 CamMode = 0; // 0 chase, 1 cockpit, 2 TV
	TArray<TWeakObjectPtr<AKartVehiclePawn>> Puppets;
	TWeakObjectPtr<ACameraActor> TvCam;
	TArray<FVector> TvSpots;
	int32 TvSpot = -1;
	TWeakObjectPtr<AActor> PrevViewTarget;
};

