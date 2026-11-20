#include "KartReplay.h"

#include "Camera/CameraActor.h"
#include "Camera/CameraComponent.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "HAL/FileManager.h"
#include "KartUnrealConversions.h"
#include "KartVehiclePawn.h"
#include "Misc/DateTime.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"

namespace
{
	constexpr double kRecordRate = 30.0;          // Hz (same as the multiplayer state stream)
	constexpr double kKeepSeconds = 30.0 * 60.0;  // memory bound: ~140 KB per kart-minute
	const float GSpeeds[] = {0.1f, 0.25f, 0.5f, 1.0f, 2.0f, 4.0f};

	uint32 PackColor(const FLinearColor& C)
	{
		const FColor S = C.ToFColor(true);
		return (static_cast<uint32>(S.R) << 24) | (static_cast<uint32>(S.G) << 16) | (static_cast<uint32>(S.B) << 8) | S.A;
	}
	FLinearColor UnpackColor(uint32 V)
	{
		return FLinearColor(FColor(static_cast<uint8>(V >> 24), static_cast<uint8>(V >> 16), static_cast<uint8>(V >> 8), static_cast<uint8>(V)));
	}
	FVector FramePosUE(const kartsim::ReplayFrame& F) { return KartConv::ToUEPos(kartsim::Vec3(F.Pos[0], F.Pos[1], F.Pos[2])); }
}

// ------------------------------------------------------------------------------- recording

void FKartReplay::Reset(const FString& InCircuit)
{
	Rec = kartsim::ReplayRecording();
	Circuit = InCircuit;
	Rec.Circuit = TCHAR_TO_UTF8(*InCircuit);
	TrackOf.Reset();
	NextSample = -1.0;
}

void FKartReplay::Record(double Clock, const TArray<AKartVehiclePawn*>& Karts)
{
	if (bPlaying) { return; }
	if (NextSample >= 0.0 && Clock + 1e-9 < NextSample) { return; }
	NextSample = Clock + 1.0 / kRecordRate;
	for (AKartVehiclePawn* K : Karts)
	{
		if (!K || !K->GetSim()) { continue; }
		int32 Index = INDEX_NONE;
		for (const TPair<TWeakObjectPtr<AKartVehiclePawn>, int32>& P : TrackOf) { if (P.Key.Get() == K) { Index = P.Value; break; } }
		kartsim::ReplayTrack* T = Index != INDEX_NONE ? &Rec.Karts[Index] : nullptr;
		// A new kart, or the kart switched class (new track: a class change is a different kart on screen).
		if (!T || T->KartFile != TCHAR_TO_UTF8(*K->KartConfigFile))
		{
			Rec.Karts.emplace_back();
			Index = static_cast<int32>(Rec.Karts.size()) - 1;
			T = &Rec.Karts[Index];
			T->Name = TCHAR_TO_UTF8(*K->DriverName);
			T->KartFile = TCHAR_TO_UTF8(*K->KartConfigFile);
			T->Color = PackColor(K->BodyColor);
			TrackOf.RemoveAll([K](const TPair<TWeakObjectPtr<AKartVehiclePawn>, int32>& P) { return P.Key.Get() == K; });
			TrackOf.Add(TPair<TWeakObjectPtr<AKartVehiclePawn>, int32>(K, Index));
		}
		T->Frames.push_back(kartsim::MakeReplayFrame(*K->GetSim(), Clock));
	}
	TrimTimer += 1.0 / kRecordRate;
	if (TrimTimer > 10.0)
	{
		TrimTimer = 0.0;
		Rec.Trim(kKeepSeconds);
	}
}

bool FKartReplay::SaveToFile(FString& OutPath) const
{
	if (!HasRecording()) { return false; }
	const FString Name = FString::Printf(TEXT("%s_%s.krpl"), *FPaths::GetBaseFilename(Circuit.IsEmpty() ? FString(TEXT("Circuit")) : Circuit),
	                                     *FDateTime::Now().ToString(TEXT("%Y-%m-%d_%H-%M-%S")));
	OutPath = FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("Replays") / Name);
	IFileManager::Get().MakeDirectory(*FPaths::GetPath(OutPath), true);
	return Rec.Save(TCHAR_TO_UTF8(*OutPath));
}

bool FKartReplay::SaveAs(const FString& Path) const
{
	if (!HasRecording()) { return false; }
	IFileManager::Get().MakeDirectory(*FPaths::GetPath(Path), true);
	return Rec.Save(TCHAR_TO_UTF8(*Path));
}

bool FKartReplay::LoadFromFile(const FString& Path, FString& OutError)
{
	kartsim::ReplayRecording R;
	std::string Err;
	if (!R.Load(TCHAR_TO_UTF8(*Path), Err)) { OutError = UTF8_TO_TCHAR(Err.c_str()); return false; }
	Rec = MoveTemp(R);
	Circuit = UTF8_TO_TCHAR(Rec.Circuit.c_str());
	TrackOf.Reset();       // the loaded karts are not the live ones
	NextSample = -1.0;
	return true;
}

// ------------------------------------------------------------------------------- playback

void FKartReplay::Begin(UWorld* World, const TArray<AKartVehiclePawn*>& LiveKarts, APlayerController* PC, const TArray<FVector>& TvLine, double StartAt)
{
	if (bPlaying || !World || !HasRecording()) { return; }
	bPlaying = true;
	bPaused = false;
	Speed = 1.0f;
	// Default: the last 60 s.
	Time = StartAt >= 0.0 ? FMath::Clamp(StartAt + Rec.StartTime(), Rec.StartTime(), Rec.EndTime()) : FMath::Max(Rec.StartTime(), Rec.EndTime() - 60.0);
	for (AKartVehiclePawn* K : LiveKarts)
	{
		if (!K) { continue; }
		K->SetActorHiddenInGame(true);
		K->HidePuppets();
	}
	Puppets.Reset();
	for (const kartsim::ReplayTrack& T : Rec.Karts)
	{
		Puppets.Add(AKartVehiclePawn::SpawnPuppet(World, UTF8_TO_TCHAR(T.KartFile.c_str()), UnpackColor(T.Color), UTF8_TO_TCHAR(T.Name.c_str())));
	}
	// TV camera spots: every ~60 m along the centre line, 9 m to the outside of the bend, 4 m up.
	TvSpots.Reset();
	const int32 N = TvLine.Num();
	double Acc = 1e9;
	for (int32 I = 0; I < N && N > 2; ++I)
	{
		if (I > 0) { Acc += FVector::Dist(TvLine[I], TvLine[I - 1]); }
		if (Acc < 6000.0) { continue; }
		Acc = 0.0;
		const FVector Prev = TvLine[(I + N - 3) % N], Cur = TvLine[I], Next = TvLine[(I + 3) % N];
		const FVector T0 = (Cur - Prev).GetSafeNormal2D(), T1 = (Next - Cur).GetSafeNormal2D();
		const FVector Left = FVector(-T0.Y, T0.X, 0.0);
		const float Turn = FVector::CrossProduct(T0, T1).Z; // > 0: bends to the left (UE is left-handed: sign only picks a side)
		const FVector Outside = Turn > 0.0f ? -Left : Left;
		TvSpots.Add(Cur + Outside * 900.0 + FVector(0.0, 0.0, 400.0));
	}
	TvSpot = -1;
	if (!TvCam.IsValid())
	{
		FActorSpawnParameters SP;
		SP.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		TvCam = World->SpawnActor<ACameraActor>(ACameraActor::StaticClass(), FTransform::Identity, SP);
		if (TvCam.IsValid()) { TvCam->GetCameraComponent()->bConstrainAspectRatio = false; }
	}
	PrevViewTarget = PC ? PC->GetViewTarget() : nullptr;
	// Focus: the local player's kart if it was recorded, else the first.
	Focus = 0;
	const AKartVehiclePawn* Me = PC ? Cast<AKartVehiclePawn>(PC->GetPawn()) : nullptr;
	for (const TPair<TWeakObjectPtr<AKartVehiclePawn>, int32>& P : TrackOf) { if (P.Key.Get() == Me) { Focus = P.Value; } }
	// Tests / screenshots: -KartReplayCamera=0|1|2 (chase, cockpit, TV), -KartReplayFocus=<kart index>.
	FParse::Value(FCommandLine::Get(), TEXT("KartReplayCamera="), CamMode);
	CamMode = FMath::Clamp(CamMode, 0, 2);
	FParse::Value(FCommandLine::Get(), TEXT("KartReplayFocus="), Focus);
	Tick(0.0f, PC); // pose before the first frame renders
	SetFocus(Focus, PC);
	UE_LOG(LogTemp, Log, TEXT("REPLAY start: %d karts, %.0f s recorded, from %.1f s"), NumKarts(), Rec.EndTime() - Rec.StartTime(), Time - Rec.StartTime());
}

void FKartReplay::End(const TArray<AKartVehiclePawn*>& LiveKarts, APlayerController* PC)
{
	if (!bPlaying) { return; }
	bPlaying = false;
	for (TWeakObjectPtr<AKartVehiclePawn>& P : Puppets) { if (AKartVehiclePawn* K = P.Get()) { K->Destroy(); } }
	Puppets.Reset();
	for (AKartVehiclePawn* K : LiveKarts) { if (K) { K->SetActorHiddenInGame(false); } }
	if (PC)
	{
		AActor* Back = PC->GetPawn() ? static_cast<AActor*>(PC->GetPawn()) : PrevViewTarget.Get();
		if (Back) { PC->SetViewTarget(Back); }
	}
	UE_LOG(LogTemp, Log, TEXT("REPLAY end"));
}

void FKartReplay::SetFocus(int32 Index, APlayerController* PC)
{
	if (Puppets.Num() == 0) { return; }
	Focus = (Index % Puppets.Num() + Puppets.Num()) % Puppets.Num();
	for (int32 I = 0; I < Puppets.Num(); ++I)
	{
		if (AKartVehiclePawn* K = Puppets[I].Get()) { K->SetPuppetCockpitView(I == Focus && CamMode == 1); }
	}
	if (!PC) { return; }
	if (CamMode == 2 && TvCam.IsValid()) { PC->SetViewTarget(TvCam.Get()); TvSpot = -1; }
	else if (AKartVehiclePawn* K = Puppets[Focus].Get()) { PC->SetViewTarget(K); }
}

void FKartReplay::Tick(float DeltaSeconds, APlayerController* PC)
{
	if (!bPlaying) { return; }
	if (PC)
	{
		auto Pressed = [PC](const FKey& K) { return PC->WasInputKeyJustPressed(K); };
		const double Jump = PC->IsInputKeyDown(EKeys::LeftShift) ? 30.0 : 5.0;
		if (Pressed(EKeys::SpaceBar)) { bPaused = !bPaused; }
		if (Pressed(EKeys::Left)) { Time -= Jump; }
		if (Pressed(EKeys::Right)) { Time += Jump; }
		int32 SpeedIndex = 3;
		for (int32 I = 0; I < UE_ARRAY_COUNT(GSpeeds); ++I) { if (FMath::IsNearlyEqual(GSpeeds[I], Speed)) { SpeedIndex = I; } }
		if (Pressed(EKeys::Up)) { Speed = GSpeeds[FMath::Min(SpeedIndex + 1, static_cast<int32>(UE_ARRAY_COUNT(GSpeeds)) - 1)]; }
		if (Pressed(EKeys::Down)) { Speed = GSpeeds[FMath::Max(SpeedIndex - 1, 0)]; }
		if (Pressed(EKeys::V)) { SetFocus(Focus + 1, PC); }
		if (Pressed(EKeys::X)) { CamMode = (CamMode + 1) % 3; SetFocus(Focus, PC); }
	}
	if (!bPaused) { Time += DeltaSeconds * Speed; }
	if (Time >= Rec.EndTime()) { Time = Rec.EndTime(); bPaused = true; }
	Time = FMath::Clamp(Time, Rec.StartTime(), Rec.EndTime());

	for (int32 I = 0; I < Puppets.Num(); ++I)
	{
		AKartVehiclePawn* K = Puppets[I].Get();
		const kartsim::ReplayTrack& T = Rec.Karts[I];
		if (!K) { continue; }
		// Only while the kart existed (joined later / left earlier / switched class: hidden outside its span).
		const bool bAlive = !T.Frames.empty() && Time >= T.StartTime() - 1e-3 && Time <= T.EndTime() + 0.2;
		K->SetActorHiddenInGame(!bAlive);
		kartsim::ReplayFrame F;
		if (bAlive && T.Sample(Time, F)) { K->PuppetPose(F, bPaused ? 0.0f : DeltaSeconds * Speed); }
	}
	UpdateCamera(PC, DeltaSeconds);
}

void FKartReplay::UpdateCamera(APlayerController* PC, float DeltaSeconds)
{
	if (CamMode != 2 || !TvCam.IsValid() || !Puppets.IsValidIndex(Focus) || TvSpots.Num() == 0) { return; }
	const AKartVehiclePawn* K = Puppets[Focus].Get();
	if (!K) { return; }
	const FVector Target = K->GetActorLocation() + FVector(0.0, 0.0, 40.0);
	// Nearest spot, with hysteresis so the cut does not flicker between two equidistant cameras.
	int32 Best = 0;
	double BestD = 1e30;
	for (int32 I = 0; I < TvSpots.Num(); ++I)
	{
		const double D = FVector::DistSquared(TvSpots[I], Target);
		if (D < BestD) { BestD = D; Best = I; }
	}
	if (TvSpot < 0 || FVector::Dist(TvSpots[TvSpot], Target) > 1.3 * FMath::Sqrt(BestD) || FVector::Dist(TvSpots[TvSpot], Target) > 9000.0)
	{
		TvSpot = Best;
	}
	const FVector Cam = TvSpots[TvSpot];
	TvCam->SetActorLocationAndRotation(Cam, (Target - Cam).Rotation());
	// Zoom: keep the kart about 6 m of picture height.
	const double Dist = FVector::Dist(Cam, Target);
	const float Fov = FMath::Clamp(FMath::RadiansToDegrees(2.0f * FMath::Atan(600.0f / static_cast<float>(FMath::Max(Dist, 1.0)))), 8.0f, 70.0f);
	TvCam->GetCameraComponent()->SetFieldOfView(Fov);
	if (PC && PC->GetViewTarget() != TvCam.Get()) { PC->SetViewTarget(TvCam.Get()); }
}

FString FKartReplay::GetFocusName() const
{
	if (Focus < 0 || Focus >= NumKarts()) { return FString(); }
	return UTF8_TO_TCHAR(Rec.Karts[Focus].Name.c_str());
}

const TCHAR* FKartReplay::GetCameraName() const
{
	return CamMode == 0 ? TEXT("chase") : (CamMode == 1 ? TEXT("cockpit") : TEXT("TV"));
}
