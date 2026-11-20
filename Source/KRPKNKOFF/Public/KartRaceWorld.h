// AKartRaceWorld (Milestone 6): the race. Steps every LOCALLY simulated kart in lockstep with
// kart-to-kart contacts each step, drives the AI karts (host), poses remote karts (multiplayer proxies),
// runs the sessions (kartsim::RaceDirector, host) and replicates the session and classification.
//
// Command line: -KartAI=<n> (default 5; 0 = solo, the physics-regression setup),
//   -KartAIStrength=<f> (scales the AI grip target, default 0.9), -KartSession=Practice|Qualifying|Race,
//   -KartRaceLaps=<n> (10), -KartQualiMinutes=<m> (8), -KartPlayerAI (an AI driver drives the
//   player's kart: demos and in-engine race tests). Keys (host): Y next session, U AI strength.
// Multiplayer: host  UnrealEditor.exe <project> /Engine/Maps/Entry?listen -game
//              client UnrealEditor.exe <project> 127.0.0.1 -game
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "KartNetTypes.h"
#include "KartReplay.h"
#include "KartSim/AiDriver.h"
#include "KartSim/KartContact.h"
#include "KartSim/RaceDirector.h"
#include "KartSim/RacingLine.h"
#include "KartSim/TrackPath.h"
#include "KartRaceWorld.generated.h"

class AKartVehiclePawn;

UCLASS()
class KRPKNKOFF_API AKartRaceWorld : public AActor
{
	GENERATED_BODY()

public:
	AKartRaceWorld();
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	virtual void GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const override;

	static AKartRaceWorld* Find(const UWorld* World);

	void RequestNextSession();   // host
	void CycleAIStrength();      // host

	// --- HUD (identical on host and clients: the replicated view) ---
	const FKartNetSession& GetSession() const { return NetSession; }
	const TArray<FKartNetStanding>& GetStandings() const { return NetStandings; }
	static const TCHAR* AiStateName(uint8 State);

	// --- Replay (single player): F5 watch / exit, F7 save; -KartReplay=<file>, -KartReplayAt=<s> ---
	bool IsReplaying() const { return Replay.IsPlaying(); }
	const FKartReplay& GetReplay() const { return Replay; }
	const FString& GetReplayMessage() const { return ReplayMessage; }

private:
	struct FEntry
	{
		TWeakObjectPtr<AKartVehiclePawn> Pawn;
		bool bAI = false;          // driven by an AiDriver on this machine
		kartsim::AiDriver Ai;
		size_t PathHint = 0;
	};

	bool EnsureSetup();
	void SyncKarts();
	void SpawnAI(int32 Count);
	void StartSession(const kartsim::SessionConfig& Config);
	FTransform GridSlot(int32 Slot) const;
	std::vector<kartsim::CompetitorInput> CompetitorInputs();
	void ConfigureAI(FEntry& E, int32 Index);
	// AI tyre choice: the set with the most grip for the current wetness on the racing line.
	void ChooseAiTyres(class AKartVehiclePawn* K) const;
	void PublishSession();
	void UpdateReplay(float DeltaSeconds);
	TArray<AKartVehiclePawn*> LivePawns() const;
	void LogSessionEvents();
	double ServerTime() const;

	UPROPERTY(Replicated) FKartNetSession NetSession;
	UPROPERTY(Replicated) TArray<FKartNetStanding> NetStandings;

	TArray<FEntry> Karts;
	FDelegateHandle LayoutHandle;
	bool bSetup = false;
	bool bPlayerAI = false;
	int32 NumAI = 5;
	int32 AISpawned = 0;
	int32 HumansNamed = 0;
	float AIStrength = 0.9f;
	double Clock = 0.0;
	double Accumulator = 0.0;
	double NetSendTimer = 0.0;
	float PhysicsRateHz = 240.0f;

	kartsim::RacingLine Line;
	kartsim::TrackPath Centre;      // lap distance for the classification
	TArray<FVector> CentreUE;
	kartsim::ContactParams Contact;
	std::vector<kartsim::KartContactInfo> Contacts;
	int32 ContactSteps = 0, ContactsShown = 0;
	double ContactWindow = 0.0;
	double SessionContactTime = 0.0, SessionMaxContactForce = 0.0;
	int32 SessionOrderChanges = 0;
	TArray<int32> LastOrderSeen;

	FKartReplay Replay;
	FString ReplayMessage;
	float ReplayMessageTime = 0.0f;
	FString ReplayFileToOpen;
	double ReplayOpenAt = -1.0;
	float QuitAfterSeconds = 0.0f; // -KartQuitAfter=<s>: automated runs (replay tests)
	float RunClock = 0.0f;

	kartsim::RaceDirector Director;
	TArray<TWeakObjectPtr<AKartVehiclePawn>> DirectorKarts; // competitor index -> kart
	kartsim::SessionConfig Pending;
	bool bPending = false;
	int32 RaceLaps = 10;
	double QualiMinutes = 8.0;
	TArray<TWeakObjectPtr<AKartVehiclePawn>> LastQualiOrder; // race grid
	kartsim::ESessionPhase LoggedPhase = kartsim::ESessionPhase::Finished;

	// Debug: -KartRecordAudio=<s> records the game's audio output to Saved/AudioCapture/KartAudio.wav
	// (starting 4 s in) and logs the player's mean rpm over the same window (sound verification).
	void UpdateAudioCapture(float DeltaSeconds);
	float AudioCaptureLength = 0.0f, AudioCaptureClock = 0.0f;
	double AudioRpmSum = 0.0;
	int32 AudioRpmCount = 0;
	int32 AudioCaptureState = 0; // 0 idle/waiting, 1 recording, 2 done
};
