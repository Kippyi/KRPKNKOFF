#include "KartRaceWorld.h"
#include "KartLicense.h"

#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/GameStateBase.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/PlayerState.h"
#include "Kismet/GameplayStatics.h"
#include "AudioMixerBlueprintLibrary.h"
#include "KartConditionsActor.h"
#include "Misc/Paths.h"
#include "KartGameInstance.h"
#include "KartSim/DriverAids.h"
#include "KartSim/Slipstream.h"
#include "KartTestTrackActor.h"
#include "KartUnrealConversions.h"
#include "KartVehiclePawn.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Net/UnrealNetwork.h"

DEFINE_LOG_CATEGORY_STATIC(LogKartRace, Log, All);

namespace
{
	// Fictional AI drivers.
	const TCHAR* GNames[] = {TEXT("A. Reyes"), TEXT("M. Strand"), TEXT("J. Keller"), TEXT("L. Ferri"), TEXT("T. Vidal"),
	                         TEXT("R. Menon"), TEXT("S. Aalto"), TEXT("D. Conti"), TEXT("K. Novak"), TEXT("E. Laine"), TEXT("P. Duarte")};
	const FLinearColor GColors[] = {{0.05f, 0.25f, 0.75f}, {0.85f, 0.55f, 0.02f}, {0.05f, 0.45f, 0.10f}, {0.80f, 0.80f, 0.82f},
	                                {0.45f, 0.05f, 0.55f}, {0.02f, 0.55f, 0.60f}, {0.90f, 0.30f, 0.45f}, {0.15f, 0.15f, 0.16f},
	                                {0.85f, 0.80f, 0.10f}, {0.40f, 0.22f, 0.08f}, {0.30f, 0.65f, 0.95f}};
	const FLinearColor GHumanColors[] = {{0.72f, 0.04f, 0.03f}, {0.95f, 0.45f, 0.05f}, {0.10f, 0.75f, 0.30f}, {0.95f, 0.95f, 0.95f}};
	// Relative pace of the AI field (x AIStrength x 1.4 g): a spread like a club grid.
	const double GPace[] = {1.00, 0.97, 0.985, 0.955, 0.94, 0.99, 0.965, 0.95, 0.975, 0.93, 0.96};
	constexpr int32 GMaxAI = UE_ARRAY_COUNT(GNames);
	constexpr double FormationSpeed = 15.0;  // m/s (54 km/h) on a rolling start's formation lap
	constexpr double InterpDelay = 0.10;     // s behind the server clock for remote karts
	constexpr double NetSendInterval = 1.0 / 30.0;
}

AKartRaceWorld::AKartRaceWorld()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PrePhysics;
	bReplicates = true;
	bAlwaysRelevant = true;
	SetNetUpdateFrequency(20.0f);
}

void AKartRaceWorld::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const
{
	Super::GetLifetimeReplicatedProps(OutLifetimeProps);
	DOREPLIFETIME(AKartRaceWorld, NetSession);
	DOREPLIFETIME(AKartRaceWorld, NetStandings);
}

AKartRaceWorld* AKartRaceWorld::Find(const UWorld* World)
{
	if (!World) { return nullptr; }
	for (TActorIterator<AKartRaceWorld> It(World); It; ++It) { return *It; }
	return nullptr;
}

const TCHAR* AKartRaceWorld::AiStateName(uint8 State)
{
	switch (State)
	{
	case 0: return TEXT("racing");
	case 1: return TEXT("following");
	case 2: return TEXT("overtaking");
	case 3: return TEXT("formation");
	default: return TEXT("");
	}
}

double AKartRaceWorld::ServerTime() const
{
	const AGameStateBase* GS = GetWorld()->GetGameState();
	return GS ? GS->GetServerWorldTimeSeconds() : GetWorld()->GetTimeSeconds();
}

void AKartRaceWorld::BeginPlay()
{
	Super::BeginPlay();
	// A new layout (a client following the host): re-read the centre line and racing line.
	LayoutHandle = AKartTestTrackActor::OnLayoutChanged.AddWeakLambda(this, [this]() { bSetup = false; });
	bPlayerAI = FParse::Param(FCommandLine::Get(), TEXT("KartPlayerAI"));
	FParse::Value(FCommandLine::Get(), TEXT("KartRecordAudio="), AudioCaptureLength);
	FParse::Value(FCommandLine::Get(), TEXT("KartReplay="), ReplayFileToOpen);
	FParse::Value(FCommandLine::Get(), TEXT("KartReplayAt="), ReplayOpenAt);
	FParse::Value(FCommandLine::Get(), TEXT("KartQuitAfter="), QuitAfterSeconds);
	if (!HasAuthority()) { return; } // clients follow the replicated session
	const UKartGameInstance* GI = UKartGameInstance::Get(this);
	const bool bMenu = GI && GI->bLaunchedFromMenu;
	if (bMenu)
	{
		NumAI = GI->Launch.NumAI;
		AIStrength = GI->Launch.AIStrength;
		RaceLaps = GI->Launch.RaceLaps;
		QualiMinutes = GI->Launch.QualiMinutes;
	}
	FParse::Value(FCommandLine::Get(), TEXT("KartAI="), NumAI);
	NumAI = FMath::Clamp(NumAI, 0, GMaxAI);
	FParse::Value(FCommandLine::Get(), TEXT("KartAIStrength="), AIStrength);
	AIStrength = FMath::Clamp(AIStrength, 0.5f, 1.1f); // U cycles 70 / 80 / 90 / 100 %
	FParse::Value(FCommandLine::Get(), TEXT("KartRaceLaps="), RaceLaps);
	RaceLaps = FMath::Clamp(RaceLaps, 1, 200);
	FParse::Value(FCommandLine::Get(), TEXT("KartQualiMinutes="), QualiMinutes);
	QualiMinutes = FMath::Clamp(QualiMinutes, 0.1, 120.0);
	FString Type;
	Pending = kartsim::SessionConfig();
	if (FParse::Value(FCommandLine::Get(), TEXT("KartSession="), Type))
	{
		if (Type.Equals(TEXT("Qualifying"), ESearchCase::IgnoreCase)) { Pending.Type = kartsim::ESessionType::Qualifying; }
		else if (Type.Equals(TEXT("Race"), ESearchCase::IgnoreCase)) { Pending.Type = kartsim::ESessionType::Race; }
		bPending = true;
	}
	else if (bMenu)
	{
		Pending.Type = static_cast<kartsim::ESessionType>(FMath::Clamp(GI->Launch.Session, 0, 2));
		bPending = true;
	}
	else
	{
		// With other karts on track, open an (unlimited) practice so the timing tower runs.
		bPending = NumAI > 0 || GetNetMode() == NM_ListenServer;
	}
	Pending.Duration = Pending.Type == kartsim::ESessionType::Qualifying ? QualiMinutes * 60.0 : 0.0;
	Pending.RaceLaps = RaceLaps;
	if (UKartGameInstance::IsFrontEnd(this))
	{
		NumAI = 0;        // main menu: a quiet showroom, no AI circulating behind it
		bPending = false;
	}
}

bool AKartRaceWorld::EnsureSetup()
{
	if (bSetup) { return true; }
	AKartVehiclePawn* Player = Cast<AKartVehiclePawn>(UGameplayStatics::GetPlayerPawn(this, 0));
	if (!Player || !Player->GetSim() || !Player->HasTimingLines()) { return false; }
	AKartTestTrackActor* Track = nullptr;
	for (TActorIterator<AKartTestTrackActor> It(GetWorld()); It; ++It) { Track = *It; break; }
	if (!Track) { return false; }

	Track->GetCentreLine(CentreUE);
	std::vector<kartsim::Vec3> C;
	for (const FVector& P : CentreUE) { C.push_back(KartConv::FromUEPos(P)); }
	Centre.Build(C);
	// Racing line: CoG kept 0.9 m from the paved edge (half the 1.4 m kart width + 0.2 m).
	Line = kartsim::ComputeRacingLine(C, 0.5 * Track->TrackWidth, 0.9);
	bSetup = true;
	SyncKarts();
	if (HasAuthority() && AISpawned == 0) { SpawnAI(NumAI); }
	UE_LOG(LogKartRace, Log, TEXT("Race world (%s): %d kart(s), %d AI, strength %.2f, racing line %d points"),
	       HasAuthority() ? (GetNetMode() == NM_Standalone ? TEXT("standalone") : TEXT("host")) : TEXT("client"), Karts.Num(), HasAuthority() ? NumAI : 0,
	       AIStrength, static_cast<int32>(Line.Points.size()));
	return true;
}

void AKartRaceWorld::ConfigureAI(FEntry& E, int32 Index)
{
	kartsim::AiSkill S;
	S.LatG = 1.4 * AIStrength * GPace[Index % GMaxAI];
	S.Consistency = 0.015;
	S.Aggression = 0.3 + 0.07 * (Index % 7);
	S.Seed = 1000u + static_cast<uint32_t>(Index) * 7919u;
	E.Ai.Skill = S;
	E.Ai.Configure(Line);
}

void AKartRaceWorld::ChooseAiTyres(AKartVehiclePawn* K) const
{
	const AKartConditionsActor* CA = AKartConditionsActor::Find(GetWorld());
	if (!K || !K->GetSim()) { return; }
	const double Wetness = CA && CA->IsActive() ? CA->Get().LineWetness() : 0.0;
	K->FitTyreCompound(kartsim::ChooseTyreCompound(K->GetSim()->GetConfig(), Wetness));
}

void AKartRaceWorld::SyncKarts()
{
	// Online: humans carry their Steam names (the player state is set when the player logs in, after the kart spawns).
	if (HasAuthority())
	{
		const UKartGameInstance* GI = UKartGameInstance::Get(this);
		for (FEntry& E : Karts)
		{
			AKartVehiclePawn* K = E.Pawn.Get();
			const APlayerState* PS = K && !K->bIsAI ? K->GetPlayerState() : nullptr;
			if (PS && GI && GI->IsSteamOnline())
			{
				const FString N = PS->GetPlayerName().Left(24);
				if (!N.IsEmpty() && N != K->DriverName) { K->DriverName = N; }
			}
		}
	}
	// Every kart in the world: local players, remote players (proxies) and AI.
	Karts.RemoveAll([](const FEntry& E) { return !E.Pawn.IsValid(); });
	const APawn* LocalPlayer = UGameplayStatics::GetPlayerPawn(this, 0);
	for (TActorIterator<AKartVehiclePawn> It(GetWorld()); It; ++It)
	{
		AKartVehiclePawn* K = *It;
		if (K->bPuppet) { continue; } // replay / ghost display karts
		if (Karts.ContainsByPredicate([K](const FEntry& E) { return E.Pawn.Get() == K; })) { continue; }
		FEntry E;
		E.Pawn = K;
		K->SetRaceWorld(this);
		if (HasAuthority() && !K->bIsAI)
		{
			// Name and colour every human (replicated).
			K->DriverName = FString::Printf(TEXT("Player %d"), ++HumansNamed);
			K->BodyColor = GHumanColors[(HumansNamed - 1) % UE_ARRAY_COUNT(GHumanColors)];
			UE_LOG(LogKartRace, Log, TEXT("%s joined (%s)"), *K->DriverName, K == LocalPlayer ? TEXT("local") : TEXT("remote"));
		}
		// AI karts are driven on the host; -KartPlayerAI lets an AI driver drive this machine's player.
		if ((HasAuthority() && K->bIsAI) || (bPlayerAI && K == LocalPlayer))
		{
			E.bAI = true;
			ConfigureAI(E, K->bIsAI ? FMath::Max(AISpawned - 1, 0) : GMaxAI - 1);
			if (K->GetSim()) { E.Ai.Reset(*K->GetSim()); }
		}
		Karts.Add(MoveTemp(E));
	}
}

void AKartRaceWorld::SpawnAI(int32 Count)
{
	const AKartVehiclePawn* Player = Cast<AKartVehiclePawn>(UGameplayStatics::GetPlayerPawn(this, 0));
	for (int32 I = 0; I < Count; ++I)
	{
		const FTransform Slot = GridSlot(I + 1);
		AKartVehiclePawn* K = GetWorld()->SpawnActorDeferred<AKartVehiclePawn>(AKartVehiclePawn::StaticClass(), Slot, nullptr, nullptr,
		                                                                       ESpawnActorCollisionHandlingMethod::AlwaysSpawn);
		if (!K) { continue; }
		K->bIsAI = true;
		K->DriverName = GNames[I];
		K->BodyColor = GColors[I];
		if (Player) { K->KartConfigFile = Player->KartConfigFile; }
		K->NetKartConfig = K->KartConfigFile;
		K->AutoPossessAI = EAutoPossessAI::Disabled;
		UGameplayStatics::FinishSpawningActor(K, Slot);
		ChooseAiTyres(K);
		AISpawned = I + 1;
		SyncKarts();
	}
}

FTransform AKartRaceWorld::GridSlot(int32 Slot) const
{
	// Two-wide grid past the start/finish line: front row 24 m after it, 6 m between rows, the second
	// kart of each row 3 m back, 1.4 m either side of the centre line.
	const int32 M = CentreUE.Num();
	if (M < 3) { return GetActorTransform(); }
	const double S = 24.0 - 6.0 * (Slot / 2) - 3.0 * (Slot % 2);
	double Acc = 0.0;
	int32 I = 0;
	while (I + 1 < M && Acc + FVector::Dist(CentreUE[I], CentreUE[I + 1]) / 100.0 < S) { Acc += FVector::Dist(CentreUE[I], CentreUE[I + 1]) / 100.0; ++I; }
	const FVector Tan = (CentreUE[(I + 1) % M] - CentreUE[I]).GetSafeNormal2D();
	const FVector Left = FVector(Tan.Y, -Tan.X, 0.0); // UE is left-handed: +Y is right of +X
	const FVector Pos = CentreUE[I] + Left * ((Slot % 2 == 0) ? 140.0 : -140.0) + FVector(0, 0, 30.0);
	return FTransform(Tan.Rotation(), Pos);
}

void AKartRaceWorld::RequestNextSession()
{
	if (!HasAuthority()) { return; }
	using kartsim::ESessionType;
	const ESessionType Cur = Director.IsActive() ? Director.Config().Type : ESessionType::Race;
	Pending = kartsim::SessionConfig();
	Pending.Type = Cur == ESessionType::Practice ? ESessionType::Qualifying : (Cur == ESessionType::Qualifying ? ESessionType::Race : ESessionType::Practice);
	Pending.Duration = Pending.Type == ESessionType::Qualifying ? QualiMinutes * 60.0 : 0.0;
	Pending.RaceLaps = RaceLaps;
	bPending = true;
}

void AKartRaceWorld::CycleAIStrength()
{
	if (!HasAuthority()) { return; }
	AIStrength = AIStrength >= 0.99f ? 0.7f : AIStrength + 0.1f;
	UE_LOG(LogKartRace, Log, TEXT("AI strength %.0f%% (applies from the next session)"), AIStrength * 100.0f);
}

void AKartRaceWorld::StartSession(const kartsim::SessionConfig& InConfig)
{
	kartsim::SessionConfig Config = InConfig;
	AKartVehiclePawn* Host = Cast<AKartVehiclePawn>(UGameplayStatics::GetPlayerPawn(this, 0));
	// Start procedure by class (the host's): gearbox karts (KZ) standing start, single-speed rolling start.
	Config.bRollingStart = Host && Host->GetSim() && Host->GetSim()->GetConfig().Gearbox.NumGears <= 1;

	// Grid: a race uses the last qualifying order; otherwise AI first, humans at the back.
	TArray<FEntry*> Order;
	if (Config.Type == kartsim::ESessionType::Race)
	{
		for (const TWeakObjectPtr<AKartVehiclePawn>& P : LastQualiOrder)
		{
			if (FEntry* E = Karts.FindByPredicate([&P](const FEntry& X) { return X.Pawn == P; })) { Order.Add(E); }
		}
	}
	for (FEntry& E : Karts) { if (E.Pawn->bIsAI && !Order.Contains(&E)) { Order.Add(&E); } }
	for (FEntry& E : Karts) { if (!Order.Contains(&E)) { Order.Add(&E); } }

	int32 AiIndex = 0;
	for (int32 Slot = 0; Slot < Order.Num(); ++Slot)
	{
		FEntry& E = *Order[Slot];
		AKartVehiclePawn* K = E.Pawn.Get();
		if (K->bIsAI)
		{
			if (Host && K->KartConfigFile != Host->KartConfigFile)
			{
				const int32 ClassIndex = K->AvailableKarts.IndexOfByKey(Host->KartConfigFile);
				if (ClassIndex != INDEX_NONE) { K->SwitchKart(ClassIndex); } // AI karts run the host's class
			}
			ChooseAiTyres(K);          // before placing: a tyre change re-initialises the kart
			ConfigureAI(E, AiIndex++); // re-applies the current AI strength
		}
		K->NetPlaceAt(GridSlot(Slot));
		if (E.bAI && K->GetSim()) { E.Ai.Reset(*K->GetSim()); }
	}
	DirectorKarts.Reset();
	for (FEntry& E : Karts) { DirectorKarts.Add(E.Pawn); } // competitor index = entry index
	Accumulator = 0.0;
	SessionContactTime = SessionMaxContactForce = 0.0;
	SessionOrderChanges = 0;
	LastOrderSeen.Reset();
	Director.Start(Config, Clock, Centre.Length(), CompetitorInputs());
	LoggedPhase = Director.Phase();
	UE_LOG(LogKartRace, Log, TEXT("SESSION START %s, %d karts%s"), UTF8_TO_TCHAR(kartsim::RaceDirector::TypeName(Config.Type)), Karts.Num(),
	       Config.Type == kartsim::ESessionType::Race ? *FString::Printf(TEXT(", %d laps, %s start"), Config.RaceLaps, Config.bRollingStart ? TEXT("rolling") : TEXT("standing"))
	       : (Config.Duration > 0.0 ? *FString::Printf(TEXT(", %.1f min"), Config.Duration / 60.0) : TEXT(", unlimited")));
}

std::vector<kartsim::CompetitorInput> AKartRaceWorld::CompetitorInputs()
{
	std::vector<kartsim::CompetitorInput> In;
	In.reserve(Karts.Num());
	for (FEntry& E : Karts)
	{
		const AKartVehiclePawn* K = E.Pawn.Get();
		kartsim::CompetitorInput C;
		C.Timer = &K->GetLapTimer();
		double S = 0.0, Lat = 0.0;
		if (K->GetSim() && Centre.Project(K->GetSim()->GetChassis().Position, S, Lat, E.PathHint)) { C.LapDistance = S; }
		In.push_back(C);
	}
	return In;
}

void AKartRaceWorld::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (!EnsureSetup()) { return; }
	UpdateAudioCapture(DeltaSeconds);
	// Debug: -KartShots=<t1,t2,...> renders in-game screenshots (Saved/Screenshots) at those times (s); unlike a
	// screen grab this is unaffected by other windows covering the game.
	{
		static TArray<float> ShotTimes;
		static bool bShotsParsed = false;
		static float ShotClock = 0.0f;
		if (!bShotsParsed)
		{
			bShotsParsed = true;
			FString List;
			if (FParse::Value(FCommandLine::Get(), TEXT("KartShots="), List, false))
			{
				TArray<FString> Parts;
				List.ParseIntoArray(Parts, TEXT(","));
				for (const FString& P : Parts) { ShotTimes.Add(FCString::Atof(*P)); }
			}
		}
		ShotClock += DeltaSeconds;
		if (ShotTimes.Num() > 0 && ShotClock >= ShotTimes[0])
		{
			ShotTimes.RemoveAt(0);
			if (APlayerController* ShotPC = UGameplayStatics::GetPlayerController(this, 0))
			{
				// -KartShotsUI: include Slate widgets (menus) in the capture.
				ShotPC->ConsoleCommand(FParse::Param(FCommandLine::Get(), TEXT("KartShotsUI")) ? TEXT("shot showui") : TEXT("HighResShot 1280x720"));
			}
		}
	}
	SyncKarts();
	for (const FEntry& E : Karts) { if (!E.Pawn->GetSim()) { return; } }
	const bool bAuth = HasAuthority();
	UpdateReplay(DeltaSeconds);
	if (Replay.IsPlaying()) { return; } // the race is frozen while watching

	if (bAuth)
	{
		if (const APlayerController* PC = UGameplayStatics::GetPlayerController(this, 0))
		{
			if (PC->WasInputKeyJustPressed(EKeys::U)) { CycleAIStrength(); }
		}
		// A joining or leaving player changes the field: the session restarts with everyone.
		if (Director.IsActive() && DirectorKarts.Num() != Karts.Num() && !bPending)
		{
			Pending = Director.Config();
			bPending = true;
			UE_LOG(LogKartRace, Log, TEXT("Field changed (%d karts): restarting the session"), Karts.Num());
		}
		if (bPending)
		{
			bPending = false;
			StartSession(Pending);
		}
	}
	else
	{
		if (AKartConditionsActor* Cond = AKartConditionsActor::Find(GetWorld())) { Cond->ApplyPresets(NetSession.Weather, NetSession.bRubbered); }
		// Follow the host's track layout.
		for (TActorIterator<AKartTestTrackActor> It(GetWorld()); It; ++It)
		{
			if (!NetSession.Circuit.IsEmpty() && NetSession.Circuit != It->GetLayoutFile()) { It->LoadLayout(NetSession.Circuit); }
			break;
		}
	}

	// Remote karts: kinematic proxies, posed once per frame from the interpolation buffer.
	const double Now = ServerTime();
	for (FEntry& E : Karts)
	{
		if (!E.Pawn->IsSimulatedLocally()) { E.Pawn->NetPose(Now - InterpDelay, DeltaSeconds); }
	}

	if (bAuth) { PublishSession(); } // the host's own loop below reads the same session view as clients
	const kartsim::ESessionPhase Phase = static_cast<kartsim::ESessionPhase>(NetSession.Phase);
	const bool bGrid = NetSession.bActive && Phase == kartsim::ESessionPhase::Grid;
	const double H = 1.0 / PhysicsRateHz;
	Accumulator += FMath::Min(static_cast<double>(DeltaSeconds), 0.1); // hitch guard
	const double T0 = FPlatformTime::Seconds();
	int32 Steps = 0;
	std::vector<kartsim::KartSimulation*> Sims;
	std::vector<kartsim::KartFootprint> Feet;
	std::vector<kartsim::AiTraffic> Traffic;
	for (FEntry& E : Karts)
	{
		Sims.push_back(E.Pawn->GetSimMutable());
		Feet.push_back(kartsim::KartFootprint::FromConfig(E.Pawn->GetSim()->GetConfig()));
	}
	kartsim::DriverAidSettings AllAids;
	AllAids.bAutoClutch = AllAids.bAutoShift = AllAids.bAutoPush = true;
	{
		// AI pace follows the grip of its tyres on the current line (wet track / wet set).
		const AKartConditionsActor* CA = AKartConditionsActor::Find(GetWorld());
		const double LineWet = CA && CA->IsActive() ? CA->Get().LineWetness() : 0.0;
		for (FEntry& E : Karts)
		{
			if (E.bAI && E.Pawn->GetSim()) { E.Ai.GripScale = kartsim::CompoundGripRatio(E.Pawn->GetSim()->GetConfig(), LineWet); }
		}
	}
	if (FKartLicense::Get().Blocked()) { Accumulator = 0.0; return; } // tester build without a valid license: nothing moves
	while (Accumulator >= H)
	{
		// Contacts with every kart; only locally simulated karts are stepped (a proxy is pushed on its
		// own machine).
		if (Sims.size() > 1 && kartsim::ApplyKartContacts(Sims, Feet, Contact, &Contacts, H) > 0)
		{
			++ContactSteps;
			SessionContactTime += H;
			for (const kartsim::KartContactInfo& C : Contacts) { SessionMaxContactForce = FMath::Max(SessionMaxContactForce, C.ForceOnB.Length()); }
		}
		for (kartsim::KartSimulation* S : Sims) { S->Settings.DragScale = kartsim::DraftDragScale(*S, Sims); }
		const bool bHold = bGrid && !NetSession.bRolling && (bAuth ? Director.LightsRemaining(Clock) > 0.0 : NetSession.Lights > 0.0f);
		const bool bFormation = bGrid && NetSession.bRolling;
		Clock += H;
		for (int32 I = 0; I < Karts.Num(); ++I)
		{
			FEntry& E = Karts[I];
			if (!E.Pawn->IsSimulatedLocally()) { continue; }
			if (!E.bAI)
			{
				E.Pawn->StepPhysics(H, Clock, nullptr, bHold, bFormation ? FormationSpeed + 1.0 : 1e9);
				continue;
			}
			Traffic.clear();
			for (int32 J = 0; J < Karts.Num(); ++J)
			{
				if (J != I) { Traffic.push_back({Sims[J]->GetChassis().Position, Sims[J]->GetChassis().Velocity}); }
			}
			E.Ai.ExternalSpeedCap = bFormation ? FormationSpeed : 1e9;
			kartsim::KartInputs In;
			E.Ai.Drive(*Sims[I], Traffic.data(), static_cast<int>(Traffic.size()), H, In);
			kartsim::ApplyDriverAids(*Sims[I], AllAids, In);
			E.Pawn->StepPhysics(H, Clock, &In, bHold);
		}
		if (bAuth && Director.IsActive() && DirectorKarts.Num() == Karts.Num()) { Director.Update(Clock, CompetitorInputs()); }
		Accumulator -= H;
		++Steps;
	}
	const float Ms = static_cast<float>((FPlatformTime::Seconds() - T0) * 1000.0);
	if (Steps > 0) { Replay.Record(Clock, LivePawns()); }
	for (FEntry& E : Karts)
	{
		if (E.Pawn->IsSimulatedLocally()) { E.Pawn->FinishFrame(static_cast<float>(Accumulator / H), Steps, Ms, DeltaSeconds); }
		else
		{
			if (bAuth) { E.Pawn->NetProxyTiming(Clock, DeltaSeconds); } // the host times remote karts on its clock
			E.Pawn->FinishFrame(1.0f, 0, 0.0f, DeltaSeconds);
		}
	}

	// 30 Hz state for every kart simulated here.
	NetSendTimer += DeltaSeconds;
	if (NetSendTimer >= NetSendInterval && GetNetMode() != NM_Standalone)
	{
		NetSendTimer = FMath::Fmod(NetSendTimer, NetSendInterval);
		for (FEntry& E : Karts) { E.Pawn->NetPublishState(Now); }
	}

	ContactWindow += DeltaSeconds;
	if (ContactWindow >= 1.0)
	{
		ContactsShown = ContactSteps;
		ContactSteps = 0;
		ContactWindow = 0.0;
	}
	if (bAuth)
	{
		if (Director.IsGreen() && Director.Order().size() == static_cast<size_t>(Karts.Num()))
		{
			TArray<int32> O;
			for (int X : Director.Order()) { O.Add(X); }
			if (LastOrderSeen.Num() == O.Num() && LastOrderSeen != O) { ++SessionOrderChanges; }
			LastOrderSeen = O;
		}
		PublishSession();
		LogSessionEvents();
	}
}

void AKartRaceWorld::UpdateAudioCapture(float DeltaSeconds)
{
	if (AudioCaptureLength <= 0.0f || AudioCaptureState == 2) { return; }
	AudioCaptureClock += DeltaSeconds;
	if (AudioCaptureState == 0 && AudioCaptureClock > 4.0f)
	{
		UAudioMixerBlueprintLibrary::StartRecordingOutput(this, AudioCaptureLength);
		AudioCaptureState = 1;
		AudioCaptureClock = 0.0f;
		UE_LOG(LogKartRace, Log, TEXT("AUDIO capture started (%.1f s)"), AudioCaptureLength);
	}
	else if (AudioCaptureState == 1)
	{
		if (const AKartVehiclePawn* P = Cast<AKartVehiclePawn>(UGameplayStatics::GetPlayerPawn(this, 0)); P && P->GetSim())
		{
			AudioRpmSum += P->GetSim()->GetTelemetry().EngineRpm;
			++AudioRpmCount;
		}
		if (AudioCaptureClock >= AudioCaptureLength)
		{
			const FString Dir = FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("AudioCapture"));
			UAudioMixerBlueprintLibrary::StopRecordingOutput(this, EAudioRecordingExportType::WavFile, TEXT("KartAudio"), Dir);
			AudioCaptureState = 2;
			UE_LOG(LogKartRace, Log, TEXT("AUDIO capture saved to %s/KartAudio.wav, player mean rpm %.0f (firing frequency %.1f Hz)"), *Dir,
			       AudioRpmSum / FMath::Max(AudioRpmCount, 1), AudioRpmSum / FMath::Max(AudioRpmCount, 1) / 60.0);
		}
	}
}

void AKartRaceWorld::PublishSession()
{
	FKartNetSession S;
	S.bActive = Director.IsActive();
	S.Type = static_cast<uint8>(Director.Config().Type);
	S.Phase = static_cast<uint8>(Director.Phase());
	S.bRolling = Director.Config().bRollingStart;
	S.Lights = static_cast<float>(Director.LightsRemaining(Clock));
	S.Remaining = static_cast<float>(Director.TimeRemaining(Clock));
	S.Elapsed = static_cast<float>(Director.Elapsed(Clock));
	S.RaceLaps = Director.Config().RaceLaps;
	S.AIStrength = AIStrength;
	S.ContactsPerSecond = ContactsShown;
	for (TActorIterator<AKartTestTrackActor> It(GetWorld()); It; ++It) { S.Circuit = It->GetLayoutFile(); break; }
	if (const AKartConditionsActor* Cond = AKartConditionsActor::Find(GetWorld()))
	{
		S.Weather = static_cast<uint8>(Cond->GetWeatherIndex());
		S.bRubbered = Cond->IsRubbered();
	}
	NetSession = S;

	NetStandings.Reset();
	const bool bValid = Director.IsActive() && Director.Results().size() == static_cast<size_t>(DirectorKarts.Num());
	if (!bValid)
	{
		for (const FEntry& E : Karts)
		{
			FKartNetStanding R;
			R.Kart = E.Pawn.Get();
			NetStandings.Add(R);
		}
		return;
	}
	for (int Idx : Director.Order())
	{
		const kartsim::CompetitorResult& C = Director.Results()[Idx];
		FKartNetStanding R;
		R.Kart = DirectorKarts[Idx].Get();
		R.Laps = C.Laps;
		R.BestLap = static_cast<float>(C.BestLap);
		R.Gap = static_cast<float>(C.Gap);
		R.LapsDown = C.LapsDown;
		R.bFinished = C.bFinished;
		R.FinishTime = static_cast<float>(C.FinishTime);
		R.InvalidLaps = C.InvalidLaps;
		const AKartVehiclePawn* RK = R.Kart.Get();
		const FEntry* E = Karts.FindByPredicate([RK](const FEntry& X) { return X.Pawn.Get() == RK; });
		R.AiState = (E && E->bAI) ? static_cast<uint8>(E->Ai.State()) : 255;
		NetStandings.Add(R);
	}
}

void AKartRaceWorld::LogSessionEvents()
{
	if (!Director.IsActive() || Director.Phase() == LoggedPhase) { return; }
	LoggedPhase = Director.Phase();
	switch (LoggedPhase)
	{
	case kartsim::ESessionPhase::Running: UE_LOG(LogKartRace, Log, TEXT("GREEN")); break;
	case kartsim::ESessionPhase::Chequered: UE_LOG(LogKartRace, Log, TEXT("CHEQUERED FLAG")); break;
	case kartsim::ESessionPhase::Finished:
	{
		UE_LOG(LogKartRace, Log, TEXT("SESSION RESULT %s"), UTF8_TO_TCHAR(kartsim::RaceDirector::TypeName(Director.Config().Type)));
		UE_LOG(LogKartRace, Log, TEXT("  contact %.1f s total, max contact force %.0f N, order changes %d"), SessionContactTime, SessionMaxContactForce, SessionOrderChanges);
		const bool bRace = Director.Config().Type == kartsim::ESessionType::Race;
		LastQualiOrder.Reset();
		for (int Idx : Director.Order())
		{
			const kartsim::CompetitorResult& R = Director.Results()[Idx];
			const AKartVehiclePawn* K = DirectorKarts[Idx].Get();
			UE_LOG(LogKartRace, Log, TEXT("  P%d %-12s laps %2d  best %7.3f  %s"), R.Position, K ? *K->DriverName : TEXT("?"), R.Laps, R.BestLap,
			       bRace ? *FString::Printf(TEXT("time %.3f%s"), R.FinishTime, R.LapsDown > 0 ? *FString::Printf(TEXT(" (+%d lap)"), R.LapsDown) : TEXT(""))
			             : *FString::Printf(TEXT("gap %+.3f"), R.Gap));
			LastQualiOrder.Add(DirectorKarts[Idx]);
		}
		if (Director.Config().Type != kartsim::ESessionType::Qualifying) { LastQualiOrder.Reset(); }
		break;
	}
	default: break;
	}
}


// ------------------------------------------------------------------------------- replay

void AKartRaceWorld::EndPlay(const EEndPlayReason::Type Reason)
{
	// -KartSaveReplayOnExit: automated runs keep the session as Saved/Replays/LastSession.krpl.
	if (FParse::Param(FCommandLine::Get(), TEXT("KartSaveReplayOnExit")) && Replay.HasRecording())
	{
		const FString Path = FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("Replays") / TEXT("LastSession.krpl"));
		UE_LOG(LogKartRace, Log, TEXT("Replay %s (%.0f s): %s"), Replay.SaveAs(Path) ? TEXT("saved") : TEXT("NOT saved"), Replay.RecordedSeconds(), *Path);
	}
	Super::EndPlay(Reason);
}

TArray<AKartVehiclePawn*> AKartRaceWorld::LivePawns() const
{
	TArray<AKartVehiclePawn*> Out;
	for (const FEntry& E : Karts) { if (AKartVehiclePawn* K = E.Pawn.Get()) { Out.Add(K); } }
	return Out;
}

void AKartRaceWorld::UpdateReplay(float DeltaSeconds)
{
	RunClock += DeltaSeconds;
	if (QuitAfterSeconds > 0.0f && RunClock >= QuitAfterSeconds)
	{
		QuitAfterSeconds = 0.0f;
		UE_LOG(LogKartRace, Log, TEXT("KartQuitAfter reached, exiting"));
		FPlatformMisc::RequestExit(false);
	}
	if (ReplayMessageTime > 0.0f) { ReplayMessageTime -= DeltaSeconds; if (ReplayMessageTime <= 0.0f) { ReplayMessage.Reset(); } }
	APlayerController* PC = UGameplayStatics::GetPlayerController(this, 0);
	const bool bStandalone = GetNetMode() == NM_Standalone;
	auto Say = [this](const FString& M) { ReplayMessage = M; ReplayMessageTime = 4.0f; UE_LOG(LogKartRace, Log, TEXT("%s"), *M); };

	// The circuit the recording belongs to: a layout change starts a new recording.
	FString Layout;
	for (TActorIterator<AKartTestTrackActor> It(GetWorld()); It; ++It) { Layout = It->GetLayoutFile(); break; }
	if (!Replay.IsPlaying() && Layout != Replay.GetCircuit() && ReplayFileToOpen.IsEmpty()) { Replay.Reset(Layout); }

	// -KartReplay=<file>: open once the karts exist.
	if (!ReplayFileToOpen.IsEmpty() && bStandalone)
	{
		FString Path = ReplayFileToOpen;
		ReplayFileToOpen.Reset();
		if (FPaths::IsRelative(Path)) { Path = FPaths::ProjectSavedDir() / TEXT("Replays") / Path; }
		FString Err;
		if (Replay.LoadFromFile(Path, Err))
		{
			if (FPaths::GetBaseFilename(Replay.GetCircuit()) != FPaths::GetBaseFilename(Layout))
			{
				Say(FString::Printf(TEXT("Replay is on %s, this track is %s: the karts will not match the scenery"), *Replay.GetCircuit(), *Layout));
			}
			Replay.Begin(GetWorld(), LivePawns(), PC, CentreUE, ReplayOpenAt);
		}
		else { Say(FString::Printf(TEXT("Replay not loaded: %s"), *Err)); }
	}

	if (PC && PC->WasInputKeyJustPressed(EKeys::F5))
	{
		if (Replay.IsPlaying()) { Replay.End(LivePawns(), PC); }
		else if (!bStandalone) { Say(TEXT("Replay is single-player only for now")); }
		else if (!Replay.HasRecording()) { Say(TEXT("Nothing recorded yet")); }
		else
		{
			for (AKartVehiclePawn* K : LivePawns()) { K->StopForceFeedback(); }
			Replay.Begin(GetWorld(), LivePawns(), PC, CentreUE);
		}
	}
	if (PC && PC->WasInputKeyJustPressed(EKeys::F7))
	{
		FString Path;
		Say(Replay.SaveToFile(Path) ? FString::Printf(TEXT("Replay saved (%.0f s): %s"), Replay.RecordedSeconds(), *Path) : FString(TEXT("Replay NOT saved (nothing recorded?)")));
	}
	if (Replay.IsPlaying()) { Replay.Tick(DeltaSeconds, PC); }
}
