#include "KartGameInstance.h"
#include "KartGraphicsSettings.h"
#include "KartLicense.h"

#include "Engine/World.h"
#include "Interfaces/OnlineIdentityInterface.h"
#include "Online/OnlineSessionNames.h"
#include "OnlineSessionSettings.h"
#include "OnlineSubsystem.h"
#include "OnlineSubsystemNames.h"
#include "Misc/Paths.h"
#include "GameFramework/PlayerController.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

UKartGameInstance* UKartGameInstance::Get(const UObject* Ctx)
{
	return Ctx ? Cast<UKartGameInstance>(UGameplayStatics::GetGameInstance(Ctx)) : nullptr;
}

bool UKartGameInstance::HasScriptedLaunch()
{
	static const TCHAR* Flags[] = {TEXT("KartAutopilot="), TEXT("KartSession="), TEXT("KartAI="), TEXT("KartQuitAfterLaps="), TEXT("KartPlayerAI"),
	                               TEXT("KartSurfaceSelfTest"), TEXT("KartNoMenu")};
	const TCHAR* Cmd = FCommandLine::Get();
	for (const TCHAR* F : Flags) { if (FCString::Stristr(Cmd, F)) { return true; } }
	return false;
}

bool UKartGameInstance::IsFrontEnd(const UObject* Ctx)
{
	const UKartGameInstance* GI = Get(Ctx);
	const UWorld* W = Ctx ? Ctx->GetWorld() : nullptr;
	return GI && W && !GI->bLaunchedFromMenu && !HasScriptedLaunch() && W->GetNetMode() == NM_Standalone;
}

void UKartGameInstance::StartSession(const UObject* Ctx, bool bHost)
{
	bLaunchedFromMenu = true;
	UGameplayStatics::OpenLevel(Ctx, FName(TEXT("/Engine/Maps/Entry")), true, bHost ? TEXT("listen") : TEXT(""));
}

void UKartGameInstance::JoinGame(const UObject* Ctx)
{
	bLaunchedFromMenu = true;
	if (APlayerController* PC = UGameplayStatics::GetPlayerController(Ctx, 0))
	{
		PC->ClientTravel(JoinAddress, TRAVEL_Absolute);
	}
}


// =============================================================================== online (Steam)
// Lobby keys: our game tag and build date filter out everything else on the shared test app ID 480 and hosts on
// other builds (a different build would desync).
namespace
{
	const FName KeyGame(TEXT("KRPK_GAME"));
	const FName KeyBuild(TEXT("KRPK_BUILD"));
	const FName KeyName(TEXT("KRPK_NAME"));
	const FName KeyTrack(TEXT("KRPK_TRACK"));
	const FName KeyClass(TEXT("KRPK_CLASS"));
	const FString GameTag(TEXT("KRPKNKOFF"));
	FString BuildTag() { return FString(ANSI_TO_TCHAR(__DATE__)).Replace(TEXT(" "), TEXT("")); }
	FString PrettyClass(const FString& File)
	{
		if (File.Contains(TEXT("KZ"))) { return TEXT("KZ2"); }
		if (File.Contains(TEXT("KT100"))) { return TEXT("KT100"); }
		return TEXT("Rotax Max");
	}
}

void UKartGameInstance::Init()
{
	Super::Init();
	FKartGraphicsSettings::Get().Apply(); // first launch: hardware benchmark -> preset
	FKartLicense::Get();                  // machine code, stored key (logged to KartTester.log)
	if (IOnlineSessionPtr S = Sessions())
	{
		// Steam overlay "Join game" / accepted invites.
		InviteHandle = S->AddOnSessionUserInviteAcceptedDelegate_Handle(FOnSessionUserInviteAcceptedDelegate::CreateUObject(this, &UKartGameInstance::OnInviteAccepted));
	}
	OnlineStatus = IsSteamOnline() ? FString::Printf(TEXT("Steam: signed in as %s"), *GetLocalPlayerName())
	                               : FString(TEXT("Steam is not running: start Steam and restart the game for the server list (direct IP still works)"));
	UE_LOG(LogTemp, Log, TEXT("Online: %s"), *OnlineStatus);
	KartTesterLog(FString::Printf(TEXT("Start: %s, graphics %s"), *OnlineStatus, FKartGraphicsSettings::PresetName(FKartGraphicsSettings::Get().Preset)));
}

void UKartGameInstance::Shutdown()
{
	LeaveOnlineSession();
	Super::Shutdown();
}

IOnlineSessionPtr UKartGameInstance::Sessions() const
{
	IOnlineSubsystem* OSS = IOnlineSubsystem::Get();
	return OSS ? OSS->GetSessionInterface() : nullptr;
}

bool UKartGameInstance::IsSteamOnline() const
{
	IOnlineSubsystem* OSS = IOnlineSubsystem::Get();
	if (!OSS || OSS->GetSubsystemName() != STEAM_SUBSYSTEM) { return false; }
	IOnlineIdentityPtr Id = OSS->GetIdentityInterface();
	return Id.IsValid() && Id->GetLoginStatus(0) == ELoginStatus::LoggedIn;
}

FString UKartGameInstance::GetLocalPlayerName() const
{
	// Steam persona name; without Steam the fallback subsystem's nickname is a machine id (and the PC name): not shown.
	if (!IsSteamOnline()) { return TEXT("Player"); }
	IOnlineIdentityPtr Id = IOnlineSubsystem::Get()->GetIdentityInterface();
	const FString N = Id.IsValid() ? Id->GetPlayerNickname(0) : FString();
	return N.IsEmpty() ? FString(TEXT("Player")) : N;
}

void UKartGameInstance::HostOnline(const UObject* Ctx)
{
	IOnlineSessionPtr S = Sessions();
	HostCtx = Ctx;
	if (!S.IsValid()) { OnlineStatus = TEXT("No online subsystem: hosting for direct IP only"); StartSession(Ctx, true); return; }
	if (S->GetNamedSession(NAME_GameSession)) { S->DestroySession(NAME_GameSession); }
	const bool bSteam = IsSteamOnline();
	FOnlineSessionSettings Set;
	Set.NumPublicConnections = 8;
	Set.bShouldAdvertise = true;
	Set.bAllowJoinInProgress = true;
	Set.bIsLANMatch = !bSteam;
	Set.bUsesPresence = bSteam;
	Set.bUseLobbiesIfAvailable = bSteam;
	Set.bAllowJoinViaPresence = bSteam;
	Set.bAllowInvites = true;
	const FString Name = ServerName.TrimStartAndEnd().IsEmpty() ? FString::Printf(TEXT("%s's server"), *GetLocalPlayerName()) : ServerName.TrimStartAndEnd().Left(40);
	Set.Set(KeyGame, GameTag, EOnlineDataAdvertisementType::ViaOnlineService);
	Set.Set(KeyBuild, BuildTag(), EOnlineDataAdvertisementType::ViaOnlineService);
	Set.Set(KeyName, Name, EOnlineDataAdvertisementType::ViaOnlineService);
	Set.Set(KeyTrack, FPaths::GetBaseFilename(Launch.Circuit), EOnlineDataAdvertisementType::ViaOnlineService);
	Set.Set(KeyClass, PrettyClass(Launch.KartClass), EOnlineDataAdvertisementType::ViaOnlineService);
	S->ClearOnCreateSessionCompleteDelegate_Handle(CreateHandle);
	CreateHandle = S->AddOnCreateSessionCompleteDelegate_Handle(FOnCreateSessionCompleteDelegate::CreateUObject(this, &UKartGameInstance::OnCreateSessionComplete));
	OnlineStatus = TEXT("Creating the server...");
	if (!S->CreateSession(0, NAME_GameSession, Set))
	{
		OnlineStatus = TEXT("Could not create the server (see log); hosting for direct IP only");
		StartSession(Ctx, true);
	}
}

void UKartGameInstance::OnCreateSessionComplete(FName Session, bool bOk)
{
	if (IOnlineSessionPtr S = Sessions()) { S->ClearOnCreateSessionCompleteDelegate_Handle(CreateHandle); }
	OnlineStatus = bOk ? FString(TEXT("Server is listed: others can join from their server list")) : FString(TEXT("Server listing failed; hosting for direct IP only"));
	UE_LOG(LogTemp, Log, TEXT("Online: create session %s"), bOk ? TEXT("OK") : TEXT("FAILED"));
	StartSession(HostCtx.IsValid() ? HostCtx.Get() : this, true);
}

void UKartGameInstance::RefreshServers()
{
	IOnlineSessionPtr S = Sessions();
	if (!S.IsValid() || bSearching) { return; }
	Search = MakeShared<FOnlineSessionSearch>();
	Search->MaxSearchResults = 200;
	Search->bIsLanQuery = !IsSteamOnline();
	Search->QuerySettings.Set(SEARCH_LOBBIES, true, EOnlineComparisonOp::Equals);
	Search->QuerySettings.Set(KeyGame, GameTag, EOnlineComparisonOp::Equals);
	S->ClearOnFindSessionsCompleteDelegate_Handle(FindHandle);
	FindHandle = S->AddOnFindSessionsCompleteDelegate_Handle(FOnFindSessionsCompleteDelegate::CreateUObject(this, &UKartGameInstance::OnFindSessionsComplete));
	bSearching = true;
	OnlineStatus = TEXT("Searching...");
	if (!S->FindSessions(0, Search.ToSharedRef())) { bSearching = false; OnlineStatus = TEXT("Search failed to start"); }
}

void UKartGameInstance::OnFindSessionsComplete(bool bOk)
{
	if (IOnlineSessionPtr S = Sessions()) { S->ClearOnFindSessionsCompleteDelegate_Handle(FindHandle); }
	bSearching = false;
	Servers.Reset();
	int32 OtherBuilds = 0;
	if (bOk && Search.IsValid())
	{
		for (int32 I = 0; I < Search->SearchResults.Num(); ++I)
		{
			const FOnlineSessionSearchResult& R = Search->SearchResults[I];
			const FOnlineSessionSettings& Set = R.Session.SessionSettings;
			FString Game, Build;
			Set.Get(KeyGame, Game);
			Set.Get(KeyBuild, Build);
			if (Game != GameTag) { continue; }                  // somebody else's test app on 480
			if (Build != BuildTag()) { ++OtherBuilds; continue; } // incompatible build
			FKartServerInfo Info;
			Set.Get(KeyName, Info.Name);
			Set.Get(KeyTrack, Info.Track);
			Set.Get(KeyClass, Info.KartClass);
			Info.Host = R.Session.OwningUserName;
			Info.MaxPlayers = Set.NumPublicConnections;
			Info.Players = FMath::Clamp(Set.NumPublicConnections - R.Session.NumOpenPublicConnections, 0, Set.NumPublicConnections);
			Info.PingMs = R.PingInMs > 0 && R.PingInMs < 9999 ? R.PingInMs : -1;
			Info.ResultIndex = I;
			Servers.Add(Info);
		}
	}
	++ServersVersion;
	const FString Prefix = IsSteamOnline() ? FString() : FString(TEXT("Steam is not running (LAN only; start Steam and restart for online servers).  "));
	OnlineStatus = Prefix + (!bOk ? FString(TEXT("Search failed"))
	             : FString::Printf(TEXT("%d server%s found%s"), Servers.Num(), Servers.Num() == 1 ? TEXT("") : TEXT("s"),
	                               OtherBuilds ? *FString::Printf(TEXT("  (%d on another game version hidden)"), OtherBuilds) : TEXT("")));
	UE_LOG(LogTemp, Log, TEXT("Online: %s"), *OnlineStatus);
}

void UKartGameInstance::JoinServer(int32 ListIndex)
{
	if (!Servers.IsValidIndex(ListIndex) || !Search.IsValid() || !Search->SearchResults.IsValidIndex(Servers[ListIndex].ResultIndex)) { return; }
	JoinResult(Search->SearchResults[Servers[ListIndex].ResultIndex]);
}

void UKartGameInstance::JoinResult(const FOnlineSessionSearchResult& Result)
{
	IOnlineSessionPtr S = Sessions();
	if (!S.IsValid()) { return; }
	if (S->GetNamedSession(NAME_GameSession)) { S->DestroySession(NAME_GameSession); }
	S->ClearOnJoinSessionCompleteDelegate_Handle(JoinHandle);
	JoinHandle = S->AddOnJoinSessionCompleteDelegate_Handle(FOnJoinSessionCompleteDelegate::CreateUObject(this, &UKartGameInstance::OnJoinSessionComplete));
	OnlineStatus = TEXT("Joining...");
	S->JoinSession(0, NAME_GameSession, Result);
}

void UKartGameInstance::OnJoinSessionComplete(FName Session, EOnJoinSessionCompleteResult::Type Result)
{
	IOnlineSessionPtr S = Sessions();
	if (S.IsValid()) { S->ClearOnJoinSessionCompleteDelegate_Handle(JoinHandle); }
	FString Url;
	if (Result != EOnJoinSessionCompleteResult::Success || !S.IsValid() || !S->GetResolvedConnectString(Session, Url))
	{
		OnlineStatus = FString::Printf(TEXT("Join failed (%s)"), LexToString(Result));
		UE_LOG(LogTemp, Warning, TEXT("Online: %s"), *OnlineStatus);
		return;
	}
	UE_LOG(LogTemp, Log, TEXT("Online: joining %s"), *Url);
	OnlineStatus = TEXT("Connecting...");
	bLaunchedFromMenu = true;
	if (APlayerController* PC = GetFirstLocalPlayerController()) { PC->ClientTravel(Url, TRAVEL_Absolute); }
}

void UKartGameInstance::OnInviteAccepted(bool bOk, int32 LocalUser, FUniqueNetIdPtr User, const FOnlineSessionSearchResult& Result)
{
	UE_LOG(LogTemp, Log, TEXT("Online: invite / Steam join accepted (%s)"), bOk ? TEXT("ok") : TEXT("invalid"));
	if (bOk && Result.IsValid()) { JoinResult(Result); }
}

void UKartGameInstance::LeaveOnlineSession()
{
	IOnlineSessionPtr S = Sessions();
	if (S.IsValid() && S->GetNamedSession(NAME_GameSession)) { S->DestroySession(NAME_GameSession); }
}
