// UKartGameInstance: survives level loads, so it carries the main menu's choices into the session it
// starts. Command-line flags (used by the automated tests) still take priority over these.
#pragma once

#include "CoreMinimal.h"
#include "Engine/GameInstance.h"
#include "Interfaces/OnlineSessionInterface.h"
#include "KartGameInstance.generated.h"

USTRUCT()
struct FKartLaunchSettings
{
	GENERATED_BODY()

	UPROPERTY() FString KartClass = TEXT("Rotax125MaxSenior.json");
	UPROPERTY() int32 NumAI = 5;
	UPROPERTY() float AIStrength = 0.9f;
	UPROPERTY() int32 Weather = 0;          // AKartConditionsActor preset index
	UPROPERTY() bool bRubbered = false;
	UPROPERTY() int32 Session = 0;          // 0 practice, 1 qualifying, 2 race
	UPROPERTY() int32 RaceLaps = 10;
	UPROPERTY() float QualiMinutes = 8.0f;
	UPROPERTY() FString Circuit = TEXT("RiversideKartPark.json"); // Content/KartData/Tracks
};

// One entry of the online server list (a Steam lobby advertised by a hosting player).
struct FKartServerInfo
{
	FString Name, Track, KartClass, Host;
	int32 Players = 0, MaxPlayers = 0, PingMs = -1; // ping -1 = unknown (Steam lobbies do not report one)
	int32 ResultIndex = -1;
};

UCLASS()
class KRPKNKOFF_API UKartGameInstance : public UGameInstance
{
	GENERATED_BODY()

public:
	FKartLaunchSettings Launch;
	bool bLaunchedFromMenu = false;   // the current level was started by the menu (apply Launch)
	FString JoinAddress = TEXT("127.0.0.1");

	static UKartGameInstance* Get(const UObject* Ctx);
	// True when the process was started by a test / script with explicit kart flags: no menu then.
	static bool HasScriptedLaunch();
	// The game sitting at the main menu (not a session the menu started, not a scripted test, single player):
	// a quiet showroom scene, no AI, karts silent.
	static bool IsFrontEnd(const UObject* Ctx);
	// (Re)starts the track with Launch applied; bHost opens it as a listen server.
	void StartSession(const UObject* Ctx, bool bHost);
	void JoinGame(const UObject* Ctx);

	// --- Online (Steam lobbies + Steam relay networking: no port forwarding). Falls back to LAN / direct IP when
	// Steam is not running. ---
	virtual void Init() override;
	virtual void Shutdown() override;
	FString ServerName;                                  // host: name shown in the list (default "<name>'s server")
	void HostOnline(const UObject* Ctx);                 // advertise a lobby, then open the track as a listen server
	void RefreshServers();
	void JoinServer(int32 ListIndex);
	void LeaveOnlineSession();                           // back to the menu / quit
	const TArray<FKartServerInfo>& GetServers() const { return Servers; }
	int32 GetServersVersion() const { return ServersVersion; }
	bool IsSearching() const { return bSearching; }
	const FString& GetOnlineStatus() const { return OnlineStatus; }
	bool IsSteamOnline() const;                          // Steam client running and logged in
	FString GetLocalPlayerName() const;

private:
	IOnlineSessionPtr Sessions() const;
	void OnCreateSessionComplete(FName Session, bool bOk);
	void OnFindSessionsComplete(bool bOk);
	void OnJoinSessionComplete(FName Session, EOnJoinSessionCompleteResult::Type Result);
	void OnInviteAccepted(bool bOk, int32 LocalUser, FUniqueNetIdPtr User, const FOnlineSessionSearchResult& Result);
	void JoinResult(const FOnlineSessionSearchResult& Result);
	TSharedPtr<FOnlineSessionSearch> Search;
	TArray<FKartServerInfo> Servers;
	int32 ServersVersion = 0;
	bool bSearching = false;
	FString OnlineStatus;
	TWeakObjectPtr<const UObject> HostCtx;
	FDelegateHandle CreateHandle, FindHandle, JoinHandle, InviteHandle;
};
