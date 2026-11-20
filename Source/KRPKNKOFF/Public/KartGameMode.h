// Milestone-1 game mode: ensures a test track, lighting and a player start exist, then spawns
// the kart pawn with the telemetry HUD. Works on an empty map.
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "KartGameMode.generated.h"

UCLASS()
class KRPKNKOFF_API AKartGameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	AKartGameMode();
	virtual void InitGame(const FString& MapName, const FString& Options, FString& ErrorMessage) override;

	// Track, lighting, sky, conditions: local (non-replicated) actors that every machine creates for itself
	// (multiplayer clients have no GameMode and call this from the kart pawn). bServer also spawns the
	// replicated race world and a player start.
	static void EnsureEnvironment(UWorld* World, bool bServer);
};
