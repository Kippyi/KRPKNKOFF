// Multiplayer message and replication types (Milestone 6). Kart states are in SIM coordinates (metres,
// ISO 8855) so no conversion error accumulates; the receiver poses a kinematic proxy from them.
#pragma once

#include "CoreMinimal.h"
#include "Engine/NetSerialization.h"
#include "KartNetTypes.generated.h"

class AKartVehiclePawn;

USTRUCT()
struct FKartNetState
{
	GENERATED_BODY()

	UPROPERTY() double ServerTime = 0.0;            // sender's estimate of the server clock (s)
	UPROPERTY() FVector Position = FVector::ZeroVector; // sim, m
	UPROPERTY() FQuat Orientation = FQuat::Identity;  // sim quaternion (X, Y, Z, W)
	UPROPERTY() FVector_NetQuantize100 Velocity;      // sim world, m/s
	UPROPERTY() FVector_NetQuantize100 AngularVelocity; // body, rad/s
	UPROPERTY() float SteerLeft = 0.0f;              // rad
	UPROPERTY() float SteerRight = 0.0f;
	UPROPERTY() float OmegaFront = 0.0f;             // rad/s
	UPROPERTY() float OmegaRear = 0.0f;
	UPROPERTY() bool bAllWheelsOff = false;          // track limits, judged on the driver's machine
};

// Host -> everyone: the session (replicated on AKartRaceWorld).
USTRUCT()
struct FKartNetSession
{
	GENERATED_BODY()

	UPROPERTY() bool bActive = false;
	UPROPERTY() uint8 Type = 0;      // kartsim::ESessionType
	UPROPERTY() uint8 Phase = 3;     // kartsim::ESessionPhase
	UPROPERTY() bool bRolling = false;
	UPROPERTY() float Lights = 0.0f;
	UPROPERTY() float Remaining = 0.0f;
	UPROPERTY() float Elapsed = 0.0f;
	UPROPERTY() int32 RaceLaps = 0;
	UPROPERTY() uint8 Weather = 0;   // AKartConditionsActor preset
	UPROPERTY() bool bRubbered = false;
	UPROPERTY() float AIStrength = 0.9f;
	UPROPERTY() int32 ContactsPerSecond = 0;
	UPROPERTY() FString Circuit;       // host's track layout file (clients rebuild their track to match)
};

// One row of the classification, in position order.
USTRUCT()
struct FKartNetStanding
{
	GENERATED_BODY()

	UPROPERTY() TObjectPtr<AKartVehiclePawn> Kart = nullptr;
	UPROPERTY() int32 Laps = 0;
	UPROPERTY() float BestLap = 0.0f;
	UPROPERTY() float Gap = 0.0f;
	UPROPERTY() int32 LapsDown = 0;
	UPROPERTY() bool bFinished = false;
	UPROPERTY() float FinishTime = 0.0f;
	UPROPERTY() int32 InvalidLaps = 0;
	UPROPERTY() uint8 AiState = 255;  // kartsim::EAiState, 255 = human
};
