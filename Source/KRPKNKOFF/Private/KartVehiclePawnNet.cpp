// AKartVehiclePawn multiplayer (Milestone 6). Model (see Docs/Milestone6AiMultiplayer.md):
//  * a kart is SIMULATED where it is driven: a human's kart on that human's machine, AI karts on the host;
//  * the simulating machine sends its state at 30 Hz (owner -> server RPC -> multicast to everyone);
//  * everywhere else the kart is a kinematic PROXY, interpolated 100 ms behind on the server clock;
//  * the host times every kart (proxies from their interpolated path) and runs the session.
#include "KartVehiclePawn.h"

#include "GameFramework/GameStateBase.h"
#include "KartUnrealConversions.h"
#include "Net/UnrealNetwork.h"

void AKartVehiclePawn::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const
{
	Super::GetLifetimeReplicatedProps(OutLifetimeProps);
	DOREPLIFETIME(AKartVehiclePawn, BodyColor);
	DOREPLIFETIME(AKartVehiclePawn, bIsAI);
	DOREPLIFETIME(AKartVehiclePawn, DriverName);
	DOREPLIFETIME(AKartVehiclePawn, NetKartConfig);
}

bool AKartVehiclePawn::IsSimulatedLocally() const
{
	if (GetNetMode() == NM_Standalone) { return true; }
	return bIsAI ? HasAuthority() : IsLocallyControlled();
}

void AKartVehiclePawn::NetPublishState(double ServerTime)
{
	if (!Sim || GetNetMode() == NM_Standalone || !IsSimulatedLocally()) { return; }
	// A client tells the host its class once (it may have chosen one with -KartClass).
	if (NetKartConfig != KartConfigFile)
	{
		if (HasAuthority()) { NetKartConfig = KartConfigFile; }  // host's own kart / AI
		else { ServerSetKartConfig(KartConfigFile); }
	}
	const kartsim::ChassisState& C = Sim->GetChassis();
	FKartNetState S;
	S.ServerTime = ServerTime;
	S.Position = FVector(C.Position.X, C.Position.Y, C.Position.Z);
	S.Orientation = FQuat(C.Orientation.X, C.Orientation.Y, C.Orientation.Z, C.Orientation.W);
	S.Velocity = FVector(C.Velocity.X, C.Velocity.Y, C.Velocity.Z);
	S.AngularVelocity = FVector(C.AngularVelocity.X, C.AngularVelocity.Y, C.AngularVelocity.Z);
	S.SteerLeft = static_cast<float>(Sim->GetWheel(kartsim::WheelFL).SteerAngle);
	S.SteerRight = static_cast<float>(Sim->GetWheel(kartsim::WheelFR).SteerAngle);
	S.OmegaFront = static_cast<float>(Sim->GetWheel(kartsim::WheelFL).Omega);
	S.OmegaRear = static_cast<float>(Sim->GetWheel(kartsim::WheelRL).Omega);
	S.bAllWheelsOff = bLastAllOff;
	bLastAllOff = false;
	if (HasAuthority()) { MulticastKartState(S); }
	else { ServerKartState(S); }
}

void AKartVehiclePawn::ServerKartState_Implementation(const FKartNetState& State)
{
	MulticastKartState(State); // runs on the host too (its proxy of this kart) and on every client
}

void AKartVehiclePawn::MulticastKartState_Implementation(const FKartNetState& State)
{
	if (IsSimulatedLocally()) { return; } // our own kart: we are the source
	if (NetBuffer.Num() > 0 && State.ServerTime <= NetBuffer.Last().ServerTime) { return; } // late/out of order
	NetBuffer.Add(State);
	if (NetBuffer.Num() > 64) { NetBuffer.RemoveAt(0, NetBuffer.Num() - 64); }
}

bool AKartVehiclePawn::NetPose(double RenderTime, double Dt)
{
	if (!Sim || NetBuffer.Num() == 0) { return false; }
	// Bracketing states; beyond the newest, extrapolate for at most 0.25 s (then hold).
	int32 B = 0;
	while (B < NetBuffer.Num() && NetBuffer[B].ServerTime < RenderTime) { ++B; }
	const FKartNetState& Hi = NetBuffer[FMath::Min(B, NetBuffer.Num() - 1)];
	const FKartNetState& Lo = NetBuffer[FMath::Max(B - 1, 0)];
	double Alpha = 0.0;
	double Extra = 0.0;
	if (B >= NetBuffer.Num()) { Extra = FMath::Min(RenderTime - Hi.ServerTime, 0.25); }
	else if (Hi.ServerTime > Lo.ServerTime) { Alpha = (RenderTime - Lo.ServerTime) / (Hi.ServerTime - Lo.ServerTime); }
	Alpha = FMath::Clamp(Alpha, 0.0, 1.0);
	const FKartNetState& Base = B >= NetBuffer.Num() ? Hi : Lo;
	const FVector Pos = B >= NetBuffer.Num() ? Hi.Position + FVector(Hi.Velocity) * Extra : FMath::Lerp(Lo.Position, Hi.Position, Alpha);
	const FQuat Rot = B >= NetBuffer.Num() ? Hi.Orientation : FQuat::Slerp(Lo.Orientation, Hi.Orientation, Alpha);
	const FVector Vel = FMath::Lerp(FVector(Base.Velocity), FVector(Hi.Velocity), Alpha);
	const FVector Ang = FMath::Lerp(FVector(Base.AngularVelocity), FVector(Hi.AngularVelocity), Alpha);
	PrevSnap = CurrSnap;
	Sim->SetKinematicPose(kartsim::Vec3(Pos.X, Pos.Y, Pos.Z), kartsim::Quat(Rot.W, Rot.X, Rot.Y, Rot.Z), kartsim::Vec3(Vel.X, Vel.Y, Vel.Z),
	                      kartsim::Vec3(Ang.X, Ang.Y, Ang.Z), FMath::Lerp(Base.SteerLeft, Hi.SteerLeft, Alpha), FMath::Lerp(Base.SteerRight, Hi.SteerRight, Alpha),
	                      FMath::Lerp(Base.OmegaFront, Hi.OmegaFront, Alpha), FMath::Lerp(Base.OmegaRear, Hi.OmegaRear, Alpha), Dt);
	bProxyAllOff = Hi.bAllWheelsOff;
	CurrSnap = Capture();
	SetActorLocation(CurrSnap.Position); // keeps the actor (relevancy, cameras) with the kart
	// Drop states older than the bracket.
	if (B > 2) { NetBuffer.RemoveAt(0, B - 2); }
	return true;
}

void AKartVehiclePawn::NetProxyTiming(double Clock, double Dt)
{
	if (!Sim || !bTimingConfigured) { return; }
	const kartsim::Vec3 Pos = Sim->GetChassis().Position;
	ClockNow = Clock;
	if (ProxyTimingHold > 0.0)
	{
		ProxyTimingHold -= Dt;
		bProxyHasPrev = false;
	}
	else if (bProxyHasPrev)
	{
		LapTimer.Update(Clock, Dt, ProxyPrevPos, Pos, bProxyAllOff); // track limits as judged on the driver's machine
	}
	ProxyPrevPos = Pos;
	bProxyHasPrev = true;
}

void AKartVehiclePawn::NetPlaceAt(const FTransform& Where)
{
	if (IsSimulatedLocally())
	{
		PlaceAt(Where);
		return;
	}
	ClientPlaceAt(Where);
	LapTimer.Reset();
	NetBuffer.Reset();
	bProxyHasPrev = false;
	ProxyTimingHold = 0.6; // the teleport must not count as a line crossing
}

void AKartVehiclePawn::ClientPlaceAt_Implementation(const FTransform& Where)
{
	PlaceAt(Where);
}

void AKartVehiclePawn::ServerSetKartConfig_Implementation(const FString& File)
{
	NetKartConfig = File;
	OnRep_NetKartConfig(); // the host's proxy of this kart
}

void AKartVehiclePawn::OnRep_NetKartConfig()
{
	if (IsSimulatedLocally() || NetKartConfig.IsEmpty() || NetKartConfig == KartConfigFile) { return; }
	const int32 Index = AvailableKarts.IndexOfByKey(NetKartConfig);
	if (Index != INDEX_NONE) { SwitchKart(Index); }
}
