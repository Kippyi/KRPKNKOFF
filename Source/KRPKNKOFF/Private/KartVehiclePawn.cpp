#include "KartVehiclePawn.h"

#include "KartAudioComponent.h"
#include "Misc/ConfigCacheIni.h"
#include "KartWheelInput.h"
#include "Framework/Application/SlateApplication.h"
#include "Widgets/SWindow.h"
#include "KartConditionsActor.h"
#include "KartGameInstance.h"
#include "KartGameMode.h"
#include "KartRaceWorld.h"

#include "Camera/CameraComponent.h"
#include "Components/StaticMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/HitResult.h"
#include "Engine/LocalPlayer.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/SpringArmComponent.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformTime.h"
#include "InputAction.h"
#include "InputMappingContext.h"
#include "InputModifiers.h"
#include "KartRealismSettings.h"
#include "KartDash.h"
#include "KartLicense.h"
#include "KartSim/DriverAids.h"
#include "KartSim/Json.h"
#include "KartSim/Setup.h"
#include "Misc/FileHelper.h"
#include "EngineUtils.h"
#include "Kismet/GameplayStatics.h"
#include "KartTestTrackActor.h"
#include "Engine/StaticMeshActor.h"
#include "KartProceduralVisuals.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "KartUnrealConversions.h"
#include "ProceduralMeshComponent.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Engine/Texture.h"
#include "Misc/DateTime.h"
#include "Misc/Paths.h"
#include "UObject/ConstructorHelpers.h"

DEFINE_LOG_CATEGORY_STATIC(LogKart, Log, All);

namespace
{
	UProceduralMeshComponent* MakeVisualComponent(AKartVehiclePawn* Owner, USceneComponent* Parent, const TCHAR* Name, bool bAbsolute)
	{
		UProceduralMeshComponent* P = Owner->CreateDefaultSubobject<UProceduralMeshComponent>(Name);
		P->SetupAttachment(Parent);
		P->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		if (bAbsolute)
		{
			P->SetUsingAbsoluteLocation(true);
			P->SetUsingAbsoluteRotation(true);
			P->SetUsingAbsoluteScale(true);
		}
		return P;
	}
}

AKartVehiclePawn::AKartVehiclePawn()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PrePhysics;
	// No auto-possession: the GameMode possesses each player's default pawn. (Auto-possessing Player0
	// made every AI kart spawned later steal the local player's controller and camera.)
	AutoPossessPlayer = EAutoReceiveInput::Disabled;
	// Multiplayer: the kart's motion travels as 30 Hz state RPCs, not engine movement replication.
	bReplicates = true;
	SetReplicateMovement(false);
	bAlwaysRelevant = true;

	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	CreateDefaultSubobject<UKartAudioComponent>(TEXT("KartAudio")); // synthesised engine / tyre / contact sound
	SetRootComponent(Root);

	static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeFinder(TEXT("/Engine/BasicShapes/Cube.Cube"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> CylFinder(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereFinder(TEXT("/Engine/BasicShapes/Sphere.Sphere"));
	static ConstructorHelpers::FObjectFinder<UMaterialInterface> MatFinder(TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	CubeMesh = CubeFinder.Object;
	CylinderMesh = CylFinder.Object;
	SphereMesh = SphereFinder.Object;
	BaseMaterial = MatFinder.Object;

	FrameMesh = MakeVisualComponent(this, Root, TEXT("Frame"), false);
	BodyMesh = MakeVisualComponent(this, Root, TEXT("Bodywork"), false);
	DriverMesh = MakeVisualComponent(this, Root, TEXT("Driver"), false);
	TorsoMesh = MakeVisualComponent(this, Root, TEXT("DriverTorso"), false);
	{
		static const TCHAR* ArmNames[] = {TEXT("UpperArmL"), TEXT("UpperArmR"), TEXT("ForearmL"), TEXT("ForearmR"), TEXT("HandL"), TEXT("HandR")};
		for (const TCHAR* N : ArmNames) { ArmMeshes.Add(MakeVisualComponent(this, Root, N, false)); }
	}
	HelmetMesh = MakeVisualComponent(this, Root, TEXT("Helmet"), false);
	SteeringWheelMesh = MakeVisualComponent(this, Root, TEXT("SteeringWheel"), false);
	AxleMesh = MakeVisualComponent(this, Root, TEXT("RearAxle"), true);
	UprightFL = CreateDefaultSubobject<USceneComponent>(TEXT("UprightFL"));
	UprightFL->SetupAttachment(Root);
	UprightFR = CreateDefaultSubobject<USceneComponent>(TEXT("UprightFR"));
	UprightFR->SetupAttachment(Root);
	RearAxlePivot = CreateDefaultSubobject<USceneComponent>(TEXT("RearAxlePivot"));
	RearAxlePivot->SetupAttachment(Root);
	static const TCHAR* WheelNames[] = {TEXT("WheelFL"), TEXT("WheelFR"), TEXT("WheelRL"), TEXT("WheelRR")};
	for (int32 I = 0; I < kartsim::NumWheels; ++I)
	{
		WheelMeshes.Add(MakeVisualComponent(this, Root, WheelNames[I], true));
	}

	ChaseArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("ChaseArm"));
	ChaseArm->SetupAttachment(Root);
	ChaseArm->TargetArmLength = 380.0f;
	ChaseArm->SocketOffset = FVector(0.0f, 0.0f, 90.0f);
	ChaseArm->SetRelativeRotation(FRotator(-10.0f, 0.0f, 0.0f));
	ChaseArm->bDoCollisionTest = false;
	ChaseArm->bInheritRoll = false;
	ChaseArm->bInheritPitch = false;
	ChaseArm->bEnableCameraRotationLag = true;
	ChaseArm->CameraRotationLagSpeed = 8.0f;

	ChaseCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("ChaseCamera"));
	ChaseCamera->SetupAttachment(ChaseArm);
	ChaseCamera->FieldOfView = 75.0f;

	CockpitCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("CockpitCamera"));
	CockpitCamera->SetupAttachment(Root);
	CockpitCamera->FieldOfView = 85.0f;
	CockpitCamera->SetAutoActivate(false);
}

void AKartVehiclePawn::BeginPlay()
{
	Super::BeginPlay();
	// Multiplayer clients have no GameMode: create the local environment (track, sky, conditions).
	if (GetNetMode() == NM_Client) { AKartGameMode::EnsureEnvironment(GetWorld(), false); }
	FfbDevice = MakeUnique<FKartNullFFBDevice>();

	TrackQuery.Setup(GetWorld(), this);
	if (bPuppet)
	{
		if (InitSimulation()) { BuildVisuals(); ResetKart(); }
		SetActorEnableCollision(false);
		return;
	}
	// Optional command-line class selection: -KartClass=KZ2_IAMEScreamer.json
	FString CmdClass;
	if (!bIsAI && FParse::Value(FCommandLine::Get(), TEXT("KartClass="), CmdClass) && !CmdClass.IsEmpty())
	{
		KartConfigFile = CmdClass;
	}
	else if (const UKartGameInstance* GI = UKartGameInstance::Get(this); !bIsAI && GI && GI->bLaunchedFromMenu)
	{
		KartConfigFile = GI->Launch.KartClass; // chosen in the main menu (remote karts are corrected below / by replication)
	}
	if (GetNetMode() == NM_Client && !NetKartConfig.IsEmpty())
	{
		KartConfigFile = NetKartConfig; // another player's (or AI) kart: its class as replicated by the host
	}
	if (InitSimulation())
	{
		BuildVisuals();
		ResetKart();
	}
	ConfigureLapTiming();
	AKartTestTrackActor::OnLayoutChanged.AddWeakLambda(this, [this]() { ConfigureLapTiming(); }); // client following the host's layout
	if (AKartConditionsActor* C = AKartConditionsActor::Find(GetWorld()))
	{
		Conditions = C;
		CondQuery = MakeUnique<kartsim::ConditionsTrackQuery>(TrackQuery, C->Get());
	}
	if (bIsAI)
	{
		return; // driven by AKartRaceWorld: no command-line options, input or camera
	}
	if (FParse::Param(FCommandLine::Get(), TEXT("KartShowTelemetry"))) { bShowTelemetry = true; }
	if (GConfig)
	{
		GConfig->GetFloat(TEXT("KartCamera"), TEXT("CockpitPitch"), CockpitPitch, GGameUserSettingsIni);
		GConfig->GetFloat(TEXT("KartCamera"), TEXT("CockpitFov"), CockpitFov, GGameUserSettingsIni);
		CockpitCamera->SetRelativeRotation(FRotator(CockpitPitch, 0.0f, 0.0f));
		CockpitCamera->FieldOfView = CockpitFov;
	}
	if (FParse::Param(FCommandLine::Get(), TEXT("KartRecord"))) { ToggleRecording(); }
	const bool bFrontEnd = UKartGameInstance::IsFrontEnd(this);
	if (bFrontEnd) { ShowroomDistance = 440.0f; ShowroomPitch = -9.0f; }
	if ((FParse::Value(FCommandLine::Get(), TEXT("KartShowroom="), ShowroomDistance) && ShowroomDistance > 0.0f) || bFrontEnd)
	{
		ChaseArm->TargetArmLength = ShowroomDistance;
		ChaseArm->SocketOffset = FVector::ZeroVector;
		ChaseArm->TargetOffset = FVector(0.0f, 0.0f, 10.0f);
		ChaseArm->bEnableCameraRotationLag = false;
		ChaseArm->bEnableCameraLag = false;
		ShowroomYaw = 150.0f;
		if (FParse::Value(FCommandLine::Get(), TEXT("KartShowroomYaw="), ShowroomYaw)) { ShowroomSpin = 0.0f; }
		FParse::Value(FCommandLine::Get(), TEXT("KartShowroomPitch="), ShowroomPitch);
		if (bFrontEnd)
		{
			// Slow turntable behind the main menu; the camera sits a little to the side so the kart is framed in the
			// open area right of the menu rail.
			ShowroomSpin = 6.0f;
			ShowroomYaw = 135.0f;
			ChaseArm->SocketOffset = FVector(0.0f, -140.0f, 0.0f);
		}
	}
	if (FParse::Param(FCommandLine::Get(), TEXT("KartCockpit"))) { OnToggleCamera(); } // telemetry CSV from the start (diagnostics)
	if (FParse::Param(FCommandLine::Get(), TEXT("KartSurfaceSelfTest"))) { SelfTestCountdown = 1.5f; RunSurfaceSelfTest(); }
	// Autopilot (test/demo driver): -KartAutopilot=<target lateral g>, -KartQuitAfterLaps=<n>.
	float AutoG = 0.0f;
	if (FParse::Value(FCommandLine::Get(), TEXT("KartAutopilot="), AutoG) && AutoG > 0.1f)
	{
		Pilot.TargetLatG = AutoG;
		bAutopilot = true;
	}
	FParse::Value(FCommandLine::Get(), TEXT("KartQuitAfterLaps="), QuitAfterLaps);
	FParse::Value(FCommandLine::Get(), TEXT("KartAnalysisAfterLaps="), AnalysisAfterLaps);
	if (Sim && Pilot.HasPath()) { Pilot.Reset(*Sim); }
	AddMappingContext();
}

void AKartVehiclePawn::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (Recorder) { Recorder->Close(); }
	if (FfbDevice) { FfbDevice->Stop(); }
	CaptureTyreWear();
	SaveTyreSets();
	Super::EndPlay(EndPlayReason);
}

bool AKartVehiclePawn::InitSimulation()
{
	const FString Path = FPaths::ProjectContentDir() / TEXT("KartData/Karts") / KartConfigFile; // relative: also found inside the pak
	kartsim::KartConfig Config;
	std::string Error;
	if (!kartsim::LoadKartConfigFile(TCHAR_TO_UTF8(*Path), Config, Error))
	{
		StatusMessage = FString::Printf(TEXT("Kart config failed to load (%s):\n%s"), *Path, UTF8_TO_TCHAR(Error.c_str()));
		UE_LOG(LogKart, Error, TEXT("%s"), *StatusMessage);
		return false;
	}
	DefaultConfig = Config;
	// Apply the saved setup for this class, if any (Saved/Setups/<classId>.json).
	FString SetupText;
	// -KartDefaultSetup: class-file setup only (regression runs must not depend on the user's saved setups).
	if (!FParse::Param(FCommandLine::Get(), TEXT("KartDefaultSetup")) && FFileHelper::LoadFileToString(SetupText, *SetupFilePath(Config.ClassId)))
	{
		std::string SetupErr;
		if (kartsim::ApplySetupJson(TCHAR_TO_UTF8(*SetupText), Config, SetupErr))
		{
			UE_LOG(LogKart, Log, TEXT("Applied saved setup %s"), *SetupFilePath(Config.ClassId));
		}
		else
		{
			UE_LOG(LogKart, Warning, TEXT("Saved setup ignored: %s"), UTF8_TO_TCHAR(SetupErr.c_str()));
		}
	}
	EditConfig = Config;
	Sim = MakeUnique<kartsim::KartSimulation>();
	Sim->Initialize(Config);
	Sim->Settings.Substeps = Substeps;
	if (TyreSetClassId != Config.ClassId) { LoadTyreSets(); } // first load, or SwitchKart: that class's sets
	RestoreTyreWear();
	StatusMessage.Reset();
	UE_LOG(LogKart, Log, TEXT("Loaded kart '%s' (%s), physics %.0f Hz x %d substeps"),
	       UTF8_TO_TCHAR(Config.Name.c_str()), UTF8_TO_TCHAR(Config.ClassId.c_str()), PhysicsRateHz, Substeps);
	return true;
}

// BuildVisuals: see KartVehicleModel.cpp

void AKartVehiclePawn::ResetKart()
{
	if (!Sim)
	{
		return;
	}
	// Place the kart on the ground below its current location, keeping its heading.
	const FVector Loc = GetActorLocation();
	FHitResult Hit;
	FCollisionQueryParams Params(SCENE_QUERY_STAT(KartReset), true, this);
	FVector Ground = Loc - FVector(0, 0, 50);
	if (GetWorld()->LineTraceSingleByChannel(Hit, Loc + FVector(0, 0, 200), Loc - FVector(0, 0, 2000), ECC_WorldStatic, Params))
	{
		Ground = Hit.ImpactPoint;
	}
	// A reset is a trip back to the pits: the mechanics fix the damage and a seized engine (Milestone 8).
	Sim->Repair();
	Sim->Reset(KartConv::FromUEPos(Ground), KartConv::YawFromUE(GetActorRotation().Yaw));
	Accumulator = 0.0;
	LapTimer.Reset(); // a reset aborts the lap in progress (history and bests are kept)
	CurrSnap = Capture();
	PrevSnap = CurrSnap;
	ApplyRender(1.0f);
	if (FfbDevice) { FfbDevice->Stop(); }
}

AKartVehiclePawn::FRenderSnapshot AKartVehiclePawn::Capture() const
{
	FRenderSnapshot S;
	const kartsim::ChassisState& Ch = Sim->GetChassis();
	S.Position = KartConv::ToUEPos(Ch.Position);
	S.Rotation = KartConv::ToUEQuat(Ch.Orientation);
	const FVector ChassisFwd = S.Rotation.GetForwardVector();
	for (int32 I = 0; I < kartsim::NumWheels; ++I)
	{
		const kartsim::WheelState& W = Sim->GetWheel(I);
		S.WheelPos[I] = KartConv::ToUEPos(W.CentreWorld);
		const FVector Axle = KartConv::ToUEDir(W.AxleWorld).GetSafeNormal();
		const FQuat Align = FRotationMatrix::MakeFromZX(Axle, ChassisFwd).ToQuat();
		// Mirroring flips rotation sense: a positive sim spin is a negative UE angle.
		S.WheelRot[I] = FQuat(Axle, -W.SpinAngle) * Align;
		S.UprightRot[I] = Align;
	}
	return S;
}

void AKartVehiclePawn::ApplyRender(float Alpha)
{
	const FVector Pos = FMath::Lerp(PrevSnap.Position, CurrSnap.Position, Alpha);
	const FQuat Rot = FQuat::Slerp(PrevSnap.Rotation, CurrSnap.Rotation, Alpha);
	SetActorLocationAndRotation(Pos, Rot, false, nullptr, ETeleportType::TeleportPhysics);
	for (int32 I = 0; I < kartsim::NumWheels; ++I)
	{
		WheelMeshes[I]->SetWorldLocationAndRotation(
			FMath::Lerp(PrevSnap.WheelPos[I], CurrSnap.WheelPos[I], Alpha),
			FQuat::Slerp(PrevSnap.WheelRot[I], CurrSnap.WheelRot[I], Alpha));
	}
	// Uprights (steered, no spin) and the spinning rear-axle pivot for imported moving parts.
	// Front wheels on the imported spindles: drawn on the stub-axle line through the model's kingpin, at the sim's
	// distance along that line (so a track change still slides the hub along the stub axle). The sim's own kingpin
	// sits elsewhere (spindleOffset, kept as is), so without this the wheel swung on a different arc than the
	// spindle and left the stub axle when steering. Visual only: contact patch and physics are unchanged.
	if (bKingpinPivot)
	{
		const FTransform ActorT = GetActorTransform();
		const int32 Wi[2] = {kartsim::WheelFL, kartsim::WheelFR};
		for (int32 S = 0; S < 2; ++S)
		{
			const FVector Kp = ActorT.TransformPosition(KingpinLocal[S]);
			const FVector Axle = FQuat::Slerp(PrevSnap.UprightRot[Wi[S]], CurrSnap.UprightRot[Wi[S]], Alpha).GetAxisZ();
			const FVector P = WheelMeshes[Wi[S]]->GetComponentLocation();
			WheelMeshes[Wi[S]]->SetWorldLocation(Kp + Axle * FVector::DotProduct(P - Kp, Axle));
		}
	}
	// Real kart: the spindle is pinned to the chassis at the kingpin and only rotates (steer about the kingpin axis,
	// caster / camber); a track-width change slides the hub along the stub axle. So the upright sits at the
	// chassis-fixed kingpin and takes the wheel's (non-spinning) orientation; the wheel itself stays where the sim puts it.
	{
		const FTransform ActorT = GetActorTransform();
		const int32 Wi[2] = {kartsim::WheelFL, kartsim::WheelFR};
		USceneComponent* Up[2] = {UprightFL, UprightFR};
		for (int32 S = 0; S < 2; ++S)
		{
			const FVector UpPos = bKingpinPivot ? ActorT.TransformPosition(KingpinLocal[S]) : FMath::Lerp(PrevSnap.WheelPos[Wi[S]], CurrSnap.WheelPos[Wi[S]], Alpha);
			Up[S]->SetWorldLocationAndRotation(UpPos, FQuat::Slerp(PrevSnap.UprightRot[Wi[S]], CurrSnap.UprightRot[Wi[S]], Alpha));
		}
	}
	RearAxlePivot->SetWorldLocationAndRotation(0.5f * (WheelMeshes[kartsim::WheelRL]->GetComponentLocation() + WheelMeshes[kartsim::WheelRR]->GetComponentLocation()),
		WheelMeshes[kartsim::WheelRL]->GetComponentQuat());
	// Rear axle spans the two rear hubs (follows rear frame twist).
	const FVector HubL = WheelMeshes[kartsim::WheelRL]->GetComponentLocation();
	const FVector HubR = WheelMeshes[kartsim::WheelRR]->GetComponentLocation();
	const FVector AxleDir = HubL - HubR;
	if (!AxleDir.IsNearlyZero())
	{
		AxleMesh->SetWorldLocationAndRotation(0.5f * (HubL + HubR), FRotationMatrix::MakeFromZX(AxleDir.GetSafeNormal(), Rot.GetForwardVector()).ToQuat());
		AxleMesh->SetWorldScale3D(FVector(1.0f, 1.0f, AxleDir.Size() / 100.0f));
	}
	// Steering wheel turns about the column axis (mirrored space: positive sim angle is a negative UE angle). The
	// column angle comes from the simulated front wheels so every kart (AI, network proxies, replays) shows it.
	const float Column = static_cast<float>(0.5 * (Sim->GetWheel(kartsim::WheelFL).SteerAngle + Sim->GetWheel(kartsim::WheelFR).SteerAngle) * Sim->GetConfig().Steering.Ratio);
	SteeringWheelMesh->SetRelativeRotation(FQuat(SteeringAxisLocal, -Column) * SteeringAlign);
	// Tie rods: the outer end rides on the spindle arm, the inner end on the column arm (rotates with the column about
	// the steering axis). Re-aimed and stretched along the rod every frame.
	if (TieRods.Num() > 0)
	{
		const FTransform RootT = Root->GetComponentTransform();
		const FVector ColPivot = SteeringWheelMesh->GetRelativeLocation();
		const FQuat ColRot(SteeringAxisLocal, -Column);
		USceneComponent* Ups[2] = {UprightFL, UprightFR};
		for (const FTieRod& T : TieRods)
		{
			if (!T.C) { continue; }
			const FVector Outer = RootT.InverseTransformPosition(Ups[T.Side]->GetComponentTransform().TransformPosition(T.OuterLocal));
			const FVector Inner = ColPivot + ColRot.RotateVector(T.InnerRest - ColPivot);
			const FVector Cur = Inner - Outer;
			const float Stretch = static_cast<float>(Cur.Size() / FMath::Max(T.RestDir.Size(), 1.0));
			T.C->SetRelativeTransform(FTransform(FQuat::FindBetweenVectors(T.RestDir, Cur), Outer, FVector(1.0f, Stretch, 1.0f)));
		}
	}
	AnimateDriver(Column, GetWorld() ? GetWorld()->GetDeltaSeconds() : 0.016f);
}

void AKartVehiclePawn::AnimateDriver(float Column, float DeltaSeconds)
{
	if (!DRig.bValid || !Sim) { return; }
	// Body-frame acceleration from the chassis velocity (finite difference, smoothed): works for simulated karts,
	// network proxies and replay puppets alike.
	const kartsim::ChassisState& Ch = Sim->GetChassis();
	const FVector Vel(Ch.Velocity.X, Ch.Velocity.Y, Ch.Velocity.Z);
	const float Dt = FMath::Clamp(DeltaSeconds, 1e-3f, 0.1f);
	if (bAnimHasPrev && DeltaSeconds > 0.0f)
	{
		const FVector Aw = (Vel - AnimPrevVel) / Dt;
		const kartsim::Vec3 Fwd = Ch.Orientation.Rotate(kartsim::Vec3(1, 0, 0)), Left = Ch.Orientation.Rotate(kartsim::Vec3(0, 1, 0));
		const FVector Ab(Aw.X * Fwd.X + Aw.Y * Fwd.Y + Aw.Z * Fwd.Z, Aw.X * Left.X + Aw.Y * Left.Y + Aw.Z * Left.Z, 0.0f);
		AnimAccel = FMath::Lerp(AnimAccel, Ab, FMath::Clamp(Dt / 0.12f, 0.0f, 1.0f));
	}
	AnimPrevVel = Vel;
	bAnimHasPrev = true;
	const float G = 9.81f;
	// Torso: pushed to the outside of the corner (~6 deg/g) and forward under braking (~5 deg/g); the head keeps
	// ~70 % of the roll level and looks into the corner with the steering.
	const float RollT = FMath::Clamp(-6.0f * AnimAccel.Y / G, -10.0f, 10.0f);
	const float PitchT = FMath::Clamp(5.0f * FMath::Max(-AnimAccel.X / G, 0.0f) - 1.5f * FMath::Max(AnimAccel.X / G, 0.0f), -3.0f, 8.0f);
	const float YawT = FMath::Clamp(FMath::RadiansToDegrees(Column) * 0.18f, -14.0f, 14.0f);
	const float K = FMath::Clamp(Dt / 0.15f, 0.0f, 1.0f);
	AnimRoll += (RollT - AnimRoll) * K;
	AnimPitch += (PitchT - AnimPitch) * K;
	AnimYaw += (YawT - AnimYaw) * K;

	// Rotations in component space (UE: X forward, Y right, Z up). Positive roll about +X tilts the top to the
	// kart's left; positive pitch about +Y tilts the top forward.
	const FQuat TorsoR = FQuat(FVector::ForwardVector, FMath::DegreesToRadians(AnimRoll)) * FQuat(FVector::RightVector, FMath::DegreesToRadians(AnimPitch));
	auto AboutPivot = [](UPrimitiveComponent* C, const FVector& Pivot, const FQuat& R, const FVector& NewPivot)
	{
		C->SetRelativeLocationAndRotation(NewPivot - R.RotateVector(Pivot), R);
	};
	AboutPivot(TorsoMesh, DRig.Pelvis, TorsoR, DRig.Pelvis);
	const FVector NeckNow = DRig.Pelvis + TorsoR.RotateVector(DRig.Neck - DRig.Pelvis);
	const FQuat HeadR = TorsoR * FQuat(FVector::UpVector, FMath::DegreesToRadians(-AnimYaw)) * FQuat(FVector::ForwardVector, FMath::DegreesToRadians(-0.7f * AnimRoll));
	AboutPivot(HelmetMesh, DRig.Neck, HeadR, NeckNow);
	CockpitCamera->SetRelativeLocation(NeckNow + HeadR.RotateVector(DRig.Eye - DRig.Neck)); // position only: no camera roll

	// Arms: hands ride the rim with the steering wheel; elbows by two-bone IK from the (leaning) shoulders.
	const FQuat WheelR(DRig.ColAxis, -Column);
	for (int32 S = 0; S < 2; ++S)
	{
		const FVector Shoulder = DRig.Pelvis + TorsoR.RotateVector(DRig.Shoulder[S] - DRig.Pelvis);
		const FVector Grip = DRig.ColTop + WheelR.RotateVector(DRig.Grip[S] - DRig.ColTop);
		const FVector Wrist = DRig.ColTop + WheelR.RotateVector(DRig.Wrist[S] - DRig.ColTop);
		const FVector D = Wrist - Shoulder;
		const float Dist = FMath::Clamp(static_cast<float>(D.Size()), 1.0f, DRig.UpperLen + DRig.ForeLen - 0.1f);
		const FVector Dir = D.GetSafeNormal();
		const float A = DRig.UpperLen, B = DRig.ForeLen;
		const float Along = (A * A - B * B + Dist * Dist) / (2.0f * Dist);
		const float Off = FMath::Sqrt(FMath::Max(A * A - Along * Along, 0.0f));
		const FVector Perp = (DRig.ElbowHint[S] - Dir * FVector::DotProduct(DRig.ElbowHint[S], Dir)).GetSafeNormal();
		const FVector Elbow = Shoulder + Dir * Along + Perp * Off;
		ArmMeshes[S]->SetRelativeLocationAndRotation(Shoulder, FRotationMatrix::MakeFromXZ((Elbow - Shoulder).GetSafeNormal(), Perp).ToQuat());
		ArmMeshes[2 + S]->SetRelativeLocationAndRotation(Elbow, FRotationMatrix::MakeFromXZ((Wrist - Elbow).GetSafeNormal(), Perp).ToQuat());
		ArmMeshes[4 + S]->SetRelativeLocationAndRotation(Grip, WheelR * DRig.GripRot[S]);
	}
}

void AKartVehiclePawn::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (ShowroomDistance > 0.0f)
	{
		// Debug turntable camera (-KartShowroom=<cm>): orbits the kart at 40 deg/s.
		ShowroomYaw += ShowroomSpin * DeltaSeconds;
		ChaseArm->SetRelativeRotation(FRotator(ShowroomPitch, ShowroomYaw, 0.0f));
	}
	if (!Sim || bPuppet)
	{
		return;
	}
	if (!bIsAI)
	{
		UpdateGhost(DeltaSeconds);
		UpdateDash(DeltaSeconds);
	}
	UpdateFairing(DeltaSeconds);
	LogDamageEvents();
	if (!bIsAI)
	{
		if (!bTyreSetRestored && OwnsTyreSets()) { RestoreTyreWear(); bTyreSetRestored = true; } // possessed after BeginPlay
		TyreSaveTimer += DeltaSeconds;
		if (TyreSaveTimer > 15.0f && bTyreSetRestored) { TyreSaveTimer = 0.0f; CaptureTyreWear(); SaveTyreSets(); } // survives a crash / kill
		UpdateInputs(DeltaSeconds);
		if (SelfTestCountdown > 0.0f)
		{
			// Physics bodies are created asynchronously: probe the spawned test pad a moment later.
			SelfTestCountdown -= DeltaSeconds;
			if (SelfTestCountdown <= 0.0f) { RunSurfaceSelfTest(); }
		}
		if (!bSetupOpen) { UpdateWorldKeys(); }
		if (bAnalysisOpen && !bSetupOpen)
		{
			// Lap analysis: driving inputs suspended like the setup page.
			UpdateAnalysisKeys(DeltaSeconds);
			Inputs = kartsim::KartInputs();
			Inputs.Brake = 1.0;
			Inputs.Clutch = 1.0;
		}
		if (bSetupOpen)
		{
			// Setup menu: driving inputs suspended, kart held on the brake.
			UpdateSetupMenu();
			Inputs = kartsim::KartInputs();
			Inputs.Brake = 1.0;
			Inputs.Clutch = 1.0;
		}
	}

	// With a race world, it steps every kart in lockstep (contacts) and calls FinishFrame. Without
	// one (e.g. the pawn placed in another map), the pawn steps itself.
	if (!RaceWorld.IsValid())
	{
		const double H = 1.0 / FMath::Max(PhysicsRateHz, 1.0f);
		Accumulator += FMath::Min(static_cast<double>(DeltaSeconds), 0.1); // hitch guard: slow down, never spiral
		if (FKartLicense::Get().Blocked()) { Accumulator = 0.0; } // tester build without a valid license
		const double T0 = FPlatformTime::Seconds();
		int32 Steps = 0;
		while (Accumulator >= H)
		{
			StepPhysics(H, Sim->GetTime() + H, nullptr, false);
			Accumulator -= H;
			++Steps;
		}
		FinishFrame(static_cast<float>(Accumulator / H), Steps, static_cast<float>((FPlatformTime::Seconds() - T0) * 1000.0), DeltaSeconds);
	}

	if (bDebugDraw && !bIsAI)
	{
		DrawPhysicsDebug();
	}
}

void AKartVehiclePawn::SetRaceWorld(AKartRaceWorld* World)
{
	RaceWorld = World;
}

void AKartVehiclePawn::StepPhysics(double H, double Clock, const kartsim::KartInputs* Override, bool bHoldOnGrid, double SpeedLimit)
{
	if (!Sim) { return; }
	ClockNow = Clock;
	AKartConditionsActor* Cond = Conditions.Get();
	const bool bCond = Cond && Cond->IsActive() && CondQuery.IsValid();
	const kartsim::ITrackQuery& Ground = bCond ? static_cast<const kartsim::ITrackQuery&>(*CondQuery) : static_cast<const kartsim::ITrackQuery&>(TrackQuery);
	if (bCond)
	{
		Sim->Settings.AmbientTemp = Cond->Get().Weather.AirTemp;
		Sim->Settings.TrackTemp = Cond->Get().Weather.TrackTemp;
		Sim->Settings.AirPressure = Cond->Get().Weather.PressureKPa;
		Sim->Settings.Humidity = Cond->Get().Weather.Humidity;
		// AI mechanics jet for the air and tape the radiator (re-done when the air changes by > 1 degC).
		if (bIsAI && FMath::Abs(Sim->Settings.AmbientTemp - AiTunedAirTemp) > 1.0)
		{
			AiTunedAirTemp = Sim->Settings.AmbientTemp;
			kartsim::EngineTuneParams T = Sim->GetConfig().Tuning;
			kartsim::AutoTune(T, Sim->Settings.Air());
			Sim->SetCarb(T.Jet, T.Cover);
		}
	}
	{
		const FKartRealismSettings& RS = FKartRealismSettings::Get();
		Sim->Settings.bDamage = RS.bDamage;
		Sim->Settings.bEngineFailures = RS.bEngineFailures;
	}
	Sim->Settings.Substeps = Substeps;

	PrevSnap = CurrSnap;
	kartsim::KartInputs StepInputs = Inputs;
	StepInputs.ShiftRequest = PendingShift;
	PendingShift = 0;
	if (Override)
	{
		StepInputs = *Override;
	}
	else if (bAutopilot && Pilot.HasPath() && !bSetupOpen)
	{
		StepInputs = kartsim::KartInputs();
		Pilot.Drive(*Sim, H, StepInputs);
		kartsim::DriverAidSettings AllAids;
		AllAids.bAutoClutch = AllAids.bAutoShift = AllAids.bAutoPush = true;
		kartsim::ApplyDriverAids(*Sim, AllAids, StepInputs);
	}
	if (bHoldOnGrid)
	{
		StepInputs.Brake = 1.0; // start lights: held on the grid (throttle allowed, as in a real start)
	}
	if (Sim->ForwardSpeed() > SpeedLimit)
	{
		StepInputs.Throttle = 0.0; // formation-lap limiter (race control)
	}
	const kartsim::Vec3 PrevCog = Sim->GetChassis().Position;
	Sim->Step(H, StepInputs, Ground);
	if (bCond) { Cond->Get().ApplyKart(*Sim, H); }
	if (bTimingConfigured)
	{
		// Track limits: all four wheels on grass/gravel at once (kerbs count as track).
		bool bAllOff = true;
		for (int32 W = 0; W < kartsim::NumWheels; ++W)
		{
			const kartsim::TyreContact& Ct = Sim->GetWheel(W).Contact;
			const bool bOff = Ct.bInContact && (Ct.Surface == kartsim::ESurfaceType::Grass || Ct.Surface == kartsim::ESurfaceType::Gravel);
			bAllOff = bAllOff && bOff;
		}
		bLastAllOff = bLastAllOff || bAllOff; // latched until the next network send
		LapTimer.Update(Clock, H, PrevCog, Sim->GetChassis().Position, bAllOff);
		LapAnalysis.Step(Clock, LapTimer, *Sim);
		if (!bIsAI)
		{
			for (; GhostSeenLaps < LapTimer.Laps().size(); ++GhostSeenLaps)
			{
				const kartsim::LapRecord& GL = LapTimer.Laps()[GhostSeenLaps];
				if (Ghost.OnLapCompleted(GL.LapTime, GL.bValid))
				{
					SaveGhost();
					UE_LOG(LogKart, Log, TEXT("New ghost lap %.3f s saved: %s"), GL.LapTime, *GhostFilePath());
				}
			}
			Ghost.Step(LapTimer.IsTiming(), LapTimer.CurrentLapTime(Clock), *Sim);
		}
		if (LapTimer.Laps().size() > LoggedLaps)
		{
			LoggedLaps = LapTimer.Laps().size();
			const kartsim::LapRecord& L = LapTimer.Laps().back();
			UE_LOG(LogKart, Log, TEXT("%sLAP %d: %.3f s (S1 %.3f S2 %.3f S3 %.3f) %s%s"), bIsAI ? *FString::Printf(TEXT("[%s] "), *DriverName) : TEXT(""),
			       static_cast<int32>(LoggedLaps), L.LapTime, L.Sectors[0], L.Sectors[1], L.Sectors[2], L.bValid ? TEXT("valid") : TEXT("INVALID"),
			       bAutopilot ? TEXT(" [autopilot]") : TEXT(""));
			if (!bIsAI && AnalysisAfterLaps > 0 && static_cast<int32>(LoggedLaps) == AnalysisAfterLaps)
			{
				bAnalysisOpen = true;
				AnalysisLap = static_cast<int32>(LapAnalysis.GetDelta().History().size()) - 1;
				AnalysisCursor = static_cast<int32>(LapAnalysis.GetDelta().SamplesPerLap() / 3);
			}
			if (!bIsAI && QuitAfterLaps > 0 && static_cast<int32>(LoggedLaps) >= QuitAfterLaps)
			{
				UE_LOG(LogKart, Log, TEXT("KartQuitAfterLaps reached, exiting"));
				FPlatformMisc::RequestExit(false);
			}
		}
	}
	if (!bIsAI && !Override) { CheckWobble(StepInputs); }
	CurrSnap = Capture();
	if (!bIsAI)
	{
		if (!bIsAI && FfbDevice && !FfbDevice->IsConnected() && FKartWheelInput::Get().HasForceFeedback())
		{
			// The HUD opens DirectInput on its first frame (needs the game window) and on "Rescan devices".
			FfbDevice = MakeUnique<FKartDirectInputFFBDevice>();
		}
		if (FfbDevice && !Override && IsLocallyControlled())
		{
			// Simulated column torque x user gain, plus a soft lock at the kart's steering lock (the base
			// usually rotates much further than a kart's steering).
			const FKartWheelInput& Wheel = FKartWheelInput::Get();
			float Cmd = static_cast<float>(Sim->GetTelemetry().FfbOutput) * Wheel.FfbGain;
			if (Wheel.IsSteeringBound())
			{
				const float LockDeg = static_cast<float>(Sim->GetConfig().Steering.MaxColumnAngle * kartsim::kRadToDeg);
				const float W = Wheel.SteeringDeg(); // + = right
				const float Over = FMath::Abs(W) - LockDeg;
				if (Over > 0.0f) { Cmd += -FMath::Sign(W) * Wheel.SoftLockStrength * FMath::Min(1.0f, 0.2f + Over / 4.0f); } // right of lock -> push left (+)
			}
			LastFfbCommand = FMath::Clamp(Cmd, -1.0f, 1.0f);
			FfbDevice->SetConstantTorque(LastFfbCommand);
		}
		if (IsRecording()) { Recorder->Write(Sim->GetTelemetry()); }
	}
}

void AKartVehiclePawn::FinishFrame(float Alpha, int32 Steps, float PhysicsMs, float DeltaSeconds)
{
	PhysicsMsPerFrame = PhysicsMs;
	StepsLastFrame = Steps;
	HzWindowSteps += Steps;
	HzWindowTime += DeltaSeconds;
	if (HzWindowTime >= 0.5)
	{
		MeasuredHz = static_cast<float>(HzWindowSteps / HzWindowTime);
		HzWindowSteps = 0;
		HzWindowTime = 0.0;
	}
	ApplyRender(Alpha);
}

// ------------------------------------------------------------------------------------ input

void AKartVehiclePawn::EnsureInputObjects()
{
	if (InputContext)
	{
		return;
	}
	auto MakeAction = [this](const TCHAR* Name, EInputActionValueType Type)
	{
		UInputAction* A = NewObject<UInputAction>(this, Name);
		A->ValueType = Type;
		return A;
	};
	ActThrottleKey = MakeAction(TEXT("IA_ThrottleKey"), EInputActionValueType::Axis1D);
	ActBrakeKey = MakeAction(TEXT("IA_BrakeKey"), EInputActionValueType::Axis1D);
	ActSteerKey = MakeAction(TEXT("IA_SteerKey"), EInputActionValueType::Axis1D);
	ActThrottleAxis = MakeAction(TEXT("IA_ThrottleAxis"), EInputActionValueType::Axis1D);
	ActBrakeAxis = MakeAction(TEXT("IA_BrakeAxis"), EInputActionValueType::Axis1D);
	ActSteerAxis = MakeAction(TEXT("IA_SteerAxis"), EInputActionValueType::Axis1D);
	ActClutch = MakeAction(TEXT("IA_Clutch"), EInputActionValueType::Axis1D);
	ActShiftUp = MakeAction(TEXT("IA_ShiftUp"), EInputActionValueType::Boolean);
	ActShiftDown = MakeAction(TEXT("IA_ShiftDown"), EInputActionValueType::Boolean);
	ActReset = MakeAction(TEXT("IA_Reset"), EInputActionValueType::Boolean);
	ActCamera = MakeAction(TEXT("IA_Camera"), EInputActionValueType::Boolean);
	ActDebug = MakeAction(TEXT("IA_Debug"), EInputActionValueType::Boolean);
	ActTelemetry = MakeAction(TEXT("IA_Telemetry"), EInputActionValueType::Boolean);
	ActRecord = MakeAction(TEXT("IA_Record"), EInputActionValueType::Boolean);
	ActKart1 = MakeAction(TEXT("IA_Kart1"), EInputActionValueType::Boolean);
	ActKart2 = MakeAction(TEXT("IA_Kart2"), EInputActionValueType::Boolean);
	ActKart3 = MakeAction(TEXT("IA_Kart3"), EInputActionValueType::Boolean);
	ActStarter = MakeAction(TEXT("IA_Starter"), EInputActionValueType::Axis1D);
	ActPush = MakeAction(TEXT("IA_Push"), EInputActionValueType::Axis1D);
	ActAutoShift = MakeAction(TEXT("IA_AutoShift"), EInputActionValueType::Boolean);
	ActAutoClutch = MakeAction(TEXT("IA_AutoClutch"), EInputActionValueType::Boolean);
	ActSetup = MakeAction(TEXT("IA_Setup"), EInputActionValueType::Boolean);
	ActAutopilot = MakeAction(TEXT("IA_Autopilot"), EInputActionValueType::Boolean);

	InputContext = NewObject<UInputMappingContext>(this, TEXT("IMC_Kart"));
	UInputModifierNegate* Negate = NewObject<UInputModifierNegate>(this);
	UInputModifierDeadZone* DeadZone = NewObject<UInputModifierDeadZone>(this);
	DeadZone->LowerThreshold = 0.06f;

	InputContext->MapKey(ActThrottleKey, EKeys::W);
	InputContext->MapKey(ActThrottleKey, EKeys::Up);
	InputContext->MapKey(ActBrakeKey, EKeys::S);
	InputContext->MapKey(ActBrakeKey, EKeys::Down);
	// Sim convention: positive steering = left.
	InputContext->MapKey(ActSteerKey, EKeys::A);
	InputContext->MapKey(ActSteerKey, EKeys::Left);
	InputContext->MapKey(ActSteerKey, EKeys::D).Modifiers.Add(Negate);
	InputContext->MapKey(ActSteerKey, EKeys::Right).Modifiers.Add(Negate);
	InputContext->MapKey(ActClutch, EKeys::LeftShift);
	InputContext->MapKey(ActClutch, EKeys::Gamepad_LeftShoulder);

	InputContext->MapKey(ActThrottleAxis, EKeys::Gamepad_RightTriggerAxis);
	InputContext->MapKey(ActBrakeAxis, EKeys::Gamepad_LeftTriggerAxis);
	FEnhancedActionKeyMapping& Stick = InputContext->MapKey(ActSteerAxis, EKeys::Gamepad_LeftX);
	Stick.Modifiers.Add(DeadZone);
	Stick.Modifiers.Add(Negate);

	InputContext->MapKey(ActShiftUp, EKeys::E);
	InputContext->MapKey(ActShiftUp, EKeys::Gamepad_RightShoulder);
	InputContext->MapKey(ActShiftDown, EKeys::Q);
	InputContext->MapKey(ActShiftDown, EKeys::Gamepad_FaceButton_Left);
	InputContext->MapKey(ActReset, EKeys::R);
	InputContext->MapKey(ActReset, EKeys::Gamepad_Special_Right);
	InputContext->MapKey(ActCamera, EKeys::C);
	InputContext->MapKey(ActCamera, EKeys::Gamepad_FaceButton_Top);
	InputContext->MapKey(ActDebug, EKeys::G);
	InputContext->MapKey(ActTelemetry, EKeys::T);
	InputContext->MapKey(ActRecord, EKeys::K);
	InputContext->MapKey(ActKart1, EKeys::One);
	InputContext->MapKey(ActKart2, EKeys::Two);
	InputContext->MapKey(ActKart3, EKeys::Three);
	InputContext->MapKey(ActStarter, EKeys::B);
	InputContext->MapKey(ActStarter, EKeys::Gamepad_FaceButton_Right);
	InputContext->MapKey(ActPush, EKeys::P);
	InputContext->MapKey(ActAutoShift, EKeys::H);
	InputContext->MapKey(ActAutoClutch, EKeys::J);
	InputContext->MapKey(ActSetup, EKeys::Tab);
	InputContext->MapKey(ActAutopilot, EKeys::O);
}

void AKartVehiclePawn::AddMappingContext()
{
	EnsureInputObjects();
	if (const APlayerController* PC = Cast<APlayerController>(GetController()))
	{
		if (ULocalPlayer* LP = PC->GetLocalPlayer())
		{
			if (UEnhancedInputLocalPlayerSubsystem* Sub = LP->GetSubsystem<UEnhancedInputLocalPlayerSubsystem>())
			{
				if (!Sub->HasMappingContext(InputContext))
				{
					Sub->AddMappingContext(InputContext, 0);
				}
			}
		}
	}
}

void AKartVehiclePawn::NotifyControllerChanged()
{
	Super::NotifyControllerChanged();
	AddMappingContext();
}

void AKartVehiclePawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);
	EnsureInputObjects();
	UEnhancedInputComponent* EIC = Cast<UEnhancedInputComponent>(PlayerInputComponent);
	if (!EIC)
	{
		StatusMessage = TEXT("Enhanced Input component missing: enable the EnhancedInput plugin / default input classes.");
		return;
	}
	for (UInputAction* A : {ActThrottleKey.Get(), ActBrakeKey.Get(), ActSteerKey.Get(), ActThrottleAxis.Get(),
	                        ActBrakeAxis.Get(), ActSteerAxis.Get(), ActClutch.Get(), ActStarter.Get(), ActPush.Get()})
	{
		EIC->BindActionValue(A);
	}
	EIC->BindAction(ActShiftUp, ETriggerEvent::Started, this, &AKartVehiclePawn::OnShiftUp);
	EIC->BindAction(ActShiftDown, ETriggerEvent::Started, this, &AKartVehiclePawn::OnShiftDown);
	EIC->BindAction(ActReset, ETriggerEvent::Started, this, &AKartVehiclePawn::OnReset);
	EIC->BindAction(ActCamera, ETriggerEvent::Started, this, &AKartVehiclePawn::OnToggleCamera);
	EIC->BindAction(ActDebug, ETriggerEvent::Started, this, &AKartVehiclePawn::OnToggleDebug);
	EIC->BindAction(ActTelemetry, ETriggerEvent::Started, this, &AKartVehiclePawn::OnToggleTelemetry);
	EIC->BindAction(ActRecord, ETriggerEvent::Started, this, &AKartVehiclePawn::OnToggleRecord);
	EIC->BindAction(ActKart1, ETriggerEvent::Started, this, &AKartVehiclePawn::OnKart1);
	EIC->BindAction(ActKart2, ETriggerEvent::Started, this, &AKartVehiclePawn::OnKart2);
	EIC->BindAction(ActKart3, ETriggerEvent::Started, this, &AKartVehiclePawn::OnKart3);
	EIC->BindAction(ActAutoShift, ETriggerEvent::Started, this, &AKartVehiclePawn::OnToggleAutoShift);
	EIC->BindAction(ActAutoClutch, ETriggerEvent::Started, this, &AKartVehiclePawn::OnToggleAutoClutch);
	EIC->BindAction(ActSetup, ETriggerEvent::Started, this, &AKartVehiclePawn::OnToggleSetup);
	EIC->BindAction(ActAutopilot, ETriggerEvent::Started, this, &AKartVehiclePawn::OnToggleAutopilot);
}

void AKartVehiclePawn::UpdateInputs(float DeltaSeconds)
{
	const UEnhancedInputComponent* EIC = Cast<UEnhancedInputComponent>(InputComponent);
	auto Value = [EIC](const UInputAction* A) -> float
	{
		return (EIC && A) ? EIC->GetBoundActionValue(A).Get<float>() : 0.0f;
	};

	// Digital keys -> ramps (input conditioning only).
	auto Approach = [](float Current, float Target, float Rate, float Dt)
	{
		return Current + FMath::Clamp(Target - Current, -Rate * Dt, Rate * Dt);
	};
	// Log which physical driving keys are down whenever the set changes (input diagnostics).
	if (const APlayerController* PC = Cast<APlayerController>(GetController()))
	{
		static const FKey Watched[] = {EKeys::W, EKeys::A, EKeys::S, EKeys::D, EKeys::Up, EKeys::Down, EKeys::Left, EKeys::Right};
		FString Down;
		for (const FKey& K : Watched)
		{
			if (PC->IsInputKeyDown(K)) { Down += K.ToString() + TEXT(" "); }
		}
		if (Down != LastKeysDown)
		{
			UE_LOG(LogKart, Verbose, TEXT("Driving keys down: [%s] (thrKey %.2f steerKey %.2f)"), *Down, Value(ActThrottleKey), Value(ActSteerKey));
			LastKeysDown = Down;
		}
	}
	const float SteerTarget = FMath::Clamp(Value(ActSteerKey), -1.0f, 1.0f);
	const bool bReturning = FMath::IsNearlyZero(SteerTarget) || (SteerTarget * KeySteer < 0.0f);
	KeySteer = Approach(KeySteer, SteerTarget, bReturning ? KeyboardSteerReturnRate : KeyboardSteerRate, DeltaSeconds);
	KeyThrottle = Approach(KeyThrottle, FMath::Clamp(Value(ActThrottleKey), 0.0f, 1.0f), KeyboardThrottleRate, DeltaSeconds);
	KeyBrake = Approach(KeyBrake, FMath::Clamp(Value(ActBrakeKey), 0.0f, 1.0f) * KeyboardBrakeMax, KeyboardBrakeRate, DeltaSeconds);

	// Analog axes are armed only after being observed at rest once.
	const float RawAxisThrottle = FMath::Clamp(Value(ActThrottleAxis), 0.0f, 1.0f);
	const float RawAxisBrake = FMath::Clamp(Value(ActBrakeAxis), 0.0f, 1.0f);
	const float RawAxisSteer = FMath::Clamp(Value(ActSteerAxis), -1.0f, 1.0f);
	InputDiag.bThrottleArmed |= RawAxisThrottle < 0.05f;
	InputDiag.bBrakeArmed |= RawAxisBrake < 0.05f;
	InputDiag.bSteerArmed |= FMath::Abs(RawAxisSteer) < 0.10f;
	InputDiag.AxisThrottle = RawAxisThrottle;
	InputDiag.AxisBrake = RawAxisBrake;
	InputDiag.AxisSteer = RawAxisSteer;
	InputDiag.KeyThrottle = KeyThrottle;
	InputDiag.KeyBrake = KeyBrake;
	InputDiag.KeySteer = KeySteer;
	const float AxisThrottle = InputDiag.bThrottleArmed ? RawAxisThrottle : 0.0f;
	const float AxisBrake = InputDiag.bBrakeArmed ? RawAxisBrake : 0.0f;
	const float AnalogSteer = InputDiag.bSteerArmed ? RawAxisSteer : 0.0f;

	float Steer = FMath::Abs(AnalogSteer) > FMath::Abs(KeySteer) ? AnalogSteer : KeySteer;

	// Keyboard/gamepad cannot modulate lock like a wheel: limit lock to the road-wheel angle the kart
	// can use at this speed (kinematic L*a/v^2 for AssistLateralG, plus a slip-angle allowance).
	double LockFraction = 1.0;
	if (Sim)
	{
		const kartsim::KartConfig& Cfg = Sim->GetConfig();
		const double V = FMath::Max(FMath::Abs(Sim->ForwardSpeed()), 1.0);
		const double UsefulWheel = Cfg.Chassis.Wheelbase * AssistLateralG * kartsim::kGravity / (V * V)
			+ AssistSlipAngleDeg * kartsim::kDegToRad;
		const double UsefulColumn = UsefulWheel * Cfg.Steering.Ratio;
		LockFraction = FMath::Clamp(UsefulColumn / Cfg.Steering.MaxColumnAngle, 0.05, 1.0);
	}
	Steer *= static_cast<float>(LockFraction);
	InputDiag.SteerLockFraction = static_cast<float>(LockFraction);

	Inputs.Throttle = FMath::Max(KeyThrottle, AxisThrottle);
	Inputs.Brake = FMath::Max(KeyBrake, AxisBrake);
	// Clutch key: binary -> ramp (a real clutch lever is progressive).
	KeyClutch = Approach(KeyClutch, FMath::Clamp(Value(ActClutch), 0.0f, 1.0f), 6.0f, DeltaSeconds);
	Inputs.Clutch = KeyClutch;
	Inputs.bStarter = Value(ActStarter) > 0.5f;
	// Push start (P held): a helper pushes while the engine is stopped and the kart is slow.
	const bool bEngineStopped = Sim && !Sim->GetEngine().IsRunning();
	Inputs.PushForce = (Value(ActPush) > 0.5f && bEngineStopped && Sim->ForwardSpeed() < 7.0) ? 300.0 : 0.0;
	if (Sim)
	{
		kartsim::DriverAidSettings Aids;
		Aids.bAutoClutch = bAutoClutch;
		Aids.bAutoShift = bAutoShift;
		Aids.bAutoPush = false;
		Inputs.ShiftRequest = 0;
		kartsim::ApplyDriverAids(*Sim, Aids, Inputs);
		if (Inputs.ShiftRequest != 0 && PendingShift == 0) { PendingShift = Inputs.ShiftRequest; }
		Inputs.ShiftRequest = 0;
	}
	Inputs.SteeringColumnAngle = Sim ? Steer * Sim->GetConfig().Steering.MaxColumnAngle : 0.0;
	// Debug: -KartTestSteer=<-1..1> holds a fixed steering input (screenshots of the steering geometry).
	{
		float TestSteer = 0.0f;
		if (Sim && FParse::Value(FCommandLine::Get(), TEXT("KartTestSteer="), TestSteer))
		{
			Inputs.SteeringColumnAngle = FMath::Clamp(TestSteer, -1.0f, 1.0f) * Sim->GetConfig().Steering.MaxColumnAngle;
		}
	}

	// Steering wheel / pedals (DirectInput): 1:1 wheel angle = column angle, no assists; pedals override
	// keys when bound. Shifter paddles send shift requests.
	const FKartWheelInput& Wheel = FKartWheelInput::Get();
	if (Sim && Wheel.IsSteeringBound())
	{
		const double Max = Sim->GetConfig().Steering.MaxColumnAngle;
		const double WheelCol = -Wheel.SteeringDeg() * kartsim::kDegToRad; // sim: + = left
		if (FMath::Abs(WheelCol) > FMath::Abs(Inputs.SteeringColumnAngle)) { Inputs.SteeringColumnAngle = FMath::Clamp(WheelCol, -Max, Max); }
		InputDiag.SteerLockFraction = 1.0f;
	}
	if (Wheel.Axes[FKartWheelInput::Throttle].IsBound()) { Inputs.Throttle = FMath::Max(Inputs.Throttle, static_cast<double>(Wheel.Pedal(FKartWheelInput::Throttle))); }
	if (Wheel.Axes[FKartWheelInput::Brake].IsBound()) { Inputs.Brake = FMath::Max(Inputs.Brake, static_cast<double>(Wheel.Pedal(FKartWheelInput::Brake))); }
	if (Wheel.Axes[FKartWheelInput::Clutch].IsBound()) { Inputs.Clutch = FMath::Max(Inputs.Clutch, static_cast<double>(Wheel.Pedal(FKartWheelInput::Clutch))); }
	if (Wheel.ButtonPressedEdge(FKartWheelInput::ShiftUp)) { PendingShift = 1; }
	if (Wheel.ButtonPressedEdge(FKartWheelInput::ShiftDown)) { PendingShift = -1; }
}

void AKartVehiclePawn::OnShiftUp() { PendingShift = 1; }
void AKartVehiclePawn::OnShiftDown() { PendingShift = -1; }
void AKartVehiclePawn::OnReset() { if (!(RaceWorld.IsValid() && RaceWorld->IsReplaying())) { ResetKart(); } }
void AKartVehiclePawn::OnToggleDebug() { bDebugDraw = !bDebugDraw; }
void AKartVehiclePawn::OnToggleTelemetry() { bShowTelemetry = !bShowTelemetry; }
void AKartVehiclePawn::OnToggleRecord() { ToggleRecording(); }
void AKartVehiclePawn::OnKart1() { SwitchKart(0); }
void AKartVehiclePawn::OnKart2() { SwitchKart(1); }
void AKartVehiclePawn::OnKart3() { SwitchKart(2); }
void AKartVehiclePawn::OnToggleAutoShift() { bAutoShift = !bAutoShift; }
void AKartVehiclePawn::OnToggleAutoClutch() { bAutoClutch = !bAutoClutch; }

void AKartVehiclePawn::OnToggleAutopilot()
{
	bAutopilot = !bAutopilot;
	if (bAutopilot && Sim && Pilot.HasPath()) { Pilot.Reset(*Sim); }
}

void AKartVehiclePawn::ApplyVisualRig()
{
	// Remove parts from a previous rig (class switch / setup apply), restore procedural visibility.
	for (UStaticMeshComponent* P : RigParts) { if (P) { P->DestroyComponent(); } }
	RigParts.Reset();
	PanelParts.Reset();
	TieRods.Reset();
	bKingpinPivot = false;
	FrameMesh->SetVisibility(true);
	BodyMesh->SetVisibility(true);
	DriverMesh->SetVisibility(true);
	TorsoMesh->SetVisibility(true);
	for (UProceduralMeshComponent* A : ArmMeshes) { A->SetVisibility(true); }
	HelmetMesh->SetVisibility(true);
	SteeringWheelMesh->SetVisibility(true);
	AxleMesh->SetVisibility(true);
	for (UProceduralMeshComponent* W : WheelMeshes) { W->SetVisibility(true); }

	// Rig file: -KartVisual=<file> overrides the optional "visualFile" key of the kart config.
	FString RigFile;
	if (!FParse::Value(FCommandLine::Get(), TEXT("KartVisual="), RigFile))
	{
		FString KartText;
		const FString KartPath = FPaths::ProjectContentDir() / TEXT("KartData/Karts") / KartConfigFile;
		if (FFileHelper::LoadFileToString(KartText, *KartPath))
		{
			const kartsim::JsonParseResult R = kartsim::ParseJson(TCHAR_TO_UTF8(*KartText));
			if (const kartsim::JsonValue* V = R.bOk ? R.Root.Find("visualFile") : nullptr) { RigFile = UTF8_TO_TCHAR(V->StringValue.c_str()); }
		}
	}
	if (RigFile.IsEmpty()) { return; }
	FString RigText;
	const FString RigPath = FPaths::ProjectContentDir() / TEXT("KartData/Visuals") / RigFile;
	if (!FFileHelper::LoadFileToString(RigText, *RigPath))
	{
		UE_LOG(LogKart, Warning, TEXT("Visual rig not found: %s (procedural visuals kept)"), *RigPath);
		return;
	}
	const kartsim::JsonParseResult Rig = kartsim::ParseJson(TCHAR_TO_UTF8(*RigText));
	if (!Rig.bOk)
	{
		UE_LOG(LogKart, Warning, TEXT("Visual rig %s: %s"), *RigFile, UTF8_TO_TCHAR(Rig.Error.c_str()));
		return;
	}
	const kartsim::KartConfig& C = Sim->GetConfig();
	const double H = C.Chassis.CogHeight, B = C.Chassis.Wheelbase * C.Chassis.FrontWeightFraction;
	auto Vec = [](const kartsim::JsonValue* V, const FVector& Def) -> FVector
	{
		return (V && V->IsArray() && V->ArrayValue.size() == 3) ? FVector(V->ArrayValue[0].NumberValue, V->ArrayValue[1].NumberValue, V->ArrayValue[2].NumberValue) : Def;
	};
	if (const kartsim::JsonValue* Hide = Rig.Root.Find("hide"))
	{
		for (const kartsim::JsonValue& Hv : Hide->ArrayValue)
		{
			const FString Name = UTF8_TO_TCHAR(Hv.StringValue.c_str());
			if (Name == TEXT("frame")) { FrameMesh->SetVisibility(false); }
			else if (Name == TEXT("bodywork")) { BodyMesh->SetVisibility(false); }
			else if (Name == TEXT("driver")) { DriverMesh->SetVisibility(false); HelmetMesh->SetVisibility(false); TorsoMesh->SetVisibility(false); for (UProceduralMeshComponent* A : ArmMeshes) { A->SetVisibility(false); } }
			else if (Name == TEXT("driverBody")) { DriverMesh->SetVisibility(false); TorsoMesh->SetVisibility(false); for (int32 Ai = 0; Ai < 4 && Ai < ArmMeshes.Num(); ++Ai) { ArmMeshes[Ai]->SetVisibility(false); } }
			else if (Name == TEXT("helmetShell")) { HelmetMesh->SetVisibility(false); }
			else if (Name == TEXT("steeringWheel")) { SteeringWheelMesh->SetVisibility(false); }
			else if (Name == TEXT("axle")) { AxleMesh->SetVisibility(false); }
			else if (Name == TEXT("wheels")) { for (UProceduralMeshComponent* W : WheelMeshes) { W->SetVisibility(false); } }
			else { UE_LOG(LogKart, Warning, TEXT("Visual rig %s: unknown hide entry '%s'"), *RigFile, *Name); }
		}
	}
	// Optional steering column override (an imported steering wheel sits elsewhere than the procedural one):
	// "steeringColumn": { "hub": [x, y, z] kart frame, "axis": [x, y, z] kart frame, pointing up to the driver }.
	// Moves the steering wheel component and the driver's grips (hands follow the new rim).
	if (const kartsim::JsonValue* Col = Rig.Root.Find("steeringColumn"))
	{
		const FVector HubK = Vec(Col->Find("hub"), FVector::ZeroVector);
		const FVector AxisK = Vec(Col->Find("axis"), FVector(0, 0, 1));
		const FVector NewTop = KartConv::ToUEPos(kartsim::Vec3(HubK.X - B, HubK.Y, HubK.Z - H));
		const FVector NewAxis = KartConv::ToUEDir(kartsim::Vec3(AxisK.X, AxisK.Y, AxisK.Z)).GetSafeNormal();
		const FQuat Turn = FQuat::FindBetweenNormals(SteeringAxisLocal, NewAxis);
		if (DRig.bValid)
		{
			for (int32 S = 0; S < 2; ++S)
			{
				DRig.Grip[S] = NewTop + Turn.RotateVector(DRig.Grip[S] - DRig.ColTop);
				DRig.Wrist[S] = NewTop + Turn.RotateVector(DRig.Wrist[S] - DRig.ColTop);
				DRig.GripRot[S] = Turn * DRig.GripRot[S];
			}
			DRig.ColTop = NewTop;
			DRig.ColAxis = NewAxis;
		}
		SteeringAxisLocal = NewAxis;
		SteeringAlign = FRotationMatrix::MakeFromZX(NewAxis, FVector::ForwardVector).ToQuat();
		SteeringWheelMesh->SetRelativeLocationAndRotation(NewTop, SteeringAlign);
	}

	// Material classes for imported parts ("material" key): runtime instances of M_KartSurface. "body" takes the
	// kart's livery colour (BodyColor), so every kart in the field keeps its own colour.
	TMap<FString, UMaterialInterface*> ClassMats;
	auto ClassMaterial = [this, &ClassMats](const FString& Cls) -> UMaterialInterface*
	{
		if (UMaterialInterface** Found = ClassMats.Find(Cls)) { return *Found; }
		// R2S livery skin (user-supplied TGAs, Tools/CreateKartSkin.py): textured clear-coat on human karts. AI karts keep
		// their own colour so the field stays distinguishable (body paint / graphite steering wheel).
		static const TMap<FString, FString> SkinTex = {
			{TEXT("skinfront"), TEXT("T_Skin_Front")}, {TEXT("skinside"), TEXT("T_Skin_Side")}, {TEXT("skinrear"), TEXT("T_Skin_Rear")},
			{TEXT("swgrip"), TEXT("T_Skin_SwGrip")}, {TEXT("swbase"), TEXT("T_Skin_SwBase")}};
		if (const FString* TexName = SkinTex.Find(Cls))
		{
			UMaterialInterface* SkinBase = bIsAI ? nullptr : LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/KartVisuals/M_KartSkin.M_KartSkin"));
			UTexture* Tex = SkinBase ? LoadObject<UTexture>(nullptr, *FString::Printf(TEXT("/Game/KartVisuals/BogKart/Skin/%s.%s"), **TexName, **TexName)) : nullptr;
			if (SkinBase && Tex)
			{
				UMaterialInstanceDynamic* Skin = UMaterialInstanceDynamic::Create(SkinBase, this);
				Skin->SetTextureParameterValue(TEXT("SkinTex"), Tex);
				ClassMats.Add(Cls, Skin);
				return Skin;
			}
			// AI (or skin assets missing): fall through to the paint / graphite presets.
		}
		// Imported driver suit (Tools/DriverSuit/import_driver.py): its own PBR maps on human drivers; AI drivers keep a
		// flat suit in their kart colour (boots: black plastic) so the field stays distinguishable.
		if (Cls == TEXT("suittex") || Cls == TEXT("boottex"))
		{
			if (UMaterialInterface* SuitMat = bIsAI ? nullptr : LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/KartVisuals/Driver/M_DriverSuit.M_DriverSuit")))
			{
				ClassMats.Add(Cls, SuitMat);
				return SuitMat;
			}
		}
		const FString Effective = Cls == TEXT("suittex") ? TEXT("suit") : Cls == TEXT("boottex") ? TEXT("plastic")
			: Cls.StartsWith(TEXT("sw")) ? TEXT("graphite") : (Cls.StartsWith(TEXT("skin")) ? TEXT("body") : Cls);
		struct FPreset { const TCHAR* Name; FLinearColor A, B; float R, RV, Met; };
		const FLinearColor Body = BodyColor, BodyB = BodyColor * 0.92f;
		const FPreset Presets[] = {
			{TEXT("body"),     Body, BodyB, 0.24f, 0.05f, 0.0f},
			{TEXT("suit"),     BodyColor * 0.82f, BodyColor * 0.95f, 0.82f, 0.08f, 0.0f},   // Nomex fabric, the driver's colour
			{TEXT("chassis"),  FLinearColor(0.56f, 0.57f, 0.60f), FLinearColor(0.60f, 0.61f, 0.64f), 0.22f, 0.05f, 1.0f},
			{TEXT("chrome"),   FLinearColor(0.62f, 0.63f, 0.66f), FLinearColor(0.68f, 0.69f, 0.72f), 0.14f, 0.04f, 1.0f},
			{TEXT("alloy"),    FLinearColor(0.48f, 0.49f, 0.51f), FLinearColor(0.53f, 0.54f, 0.56f), 0.34f, 0.08f, 1.0f},
			{TEXT("anodized"), FLinearColor(0.035f, 0.035f, 0.04f), FLinearColor(0.045f, 0.045f, 0.05f), 0.32f, 0.06f, 0.8f},
			{TEXT("graphite"), FLinearColor(0.035f, 0.036f, 0.04f), FLinearColor(0.05f, 0.05f, 0.055f), 0.48f, 0.08f, 0.2f},
			{TEXT("plastic"),  FLinearColor(0.018f, 0.018f, 0.02f), FLinearColor(0.025f, 0.025f, 0.028f), 0.42f, 0.08f, 0.0f},
			{TEXT("rubber"),   FLinearColor(0.022f, 0.022f, 0.022f), FLinearColor(0.03f, 0.03f, 0.03f), 0.86f, 0.06f, 0.0f},
			{TEXT("display"),  FLinearColor(0.008f, 0.01f, 0.013f), FLinearColor(0.01f, 0.012f, 0.015f), 0.18f, 0.02f, 0.0f},
		};
		UMaterialInterface* M = nullptr;
		// Bodywork: clear-coat car paint with an object-space livery (/Game/KartVisuals/M_KartPaint, created by
		// Tools/CreateKartPaint.py). Secondary band = pearl white (dark graphite on very light karts); accent =
		// near-black swoosh and pinstripe (white on very dark karts).
		if (Effective == TEXT("body"))
		{
			if (UMaterialInterface* PaintBase = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/KartVisuals/M_KartPaint.M_KartPaint")))
			{
				UMaterialInstanceDynamic* Paint = UMaterialInstanceDynamic::Create(PaintBase, this);
				const float Luma = BodyColor.GetLuminance();
				const FLinearColor Pearl(0.82f, 0.83f, 0.85f), Graphite(0.05f, 0.05f, 0.055f), Ink(0.015f, 0.015f, 0.018f);
				Paint->SetVectorParameterValue(TEXT("PrimaryColor"), BodyColor);
				Paint->SetVectorParameterValue(TEXT("SecondaryColor"), Luma > 0.55f ? Graphite : Pearl);
				Paint->SetVectorParameterValue(TEXT("AccentColor"), Luma < 0.03f ? Pearl : Ink);
				M = Paint;
			}
		}
		for (const FPreset& P : Presets)
		{
			if (!M && Effective == P.Name) { M = KartVisuals::MakeSurface(this, P.A, P.B, P.R, P.RV, P.Met, 0.004f, 0.5f, 0.06f); break; }
		}
		if (!M) { UE_LOG(LogKart, Warning, TEXT("Visual rig: unknown material class '%s'"), *Cls); }
		ClassMats.Add(Cls, M);
		return M;
	};

	// "kingpins": { "L": [x, y, z], "R": [...] } kart frame: chassis-fixed spindle pivots (see ApplyRender).
	if (const kartsim::JsonValue* Kp = Rig.Root.Find("kingpins"))
	{
		const kartsim::JsonValue* KL = Kp->Find("L");
		const kartsim::JsonValue* KR = Kp->Find("R");
		if (KL && KR)
		{
			const FVector L = Vec(KL, FVector::ZeroVector), R = Vec(KR, FVector::ZeroVector);
			KingpinLocal[0] = KartConv::ToUEPos(kartsim::Vec3(L.X - B, L.Y, L.Z - H));
			KingpinLocal[1] = KartConv::ToUEPos(kartsim::Vec3(R.X - B, R.Y, R.Z - H));
			bKingpinPivot = true;
		}
	}
	// Driver parts are given in the kart frame at the rest pose: pose the procedural driver at rest first, so each
	// animated parent's current relative transform is its rest transform.
	if (DRig.bValid) { AnimRoll = AnimPitch = AnimYaw = 0.0f; bAnimHasPrev = false; AnimateDriver(0.0f, 0.0f); }
	int32 Loaded = 0;
	if (const kartsim::JsonValue* Parts = Rig.Root.Find("parts"))
	{
		for (const kartsim::JsonValue& Pv : Parts->ArrayValue)
		{
			const kartsim::JsonValue* MeshV = Pv.Find("mesh");
			const kartsim::JsonValue* AttachV = Pv.Find("attach");
			if (!MeshV || !AttachV) { continue; }
			UStaticMesh* Mesh = LoadObject<UStaticMesh>(nullptr, UTF8_TO_TCHAR(MeshV->StringValue.c_str()));
			if (!Mesh)
			{
				UE_LOG(LogKart, Warning, TEXT("Visual rig %s: mesh not found %s"), *RigFile, UTF8_TO_TCHAR(MeshV->StringValue.c_str()));
				continue;
			}
			const FString Attach = UTF8_TO_TCHAR(AttachV->StringValue.c_str());
			const kartsim::JsonValue* AlignedV = Pv.Find("kartAligned");
			const bool bKartAligned = AlignedV && AlignedV->BoolValue;
			USceneComponent* Parent = Root;
			int32 WheelIndex = INDEX_NONE;
			bool bTieRod = false;
			bool bDriverPart = false, bHelmetPart = false;
			FQuat ParentRest = FQuat::Identity; // parent's rest rotation relative to the kart root (kartAligned parts)
			const FVector LocM = Vec(Pv.Find("location"), FVector::ZeroVector);
			FVector LocalPos;
			if (Attach == TEXT("chassis") || Attach == TEXT("driver"))
			{
				// Kart frame (metres forward of the rear axle, y left, z up from ground) -> UE local.
				LocalPos = KartConv::ToUEPos(kartsim::Vec3(LocM.X - B, LocM.Y, LocM.Z - H));
			}
			else
			{
				static const TCHAR* WheelKeys[] = {TEXT("wheelFL"), TEXT("wheelFR"), TEXT("wheelRL"), TEXT("wheelRR")};
				for (int32 I = 0; I < kartsim::NumWheels; ++I) { if (Attach == WheelKeys[I]) { Parent = WheelMeshes[I]; WheelIndex = I; } }
				if (Attach == TEXT("steeringWheel")) { Parent = SteeringWheelMesh; ParentRest = SteeringAlign; }
				bool bWheelFrame = WheelIndex != INDEX_NONE;
				if (Attach == TEXT("uprightFL")) { Parent = UprightFL; bWheelFrame = true; }
				if (Attach == TEXT("uprightFR")) { Parent = UprightFR; bWheelFrame = true; }
				if (Attach == TEXT("rearAxle")) { Parent = RearAxlePivot; bWheelFrame = true; }
				// Imported driver model: "location" / "rotationDeg" in the kart frame at the rest pose (like chassis parts),
				// converted into the animated parent's rest frame so the part follows the lean / head / steering animation.
				static const TCHAR* DriverKeys[] = {TEXT("driverBody"), TEXT("torso"), TEXT("helmet"), TEXT("upperArmL"), TEXT("upperArmR"), TEXT("forearmL"), TEXT("forearmR")};
				int32 DriverKey = INDEX_NONE;
				for (int32 Dk = 0; Dk < UE_ARRAY_COUNT(DriverKeys); ++Dk) { if (Attach == DriverKeys[Dk]) { DriverKey = Dk; } }
				if (DriverKey != INDEX_NONE)
				{
					USceneComponent* DriverParents[] = {DriverMesh, TorsoMesh, HelmetMesh, ArmMeshes[0], ArmMeshes[1], ArmMeshes[2], ArmMeshes[3]};
					Parent = DriverParents[DriverKey];
					bDriverPart = true;
					bHelmetPart = DriverKey == 2;
				}
				bTieRod = Attach == TEXT("tieRodL") || Attach == TEXT("tieRodR");
				if (bTieRod)
				{
					// location = outer end (kart frame); "inner" = column-arm end (kart frame); "outerFromHub" = outer end
					// relative to the model's front hub (kart frame, m). Placed every frame by ApplyRender.
					FTieRod T;
					T.Side = Attach == TEXT("tieRodL") ? 0 : 1;
					const FVector InK = Vec(Pv.Find("inner"), FVector::ZeroVector);
					// Outer end relative to the upright origin: the kingpin ("outerFromKingpin") or, for older rigs, the hub.
					const kartsim::JsonValue* OffV = Pv.Find("outerFromKingpin");
					const FVector OffK = Vec(OffV ? OffV : Pv.Find("outerFromHub"), FVector::ZeroVector);
					const FVector OuterRest = KartConv::ToUEPos(kartsim::Vec3(LocM.X - B, LocM.Y, LocM.Z - H));
					T.InnerRest = KartConv::ToUEPos(kartsim::Vec3(InK.X - B, InK.Y, InK.Z - H));
					T.RestDir = T.InnerRest - OuterRest;
					const FQuat UpRest = FRotationMatrix::MakeFromZX(FVector(0, -1, 0), FVector::ForwardVector).ToQuat();
					T.OuterLocal = UpRest.Inverse().RotateVector(KartConv::ToUEDir(kartsim::Vec3(OffK.X, OffK.Y, OffK.Z)) * 100.0);
					TieRods.Add(T);
					Parent = Root;
				}
				if (Parent == Root && !bTieRod && !bDriverPart)
				{
					UE_LOG(LogKart, Warning, TEXT("Visual rig %s: unknown attach '%s'"), *RigFile, *Attach);
					continue;
				}
				// Wheels / uprights / rear axle at rest: local Z = axle (kart left), local X = forward (see Capture).
				if (bWheelFrame) { ParentRest = FRotationMatrix::MakeFromZX(FVector(0, -1, 0), FVector::ForwardVector).ToQuat(); }
				// kartAligned: location / rotation are given in the kart frame (metres, y left) at the parent's origin.
				LocalPos = bTieRod ? KartConv::ToUEPos(kartsim::Vec3(LocM.X - B, LocM.Y, LocM.Z - H))
				         : (bKartAligned ? ParentRest.Inverse().RotateVector(KartConv::ToUEDir(kartsim::Vec3(LocM.X, LocM.Y, LocM.Z)) * 100.0) : LocM * 100.0);
			}
			const FVector Rot = Vec(Pv.Find("rotationDeg"), FVector::ZeroVector);
			FQuat RelRot = FRotator(Rot.X, Rot.Y, Rot.Z).Quaternion();
			if (bKartAligned) { RelRot = ParentRest.Inverse() * RelRot; }
			FVector Scale = Vec(Pv.Find("scale"), FVector::OneVector);
			// fitRadius (m): the model's tyre radius; the part is scaled uniformly to the configured tyre radius so the
			// imported wheel matches the simulated contact patch height for every class.
			if (const kartsim::JsonValue* FitV = Pv.Find("fitRadius"); FitV && WheelIndex != INDEX_NONE && FitV->NumberValue > 0.0)
			{
				const kartsim::TyreParams& T = WheelIndex < 2 ? C.FrontTyre : C.RearTyre;
				Scale *= static_cast<float>(T.UnloadedRadius / FitV->NumberValue);
			}
			if (bDriverPart)
			{
				// kart-frame rest transform -> parent rest frame
				const FTransform Want(RelRot, KartConv::ToUEPos(kartsim::Vec3(LocM.X - B, LocM.Y, LocM.Z - H)), Scale);
				const FTransform Rel = Want.GetRelativeTransform(Parent->GetRelativeTransform());
				RelRot = Rel.GetRotation(); LocalPos = Rel.GetLocation();
			}
			UStaticMeshComponent* Part = NewObject<UStaticMeshComponent>(this);
			if (bHelmetPart) { Part->ComponentTags.Add(TEXT("KartHelmetPart")); }
			if (const kartsim::JsonValue* MatTag = Pv.Find("material"); MatTag && MatTag->StringValue == "display") { Part->ComponentTags.Add(TEXT("KartDisplayPart")); }
			Part->SetStaticMesh(Mesh);
			Part->SetCollisionEnabled(ECollisionEnabled::NoCollision);
			Part->SetupAttachment(Parent);
			Part->RegisterComponent();
			Part->SetRelativeTransform(FTransform(RelRot, LocalPos, Scale));
			if (const kartsim::JsonValue* MatV = Pv.Find("material"))
			{
				if (UMaterialInterface* M = ClassMaterial(UTF8_TO_TCHAR(MatV->StringValue.c_str())))
				{
					for (int32 Slot = 0; Slot < Part->GetNumMaterials(); ++Slot) { Part->SetMaterial(Slot, M); }
				}
			}
			RigParts.Add(Part);
			if (const kartsim::JsonValue* PanelV = Pv.Find("panel"))
			{
				const std::string& K = PanelV->StringValue;
				const int32 Kind = K == "nose" ? 0 : (K == "sidepodL" ? 1 : (K == "sidepodR" ? 2 : (K == "bumper" ? 3 : -1)));
				if (Kind >= 0) { FPanelPart Pp; Pp.C = Part; Pp.Rest = Part->GetRelativeTransform(); Pp.Kind = Kind; PanelParts.Add(Pp); }
			}
			if (bTieRod)
			{
				// Several meshes (one per material class) per rod: each gets its own entry with the same geometry data.
				FTieRod& Last = TieRods.Last();
				if (Last.C) { FTieRod Copy = Last; Copy.C = Part; TieRods.Add(Copy); } else { Last.C = Part; }
			}
			++Loaded;
		}
	}
	UE_LOG(LogKart, Log, TEXT("Visual rig %s: %d part(s) loaded"), *RigFile, Loaded);
	ApplyCockpitHelmetVisibility();
}

void AKartVehiclePawn::RunSurfaceSelfTest()
{
	// Probes the surface-resolution paths: untagged asphalt (default), grass and kerb via component tags
	// on the procedural track, and the imported-art convention (static mesh asset named
	// ..._KartSurface_Gravel, created by Tools/CreateSurfaceTestAsset.py) on a pad 60 m off the start.
	// Called twice: the first call spawns the pad; physics bodies are created asynchronously, so the
	// probes run on the second call ~1.5 s later.
	for (TActorIterator<AKartTestTrackActor> It(GetWorld()); It; ++It)
	{
		TArray<FVector> A, Bl, Fwd, Centre;
		It->GetTimingLines(A, Bl, Fwd);
		It->GetCentreLine(Centre);
		if (A.Num() == 0 || Centre.Num() == 0) { break; }
		// 60 m off the start line, resting on the terrain (which is no longer flat: track elevation / hills).
		FVector PadPos = Centre[0] + FVector(0.0, -6000.0, 0.0);
		{
			FHitResult GroundHit;
			FCollisionQueryParams Q(SCENE_QUERY_STAT(KartPadGround), true, this);
			if (GetWorld()->LineTraceSingleByChannel(GroundHit, PadPos + FVector(0, 0, 5000.0), PadPos - FVector(0, 0, 5000.0), ECC_WorldStatic, Q))
			{
				PadPos.Z = GroundHit.ImpactPoint.Z + 12.0; // the 20 cm pad's base 2 cm above the ground
			}
		}
		if (!bSelfTestPadSpawned)
		{
			bSelfTestPadSpawned = true;
			UStaticMesh* PadAsset = LoadObject<UStaticMesh>(nullptr, TEXT("/Game/KartVisuals/Test/SM_Pad_KartSurface_Gravel.SM_Pad_KartSurface_Gravel"));
			if (!PadAsset)
			{
				UE_LOG(LogKart, Warning, TEXT("SURFACE SELFTEST: test asset missing (run Tools/CreateSurfaceTestAsset.py)"));
				return;
			}
			FActorSpawnParameters SP;
			SP.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
			if (AActor* Pad = GetWorld()->SpawnActor<AActor>(AActor::StaticClass(), FTransform::Identity, SP))
			{
				// Everything set BEFORE registration so the physics body is created with mesh + collision.
				UStaticMeshComponent* PadMesh = NewObject<UStaticMeshComponent>(Pad, TEXT("SurfacePad"));
				PadMesh->SetMobility(EComponentMobility::Movable);
				PadMesh->SetStaticMesh(PadAsset);
				PadMesh->SetWorldTransform(FTransform(FRotator::ZeroRotator, PadPos, FVector(4.0, 4.0, 0.2)));
				PadMesh->SetCollisionObjectType(ECC_WorldStatic);
				PadMesh->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
				PadMesh->SetCollisionResponseToAllChannels(ECR_Block);
				Pad->SetRootComponent(PadMesh);
				PadMesh->RegisterComponent();
			}
			return;
		}
		FVector Kerb;
		const bool bKerb = It->GetKerbProbe(Kerb);
		struct FProbe { const TCHAR* Name; FVector P; kartsim::ESurfaceType Expect; };
		int32 Fails = 0;
		TArray<FProbe> Probes = {{TEXT("track centre"), Centre[0], kartsim::ESurfaceType::Asphalt},
		                         {TEXT("run-off"), A[0], kartsim::ESurfaceType::Grass},
		                         {TEXT("static mesh *_KartSurface_Gravel"), PadPos + FVector(0.0, 0.0, 10.0), kartsim::ESurfaceType::Gravel}};
		if (bKerb) { Probes.Add({TEXT("kerb"), Kerb, kartsim::ESurfaceType::Kerb}); }
		for (const FProbe& P : Probes)
		{
			kartsim::GroundHit Hit;
			const bool bHit = TrackQuery.QueryGround(KartConv::FromUEPos(P.P), 1.0, 1.0, Hit);
			const bool bOk = bHit && Hit.Surface == P.Expect;
			Fails += bOk ? 0 : 1;
			{
				// Diagnostics: what the raw trace hit.
				FHitResult Raw;
				FCollisionQueryParams Qp(SCENE_QUERY_STAT(KartSelfTest), true, this);
				Qp.bReturnPhysicalMaterial = true;
				Qp.bReturnFaceIndex = true;
				GetWorld()->LineTraceSingleByChannel(Raw, P.P + FVector(0, 0, 100), P.P - FVector(0, 0, 100), ECC_WorldStatic, Qp);
				UE_LOG(LogKart, Log, TEXT("   trace: component %s, physmat %s, face %d"), Raw.GetComponent() ? *Raw.GetComponent()->GetName() : TEXT("none"),
				       Raw.PhysMaterial.IsValid() ? *Raw.PhysMaterial->GetName() : TEXT("null"), Raw.FaceIndex);
			}
			UE_LOG(LogKart, Log, TEXT("SURFACE SELFTEST %s -> %s (expected %s) %s"), P.Name,
			       bHit ? UTF8_TO_TCHAR(kartsim::SurfaceName(Hit.Surface)) : TEXT("no hit"),
			       UTF8_TO_TCHAR(kartsim::SurfaceName(P.Expect)), bOk ? TEXT("OK") : TEXT("FAIL"));
		}
		UE_LOG(LogKart, Log, TEXT("SURFACE SELFTEST %s"), Fails == 0 ? TEXT("PASS") : TEXT("FAILED"));
		return;
	}
	UE_LOG(LogKart, Warning, TEXT("SURFACE SELFTEST: no test track found"));
}

void AKartVehiclePawn::ConfigureLapTiming()
{
	bTimingConfigured = false;
	for (TActorIterator<AKartTestTrackActor> It(GetWorld()); It; ++It)
	{
		TArray<FVector> A, B, Fwd;
		It->GetTimingLines(A, B, Fwd);
		if (A.Num() == 0) { continue; }
		kartsim::TimingLine Lines[3];
		const int32 N = FMath::Min(A.Num(), 3);
		for (int32 I = 0; I < N; ++I)
		{
			Lines[I] = kartsim::TimingLine{KartConv::FromUEPos(A[I]), KartConv::FromUEPos(B[I]), KartConv::FromUEDir(Fwd[I])};
		}
		LapTimer.Configure(Lines, N);
		bTimingConfigured = true;
		LoggedLaps = 0;
		TArray<FVector> Centre;
		It->GetCentreLine(Centre);
		std::vector<kartsim::Vec3> Path;
		Path.reserve(Centre.Num());
		for (const FVector& P : Centre) { Path.push_back(KartConv::FromUEPos(P)); }
		Pilot.SetPath(Path);
		LapAnalysis.Configure(Centre);
		break;
	}
	if (!bIsAI && !bPuppet) { LoadGhost(); }
}

FString AKartVehiclePawn::SetupFilePath(const std::string& ClassId) const
{
	return FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("Setups") / (FString(UTF8_TO_TCHAR(ClassId.c_str())) + TEXT(".json")));
}

void AKartVehiclePawn::OnToggleSetup()
{
	if (!Sim || (RaceWorld.IsValid() && RaceWorld->IsReplaying())) { return; }
	bSetupOpen = !bSetupOpen;
	if (bSetupOpen)
	{
		EditConfig = Sim->GetConfig();
		SetupMessage = TEXT("Up/Down select, Left/Right adjust, Enter apply + save, Backspace defaults, Tab close");
	}
}

void AKartVehiclePawn::UpdateSetupMenu()
{
	const APlayerController* PC = Cast<APlayerController>(GetController());
	if (!PC) { return; }
	int32 Count = 0;
	const kartsim::SetupParam* Params = kartsim::GetSetupParams(Count);
	auto Pressed = [PC](const FKey& K) { return PC->WasInputKeyJustPressed(K); };
	// Skip settings this class does not have (e.g. front brake share on a Rotax).
	auto Move = [&](int32 Dir) { for (int32 N = 0; N < Count; ++N) { SetupCursor = (SetupCursor + Count + Dir) % Count; if (Params[SetupCursor].IsAvailable(EditConfig)) { break; } } };
	if (Pressed(EKeys::Up) || Pressed(EKeys::W)) { Move(-1); }
	if (Pressed(EKeys::Down) || Pressed(EKeys::S)) { Move(1); }
	int32 Dir = 0;
	if (Pressed(EKeys::Left) || Pressed(EKeys::A)) { Dir = -1; }
	if (Pressed(EKeys::Right) || Pressed(EKeys::D)) { Dir = 1; }
	if (Dir != 0)
	{
		const kartsim::SetupParam& P = Params[SetupCursor];
		kartsim::SetSetupValue(EditConfig, SetupCursor, P.Get(EditConfig) + Dir * P.Step);
	}
	if (Pressed(EKeys::BackSpace))
	{
		EditConfig = DefaultConfig;
		SetupMessage = TEXT("Class defaults loaded (Enter to apply)");
	}
	if (Pressed(EKeys::Enter))
	{
		ApplySetup();
	}
}

void AKartVehiclePawn::GarageSetValue(int32 Index, double Value)
{
	kartsim::SetSetupValue(EditConfig, Index, Value);
}

void AKartVehiclePawn::ApplySetup()
{
	CaptureTyreWear(); // the set on the kart keeps its wear across a setup change
	ReinitFromEditConfig(true);
}

void AKartVehiclePawn::ReinitFromEditConfig(bool bSaveSetup)
{
	bool bSaved = false;
	const FString Path = SetupFilePath(EditConfig.ClassId);
	if (bSaveSetup)
	{
		IFileManager::Get().MakeDirectory(*FPaths::GetPath(Path), true);
		bSaved = FFileHelper::SaveStringToFile(FString(UTF8_TO_TCHAR(kartsim::SerializeSetup(EditConfig).c_str())), *Path);
	}
	Sim = MakeUnique<kartsim::KartSimulation>();
	Sim->Initialize(EditConfig);
	Sim->Settings.Substeps = Substeps;
	RestoreTyreWear();
	SaveTyreSets();
	BuildVisuals();
	ResetKart();
	if (bSaveSetup)
	{
		SetupMessage = bSaved ? FString::Printf(TEXT("Applied and saved: %s"), *FPaths::GetCleanFilename(Path)) : FString(TEXT("Applied (save FAILED)"));
		UE_LOG(LogKart, Log, TEXT("%s"), *SetupMessage);
	}
}

// --- Tyre sets ---

bool AKartVehiclePawn::OwnsTyreSets() const
{
	// Only this machine's own kart (a remote player's kart here is a proxy and must not touch the file).
	return !bIsAI && IsLocallyControlled() && !FParse::Param(FCommandLine::Get(), TEXT("KartDefaultSetup"));
}

static FString TyreSetFilePath(const std::string& ClassId)
{
	return FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("Setups") / (FString(UTF8_TO_TCHAR(ClassId.c_str())) + TEXT(".tyres.json")));
}

void AKartVehiclePawn::LoadTyreSets()
{
	FMemory::Memzero(TyreSetWear);
	TyreSetClassId = Sim ? Sim->GetConfig().ClassId : std::string();
	FString Text;
	if (bIsAI || FParse::Param(FCommandLine::Get(), TEXT("KartDefaultSetup")) || TyreSetClassId.empty() || !FFileHelper::LoadFileToString(Text, *TyreSetFilePath(TyreSetClassId))) { return; }
	const kartsim::JsonParseResult R = kartsim::ParseJson(TCHAR_TO_UTF8(*Text));
	const kartsim::JsonValue* Sets = R.bOk ? R.Root.Find("wear") : nullptr;
	if (!Sets || !Sets->IsArray()) { UE_LOG(LogKart, Warning, TEXT("Tyre sets file ignored (%s)"), *TyreSetFilePath(TyreSetClassId)); return; }
	for (int32 K = 0; K < kartsim::KartConfig::MaxCompounds && K < static_cast<int32>(Sets->ArrayValue.size()); ++K)
	{
		const kartsim::JsonValue& Set = Sets->ArrayValue[K];
		for (int32 W = 0; W < kartsim::NumWheels && Set.IsArray() && W < static_cast<int32>(Set.ArrayValue.size()); ++W)
		{
			TyreSetWear[K][W] = FMath::Clamp(Set.ArrayValue[W].NumberValue, 0.0, 1.0);
		}
	}
	UE_LOG(LogKart, Log, TEXT("Loaded used tyre sets for %s (dry set RL wear %.1f%%)"), UTF8_TO_TCHAR(TyreSetClassId.c_str()), TyreSetWear[0][kartsim::WheelRL] * 100.0);
}

void AKartVehiclePawn::SaveTyreSets() const
{
	if (!OwnsTyreSets() || TyreSetClassId.empty()) { return; }
	FString S = FString::Printf(TEXT("{\n  \"classId\": \"%s\",\n  \"wear\": ["), UTF8_TO_TCHAR(TyreSetClassId.c_str()));
	for (int32 K = 0; K < kartsim::KartConfig::MaxCompounds; ++K)
	{
		S += FString::Printf(TEXT("%s[%.6f, %.6f, %.6f, %.6f]"), K ? TEXT(", ") : TEXT(""), TyreSetWear[K][0], TyreSetWear[K][1], TyreSetWear[K][2], TyreSetWear[K][3]);
	}
	S += TEXT("]\n}\n");
	const FString Path = TyreSetFilePath(TyreSetClassId);
	IFileManager::Get().MakeDirectory(*FPaths::GetPath(Path), true);
	FFileHelper::SaveStringToFile(S, *Path);
}

void AKartVehiclePawn::CaptureTyreWear()
{
	// Before the saved set is on the kart, its (new) tyres must not overwrite the stored wear.
	if (!Sim || Sim->GetConfig().ClassId != TyreSetClassId || (!bIsAI && !bTyreSetRestored)) { return; }
	const int32 K = Sim->GetConfig().Compound;
	for (int32 W = 0; W < kartsim::NumWheels; ++W) { TyreSetWear[K][W] = Sim->GetTyreModel(W).GetWear(); }
}

void AKartVehiclePawn::RestoreTyreWear()
{
	if (!Sim || !OwnsTyreSets() || Sim->GetConfig().ClassId != TyreSetClassId) { return; }
	const int32 K = Sim->GetConfig().Compound;
	for (int32 W = 0; W < kartsim::NumWheels; ++W) { Sim->SetTyreWear(W, TyreSetWear[K][W]); }
}

double AKartVehiclePawn::GetSetTyreWear(int32 Compound, int32 Wheel) const
{
	if (Compound < 0 || Compound >= kartsim::KartConfig::MaxCompounds || Wheel < 0 || Wheel >= kartsim::NumWheels) { return 0.0; }
	// The set on the kart: live value; the other set: as stored.
	if (Sim && Sim->GetConfig().Compound == Compound) { return Sim->GetTyreModel(Wheel).GetWear(); }
	return TyreSetWear[Compound][Wheel];
}

void AKartVehiclePawn::FitNewTyres()
{
	CaptureTyreWear();
	for (int32 W = 0; W < kartsim::NumWheels; ++W) { TyreSetWear[EditConfig.Compound][W] = 0.0; }
	ReinitFromEditConfig(true);
	SetupMessage = FString::Printf(TEXT("New %s fitted (setup applied)"), UTF8_TO_TCHAR(EditConfig.CompoundName[EditConfig.Compound].c_str()));
}

void AKartVehiclePawn::FitTyreCompound(int32 Compound)
{
	if (!Sim || Compound < 0 || Compound >= Sim->GetConfig().NumCompounds || Sim->GetConfig().Compound == Compound) { return; }
	CaptureTyreWear();
	EditConfig = Sim->GetConfig();
	EditConfig.FitTyres(Compound);
	ReinitFromEditConfig(false);
	UE_LOG(LogKart, Log, TEXT("%s: fitted %s"), *DriverName, UTF8_TO_TCHAR(EditConfig.CompoundName[Compound].c_str()));
}

void AKartVehiclePawn::SwitchKart(int32 Index)
{
	if (!AvailableKarts.IsValidIndex(Index) || AvailableKarts[Index] == KartConfigFile)
	{
		return;
	}
	const FString Previous = KartConfigFile;
	KartConfigFile = AvailableKarts[Index];
	CaptureTyreWear();
	SaveTyreSets();
	TUniquePtr<kartsim::KartSimulation> OldSim = MoveTemp(Sim);
	if (InitSimulation())
	{
		BuildVisuals();
		ConfigureLapTiming(); // new class: fresh lap history
		ResetKart();
		UE_LOG(LogKart, Log, TEXT("Switched kart class to %s"), *KartConfigFile);
		if (HasAuthority()) { NetKartConfig = KartConfigFile; } // replicated to the other machines' proxies
	}
	else
	{
		// Keep driving the previous kart if the new config fails to load (error shown on HUD).
		KartConfigFile = Previous;
		Sim = MoveTemp(OldSim);
	}
}

void AKartVehiclePawn::OnToggleCamera()
{
	bCockpitCamera = !bCockpitCamera;
	CockpitCamera->SetActive(bCockpitCamera);
	ChaseCamera->SetActive(!bCockpitCamera);
	// Hide the driver's helmet from the cockpit view.
	ApplyCockpitHelmetVisibility();
}

void AKartVehiclePawn::ApplyCockpitHelmetVisibility()
{
	HelmetMesh->SetOwnerNoSee(bCockpitCamera);
	for (UStaticMeshComponent* P : RigParts) { if (P && P->ComponentHasTag(TEXT("KartHelmetPart"))) { P->SetOwnerNoSee(bCockpitCamera); } }
}

void AKartVehiclePawn::ToggleRecording()
{
	if (IsRecording())
	{
		Recorder->Close();
		UE_LOG(LogKart, Log, TEXT("Telemetry recording saved: %s"), *RecordingPath);
		return;
	}
	const FString Dir = FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("Telemetry"));
	IFileManager::Get().MakeDirectory(*Dir, true);
	RecordingPath = Dir / FString::Printf(TEXT("kart_%s.csv"), *FDateTime::Now().ToString(TEXT("%Y%m%d_%H%M%S")));
	Recorder = MakeUnique<kartsim::CsvTelemetryWriter>();
	if (!Recorder->Open(TCHAR_TO_UTF8(*RecordingPath)))
	{
		UE_LOG(LogKart, Error, TEXT("Cannot open telemetry file %s"), *RecordingPath);
		Recorder.Reset();
	}
}

// ------------------------------------------------------------------------------------ debug

void AKartVehiclePawn::DrawPhysicsDebug() const
{
	UWorld* W = GetWorld();
	if (!W || !Sim)
	{
		return;
	}
	const kartsim::ChassisState& Ch = Sim->GetChassis();
	const FVector Cog = KartConv::ToUEPos(Ch.Position);

	// Centre of gravity and velocity (10 cm per m/s).
	DrawDebugSphere(W, Cog, 6.0f, 10, FColor::Yellow, false, -1.0f, SDPG_Foreground);
	DrawDebugDirectionalArrow(W, Cog, Cog + KartConv::ToUEDir(Ch.Velocity) * 10.0, 12.0f, FColor::Cyan, false, -1.0f, SDPG_Foreground, 1.5f);

	for (int32 I = 0; I < kartsim::NumWheels; ++I)
	{
		const kartsim::WheelState& Wh = Sim->GetWheel(I);
		const FVector P = KartConv::ToUEPos(Wh.Contact.ContactPoint);
		if (Wh.bLifted)
		{
			DrawDebugSphere(W, P, 5.0f, 8, FColor::Magenta, false, -1.0f, SDPG_Foreground);
			continue;
		}
		// Vertical load: 1 cm per 10 N.
		DrawDebugLine(W, P, P + KartConv::ToUEDir(Wh.Contact.Normal) * (Wh.Tyre.Fz * 0.1), FColor::Blue, false, -1.0f, SDPG_Foreground, 2.0f);
		// Horizontal tyre force: 1 cm per 10 N, coloured by combined slip (green grip -> red sliding).
		const kartsim::Vec3 Fh = Wh.Contact.Forward * Wh.Tyre.Fx + Wh.Contact.Lateral * Wh.Tyre.Fy;
		const float Lam = static_cast<float>(FMath::Clamp(Wh.Tyre.CombinedSlip, 0.0, 1.0));
		const FColor Col = FLinearColor::LerpUsingHSV(FLinearColor::Green, FLinearColor::Red, Lam).ToFColor(true);
		DrawDebugDirectionalArrow(W, P, P + KartConv::ToUEDir(Fh) * 0.1, 8.0f, Col, false, -1.0f, SDPG_Foreground, 2.0f);
		// Contact patch outline (2a x width).
		const kartsim::TyreParams& T = I < 2 ? Sim->GetConfig().FrontTyre : Sim->GetConfig().RearTyre;
		const FVector Fwd = KartConv::ToUEDir(Wh.Contact.Forward);
		const FQuat PatchRot = FRotationMatrix::MakeFromXZ(Fwd, KartConv::ToUEDir(Wh.Contact.Normal)).ToQuat();
		DrawDebugBox(W, P, FVector(Wh.Contact.HalfLength * 100.0, T.Width * 50.0, 0.3), PatchRot, Col, false, -1.0f, SDPG_Foreground, 1.0f);
	}

	// Kingpin axes.
	const kartsim::ChassisModel& Model = Sim->GetChassisModel();
	for (int32 Side = 0; Side < 2; ++Side)
	{
		const kartsim::KingpinGeometry& G = Sim->GetSteeringModel().GetGeometry(Side);
		const FVector A = KartConv::ToUEPos(Model.BodyToWorldPoint(G.Pivot - G.Axis * 0.15));
		const FVector B = KartConv::ToUEPos(Model.BodyToWorldPoint(G.Pivot + G.Axis * 0.15));
		DrawDebugLine(W, A, B, FColor::Orange, false, -1.0f, SDPG_Foreground, 1.5f);
	}
}

void AKartVehiclePawn::UpdateWorldKeys()
{
	const APlayerController* PC = Cast<APlayerController>(GetController());
	if (!PC) { return; }
	if (RaceWorld.IsValid() && RaceWorld->IsReplaying()) { return; } // replay keys belong to the replay
	AKartConditionsActor* C = Conditions.Get();
	if (C && GetNetMode() != NM_Client) // the host sets the weather; clients follow it
	{
		if (PC->WasInputKeyJustPressed(EKeys::N)) { C->CycleWeather(); }
		if (PC->WasInputKeyJustPressed(EKeys::M)) { C->ToggleRubber(); }
	}
	// Carburettor on the fly: [ leaner, ] richer (one jet size / 1/16 needle turn), saved with the setup.
	if (Sim && (PC->WasInputKeyJustPressed(EKeys::LeftBracket) || PC->WasInputKeyJustPressed(EKeys::RightBracket)))
	{
		const kartsim::EngineTuneParams& T = Sim->GetConfig().Tuning;
		const double Dir = PC->WasInputKeyJustPressed(EKeys::RightBracket) ? 1.0 : -1.0;
		const double Jet = FMath::Clamp(T.Jet + Dir * T.JetStep, T.JetMin, T.JetMax);
		Sim->SetCarb(Jet, T.Cover);
		EditConfig.Tuning.Jet = Jet;
		JetChangeTime = Sim->GetTime();
		FFileHelper::SaveStringToFile(FString(UTF8_TO_TCHAR(kartsim::SerializeSetup(Sim->GetConfig()).c_str())), *SetupFilePath(Sim->GetConfig().ClassId));
	}
	// Cockpit camera adjustment (saved): Page Up/Down pitch, Home/End field of view.
	{
		float DP = 0.0f, DF = 0.0f;
		if (PC->WasInputKeyJustPressed(EKeys::PageUp)) { DP = 2.0f; }
		if (PC->WasInputKeyJustPressed(EKeys::PageDown)) { DP = -2.0f; }
		if (PC->WasInputKeyJustPressed(EKeys::Home)) { DF = 5.0f; }
		if (PC->WasInputKeyJustPressed(EKeys::End)) { DF = -5.0f; }
		if (DP != 0.0f || DF != 0.0f)
		{
			CockpitPitch = FMath::Clamp(CockpitPitch + DP, -30.0f, 10.0f);
			CockpitFov = FMath::Clamp(CockpitFov + DF, 60.0f, 130.0f);
			CockpitCamera->SetRelativeRotation(FRotator(CockpitPitch, 0.0f, 0.0f));
			CockpitCamera->FieldOfView = CockpitFov;
			if (GConfig)
			{
				GConfig->SetFloat(TEXT("KartCamera"), TEXT("CockpitPitch"), CockpitPitch, GGameUserSettingsIni);
				GConfig->SetFloat(TEXT("KartCamera"), TEXT("CockpitFov"), CockpitFov, GGameUserSettingsIni);
				GConfig->Flush(false, GGameUserSettingsIni);
			}
		}
	}
	if (PC->WasInputKeyJustPressed(EKeys::Y))
	{
		if (AKartRaceWorld* W = RaceWorld.Get()) { W->RequestNextSession(); }
	}
	if (PC->WasInputKeyJustPressed(EKeys::L))
	{
		bAnalysisOpen = !bAnalysisOpen;
		AnalysisLap = LapAnalysis.GetDelta().History().empty() ? -1 : static_cast<int32>(LapAnalysis.GetDelta().History().size()) - 1;
		AnalysisRefLap = -1;
	}
	if (PC->WasInputKeyJustPressed(EKeys::F6))
	{
		bGhostEnabled = !bGhostEnabled;
		UE_LOG(LogKart, Log, TEXT("Ghost %s"), bGhostEnabled ? TEXT("on") : TEXT("off"));
	}
}

void AKartVehiclePawn::UpdateAnalysisKeys(float DeltaSeconds)
{
	const APlayerController* PC = Cast<APlayerController>(GetController());
	if (!PC) { return; }
	const kartsim::LapDelta& D = LapAnalysis.GetDelta();
	const int32 NumLaps = static_cast<int32>(D.History().size());
	const int32 NumSt = static_cast<int32>(D.SamplesPerLap());
	if (NumLaps == 0) { AnalysisLap = -1; return; }
	AnalysisLap = FMath::Clamp(AnalysisLap < 0 ? NumLaps - 1 : AnalysisLap, 0, NumLaps - 1);
	// Up/Down (W/S): lap; Page Up/Down: reference lap (best, then each lap); Left/Right (A/D): cursor.
	if (PC->WasInputKeyJustPressed(EKeys::Up) || PC->WasInputKeyJustPressed(EKeys::W)) { AnalysisLap = FMath::Max(AnalysisLap - 1, 0); }
	if (PC->WasInputKeyJustPressed(EKeys::Down) || PC->WasInputKeyJustPressed(EKeys::S)) { AnalysisLap = FMath::Min(AnalysisLap + 1, NumLaps - 1); }
	if (PC->WasInputKeyJustPressed(EKeys::PageUp)) { AnalysisRefLap = AnalysisRefLap <= -1 ? NumLaps - 1 : AnalysisRefLap - 1; }
	if (PC->WasInputKeyJustPressed(EKeys::PageDown)) { AnalysisRefLap = AnalysisRefLap >= NumLaps - 1 ? -1 : AnalysisRefLap + 1; }
	const bool bFast = PC->IsInputKeyDown(EKeys::LeftShift);
	int32 Dir = 0;
	if (PC->IsInputKeyDown(EKeys::Left) || PC->IsInputKeyDown(EKeys::A)) { Dir -= 1; }
	if (PC->IsInputKeyDown(EKeys::Right) || PC->IsInputKeyDown(EKeys::D)) { Dir += 1; }
	if (Dir != 0)
	{
		AnalysisRepeat += DeltaSeconds * (bFast ? 240.0f : 60.0f); // stations per second
		const int32 Move = FMath::FloorToInt(AnalysisRepeat);
		AnalysisRepeat -= Move;
		AnalysisCursor = (AnalysisCursor + Dir * FMath::Max(Move, 0) + NumSt) % FMath::Max(NumSt, 1);
	}
	else { AnalysisRepeat = 1.0f; } // first press moves at once
	if (PC->WasInputKeyJustPressed(EKeys::Home)) { AnalysisCursor = 0; }
}

void AKartVehiclePawn::UpdateFairing(float DeltaSeconds)
{
	PanelParts.RemoveAll([](const FPanelPart& P) { return !IsValid(P.C); });
	// -KartTestDamage (or -KartTestFairing): the player's kart gets every bodywork panel knocked 1 s in (screenshots).
	if (Sim && !bIsAI && !bTestDamageDone && Sim->GetTime() > 1.0 &&
	    (FParse::Param(FCommandLine::Get(), TEXT("KartTestDamage")) || FParse::Param(FCommandLine::Get(), TEXT("KartTestFairing"))))
	{
		bTestDamageDone = true;
		kartsim::KartDamageState D = Sim->GetDamage();
		D.bFairingDisplaced = true;
		D.bSidepodKnocked[0] = D.bSidepodKnocked[1] = D.bBumperKnocked = FParse::Param(FCommandLine::Get(), TEXT("KartTestDamage"));
		Sim->SetDamage(D);
	}
	if (!Sim) { return; }
	const kartsim::KartDamageState& Dm = Sim->GetDamage();
	// Per panel (part space, cm; UE axes: X forward, Y right, Z up): pivot, rotation axis / angle, then a translation.
	// Pivots are the panels' mounting edges measured on the model (Tools/SplitNose.py, SplitSidepods.py). ESTIMATES.
	// (User: the nose went back "too far" at 9 cm; a released CIK kit sits a few cm back on its rods.)
	struct FPose { FVector Pivot; FVector Axis; float Deg; FVector Move; };
	static const FPose Poses[4] = {
		{FVector(128.4, 0.0, 0.0), FVector(0, 1, 0), 2.5f, FVector(-4.0, 0.0, -1.0)},     // nose: back on its rods, tip down
		{FVector(48.0, -51.0, 7.0), FVector(1, 0, 0), 7.0f, FVector(0.0, -2.5, -1.5)},    // left sidepod: hangs out and down
		{FVector(48.0, 56.0, 7.0), FVector(1, 0, 0), -7.0f, FVector(0.0, 2.5, -1.5)},     // right sidepod
		{FVector(-17.6, 0.0, 4.0), FVector(0, 1, 0), -6.0f, FVector(3.0, 0.0, -1.0)},     // rear bumper: pushed in, drooping
	};
	// Wobble drive: the kart's specific force minus the static 1 g (kart frame x fwd, y left, z up -> part space
	// x fwd, y right, z up). A panel lags the chassis: relative acceleration = -(f - g_static).
	const kartsim::TelemetryFrame& Tf = Sim->GetTelemetry();
	const double G = 9.81;
	const FVector Drive(-Tf.AccLongG * G, Tf.AccLatG * G, -(Tf.AccVertG - 1.0) * G); // m/s^2 in part axes
	const double Speed = FMath::Abs(Tf.VelLong);
	const double Rpm = Tf.EngineRpm;
	const float Dt = FMath::Clamp(DeltaSeconds, 0.0f, 0.05f);
	const int32 Sub = FMath::Max(1, FMath::CeilToInt(Dt / (1.0f / 600.0f)));
	const double H = Dt / Sub;
	for (FPanelPart& P : PanelParts)
	{
		const bool bOn = P.Kind == 0 ? Dm.bFairingDisplaced : (P.Kind == 1 ? Dm.bSidepodKnocked[0] : (P.Kind == 2 ? Dm.bSidepodKnocked[1] : Dm.bBumperKnocked));
		// ESTIMATES: on the mounts ~12 Hz, 20 % damped (1 g -> ~2 mm); knocked loose it still hangs on a bent bracket /
		// tie: ~7 Hz, 30 % damped (1 g -> ~5 mm). (User: 4 Hz / 8 % "jumps around too much, looks unrealistic".)
		const double Hz = bOn ? 7.0 : 12.0;
		const double Zeta = bOn ? 0.30 : 0.20;
		const double W = 2.0 * PI * Hz;
		// Vibration (engine + road), white-noise acceleration: rms g grows with speed and revs.
		const double NoiseG = (bOn ? 0.35 : 0.5) * FMath::Clamp(Speed / 25.0 + Rpm / 40000.0, 0.0, 1.5);
		const double Lim = bOn ? 0.015 : 0.008; // m: the brackets / straps stop it
		FVector X = P.Disp * 0.01, V = P.Vel * 0.01; // m
		for (int32 I = 0; I < Sub && H > 0.0; ++I)
		{
			const double Ns = NoiseG * G * FMath::Sqrt((1.0 / 240.0) / H);
			const FVector Noise(PanelNoise.GetFraction() - 0.5, PanelNoise.GetFraction() - 0.5, PanelNoise.GetFraction() - 0.5);
			const FVector A = Drive + Noise * (Ns * 3.46) - X * (W * W) - V * (2.0 * Zeta * W); // uniform: x 3.46 = unit rms
			V += A * H;
			X += V * H;
			for (int32 K = 0; K < 3; ++K)
			{
				if (FMath::Abs(X[K]) > Lim) { X[K] = FMath::Sign(X[K]) * Lim; V[K] *= -0.3; } // hits the stop, bounces
			}
		}
		if (!X.ContainsNaN() && !V.ContainsNaN()) { P.Disp = X * 100.0; P.Vel = V * 100.0; } else { P.Disp = P.Vel = FVector::ZeroVector; }
		const FPose& Ps = Poses[P.Kind];
		// The damage pose (if knocked) plus the wobble: the offset, and a rotation about the panel's hinge axis from
		// its vertical motion (~1.5 deg per cm: 40 cm long panels).
		const float Wob = static_cast<float>(-P.Disp.Z * (bOn ? 0.8 : 1.5));
		const float Deg = (bOn ? Ps.Deg : 0.0f) + (Ps.Deg >= 0.0f ? Wob : -Wob);
		const FQuat Tip(Ps.Axis, FMath::DegreesToRadians(Deg));
		const FQuat R = P.Rest.GetRotation();
		const FVector S = P.Rest.GetScale3D();
		const FVector Loc = P.Rest.GetLocation() + R.RotateVector(S * (Ps.Pivot - Tip.RotateVector(Ps.Pivot))) + R.RotateVector((bOn ? Ps.Move : FVector::ZeroVector) + P.Disp);
		P.C->SetRelativeTransform(FTransform(R * Tip, Loc, S));
	}
	// -KartPanelLog: panel wobble offsets twice a second (verification without video).
	static const bool bPanelLog = FParse::Param(FCommandLine::Get(), TEXT("KartPanelLog"));
	if (bPanelLog && !bIsAI && PanelParts.Num() > 0 && FMath::FloorToInt(Sim->GetTime() * 2.0) != FMath::FloorToInt((Sim->GetTime() - Dt) * 2.0))
	{
		FString Line;
		for (const FPanelPart& P : PanelParts) { Line += FString::Printf(TEXT(" [%d] %.2f/%.2f/%.2f cm"), P.Kind, P.Disp.X, P.Disp.Y, P.Disp.Z); }
		UE_LOG(LogKart, Log, TEXT("PANELS t=%.1f %.0f km/h:%s"), Sim->GetTime(), Tf.SpeedKmh, *Line);
	}
}

void AKartVehiclePawn::LogDamageEvents()
{
	if (!Sim || bPuppet) { return; }
	const kartsim::KartDamageState& D = Sim->GetDamage();
	if (D.NumEvents < LoggedDamageEvents) { LoggedDamageEvents = 0; } // repaired
	if (D.NumEvents > LoggedDamageEvents)
	{
		LoggedDamageEvents = D.NumEvents;
		const FString Line = FString::Printf(TEXT("DAMAGE %s: %s (t=%.1f s; toe %.2f/%.2f deg, axle %.2f deg, fairing %d, sidepods %d/%d, bumper %d, chain %d)"), *DriverName,
		       UTF8_TO_TCHAR(D.LastEvent.c_str()), Sim->GetTime(), D.ToeDeg[0], D.ToeDeg[1], D.AxleBendDeg, D.bFairingDisplaced ? 1 : 0,
		       D.bSidepodKnocked[0] ? 1 : 0, D.bSidepodKnocked[1] ? 1 : 0, D.bBumperKnocked ? 1 : 0, D.bChainOff ? 1 : 0);
		UE_LOG(LogKart, Log, TEXT("%s"), *Line);
		KartTesterLog(Line);
	}
	// Airborne (all four tyres off the ground): logged once per flight with the peak height.
	const kartsim::TelemetryFrame& F = Sim->GetTelemetry();
	const bool bAir = F.LiftFL > 0.5 && F.LiftFR > 0.5 && F.LiftRL > 0.5 && F.LiftRR > 0.5;
	const double H = Sim->GetChassis().Position.Z;
	if (bAir) { if (!bLoggedAirborne) { bLoggedAirborne = true; AirborneMax = H; } AirborneMax = FMath::Max(AirborneMax, H); }
	else if (bLoggedAirborne)
	{
		bLoggedAirborne = false;
		if (AirborneMax - H > 0.03)
		{
			const FString Line = FString::Printf(TEXT("AIRBORNE %s: landed at t=%.1f s, CoG peaked %.2f m above the landing height"), *DriverName, Sim->GetTime(), AirborneMax - H);
			UE_LOG(LogKart, Log, TEXT("%s"), *Line);
			KartTesterLog(Line);
		}
	}
	// Tyre touching another kart's tyre / body (wheel-to-wheel), at most once a second.
	double Wt = 0.0;
	for (int32 I = 0; I < kartsim::NumWheels; ++I) { Wt = FMath::Max(Wt, FMath::Abs(Sim->GetWheelContactTorque(I))); }
	if (Wt > 1.0 && Sim->GetTime() - LastWheelContactLog > 1.0)
	{
		LastWheelContactLog = Sim->GetTime();
		const FString Line = FString::Printf(TEXT("WHEEL CONTACT %s: t=%.1f s, tyre torque %.0f Nm, %.0f km/h"), *DriverName, Sim->GetTime(), Wt, F.SpeedKmh);
		UE_LOG(LogKart, Log, TEXT("%s"), *Line);
		KartTesterLog(Line);
	}
}

void AKartVehiclePawn::UpdateDash(float DeltaSeconds)
{
	if (bPuppet || !IsLocallyControlled() || !IsSimulatedLocally()) { return; }
	// The screen sits on the rig's "display" part (the Mychron body on the imported steering wheel). Its front face
	// was measured from Art/BogKart/Steer_display.fbx (part space, kart frame at the steering hub): plane normal
	// (-0.899, 0, 0.437) toward the driver, face 15 x 8 cm centred 3.3 cm out at (1.35, 0, 10.9) cm.
	UStaticMeshComponent* Display = nullptr;
	for (UStaticMeshComponent* P : RigParts) { if (P && P->ComponentHasTag(TEXT("KartDisplayPart"))) { Display = P; break; } }
	if (!Display) { if (Dash) { Dash->SetVisibility(false); } return; }
	if (!Dash || Dash->GetAttachParent() != Display)
	{
		if (Dash) { Dash->DestroyComponent(); }
		Dash = NewObject<UKartDashComponent>(this, TEXT("MychronDash"));
		Dash->SetupAttachment(Display);
		Dash->RegisterComponent();
		const FVector N(-0.8994, 0.0, 0.4371);
		const FVector Up(0.4371, 0.0, 0.8994);
		Dash->Init(this, FVector(1.353, 0.0, 10.906) + N * 0.25, Up, FVector(0.0, 1.0, 0.0), 11.0f, 5.5f);
	}
	Dash->SetVisibility(true);
	// ~25 Hz, only when it can be seen (cockpit camera); the HUD overlay covers the chase camera.
	DashTimer += DeltaSeconds;
	if (DashTimer >= 0.04f && (bCockpitCamera || FParse::Param(FCommandLine::Get(), TEXT("KartDashAlways"))))
	{
		DashTimer = 0.0f;
		Dash->Refresh();
	}
}

void AKartVehiclePawn::PlaceAt(const FTransform& Where)
{
	SetActorLocationAndRotation(Where.GetLocation(), Where.GetRotation());
	ResetKart();
	if (bAutopilot && Pilot.HasPath() && Sim) { Pilot.Reset(*Sim); }
}

void AKartVehiclePawn::CheckWobble(const kartsim::KartInputs& StepInputs)
{
	if (!Sim) { return; }
	if (!WobbleBuffer) { WobbleBuffer = MakeUnique<kartsim::TelemetryBuffer>(240 * 5); }
	WobbleBuffer->Push(Sim->GetTelemetry());
	if (WobblePendingSteps > 0 && --WobblePendingSteps == 0)
	{
		const FString Dir = FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("Wobble"));
		IFileManager::Get().MakeDirectory(*Dir, true);
		const FString Path = Dir / FString::Printf(TEXT("wobble_%d_%s.csv"), WobbleDumps, *FDateTime::Now().ToString(TEXT("%H%M%S")));
		WobbleBuffer->WriteCsv(TCHAR_TO_UTF8(*Path));
		UE_LOG(LogKart, Warning, TEXT("WOBBLE telemetry saved: %s"), *Path);
	}
	double MaxJump = 0.0;
	int32 Worst = -1;
	for (int32 W = 0; W < kartsim::NumWheels; ++W)
	{
		const kartsim::WheelState& Wh = Sim->GetWheel(W);
		const double J = FMath::Abs(Wh.Tyre.Fz - WobblePrevFz[W]);
		if (Wh.Contact.bInContact && J > MaxJump) { MaxJump = J; Worst = W; }
		WobblePrevFz[W] = Wh.Tyre.Fz;
	}
	const double T = Sim->GetTime();
	if (MaxJump > 500.0 && T > 2.0 && T - WobbleLastTime > 3.0 && WobbleDumps < 10 && WobblePendingSteps < 0)
	{
		WobbleLastTime = T;
		++WobbleDumps;
		WobblePendingSteps = 480; // dump after 2 more seconds
		const kartsim::TelemetryFrame& F = Sim->GetTelemetry();
		FString Surf;
		for (int32 W = 0; W < kartsim::NumWheels; ++W)
		{
			const kartsim::WheelState& Wh = Sim->GetWheel(W);
			Surf += FString::Printf(TEXT("%s:%s Fz %.0f pen %.1fmm  "), W == 0 ? TEXT("FL") : W == 1 ? TEXT("FR") : W == 2 ? TEXT("RL") : TEXT("RR"),
			                        Wh.Contact.bInContact ? UTF8_TO_TCHAR(kartsim::SurfaceName(Wh.Contact.Surface)) : TEXT("air"), Wh.Tyre.Fz, Wh.Contact.Penetration * 1000.0);
		}
		UE_LOG(LogKart, Warning, TEXT("WOBBLE #%d at t=%.2f: wheel %d load jump %.0f N in one step | %.1f km/h, steer col %.1f deg, thr %.2f brk %.2f, wheel input %s, FFB cmd %.2f, ext force %.0f N, steps/frame %d | %s"),
		       WobbleDumps, T, Worst, MaxJump, F.SpeedKmh, StepInputs.SteeringColumnAngle * kartsim::kRadToDeg, StepInputs.Throttle, StepInputs.Brake,
		       FKartWheelInput::Get().IsSteeringBound() ? TEXT("yes") : TEXT("no"), LastFfbCommand, Sim->GetExternalForce().Length(), StepsLastFrame, *Surf);
	}
}


// ------------------------------------------------------------------ puppets (replay / ghost)

AKartVehiclePawn* AKartVehiclePawn::SpawnPuppet(UWorld* World, const FString& KartFile, const FLinearColor& Color, const FString& Name)
{
	if (!World) { return nullptr; }
	const FTransform Where(FRotator::ZeroRotator, FVector(0.0, 0.0, -100000.0)); // posed on the first frame
	AKartVehiclePawn* P = World->SpawnActorDeferred<AKartVehiclePawn>(AKartVehiclePawn::StaticClass(), Where, nullptr, nullptr,
	                                                                  ESpawnActorCollisionHandlingMethod::AlwaysSpawn);
	if (!P) { return nullptr; }
	P->bPuppet = true;
	P->SetReplicates(false);
	P->AutoPossessAI = EAutoPossessAI::Disabled;
	P->KartConfigFile = KartFile;
	P->BodyColor = Color;
	P->DriverName = Name;
	UGameplayStatics::FinishSpawningActor(P, Where);
	return P;
}

void AKartVehiclePawn::PuppetPose(const kartsim::ReplayFrame& F, float Dt)
{
	if (!Sim) { return; }
	PrevSnap = CurrSnap;
	kartsim::ApplyReplayFrame(*Sim, F, Dt);
	CurrSnap = Capture();
	PrevSnap = CurrSnap; // posed at render rate: no sub-step interpolation
	ApplyRender(1.0f);
	PuppetRpm = F.Rpm;
	PuppetThrottle = F.Throttle;
}

bool AKartVehiclePawn::GetPuppetSound(float& OutRpm, float& OutThrottle) const
{
	if (!bPuppet) { return false; }
	OutRpm = PuppetRpm;
	OutThrottle = PuppetThrottle;
	return true;
}

void AKartVehiclePawn::SetPuppetCockpitView(bool bCockpit)
{
	bCockpitCamera = bCockpit;
	CockpitCamera->SetActive(bCockpit);
	ChaseCamera->SetActive(!bCockpit);
	HelmetMesh->SetOwnerNoSee(false);
	HelmetMesh->SetVisibility(!bCockpit);
	for (UStaticMeshComponent* P : RigParts) { if (P && P->ComponentHasTag(TEXT("KartHelmetPart"))) { P->SetOwnerNoSee(false); P->SetVisibility(!bCockpit); } }
}

// ------------------------------------------------------------------ ghost lap

FString AKartVehiclePawn::GhostFilePath() const
{
	FString Circuit = TEXT("Circuit");
	for (TActorIterator<AKartTestTrackActor> It(GetWorld()); It; ++It) { Circuit = FPaths::GetBaseFilename(It->GetLayoutFile()); break; }
	const FString Cls = Sim ? FString(UTF8_TO_TCHAR(Sim->GetConfig().ClassId.c_str())) : FString(TEXT("kart"));
	return FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("Ghosts") / (Circuit + TEXT("_") + Cls + TEXT(".krpl")));
}

void AKartVehiclePawn::LoadGhost()
{
	Ghost.Clear();
	GhostSeenLaps = LapTimer.Laps().size();
	kartsim::ReplayRecording R;
	std::string Err;
	const FString Path = GhostFilePath();
	if (!FPaths::FileExists(Path)) { return; }
	if (R.Load(TCHAR_TO_UTF8(*Path), Err) && !R.Karts.empty())
	{
		Ghost.Offer(R.Karts[0].Frames, R.LapTime);
		UE_LOG(LogKart, Log, TEXT("Ghost loaded: %.3f s (%s)"), R.LapTime, *Path);
	}
	else
	{
		UE_LOG(LogKart, Warning, TEXT("Ghost file ignored (%s): %s"), *Path, UTF8_TO_TCHAR(Err.c_str()));
	}
}

void AKartVehiclePawn::SaveGhost() const
{
	if (!Ghost.HasGhost()) { return; }
	kartsim::ReplayRecording R;
	for (TActorIterator<AKartTestTrackActor> It(GetWorld()); It; ++It) { R.Circuit = TCHAR_TO_UTF8(*It->GetLayoutFile()); break; }
	R.LapTime = Ghost.GhostTime();
	R.Info = "ghost";
	R.Karts.resize(1);
	R.Karts[0].Name = "Ghost";
	R.Karts[0].KartFile = TCHAR_TO_UTF8(*KartConfigFile);
	R.Karts[0].Frames = Ghost.Ghost();
	const FString Path = GhostFilePath();
	IFileManager::Get().MakeDirectory(*FPaths::GetPath(Path), true);
	R.Save(TCHAR_TO_UTF8(*Path));
}

void AKartVehiclePawn::HidePuppets()
{
	if (AKartVehiclePawn* G = GhostPuppet.Get()) { G->SetActorHiddenInGame(true); }
}

void AKartVehiclePawn::UpdateGhost(float DeltaSeconds)
{
	AKartVehiclePawn* G = GhostPuppet.Get();
	const AKartRaceWorld* W = RaceWorld.Get();
	const bool bReplaying = W && W->IsReplaying();
	const bool bShow = bGhostEnabled && !bReplaying && Ghost.HasGhost() && LapTimer.IsTiming() && IsLocallyControlled();
	GhostGap = 0.0;
	if (!bShow)
	{
		if (G) { G->SetActorHiddenInGame(true); }
		return;
	}
	const double T = LapTimer.CurrentLapTime(ClockNow);
	if (T > Ghost.GhostTime())
	{
		if (G) { G->SetActorHiddenInGame(true); } // the ghost has finished its lap
		return;
	}
	if (!G || G->KartConfigFile != KartConfigFile)
	{
		if (G) { G->Destroy(); }
		G = SpawnPuppet(GetWorld(), KartConfigFile, FLinearColor(0.0f, 0.75f, 1.0f), TEXT("Ghost")); // cyan: unlike any AI livery
		GhostPuppet = G;
		if (!G) { return; }
		G->bPuppetSilent = true;
	}
	kartsim::ReplayTrack Track; // sampled in place (no copy): the ghost frames are small
	kartsim::ReplayFrame F;
	const std::vector<kartsim::ReplayFrame>& Frames = Ghost.Ghost();
	{
		// Binary search inline (avoids copying the frames into a ReplayTrack each frame).
		size_t Lo = 0, Hi = Frames.size() - 1;
		if (T <= Frames.front().Time) { F = Frames.front(); }
		else if (T >= Frames.back().Time) { F = Frames.back(); }
		else
		{
			while (Hi - Lo > 1) { const size_t M = (Lo + Hi) / 2; if (Frames[M].Time <= T) { Lo = M; } else { Hi = M; } }
			const double Span = Frames[Hi].Time - Frames[Lo].Time;
			F = kartsim::LerpFrame(Frames[Lo], Frames[Hi], Span > 0.0 ? (T - Frames[Lo].Time) / Span : 0.0);
		}
	}
	// Close to the player the ghost would sit inside the kart: hide it within 2.5 m.
	const kartsim::Vec3 Me = Sim->GetChassis().Position;
	const double Dx = F.Pos[0] - Me.X, Dy = F.Pos[1] - Me.Y;
	const kartsim::Vec3 Fwd = Sim->GetChassis().Orientation.Rotate(kartsim::Vec3(1, 0, 0));
	GhostGap = Dx * Fwd.X + Dy * Fwd.Y; // m along our heading
	G->SetActorHiddenInGame(Dx * Dx + Dy * Dy < 2.5 * 2.5);
	G->PuppetPose(F, DeltaSeconds);
}


void AKartVehiclePawn::DumpDriverRig() const
{
	// Rest-pose driver joints in the kart frame (metres: x forward of the rear axle, y left, z up from the ground), for
	// posing imported driver models offline (Tools/DriverSuit). Elbows from the same two-bone IK as AnimateDriver.
	if (!Sim || !DRig.bValid) { return; }
	const kartsim::KartConfig& C = Sim->GetConfig();
	const double H = C.Chassis.CogHeight, B = C.Chassis.Wheelbase * C.Chassis.FrontWeightFraction;
	auto KF = [H, B](const FVector& P) { const kartsim::Vec3 Q = KartConv::FromUEPos(P); return FString::Printf(TEXT("[%.4f, %.4f, %.4f]"), Q.X + B, Q.Y, Q.Z + H); };
	FString J = TEXT("{\n");
	J += FString::Printf(TEXT("  \"classId\": \"%s\",\n"), UTF8_TO_TCHAR(C.ClassId.c_str()));
	J += FString::Printf(TEXT("  \"pelvis\": %s, \"neck\": %s, \"helmetCentre\": %s, \"eye\": %s,\n"), *KF(DRig.Pelvis), *KF(DRig.Neck), *KF(DRig.HelmetC), *KF(DRig.Eye));
	J += FString::Printf(TEXT("  \"upperLen\": %.4f, \"foreLen\": %.4f,\n"), DRig.UpperLen / 100.0f, DRig.ForeLen / 100.0f);
	static const TCHAR* Side[] = {TEXT("L"), TEXT("R")};
	for (int32 S = 0; S < 2; ++S)
	{
		const FVector D = DRig.Wrist[S] - DRig.Shoulder[S];
		const float Dist = FMath::Clamp(static_cast<float>(D.Size()), 1.0f, DRig.UpperLen + DRig.ForeLen - 0.1f);
		const FVector Dir = D.GetSafeNormal();
		const float A = DRig.UpperLen, Bf = DRig.ForeLen;
		const float Along = (A * A - Bf * Bf + Dist * Dist) / (2.0f * Dist);
		const float Off = FMath::Sqrt(FMath::Max(A * A - Along * Along, 0.0f));
		const FVector Perp = (DRig.ElbowHint[S] - Dir * FVector::DotProduct(DRig.ElbowHint[S], Dir)).GetSafeNormal();
		const FVector Elbow = DRig.Shoulder[S] + Dir * Along + Perp * Off;
		J += FString::Printf(TEXT("  \"shoulder%s\": %s, \"elbow%s\": %s, \"wrist%s\": %s, \"grip%s\": %s,\n"),
			Side[S], *KF(DRig.Shoulder[S]), Side[S], *KF(Elbow), Side[S], *KF(DRig.Wrist[S]), Side[S], *KF(DRig.Grip[S]));
		J += FString::Printf(TEXT("  \"hip%s\": %s, \"knee%s\": %s, \"ankle%s\": %s,\n"), Side[S], *KF(DRig.Hip[S]), Side[S], *KF(DRig.Knee[S]), Side[S], *KF(DRig.Ankle[S]));
	}
	J += FString::Printf(TEXT("  \"columnTop\": %s\n}\n"), *KF(DRig.ColTop));
	const FString Path = FPaths::ProjectSavedDir() / TEXT("DriverRig.json");
	FFileHelper::SaveStringToFile(J, *Path);
	UE_LOG(LogKart, Log, TEXT("Driver rig dumped: %s"), *Path);
}
