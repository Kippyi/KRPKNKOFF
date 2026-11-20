// AKartVehiclePawn: hosts one kartsim::KartSimulation inside Unreal.
//  - Fixed-step accumulator (default 240 Hz, independent of render frame rate).
//  - Render pose interpolated between the last two physics states.
//  - Ground contact via FKartUnrealTrackQuery (line traces); the pawn itself has no collision.
//  - Enhanced Input built at runtime (no input assets required).
//  - Telemetry CSV recording and physics debug drawing.
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "KartForceFeedbackDevice.h"
#include "KartSim/KartPhysicsCore.h"
#include "KartSim/Autopilot.h"
#include "KartSim/LapTiming.h"
#include "KartSim/Replay.h"
#include "KartLapAnalysis.h"
#include "KartNetTypes.h"
#include "KartSim/TrackConditions.h"
#include "KartUnrealTrackQuery.h"
#include "KartVehiclePawn.generated.h"

class UCameraComponent;
class UInputAction;
class UInputMappingContext;
class USpringArmComponent;
class UStaticMeshComponent;
class UProceduralMeshComponent;
class UStaticMesh;
class UMaterialInterface;

UCLASS()
class KRPKNKOFF_API AKartVehiclePawn : public APawn
{
	GENERATED_BODY()

public:
	AKartVehiclePawn();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaSeconds) override;
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;
	virtual void NotifyControllerChanged() override;

	// --- Configuration ---
	/** Kart class file in Content/KartData/Karts. */
	UPROPERTY(EditAnywhere, Category = "Kart")
	FString KartConfigFile = TEXT("Rotax125MaxSenior.json");

	/** Classes selectable in game with keys 1, 2, 3 ... */
	UPROPERTY(EditAnywhere, Category = "Kart")
	TArray<FString> AvailableKarts = {TEXT("Rotax125MaxSenior.json"), TEXT("KZ2_IAMEScreamer.json"), TEXT("KT100_DirectDrive.json")};

	/** Input-only aids (never alter the physics). Auto clutch: manual-clutch karts launch/anti-stall. */
	UPROPERTY(EditAnywhere, Category = "Kart|Input")
	bool bAutoClutch = true;

	/** Auto shift for gearbox karts (off by default: shift with E/Q). */
	UPROPERTY(EditAnywhere, Category = "Kart|Input")
	bool bAutoShift = false;

	UPROPERTY(EditAnywhere, Category = "Kart|Simulation", meta = (ClampMin = "60", ClampMax = "2000"))
	float PhysicsRateHz = 240.0f;

	UPROPERTY(EditAnywhere, Category = "Kart|Simulation", meta = (ClampMin = "1", ClampMax = "32"))
	int32 Substeps = 16; // 240 Hz x 16 = 3840 Hz inner rate (Milestone 9: wheel mounts, frame bending)

	UPROPERTY(EditAnywhere, Category = "Kart|Debug")
	bool bDebugDraw = false;

	UPROPERTY(EditAnywhere, Category = "Kart|Debug")
	bool bShowTelemetry = false; // T toggles; -KartShowTelemetry

	/** Cockpit camera (Page Up/Down, Home/End in game; saved in GameUserSettings.ini [KartCamera]). */
	UPROPERTY(EditAnywhere, Category = "Kart|Camera")
	float CockpitPitch = -10.0f;
	UPROPERTY(EditAnywhere, Category = "Kart|Camera")
	float CockpitFov = 100.0f;

	// Keyboard/gamepad conditioning acts on the INPUT only (never on the physics). Digital keys
	// cannot modulate, so they become ramps limited to what the kart can physically use.
	UPROPERTY(EditAnywhere, Category = "Kart|Input")
	float KeyboardSteerRate = 1.8f;          // normalised lock per second

	UPROPERTY(EditAnywhere, Category = "Kart|Input")
	float KeyboardSteerReturnRate = 3.0f;

	UPROPERTY(EditAnywhere, Category = "Kart|Input")
	float KeyboardThrottleRate = 3.0f;

	UPROPERTY(EditAnywhere, Category = "Kart|Input")
	float KeyboardBrakeRate = 4.0f;

	/** Keyboard brake pedal maximum: rear-only brakes lock above ~0.45 at speed. */
	UPROPERTY(EditAnywhere, Category = "Kart|Input", meta = (ClampMin = "0.1", ClampMax = "1"))
	float KeyboardBrakeMax = 0.40f;

	/** Keyboard/gamepad steering lock is limited to the road-wheel angle that yields this lateral
	 *  acceleration (kinematic L*a/v^2) plus AssistSlipAngleDeg. Steering wheels are never limited. */
	UPROPERTY(EditAnywhere, Category = "Kart|Input", meta = (ClampMin = "0.5", ClampMax = "3"))
	float AssistLateralG = 1.6f;

	UPROPERTY(EditAnywhere, Category = "Kart|Input", meta = (ClampMin = "0", ClampMax = "20"))
	float AssistSlipAngleDeg = 5.0f;

	// --- Accessors for HUD / tools ---
	const kartsim::KartSimulation* GetSim() const { return Sim.Get(); }
	const kartsim::KartInputs& GetLastInputs() const { return Inputs; }
	const FString& GetStatusMessage() const { return StatusMessage; }
	bool IsRecording() const { return Recorder.IsValid() && Recorder->IsOpen(); }
	const FString& GetRecordingPath() const { return RecordingPath; }
	float GetPhysicsMsPerFrame() const { return PhysicsMsPerFrame; }
	int32 GetStepsLastFrame() const { return StepsLastFrame; }
	float GetMeasuredPhysicsHz() const { return MeasuredHz; }
	const IKartFFBDevice* GetFFBDevice() const { return FfbDevice.Get(); }
	bool IsCockpitCamera() const { return bCockpitCamera; }

	void ResetKart();
	void ToggleRecording();
	void SwitchKart(int32 Index);

	// --- Garage (main-menu setup page): edits a copy of the config; Apply re-initialises and saves ---
	void GarageBegin() { if (Sim) { EditConfig = Sim->GetConfig(); } }
	void GarageSetValue(int32 Index, double Value);
	void GarageApply() { ApplySetup(); }
	void GarageDefaults() { EditConfig = DefaultConfig; }

	// --- Tyre sets: the player's wear per compound is kept between sessions and game runs
	// (Saved/Setups/<classId>.tyres.json); -KartDefaultSetup and AI karts always start on new tyres ---
	void FitNewTyres();                          // garage: new set of the edited compound (+ applies the setup)
	void FitTyreCompound(int32 Compound);        // AI / race world: swap to a (new) set of that compound
	double GetSetTyreWear(int32 Compound, int32 Wheel) const;

	// --- Setup menu (Tab) ---
	bool IsSetupOpen() const { return bSetupOpen; }
	int32 GetSetupCursor() const { return SetupCursor; }
	const kartsim::KartConfig& GetEditConfig() const { return EditConfig; }
	const FString& GetSetupMessage() const { return SetupMessage; }

	// --- Lap timing ---
	const kartsim::LapTimer& GetLapTimer() const { return LapTimer; }
	bool HasTimingLines() const { return bTimingConfigured; }
	bool IsAutopilot() const { return bAutopilot; }
	const FKartLapAnalysis& GetLapAnalysis() const { return LapAnalysis; }
	const class AKartConditionsActor* GetConditions() const { return Conditions.Get(); }
	bool IsPushing() const { return Inputs.PushForce > 0.0; }
	double GetJetChangeTime() const { return JetChangeTime; }   // sim time of the last [ / ] jet change (Mychron)

	// --- Milestone 6: race world (lockstep stepping of every kart, contacts, AI, sessions) ---
	/** Bodywork colour (AI karts get one each). */
	UPROPERTY(EditAnywhere, Replicated, Category = "Kart")
	FLinearColor BodyColor = FLinearColor(0.72f, 0.04f, 0.03f);
	/** Set by AKartRaceWorld before BeginPlay (deferred spawn): no input, camera or command-line options. */
	UPROPERTY(Replicated)
	bool bIsAI = false;
	UPROPERTY(Replicated)
	FString DriverName = TEXT("You");

	// --- Multiplayer (Milestone 6): each kart is simulated where it is driven (a human's own machine;
	// AI on the host) and seen everywhere else as a kinematic proxy interpolated from 30 Hz states. ---
	virtual void GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const override;
	bool IsSimulatedLocally() const;
	// Sends this kart's state (locally simulated karts only), stamped with the server clock.
	void NetPublishState(double ServerTime);
	// Poses a proxy at RenderTime (server clock) from the state buffer; returns false without data.
	bool NetPose(double RenderTime, double Dt);
	// Host: lap timing of a remote human's kart from its proxy path, on the host's race clock.
	void NetProxyTiming(double Clock, double Dt);
	// Host -> owning client: move to a grid slot. Also resets the host-side proxy timing.
	void NetPlaceAt(const FTransform& Where);
	UFUNCTION(Server, Unreliable) void ServerKartState(const FKartNetState& State);
	UFUNCTION(NetMulticast, Unreliable) void MulticastKartState(const FKartNetState& State);
	UFUNCTION(Client, Reliable) void ClientPlaceAt(const FTransform& Where);
	UFUNCTION(Server, Reliable) void ServerSetKartConfig(const FString& File);
	UPROPERTY(ReplicatedUsing = OnRep_NetKartConfig)
	FString NetKartConfig;
	UFUNCTION() void OnRep_NetKartConfig();
	kartsim::KartSimulation* GetSimMutable() { return Sim.Get(); }
	// One fixed physics step. Override: inputs from an external driver (AI); null = player / autopilot.
	// bHoldOnGrid: start lights, kart held on the brakes.
	// Clock: the race world's common clock at the END of this step (all karts' lap times must share it);
	// the self-stepping fallback passes the sim's own time.
	// SpeedLimit: race control's formation-lap limiter (throttle cut above it, m/s).
	void StepPhysics(double H, double Clock, const kartsim::KartInputs* Override, bool bHoldOnGrid, double SpeedLimit = 1e9);
	// The clock lap timing runs on (for HUD lap times).
	double GetClock() const { return ClockNow; }
	// After the race world's steps this frame: render interpolation and timing statistics.
	void FinishFrame(float Alpha, int32 Steps, float PhysicsMs, float DeltaSeconds);
	void SetRaceWorld(class AKartRaceWorld* World);
	// Moves the kart to a grid slot at rest (resets it and aborts the lap in progress).
	void PlaceAt(const FTransform& Where);
	bool IsAutopilotDriving() const { return bAutopilot && !bSetupOpen; }
	class AKartRaceWorld* GetRaceWorld() const { return RaceWorld.Get(); }

	// --- Puppets: display-only karts posed from recorded frames (replays, the ghost lap). Not simulated,
	// not in the race world, not replicated, no collision. Set before BeginPlay (deferred spawn). ---
	bool bPuppet = false;
	bool bPuppetSilent = false;         // the ghost makes no sound
	void PuppetPose(const kartsim::ReplayFrame& F, float Dt);
	bool GetPuppetSound(float& OutRpm, float& OutThrottle) const;
	void SetPuppetCockpitView(bool bCockpit);
	// Spawns a puppet of the given class (deferred: flags/colour set before BeginPlay).
	static AKartVehiclePawn* SpawnPuppet(UWorld* World, const FString& KartFile, const FLinearColor& Color, const FString& Name);

	// --- Ghost lap (player): fastest valid lap per circuit + class, saved to Saved/Ghosts ---
	bool IsGhostEnabled() const { return bGhostEnabled; }
	bool HasGhost() const { return Ghost.HasGhost(); }
	double GetGhostTime() const { return Ghost.GhostTime(); }
	// Ghost position relative to the player now: + = ghost ahead (s of lap time at the same place is the delta).
	double GetGhostGap() const { return GhostGap; }
	void HidePuppets();      // replay starting: no ghost on screen
	void StopForceFeedback() { if (FfbDevice) { FfbDevice->Stop(); } }

	// --- Lap analysis (telemetry viewer, key L) ---
	bool IsAnalysisOpen() const { return bAnalysisOpen; }
	int32 GetAnalysisLap() const { return AnalysisLap; }       // index into the lap history
	int32 GetAnalysisCursor() const { return AnalysisCursor; } // distance station
	int32 GetAnalysisRefLap() const { return AnalysisRefLap; } // -1 = best lap

private:
	struct FRenderSnapshot
	{
		FVector Position = FVector::ZeroVector;
		FQuat Rotation = FQuat::Identity;
		FVector WheelPos[kartsim::NumWheels];
		FQuat WheelRot[kartsim::NumWheels];
		FQuat UprightRot[kartsim::NumWheels];   // wheel frame without spin (steered spindles / uprights)
	};

	bool InitSimulation();
	FRenderSnapshot Capture() const;
	void ApplyRender(float Alpha);
	void UpdateInputs(float DeltaSeconds);
	void DrawPhysicsDebug() const;
	void EnsureInputObjects();
	void AddMappingContext();
	void BuildVisuals();

	void OnShiftUp();
	void OnShiftDown();
	void OnReset();
	void OnToggleCamera();
	void OnToggleDebug();
	void OnToggleTelemetry();
	void OnToggleRecord();
	void OnKart1();
	void OnKart2();
	void OnKart3();
	void OnToggleAutoShift();
	void OnToggleAutoClutch();
	void OnToggleSetup();
	void UpdateSetupMenu();
	void ApplySetup();
	void ReinitFromEditConfig(bool bSaveSetup);
	FString SetupFilePath(const std::string& ClassId) const;
	bool OwnsTyreSets() const;
	void LoadTyreSets();
	void SaveTyreSets() const;
	void CaptureTyreWear();
	void RestoreTyreWear();
	double TyreSetWear[kartsim::KartConfig::MaxCompounds][kartsim::NumWheels] = {};
	std::string TyreSetClassId;                  // class the TyreSetWear table belongs to
	float TyreSaveTimer = 0.0f;

	// Puppet state
	float PuppetRpm = 0.0f, PuppetThrottle = 0.0f;
	// Ghost
	kartsim::GhostKeeper Ghost;
	size_t GhostSeenLaps = 0;
	bool bGhostEnabled = true;
	double GhostGap = 0.0;
	TWeakObjectPtr<AKartVehiclePawn> GhostPuppet;
	FString GhostFilePath() const;
	void LoadGhost();
	void SaveGhost() const;
	void UpdateGhost(float DeltaSeconds);
	// Lap analysis
	bool bAnalysisOpen = false;
	int32 AnalysisLap = -1;
	int32 AnalysisRefLap = -1;
	int32 AnalysisCursor = 0;
	float AnalysisRepeat = 0.0f;
	void UpdateAnalysisKeys(float DeltaSeconds);
	bool bTyreSetRestored = false;               // the saved set was put on the (possessed) player kart

	TUniquePtr<kartsim::KartSimulation> Sim;
	FKartUnrealTrackQuery TrackQuery;

	// Milestone 5: world track conditions (grip via a ConditionsTrackQuery wrapping TrackQuery) and the
	// live delta. Keys: N weather, M track rubber, Y next session (handled by the race world).
	TWeakObjectPtr<class AKartConditionsActor> Conditions;
	TUniquePtr<kartsim::ConditionsTrackQuery> CondQuery;
	FKartLapAnalysis LapAnalysis;
	double ClockNow = 0.0;
	bool bWheelFfbChecked = false;
	TArray<FKartNetState> NetBuffer;     // proxy states, oldest first
	kartsim::Vec3 ProxyPrevPos;
	bool bProxyHasPrev = false;
	bool bProxyAllOff = false;           // latest track-limits flag of a remote kart
	bool bLastAllOff = false;            // this kart's last physics step: all four wheels off
	// Wobble detector (player kart): a tyre-load jump > 500 N in one step logs the context and saves the
	// 3 s before + 2 s after to Saved/Wobble/*.csv for diagnosis.
	void CheckWobble(const kartsim::KartInputs& StepInputs);
	TUniquePtr<kartsim::TelemetryBuffer> WobbleBuffer;
	double WobblePrevFz[4] = {0, 0, 0, 0};
	int32 WobblePendingSteps = -1;
	int32 WobbleDumps = 0;
	double WobbleLastTime = -100.0;
	float LastFfbCommand = 0.0f;
	double ProxyTimingHold = 0.0;        // s: ignore crossings right after a grid teleport
	TWeakObjectPtr<class AKartRaceWorld> RaceWorld;
	void UpdateWorldKeys();
	TUniquePtr<IKartFFBDevice> FfbDevice;
	TUniquePtr<kartsim::CsvTelemetryWriter> Recorder;
	FString RecordingPath;
	FString StatusMessage;

	kartsim::KartInputs Inputs;
	int32 PendingShift = 0;

public:
	// Raw input sources (HUD diagnostics). Analog axes are ignored until first seen at rest, so a
	// stuck or virtual controller (e.g. a remote-play synthetic gamepad) cannot hijack the controls.
	struct FInputDiagnostics
	{
		float KeyThrottle = 0, KeyBrake = 0, KeySteer = 0;
		float AxisThrottle = 0, AxisBrake = 0, AxisSteer = 0;
		bool bThrottleArmed = false, bBrakeArmed = false, bSteerArmed = false;
		float SteerLockFraction = 1.0f;
	};
	const FInputDiagnostics& GetInputDiagnostics() const { return InputDiag; }

private:
	FInputDiagnostics InputDiag;
	kartsim::KartConfig EditConfig;      // setup being edited in the menu
	kartsim::KartConfig DefaultConfig;   // class file values (Backspace restores these)
	bool bSetupOpen = false;
	int32 SetupCursor = 0;
	FString SetupMessage;
	kartsim::LapTimer LapTimer;
	bool bTimingConfigured = false;
	void ConfigureLapTiming();
	void OnToggleAutopilot();
	kartsim::Autopilot Pilot;
	bool bAutopilot = false;
	int32 QuitAfterLaps = 0;
	int32 AnalysisAfterLaps = 0; // -KartAnalysisAfterLaps=<n>: opens the lap analysis (screenshots / tests)
	size_t LoggedLaps = 0;
	FString LastKeysDown;
	float KeySteer = 0.0f, KeyThrottle = 0.0f, KeyBrake = 0.0f, KeyClutch = 0.0f;
	double Accumulator = 0.0;
	FRenderSnapshot PrevSnap, CurrSnap;
	float PhysicsMsPerFrame = 0.0f;
	int32 StepsLastFrame = 0;
	float MeasuredHz = 0.0f;
	double HzWindowTime = 0.0;
	int32 HzWindowSteps = 0;
	bool bCockpitCamera = false;
	double AiTunedAirTemp = -1000.0;     // air temperature the AI's jet / radiator cover were chosen for
	UPROPERTY(Transient) TObjectPtr<class UKartDashComponent> Dash;   // Mychron screen (locally driven karts)
	void UpdateDash(float DeltaSeconds);
	float DashTimer = 0.0f;
	double JetChangeTime = -100.0;

	// --- Components ---
	UPROPERTY(VisibleAnywhere, Category = "Kart")
	TObjectPtr<USceneComponent> Root;

	// Procedural placeholder visuals (Milestone 4). Purely visual: the physics never reads them.
	UPROPERTY(VisibleAnywhere, Category = "Kart") TObjectPtr<UProceduralMeshComponent> FrameMesh;
	UPROPERTY(VisibleAnywhere, Category = "Kart") TObjectPtr<UProceduralMeshComponent> BodyMesh;
	UPROPERTY(VisibleAnywhere, Category = "Kart") TObjectPtr<UProceduralMeshComponent> DriverMesh;
	UPROPERTY(VisibleAnywhere, Category = "Kart") TObjectPtr<UProceduralMeshComponent> HelmetMesh;
	UPROPERTY(VisibleAnywhere, Category = "Kart") TObjectPtr<UProceduralMeshComponent> SteeringWheelMesh;
	UPROPERTY(VisibleAnywhere, Category = "Kart") TObjectPtr<UProceduralMeshComponent> AxleMesh;
	FQuat SteeringAlign = FQuat::Identity;   // steering wheel local Z -> column axis (UE local space)
	FVector SteeringAxisLocal = FVector::UpVector;
	float ShowroomDistance = 0.0f; // -KartShowroom=<cm>: orbiting inspection camera (debug)
	float ShowroomYaw = 0.0f;
	float ShowroomPitch = -14.0f;
	float ShowroomSpin = 40.0f;

	// Asset pipeline: static-mesh parts from a visual rig file (Content/KartData/Visuals/*.json).
	UPROPERTY(Transient) TArray<TObjectPtr<UStaticMeshComponent>> RigParts;
	// Moving chassis parts for imported rigs (attach "uprightFL/FR", "rearAxle", "tieRodL/R"): spindles follow the
	// steered wheel without spin, the rear axle pivot spins with the rear wheels, tie rods are re-aimed every frame
	// between the steering-column arm and the spindle arm.
	UPROPERTY(VisibleAnywhere, Category = "Kart") TObjectPtr<USceneComponent> UprightFL;
	UPROPERTY(VisibleAnywhere, Category = "Kart") TObjectPtr<USceneComponent> UprightFR;
	UPROPERTY(VisibleAnywhere, Category = "Kart") TObjectPtr<USceneComponent> RearAxlePivot;
	struct FTieRod
	{
		UStaticMeshComponent* C = nullptr;        // owned by RigParts
		int32 Side = 0;                           // 0 = left (UprightFL), 1 = right
		FVector OuterLocal = FVector::ZeroVector; // spindle-arm end, upright space (cm)
		FVector InnerRest = FVector::ZeroVector;  // column-arm end at rest, component space (cm)
		FVector RestDir = FVector::ForwardVector; // outer -> inner at rest, component space
	};
	TArray<FTieRod> TieRods;
	// Bodywork panels that move with damage (rig "panel": nose / sidepodL / sidepodR / bumper): the CIK front fairing pushed
	// back, a sidepod knocked loose, the rear bumper pushed in.
	// Each panel also wobbles: a damped spring (cm, part space) driven by the kart's accelerations (hits, kerbs,
	// braking) plus engine / road vibration; stiff on its mounts, soft and flapping once knocked loose.
	struct FPanelPart { UStaticMeshComponent* C = nullptr; FTransform Rest; int32 Kind = 0; FVector Disp = FVector::ZeroVector; FVector Vel = FVector::ZeroVector; };
	TArray<FPanelPart> PanelParts;
	bool bTestDamageDone = false;
	FRandomStream PanelNoise{1234};
	void UpdateFairing(float DeltaSeconds);
	// Log of damage / contact events (Saved/Logs) for feedback sessions.
	int32 LoggedDamageEvents = 0;
	bool bLoggedAirborne = false;
	double AirborneMax = 0.0;
	double LastWheelContactLog = -100.0;
	void LogDamageEvents();
	void DumpDriverRig() const;   // -KartDumpDriverRig: rest-pose joints (kart frame) -> Saved/DriverRig.json
	void ApplyCockpitHelmetVisibility();
	// Kingpin points (component space, cm) from the rig's "kingpins": the uprights pivot there (chassis-fixed) and take
	// only the simulated steer / caster / camber rotation; without them the uprights sit at the wheel centres.
	FVector KingpinLocal[2] = {FVector::ZeroVector, FVector::ZeroVector};
	bool bKingpinPivot = false;
	void ApplyVisualRig();
	void RunSurfaceSelfTest();
	float SelfTestCountdown = 0.0f;
	bool bSelfTestPadSpawned = false;

	UPROPERTY(VisibleAnywhere, Category = "Kart")
	TArray<TObjectPtr<UProceduralMeshComponent>> WheelMeshes;

	// --- Driver animation rig (procedural): the torso pivots at the pelvis, the head at the neck; arms are rigid
	// segments placed by two-bone IK each frame with the hands on the steering-wheel rim. ---
	UPROPERTY(VisibleAnywhere, Category = "Kart") TObjectPtr<UProceduralMeshComponent> TorsoMesh;
	UPROPERTY(VisibleAnywhere, Category = "Kart") TArray<TObjectPtr<UProceduralMeshComponent>> ArmMeshes; // upper L/R, fore L/R, hand L/R
	struct FDriverRig
	{
		bool bValid = false;
		FVector Pelvis, Neck, Eye, ColTop, ColAxis;     // rest pose, component space (cm)
		FVector Shoulder[2], Wrist[2], Grip[2], ElbowHint[2];
		FQuat GripRot[2];
		FVector Hip[2], Knee[2], Ankle[2], HelmetC; // rest pose, component space (cm): for imported driver models
		float UpperLen = 31.0f, ForeLen = 27.0f;
	} DRig;
	float AnimRoll = 0.0f, AnimPitch = 0.0f, AnimYaw = 0.0f; // smoothed (deg)
	FVector AnimPrevVel = FVector::ZeroVector;
	FVector AnimAccel = FVector::ZeroVector;                 // smoothed body-frame acceleration (m/s^2): x fwd, y left
	bool bAnimHasPrev = false;
	void AnimateDriver(float SteerColumnRad, float DeltaSeconds);

	UPROPERTY(VisibleAnywhere, Category = "Kart")
	TObjectPtr<USpringArmComponent> ChaseArm;

	UPROPERTY(VisibleAnywhere, Category = "Kart")
	TObjectPtr<UCameraComponent> ChaseCamera;

	UPROPERTY(VisibleAnywhere, Category = "Kart")
	TObjectPtr<UCameraComponent> CockpitCamera;

	UPROPERTY(Transient)
	TObjectPtr<UStaticMesh> CubeMesh;

	UPROPERTY(Transient)
	TObjectPtr<UStaticMesh> CylinderMesh;

	UPROPERTY(Transient)
	TObjectPtr<UStaticMesh> SphereMesh;

	UPROPERTY(Transient)
	TObjectPtr<UMaterialInterface> BaseMaterial;

	// --- Runtime-built input ---
	UPROPERTY(Transient) TObjectPtr<UInputMappingContext> InputContext;
	UPROPERTY(Transient) TObjectPtr<UInputAction> ActThrottleKey;
	UPROPERTY(Transient) TObjectPtr<UInputAction> ActBrakeKey;
	UPROPERTY(Transient) TObjectPtr<UInputAction> ActSteerKey;
	UPROPERTY(Transient) TObjectPtr<UInputAction> ActThrottleAxis;
	UPROPERTY(Transient) TObjectPtr<UInputAction> ActBrakeAxis;
	UPROPERTY(Transient) TObjectPtr<UInputAction> ActSteerAxis;
	UPROPERTY(Transient) TObjectPtr<UInputAction> ActClutch;
	UPROPERTY(Transient) TObjectPtr<UInputAction> ActShiftUp;
	UPROPERTY(Transient) TObjectPtr<UInputAction> ActShiftDown;
	UPROPERTY(Transient) TObjectPtr<UInputAction> ActReset;
	UPROPERTY(Transient) TObjectPtr<UInputAction> ActCamera;
	UPROPERTY(Transient) TObjectPtr<UInputAction> ActDebug;
	UPROPERTY(Transient) TObjectPtr<UInputAction> ActTelemetry;
	UPROPERTY(Transient) TObjectPtr<UInputAction> ActRecord;
	UPROPERTY(Transient) TObjectPtr<UInputAction> ActKart1;
	UPROPERTY(Transient) TObjectPtr<UInputAction> ActKart2;
	UPROPERTY(Transient) TObjectPtr<UInputAction> ActKart3;
	UPROPERTY(Transient) TObjectPtr<UInputAction> ActStarter;
	UPROPERTY(Transient) TObjectPtr<UInputAction> ActPush;
	UPROPERTY(Transient) TObjectPtr<UInputAction> ActAutoShift;
	UPROPERTY(Transient) TObjectPtr<UInputAction> ActAutoClutch;
	UPROPERTY(Transient) TObjectPtr<UInputAction> ActSetup;
	UPROPERTY(Transient) TObjectPtr<UInputAction> ActAutopilot;
};
