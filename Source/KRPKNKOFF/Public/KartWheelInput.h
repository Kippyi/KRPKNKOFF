// FKartWheelInput: steering wheels, pedals and shifters through DirectInput 8 (Windows), plus
// constant-force force feedback. Covers DirectInput-compatible bases (Simagic, Logitech, Thrustmaster,
// Fanatec, Moza ... in their DirectInput mode).
//  * Every attached game controller is read each frame (8 axes, 128 buttons).
//  * Controls are bound by detection (menu "Wheel & pedals"): the axis that moves most / the button
//    pressed during a short capture. Bindings, ranges and FFB settings: GameUserSettings.ini [KartWheel].
//  * The first device with force feedback is opened EXCLUSIVE and gets a constant-force effect on its
//    steering axis (autocentre spring disabled). The game sends the simulated column torque each frame
//    plus a soft-lock spring at the kart's steering lock.
#pragma once

#include "CoreMinimal.h"

struct FKartAxisBinding
{
	int32 Device = -1;     // index into the device list (by product name match on reload)
	FString DeviceName;
	int32 Axis = -1;       // 0..7: X Y Z Rx Ry Rz Slider0 Slider1
	float Rest = 0.0f;     // raw normalised value at rest (pedals) / centre (steering)
	float Full = 1.0f;     // raw normalised value fully pressed (pedals) / full right lock (steering)
	bool IsBound() const { return Device >= 0 && Axis >= 0; }
};

struct FKartButtonBinding
{
	int32 Device = -1;
	FString DeviceName;
	int32 Button = -1;
	bool IsBound() const { return Device >= 0 && Button >= 0; }
};

class KRPKNKOFF_API FKartWheelInput
{
public:
	static FKartWheelInput& Get();

	// Opens DirectInput with the game window (needed for exclusive FFB). Safe to call repeatedly.
	bool Initialize(void* WindowHandle);
	void Shutdown();
	bool IsInitialized() const { return DI != nullptr; }
	// Re-enumerates devices (after plugging a wheel in while the game runs).
	void Rescan() { void* W = Window; Shutdown(); Initialize(W); }
	void Poll(); // once per frame

	int32 NumDevices() const { return Devices.Num(); }
	FString DeviceName(int32 I) const;
	bool HasForceFeedback() const { return FfbDevice >= 0; }
	FString FfbDeviceName() const { return FfbDevice >= 0 ? DeviceName(FfbDevice) : FString(); }
	float RawAxis(int32 Device, int32 Axis) const;   // -1..1
	bool RawButton(int32 Device, int32 Button) const;

	// --- Bound controls ---
	enum EControl { Steering, Throttle, Brake, Clutch, NumAxisControls };
	enum EButtonControl { ShiftUp, ShiftDown, NumButtonControls };
	FKartAxisBinding Axes[NumAxisControls];
	FKartButtonBinding Buttons[NumButtonControls];
	float RotationDeg = 900.0f;  // the base's configured rotation (lock to lock), e.g. SimPro Manager
	float FfbGain = 0.7f;        // 0..1.5
	bool bInvertFfb = false;
	float SoftLockStrength = 0.8f;

	bool IsSteeringBound() const { return Axes[Steering].IsBound(); }
	// Steering wheel angle in degrees (+ = right), pedal positions 0..1.
	float SteeringDeg() const;
	float Pedal(EControl C) const;
	bool ButtonPressedEdge(EButtonControl B) const;

	// Detection: Begin*, then Update every frame until it returns true (or Cancel).
	void BeginDetectAxis(EControl C);
	void BeginDetectButton(EButtonControl B);
	bool UpdateDetection(); // true when the pending detection finished
	void CancelDetection() { Detecting = -1; }
	bool IsDetecting() const { return Detecting >= 0; }
	FString DetectionPrompt() const;

	// Force feedback: Normalised in [-1, 1], + = turns the wheel LEFT (sim convention).
	void SetForce(float Normalised);
	void StopForce();

	void Load();
	void Save() const;

	FString StatusLine() const;

private:
	struct FDevice;
	TArray<TSharedPtr<FDevice>> Devices;
	void* DI = nullptr;           // IDirectInput8*
	void* Window = nullptr;
	int32 FfbDevice = -1;
	void* FfbEffect = nullptr;    // IDirectInputEffect*
	int32 FfbAxisOffset = 0;
	float LastForce = 0.0f;

	int32 Detecting = -1;         // 0..3 axis control, 10.. button control
	double DetectStart = 0.0;
	TArray<float> DetectBaseline; // per device*8 axes
	TArray<float> DetectMin, DetectMax;
	TArray<uint8> PrevButtons;    // previous frame, per device*128
	TArray<uint8> CurButtons;

	void Rebind(); // resolve saved device names to current indices
};
