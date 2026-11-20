// Force feedback output abstraction. The simulation produces a normalised steering torque
// command (kartsim::ForceFeedbackProcessor); a device implementation sends it to hardware.
//
// Milestone 1 ships only the null device (signal visible in telemetry/HUD). Planned devices:
//  - DirectInput8 constant-force effect (covers Logitech G29/G923, Thrustmaster, Simagic in
//    DirectInput compatibility mode) - Milestone 2.
//  - Vendor SDKs where they add value (e.g. Simagic high-rate API) - evaluated in Milestone 2.
// Unreal's built-in force feedback is gamepad vibration only, which the brief rules out.
#pragma once

#include "CoreMinimal.h"

class IKartFFBDevice
{
public:
	virtual ~IKartFFBDevice() = default;
	virtual FString GetName() const = 0;
	virtual bool IsConnected() const = 0;
	// Command in [-1, 1]: fraction of the device's maximum torque, positive = pulls the wheel left.
	virtual void SetConstantTorque(float NormalisedTorque) = 0;
	virtual void Stop() = 0;
};

// Accepts commands and records the last one; used when no FFB hardware backend is available.
class FKartNullFFBDevice final : public IKartFFBDevice
{
public:
	virtual FString GetName() const override { return TEXT("None (signal only)"); }
	virtual bool IsConnected() const override { return false; }
	virtual void SetConstantTorque(float NormalisedTorque) override { LastCommand = NormalisedTorque; }
	virtual void Stop() override { LastCommand = 0.0f; }
	float LastCommand = 0.0f;
};

// DirectInput constant force through FKartWheelInput (Milestone 7): wheel bases such as Simagic,
// Logitech, Thrustmaster, Fanatec, Moza in DirectInput mode.
class FKartDirectInputFFBDevice final : public IKartFFBDevice
{
public:
	virtual FString GetName() const override;
	virtual bool IsConnected() const override;
	virtual void SetConstantTorque(float NormalisedTorque) override;
	virtual void Stop() override;
};
