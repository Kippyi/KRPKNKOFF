#include "KartForceFeedbackDevice.h"

#include "KartWheelInput.h"

FString FKartDirectInputFFBDevice::GetName() const { return FKartWheelInput::Get().FfbDeviceName(); }
bool FKartDirectInputFFBDevice::IsConnected() const { return FKartWheelInput::Get().HasForceFeedback(); }
void FKartDirectInputFFBDevice::SetConstantTorque(float NormalisedTorque) { FKartWheelInput::Get().SetForce(NormalisedTorque); }
void FKartDirectInputFFBDevice::Stop() { FKartWheelInput::Get().StopForce(); }
