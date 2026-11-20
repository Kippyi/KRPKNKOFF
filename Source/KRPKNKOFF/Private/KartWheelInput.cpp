#include "KartWheelInput.h"

#include "HAL/PlatformTime.h"
#include "Misc/ConfigCacheIni.h"

#if PLATFORM_WINDOWS
#include "Windows/AllowWindowsPlatformTypes.h"
#define DIRECTINPUT_VERSION 0x0800
#include <dinput.h>
#include "Windows/HideWindowsPlatformTypes.h"
#endif

DEFINE_LOG_CATEGORY_STATIC(LogKartWheel, Log, All);

namespace
{
	const TCHAR* GSection = TEXT("KartWheel");
	const TCHAR* GAxisKeys[] = {TEXT("Steering"), TEXT("Throttle"), TEXT("Brake"), TEXT("Clutch")};
	const TCHAR* GButtonKeys[] = {TEXT("ShiftUp"), TEXT("ShiftDown")};
	constexpr double DetectSeconds = 4.0;
}

struct FKartWheelInput::FDevice
{
	FString Name;
	bool bFfb = false;
	float Axes[8] = {};
	uint8 Buttons[128] = {};
#if PLATFORM_WINDOWS
	IDirectInputDevice8W* Dev = nullptr;
#endif
};

FKartWheelInput& FKartWheelInput::Get()
{
	static FKartWheelInput W;
	return W;
}

FString FKartWheelInput::DeviceName(int32 I) const
{
	return Devices.IsValidIndex(I) ? Devices[I]->Name : FString();
}

float FKartWheelInput::RawAxis(int32 D, int32 A) const
{
	return (Devices.IsValidIndex(D) && A >= 0 && A < 8) ? Devices[D]->Axes[A] : 0.0f;
}

bool FKartWheelInput::RawButton(int32 D, int32 B) const
{
	return Devices.IsValidIndex(D) && B >= 0 && B < 128 && Devices[D]->Buttons[B] != 0;
}

#if PLATFORM_WINDOWS
namespace
{
	BOOL CALLBACK EnumDevicesCb(const DIDEVICEINSTANCEW* Inst, VOID* Ctx)
	{
		static_cast<TArray<DIDEVICEINSTANCEW>*>(Ctx)->Add(*Inst);
		return DIENUM_CONTINUE;
	}

	BOOL CALLBACK SetAxisRangeCb(const DIDEVICEOBJECTINSTANCEW* Obj, VOID* Ctx)
	{
		DIPROPRANGE R;
		R.diph.dwSize = sizeof(DIPROPRANGE);
		R.diph.dwHeaderSize = sizeof(DIPROPHEADER);
		R.diph.dwHow = DIPH_BYID;
		R.diph.dwObj = Obj->dwType;
		R.lMin = -32768;
		R.lMax = 32767;
		static_cast<IDirectInputDevice8W*>(Ctx)->SetProperty(DIPROP_RANGE, &R.diph);
		return DIENUM_CONTINUE;
	}
}
#endif

bool FKartWheelInput::Initialize(void* WindowHandle)
{
#if PLATFORM_WINDOWS
	if (DI) { return true; }
	Window = WindowHandle;
	IDirectInput8W* Input = nullptr;
	if (FAILED(DirectInput8Create(GetModuleHandleW(nullptr), DIRECTINPUT_VERSION, IID_IDirectInput8W, reinterpret_cast<void**>(&Input), nullptr)))
	{
		UE_LOG(LogKartWheel, Warning, TEXT("DirectInput8Create failed: no wheel support"));
		return false;
	}
	DI = Input;
	TArray<DIDEVICEINSTANCEW> Found;
	Input->EnumDevices(DI8DEVCLASS_GAMECTRL, EnumDevicesCb, &Found, DIEDFL_ATTACHEDONLY);
	for (const DIDEVICEINSTANCEW& Inst : Found)
	{
		IDirectInputDevice8W* Dev = nullptr;
		if (FAILED(Input->CreateDevice(Inst.guidInstance, &Dev, nullptr))) { continue; }
		Dev->SetDataFormat(&c_dfDIJoystick2);
		DIDEVCAPS Caps = {};
		Caps.dwSize = sizeof(Caps);
		Dev->GetCapabilities(&Caps);
		TSharedPtr<FDevice> D = MakeShared<FDevice>();
		D->Dev = Dev;
		D->Name = FString(Inst.tszProductName);
		D->bFfb = (Caps.dwFlags & DIDC_FORCEFEEDBACK) != 0;
		const bool bUseFfb = D->bFfb && FfbDevice < 0 && WindowHandle;
		// Exclusive access is required for force feedback; everything else is shared.
		HRESULT Coop = Dev->SetCooperativeLevel(static_cast<HWND>(WindowHandle), bUseFfb ? (DISCL_EXCLUSIVE | DISCL_BACKGROUND) : (DISCL_NONEXCLUSIVE | DISCL_BACKGROUND));
		if (FAILED(Coop) && bUseFfb)
		{
			UE_LOG(LogKartWheel, Warning, TEXT("%s: exclusive mode refused (another program using FFB?), reading input only"), *D->Name);
			Dev->SetCooperativeLevel(static_cast<HWND>(WindowHandle), DISCL_NONEXCLUSIVE | DISCL_BACKGROUND);
		}
		Dev->EnumObjects(SetAxisRangeCb, Dev, DIDFT_AXIS);
		Dev->Acquire();
		if (bUseFfb && SUCCEEDED(Coop))
		{
			DIPROPDWORD Auto;
			Auto.diph.dwSize = sizeof(DIPROPDWORD);
			Auto.diph.dwHeaderSize = sizeof(DIPROPHEADER);
			Auto.diph.dwObj = 0;
			Auto.diph.dwHow = DIPH_DEVICE;
			Auto.dwData = DIPROPAUTOCENTER_OFF;
			Dev->SetProperty(DIPROP_AUTOCENTER, &Auto.diph);
			DWORD AxesOfs[1] = {DIJOFS_X};
			LONG Dir[1] = {0};
			DICONSTANTFORCE Cf = {0};
			DIEFFECT Eff = {};
			Eff.dwSize = sizeof(DIEFFECT);
			Eff.dwFlags = DIEFF_CARTESIAN | DIEFF_OBJECTOFFSETS;
			Eff.dwDuration = INFINITE;
			Eff.dwGain = DI_FFNOMINALMAX;
			Eff.dwTriggerButton = DIEB_NOTRIGGER;
			Eff.cAxes = 1;
			Eff.rgdwAxes = AxesOfs;
			Eff.rglDirection = Dir;
			Eff.cbTypeSpecificParams = sizeof(DICONSTANTFORCE);
			Eff.lpvTypeSpecificParams = &Cf;
			IDirectInputEffect* Effect = nullptr;
			if (SUCCEEDED(Dev->CreateEffect(GUID_ConstantForce, &Eff, &Effect, nullptr)) && Effect)
			{
				Effect->Start(1, 0);
				FfbEffect = Effect;
				FfbDevice = Devices.Num();
			}
			else
			{
				UE_LOG(LogKartWheel, Warning, TEXT("%s: constant-force effect could not be created"), *D->Name);
			}
		}
		UE_LOG(LogKartWheel, Log, TEXT("Device %d: %s%s"), Devices.Num(), *D->Name, D->bFfb ? (FfbDevice == Devices.Num() ? TEXT(" [force feedback ACTIVE]") : TEXT(" [ffb capable]")) : TEXT(""));
		Devices.Add(D);
	}
	CurButtons.SetNumZeroed(Devices.Num() * 128);
	PrevButtons.SetNumZeroed(Devices.Num() * 128);
	Load();
	Poll();
	UE_LOG(LogKartWheel, Log, TEXT("%d game controller(s); %s"), Devices.Num(), *StatusLine());
	return true;
#else
	return false;
#endif
}

void FKartWheelInput::Shutdown()
{
#if PLATFORM_WINDOWS
	StopForce();
	if (FfbEffect) { static_cast<IDirectInputEffect*>(FfbEffect)->Release(); FfbEffect = nullptr; }
	for (TSharedPtr<FDevice>& D : Devices) { if (D->Dev) { D->Dev->Unacquire(); D->Dev->Release(); } }
	Devices.Reset();
	if (DI) { static_cast<IDirectInput8W*>(DI)->Release(); DI = nullptr; }
	FfbDevice = -1;
#endif
}

void FKartWheelInput::Poll()
{
#if PLATFORM_WINDOWS
	PrevButtons = CurButtons;
	for (int32 I = 0; I < Devices.Num(); ++I)
	{
		FDevice& D = *Devices[I];
		DIJOYSTATE2 S = {};
		D.Dev->Poll();
		if (FAILED(D.Dev->GetDeviceState(sizeof(S), &S)))
		{
			D.Dev->Acquire();
			if (FAILED(D.Dev->GetDeviceState(sizeof(S), &S))) { continue; }
		}
		const LONG Raw[8] = {S.lX, S.lY, S.lZ, S.lRx, S.lRy, S.lRz, S.rglSlider[0], S.rglSlider[1]};
		for (int32 A = 0; A < 8; ++A) { D.Axes[A] = FMath::Clamp((Raw[A] + 0.5f) / 32767.5f, -1.0f, 1.0f); }
		for (int32 B = 0; B < 128; ++B)
		{
			D.Buttons[B] = (S.rgbButtons[B] & 0x80) ? 1 : 0;
			CurButtons[I * 128 + B] = D.Buttons[B];
		}
	}
#endif
}

float FKartWheelInput::SteeringDeg() const
{
	const FKartAxisBinding& B = Axes[Steering];
	if (!B.IsBound()) { return 0.0f; }
	// The axis spans the base's whole configured rotation; Full holds the sign (+1: raw increases to the right).
	return (RawAxis(B.Device, B.Axis) - B.Rest) * FMath::Sign(B.Full) * 0.5f * RotationDeg;
}

float FKartWheelInput::Pedal(EControl C) const
{
	const FKartAxisBinding& B = Axes[C];
	if (!B.IsBound() || FMath::IsNearlyEqual(B.Full, B.Rest)) { return 0.0f; }
	const float V = (RawAxis(B.Device, B.Axis) - B.Rest) / (B.Full - B.Rest);
	return FMath::Clamp((V - 0.02f) / 0.96f, 0.0f, 1.0f); // 2 % dead zone each end
}

bool FKartWheelInput::ButtonPressedEdge(EButtonControl C) const
{
	const FKartButtonBinding& B = Buttons[C];
	if (!B.IsBound()) { return false; }
	const int32 Idx = B.Device * 128 + B.Button;
	return CurButtons.IsValidIndex(Idx) && CurButtons[Idx] && !PrevButtons[Idx];
}

void FKartWheelInput::BeginDetectAxis(EControl C)
{
	Detecting = C;
	DetectStart = FPlatformTime::Seconds();
	DetectBaseline.SetNum(Devices.Num() * 8);
	for (int32 D = 0; D < Devices.Num(); ++D) { for (int32 A = 0; A < 8; ++A) { DetectBaseline[D * 8 + A] = Devices[D]->Axes[A]; } }
	DetectMin = DetectBaseline;
	DetectMax = DetectBaseline;
}

void FKartWheelInput::BeginDetectButton(EButtonControl B)
{
	Detecting = 10 + B;
	DetectStart = FPlatformTime::Seconds();
}

FString FKartWheelInput::DetectionPrompt() const
{
	const int32 Left = FMath::Max(0, FMath::CeilToInt(DetectSeconds - (FPlatformTime::Seconds() - DetectStart)));
	switch (Detecting)
	{
	case Steering: return FString::Printf(TEXT("Turn the wheel to the RIGHT (a quarter turn) ... %d"), Left);
	case Throttle: return FString::Printf(TEXT("Press the THROTTLE fully, then release ... %d"), Left);
	case Brake: return FString::Printf(TEXT("Press the BRAKE fully, then release ... %d"), Left);
	case Clutch: return FString::Printf(TEXT("Press the CLUTCH fully, then release ... %d"), Left);
	case 10: return TEXT("Press the SHIFT UP paddle / button");
	case 11: return TEXT("Press the SHIFT DOWN paddle / button");
	default: return FString();
	}
}

bool FKartWheelInput::UpdateDetection()
{
	if (Detecting < 0) { return false; }
	const double T = FPlatformTime::Seconds() - DetectStart;
	if (Detecting >= 10)
	{
		for (int32 I = 0; I < CurButtons.Num(); ++I)
		{
			if (CurButtons[I] && !PrevButtons[I])
			{
				FKartButtonBinding& B = Buttons[Detecting - 10];
				B.Device = I / 128;
				B.Button = I % 128;
				B.DeviceName = DeviceName(B.Device);
				Detecting = -1;
				Save();
				return true;
			}
		}
		if (T > 10.0) { Detecting = -1; return true; }
		return false;
	}
	for (int32 D = 0; D < Devices.Num(); ++D)
	{
		for (int32 A = 0; A < 8; ++A)
		{
			const int32 K = D * 8 + A;
			if (!DetectMin.IsValidIndex(K)) { continue; }
			DetectMin[K] = FMath::Min(DetectMin[K], Devices[D]->Axes[A]);
			DetectMax[K] = FMath::Max(DetectMax[K], Devices[D]->Axes[A]);
		}
	}
	if (T < DetectSeconds) { return false; }
	// Pick the axis that moved furthest from its starting value.
	int32 Best = -1;
	float BestDev = 0.2f;
	for (int32 K = 0; K < DetectBaseline.Num(); ++K)
	{
		const float Dev = FMath::Max(DetectMax[K] - DetectBaseline[K], DetectBaseline[K] - DetectMin[K]);
		if (Dev > BestDev) { BestDev = Dev; Best = K; }
	}
	if (Best >= 0)
	{
		FKartAxisBinding& B = Axes[Detecting];
		B.Device = Best / 8;
		B.Axis = Best % 8;
		B.DeviceName = DeviceName(B.Device);
		B.Rest = DetectBaseline[Best];
		const bool bUp = DetectMax[Best] - DetectBaseline[Best] >= DetectBaseline[Best] - DetectMin[Best];
		B.Full = Detecting == Steering ? (bUp ? 1.0f : -1.0f) : (bUp ? DetectMax[Best] : DetectMin[Best]);
		UE_LOG(LogKartWheel, Log, TEXT("Bound %s to %s axis %d (rest %.3f, full %.3f)"), GAxisKeys[Detecting], *B.DeviceName, B.Axis, B.Rest, B.Full);
		Save();
	}
	Detecting = -1;
	return true;
}

void FKartWheelInput::SetForce(float Normalised)
{
#if PLATFORM_WINDOWS
	if (!FfbEffect) { return; }
	// DirectInput: positive constant force on X pushes the wheel to the right (+ = left in the sim).
	const float F = FMath::Clamp(-Normalised * (bInvertFfb ? -1.0f : 1.0f), -1.0f, 1.0f);
	if (FMath::Abs(F - LastForce) < 0.002f) { return; }
	LastForce = F;
	DICONSTANTFORCE Cf;
	Cf.lMagnitude = static_cast<LONG>(F * DI_FFNOMINALMAX);
	DIEFFECT Eff = {};
	Eff.dwSize = sizeof(DIEFFECT);
	Eff.cbTypeSpecificParams = sizeof(DICONSTANTFORCE);
	Eff.lpvTypeSpecificParams = &Cf;
	static_cast<IDirectInputEffect*>(FfbEffect)->SetParameters(&Eff, DIEP_TYPESPECIFICPARAMS);
#endif
}

void FKartWheelInput::StopForce()
{
	LastForce = 1.0f; // force the next SetForce through
	SetForce(0.0f);
}

void FKartWheelInput::Rebind()
{
	auto Resolve = [this](const FString& Name) { for (int32 I = 0; I < Devices.Num(); ++I) { if (Devices[I]->Name == Name) { return I; } } return -1; };
	for (FKartAxisBinding& B : Axes) { if (!B.DeviceName.IsEmpty()) { B.Device = Resolve(B.DeviceName); } }
	for (FKartButtonBinding& B : Buttons) { if (!B.DeviceName.IsEmpty()) { B.Device = Resolve(B.DeviceName); } }
	// Not calibrated yet: assume the force-feedback device's X axis steers (true for wheel bases),
	// increasing to the right, centred.
	if (!Axes[Steering].IsBound() && FfbDevice >= 0)
	{
		Axes[Steering].Device = FfbDevice;
		Axes[Steering].DeviceName = DeviceName(FfbDevice);
		Axes[Steering].Axis = 0;
		Axes[Steering].Rest = 0.0f;
		Axes[Steering].Full = 1.0f;
	}
}

void FKartWheelInput::Load()
{
	if (!GConfig) { return; }
	for (int32 I = 0; I < NumAxisControls; ++I)
	{
		FString V;
		if (GConfig->GetString(GSection, GAxisKeys[I], V, GGameUserSettingsIni))
		{
			TArray<FString> P;
			V.ParseIntoArray(P, TEXT("|"), false);
			if (P.Num() == 4) { Axes[I].DeviceName = P[0]; Axes[I].Axis = FCString::Atoi(*P[1]); Axes[I].Rest = FCString::Atof(*P[2]); Axes[I].Full = FCString::Atof(*P[3]); }
		}
	}
	for (int32 I = 0; I < NumButtonControls; ++I)
	{
		FString V;
		if (GConfig->GetString(GSection, GButtonKeys[I], V, GGameUserSettingsIni))
		{
			TArray<FString> P;
			V.ParseIntoArray(P, TEXT("|"), false);
			if (P.Num() == 2) { Buttons[I].DeviceName = P[0]; Buttons[I].Button = FCString::Atoi(*P[1]); }
		}
	}
	GConfig->GetFloat(GSection, TEXT("RotationDeg"), RotationDeg, GGameUserSettingsIni);
	GConfig->GetFloat(GSection, TEXT("FfbGain"), FfbGain, GGameUserSettingsIni);
	GConfig->GetBool(GSection, TEXT("InvertFfb"), bInvertFfb, GGameUserSettingsIni);
	GConfig->GetFloat(GSection, TEXT("SoftLock"), SoftLockStrength, GGameUserSettingsIni);
	Rebind();
}

void FKartWheelInput::Save() const
{
	if (!GConfig) { return; }
	for (int32 I = 0; I < NumAxisControls; ++I)
	{
		const FKartAxisBinding& B = Axes[I];
		if (B.IsBound()) { GConfig->SetString(GSection, GAxisKeys[I], *FString::Printf(TEXT("%s|%d|%f|%f"), *B.DeviceName, B.Axis, B.Rest, B.Full), GGameUserSettingsIni); }
	}
	for (int32 I = 0; I < NumButtonControls; ++I)
	{
		const FKartButtonBinding& B = Buttons[I];
		if (B.IsBound()) { GConfig->SetString(GSection, GButtonKeys[I], *FString::Printf(TEXT("%s|%d"), *B.DeviceName, B.Button), GGameUserSettingsIni); }
	}
	GConfig->SetFloat(GSection, TEXT("RotationDeg"), RotationDeg, GGameUserSettingsIni);
	GConfig->SetFloat(GSection, TEXT("FfbGain"), FfbGain, GGameUserSettingsIni);
	GConfig->SetBool(GSection, TEXT("InvertFfb"), bInvertFfb, GGameUserSettingsIni);
	GConfig->SetFloat(GSection, TEXT("SoftLock"), SoftLockStrength, GGameUserSettingsIni);
	GConfig->Flush(false, GGameUserSettingsIni);
}

FString FKartWheelInput::StatusLine() const
{
	auto AxisStr = [this](int32 C) { const FKartAxisBinding& B = Axes[C]; return B.IsBound() ? FString::Printf(TEXT("%s ax%d"), *B.DeviceName, B.Axis) : FString(TEXT("--")); };
	return FString::Printf(TEXT("steer %s | thr %s | brk %s | clutch %s | FFB %s"), *AxisStr(Steering), *AxisStr(Throttle), *AxisStr(Brake), *AxisStr(Clutch),
	                       FfbDevice >= 0 ? *DeviceName(FfbDevice) : TEXT("none"));
}
