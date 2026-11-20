#include "KartLicense.h"

#include "HAL/FileManager.h"
#include "HttpModule.h"
#include "Interfaces/IHttpRequest.h"
#include "Interfaces/IHttpResponse.h"
#include "HAL/PlatformApplicationMisc.h"
#include "HAL/PlatformProcess.h"
#include "Misc/CommandLine.h"
#include "Misc/DateTime.h"
#include "Misc/FileHelper.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "Styling/CoreStyle.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SMultiLineEditableTextBox.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/SCompoundWidget.h"
#include "Widgets/Text/STextBlock.h"

#if PLATFORM_WINDOWS
#include "Windows/AllowWindowsPlatformTypes.h"
#include <windows.h>
#include "Windows/HideWindowsPlatformTypes.h"
#endif

namespace
{
	FString MachineIdentity()
	{
		FString Id;
#if PLATFORM_WINDOWS
		// Windows' per-installation machine GUID + the system volume's serial number.
		HKEY Key = nullptr;
		if (RegOpenKeyExW(HKEY_LOCAL_MACHINE, L"SOFTWARE\\Microsoft\\Cryptography", 0, KEY_READ | KEY_WOW64_64KEY, &Key) == ERROR_SUCCESS)
		{
			wchar_t Buf[128] = {};
			DWORD Size = sizeof(Buf) - sizeof(wchar_t);
			DWORD Type = 0;
			if (RegQueryValueExW(Key, L"MachineGuid", nullptr, &Type, reinterpret_cast<LPBYTE>(Buf), &Size) == ERROR_SUCCESS && Type == REG_SZ) { Id = Buf; }
			RegCloseKey(Key);
		}
		wchar_t WinDir[MAX_PATH] = {};
		if (GetWindowsDirectoryW(WinDir, MAX_PATH) >= 3)
		{
			wchar_t Root[4] = {WinDir[0], L':', L'\\', 0};
			DWORD Serial = 0;
			if (GetVolumeInformationW(Root, nullptr, 0, &Serial, nullptr, nullptr, nullptr, 0)) { Id += FString::Printf(TEXT("|%08x"), Serial); }
		}
#endif
		if (Id.IsEmpty()) { Id = FPlatformMisc::GetLoginId(); } // fallback (other platforms)
		return Id.ToLower();
	}

	int32 YmdOf(const FDateTime& D) { return D.GetYear() * 10000 + D.GetMonth() * 100 + D.GetDay(); }
}

void KartTesterLog(const FString& Line)
{
	const FString Path = FPaths::ProjectSavedDir() / TEXT("Logs") / TEXT("KartTester.log");
	const FString Stamped = FString::Printf(TEXT("[%s] %s\r\n"), *FDateTime::Now().ToString(TEXT("%Y-%m-%d %H:%M:%S")), *Line);
	FFileHelper::SaveStringToFile(Stamped, *Path, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM, &IFileManager::Get(), FILEWRITE_Append);
}

FKartLicense& FKartLicense::Get()
{
	static FKartLicense L;
	static bool bInit = false;
	if (!bInit) { bInit = true; L.Init(); }
	return L;
}

void FKartLicense::Init()
{
#if UE_BUILD_SHIPPING
	bRequired = true;
#else
	bRequired = FParse::Param(FCommandLine::Get(), TEXT("KartLicenseCheck"));
#endif
	Machine = UTF8_TO_TCHAR(kartsim::license::MachineCodeFrom(TCHAR_TO_UTF8(*MachineIdentity())).c_str());
	const FString Dir = FPaths::Combine(FPlatformProcess::UserSettingsDir(), TEXT("KRPKNKOFF"), TEXT("License"));
	KeyPath = Dir / TEXT("license.key");
	StatePath = Dir / TEXT("license.state");
	CachePath = Dir / TEXT("revoked.cache");
	RevUrl = UTF8_TO_TCHAR(kartsim::license::GameRevocationUrl());
#if !UE_BUILD_SHIPPING
	FParse::Value(FCommandLine::Get(), TEXT("KartRevocationUrl="), RevUrl); // testing (a local server)
#endif
	LoadState();
	FString Cached;
	if (FFileHelper::LoadFileToString(Cached, *CachePath))
	{
		Revoked = kartsim::license::ParseRevocationList(TCHAR_TO_UTF8(*Cached), kartsim::license::GamePublicModulus(), kartsim::license::GamePublicExponent());
		if (Revoked.Serial < St.Serial) { Revoked = kartsim::license::RevocationList(); } // the cache file was swapped for an older list
	}
	FFileHelper::LoadFileToString(CurrentKey, *KeyPath);
	Evaluate();
	if (bRequired)
	{
		KartTesterLog(FString::Printf(TEXT("License: machine %s, %s; revocation check %s"), *Machine,
		                              Info.bValid ? *FString::Printf(TEXT("valid for %s until %d (key %s)"), UTF8_TO_TCHAR(Info.Name.c_str()), Info.Expires, UTF8_TO_TCHAR(Info.KeyId.c_str()))
		                                          : *FString(UTF8_TO_TCHAR(Info.Error.c_str())),
		                              RevUrl.IsEmpty() ? TEXT("not set up") : TEXT("started")));
		if (!RevUrl.IsEmpty()) { StartRevocationCheck(); }
	}
}

namespace
{
	int32 DaysBetween(int32 A, int32 B) // B - A in days, YYYYMMDD
	{
		if (A <= 0) { return 1000000; }
		const FDateTime Da(A / 10000, FMath::Clamp((A / 100) % 100, 1, 12), FMath::Clamp(A % 100, 1, 28));
		const FDateTime Db(B / 10000, FMath::Clamp((B / 100) % 100, 1, 12), FMath::Clamp(B % 100, 1, 28));
		return (Db - Da).GetDays();
	}
}

FString FKartLicense::StateMac(const FString& Body) const
{
	return UTF8_TO_TCHAR(kartsim::license::Sha256Hex(std::string("KRPK-STATE|") + TCHAR_TO_UTF8(*Machine) + "|" + TCHAR_TO_UTF8(*Body)).substr(0, 16).c_str());
}

void FKartLicense::LoadState()
{
	St = FState();
	FString S;
	if (!FFileHelper::LoadFileToString(S, *StatePath)) { return; }
	TArray<FString> F;
	S.TrimStartAndEnd().ParseIntoArray(F, TEXT("|"), false);
	if (F.Num() == 2 && F[1] == StateMac(F[0])) { St.LastSeen = FCString::Atoi(*F[0]); } // v1: last date seen only
	else if (F.Num() == 5 && F[0] == TEXT("v2"))
	{
		const FString Body = F[1] + TEXT("|") + F[2] + TEXT("|") + F[3];
		if (F[4] == StateMac(Body)) { St.LastSeen = FCString::Atoi(*F[1]); St.LastCheck = FCString::Atoi(*F[2]); St.Serial = FCString::Atoi64(*F[3]); }
	}
}

void FKartLicense::SaveState()
{
	const FString Body = FString::Printf(TEXT("%d|%d|%lld"), St.LastSeen, St.LastCheck, St.Serial);
	FFileHelper::SaveStringToFile(TEXT("v2|") + Body + TEXT("|") + StateMac(Body), *StatePath);
}

void FKartLicense::Evaluate()
{
	using namespace kartsim::license;
	Info = LicenseInfo();
	if (!HasGamePublicKey())
	{
		Info.Error = "This build has no license public key (developer: run Tools/License/krpk_license.py init and rebuild).";
		return;
	}
	if (CurrentKey.TrimStartAndEnd().IsEmpty()) { Info.Error = "Not activated yet."; return; }
	const int32 Today = FMath::Max(YmdOf(FDateTime::Now()), St.LastSeen); // winding the clock back does not help
	Info = CheckLicense(TCHAR_TO_UTF8(*CurrentKey), GamePublicModulus(), GamePublicExponent(), TCHAR_TO_UTF8(*Machine), Today);
	if (!Info.bValid || RevUrl.IsEmpty()) { return; }
	if (IsRevoked(Revoked, Info))
	{
		Info.bValid = false;
		Info.Error = "This license was revoked by the developer.";
		return;
	}
	if (bCheckFailed && DaysBetween(St.LastCheck, Today) > GraceDays)
	{
		Info.bValid = false;
		Info.Error = St.LastCheck > 0 ? "Your license could not be checked online for more than 7 days. Connect to the internet and restart the game."
		                              : "Your license has to be checked online once. Connect to the internet and restart the game.";
	}
}

void FKartLicense::StartRevocationCheck()
{
	TSharedRef<IHttpRequest, ESPMode::ThreadSafe> Req = FHttpModule::Get().CreateRequest();
	Req->SetURL(RevUrl + (RevUrl.Contains(TEXT("?")) ? TEXT("&") : TEXT("?")) + TEXT("t=") + FString::Printf(TEXT("%lld"), FDateTime::UtcNow().ToUnixTimestamp()));
	Req->SetVerb(TEXT("GET"));
	Req->SetTimeout(15.0f);
	Req->OnProcessRequestComplete().BindLambda([](FHttpRequestPtr, FHttpResponsePtr Resp, bool bConnected)
	{
		FKartLicense& L = FKartLicense::Get();
		const bool bWasValid = L.Info.bValid;
		FString Result;
		bool bOk = false;
		if (bConnected && Resp.IsValid() && Resp->GetResponseCode() == 200)
		{
			const FString Text = Resp->GetContentAsString();
			const kartsim::license::RevocationList List =
				kartsim::license::ParseRevocationList(TCHAR_TO_UTF8(*Text), kartsim::license::GamePublicModulus(), kartsim::license::GamePublicExponent());
			if (List.bValid && List.Serial >= L.St.Serial)
			{
				bOk = true;
				L.Revoked = List;
				L.St.Serial = List.Serial;
				L.St.LastCheck = FMath::Max(YmdOf(FDateTime::Now()), L.St.LastSeen);
				L.SaveState();
				FFileHelper::SaveStringToFile(Text, *L.CachePath);
				Result = FString::Printf(TEXT("ok (list %lld: %d keys, %d PCs revoked)"), List.Serial, static_cast<int32>(List.KeyIds.size()), static_cast<int32>(List.Machines.size()));
			}
			else { Result = List.bValid ? TEXT("an OLDER list was served (ignored)") : TEXT("the list's signature is invalid (ignored)"); }
		}
		else { Result = FString::Printf(TEXT("failed (HTTP %d)"), Resp.IsValid() ? Resp->GetResponseCode() : 0); }
		L.bCheckFailed = !bOk;
		L.Evaluate();
		KartTesterLog(FString::Printf(TEXT("Revocation check: %s -> license %s"), *Result, L.Info.bValid ? TEXT("valid") : UTF8_TO_TCHAR(L.Info.Error.c_str())));
		if (bWasValid && !L.Info.bValid) { KartTesterLog(TEXT("License withdrawn during the session.")); }
	});
	Req->ProcessRequest();
}

const kartsim::license::LicenseInfo& FKartLicense::Activate(const FString& Key)
{
	CurrentKey = Key.TrimStartAndEnd();
	Evaluate();
	if (Info.bValid)
	{
		FFileHelper::SaveStringToFile(CurrentKey, *KeyPath);
		St.LastSeen = FMath::Max(YmdOf(FDateTime::Now()), St.LastSeen);
		SaveState();
	}
	return Info;
}

FString FKartLicense::Watermark() const
{
	if (!bRequired || !Info.bValid) { return FString(); }
	return FString::Printf(TEXT("KRPKNKOFF TEST BUILD  -  licensed to %s  -  %s  -  not for distribution"), UTF8_TO_TCHAR(Info.Name.c_str()), *Machine);
}

namespace
{
	class SKartLicenseScreen : public SCompoundWidget
	{
	public:
		SLATE_BEGIN_ARGS(SKartLicenseScreen) {}
		SLATE_END_ARGS()

		TFunction<void()> OnActivated;

		void Construct(const FArguments& InArgs)
		{
			const FSlateFontInfo Title = FCoreStyle::GetDefaultFontStyle("Bold", 28);
			const FSlateFontInfo Body = FCoreStyle::GetDefaultFontStyle("Regular", 14);
			const FSlateFontInfo Code = FCoreStyle::GetDefaultFontStyle("Mono", 26);
			const FLinearColor Faint(0.70f, 0.72f, 0.76f);
			FKartLicense& L = FKartLicense::Get();
			Status = FText::FromString(L.GetInfo().Error.empty() || L.GetInfo().Error == "Not activated yet." ? FString() : FString(UTF8_TO_TCHAR(L.GetInfo().Error.c_str())));
			ChildSlot
			[
				SNew(SBorder).BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush")).BorderBackgroundColor(FLinearColor(0.035f, 0.04f, 0.05f, 1.0f))
				.HAlign(HAlign_Center).VAlign(VAlign_Center)
				[
					SNew(SBox).WidthOverride(900.0f)
					[
						SNew(SVerticalBox)
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)[ SNew(STextBlock).Text(FText::FromString(TEXT("KRPKNKOFF  -  tester activation"))).Font(Title) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 22)[ SNew(STextBlock).AutoWrapText(true).Font(Body).ColorAndOpacity(Faint)
							.Text(FText::FromString(TEXT("This test build needs a license key for this PC. Send the machine code below to the developer; "
							                             "you will get a key back. Paste it here and press Activate. Keys only work on the PC they were made for."))) ]
						+ SVerticalBox::Slot().AutoHeight()[ SNew(STextBlock).Text(FText::FromString(TEXT("Your machine code"))).Font(Body) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 22)
						[
							SNew(SHorizontalBox)
							+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)[ SNew(STextBlock).Text(FText::FromString(L.MachineCode())).Font(Code).ColorAndOpacity(FLinearColor(1.0f, 0.78f, 0.25f)) ]
							+ SHorizontalBox::Slot().AutoWidth().Padding(24, 0, 0, 0).VAlign(VAlign_Center)
							[
								SNew(SButton).Text(FText::FromString(TEXT("Copy"))).OnClicked_Lambda([this]()
								{
									FPlatformApplicationMisc::ClipboardCopy(*FKartLicense::Get().MachineCode());
									Status = FText::FromString(TEXT("Machine code copied."));
									return FReply::Handled();
								})
							]
						]
						+ SVerticalBox::Slot().AutoHeight()[ SNew(STextBlock).Text(FText::FromString(TEXT("License key"))).Font(Body) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 10)
						[
							SNew(SBox).HeightOverride(120.0f)
							[
								SAssignNew(KeyBox, SMultiLineEditableTextBox).AutoWrapText(true).Font(FCoreStyle::GetDefaultFontStyle("Mono", 11))
								.HintText(FText::FromString(TEXT("Paste the key you received here")))
							]
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 10)
						[
							SNew(SHorizontalBox)
							+ SHorizontalBox::Slot().AutoWidth()
							[
								SNew(SButton).Text(FText::FromString(TEXT("Paste"))).OnClicked_Lambda([this]()
								{
									FString Clip;
									FPlatformApplicationMisc::ClipboardPaste(Clip);
									KeyBox->SetText(FText::FromString(Clip));
									return FReply::Handled();
								})
							]
							+ SHorizontalBox::Slot().AutoWidth().Padding(12, 0, 0, 0)
							[
								SNew(SButton).Text(FText::FromString(TEXT("Activate"))).OnClicked_Lambda([this]()
								{
									const kartsim::license::LicenseInfo& R = FKartLicense::Get().Activate(KeyBox->GetText().ToString());
									KartTesterLog(FString::Printf(TEXT("License activation: %s"), R.bValid ? TEXT("OK") : UTF8_TO_TCHAR(R.Error.c_str())));
									if (R.bValid)
									{
										Status = FText::FromString(FString::Printf(TEXT("Activated for %s."), UTF8_TO_TCHAR(R.Name.c_str())));
										if (OnActivated) { OnActivated(); }
									}
									else { Status = FText::FromString(UTF8_TO_TCHAR(R.Error.c_str())); }
									return FReply::Handled();
								})
							]
							+ SHorizontalBox::Slot().FillWidth(1.0f)[ SNullWidget::NullWidget ]
							+ SHorizontalBox::Slot().AutoWidth()
							[
								SNew(SButton).Text(FText::FromString(TEXT("Quit"))).OnClicked_Lambda([]() { FPlatformMisc::RequestExit(false); return FReply::Handled(); })
							]
						]
						+ SVerticalBox::Slot().AutoHeight()[ SNew(STextBlock).AutoWrapText(true).Font(Body).ColorAndOpacity(FLinearColor(1.0f, 0.45f, 0.4f)).Text_Lambda([this]() { return Status; }) ]
					]
				]
			];
		}

	private:
		TSharedPtr<SMultiLineEditableTextBox> KeyBox;
		FText Status;
	};
}

TSharedRef<SWidget> FKartLicense::MakeActivationScreen(TFunction<void()> OnActivated)
{
	TSharedRef<SKartLicenseScreen> W = SNew(SKartLicenseScreen);
	W->OnActivated = MoveTemp(OnActivated);
	return W;
}
