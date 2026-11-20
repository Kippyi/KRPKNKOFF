// Tester license (game side): machine code, stored key, activation screen, watermark. The crypto is in
// KartSim/License.h; keys are made by Tools/License/krpk_license.py with the developer's private key.
// Enforced in Shipping builds (the tester build); in Development only with -KartLicenseCheck (testing).
#pragma once

#include "CoreMinimal.h"
#include "KartSim/License.h"

class SWidget;

struct KRPKNKOFF_API FKartLicense
{
	static FKartLicense& Get();

	bool IsRequired() const { return bRequired; }
	bool IsValid() const { return Info.bValid; }
	bool Blocked() const { return bRequired && !Info.bValid; }  // no driving, activation screen instead of the menu
	const kartsim::license::LicenseInfo& GetInfo() const { return Info; }
	const FString& MachineCode() const { return Machine; }
	FString Watermark() const;                                    // on-screen line (empty when not required)

	// Checks a pasted key; saves it when valid. Returns the result (Error filled when invalid).
	const kartsim::license::LicenseInfo& Activate(const FString& Key);

	// Full-screen activation UI. OnActivated runs once a valid key was entered.
	static TSharedRef<SWidget> MakeActivationScreen(TFunction<void()> OnActivated);

	static constexpr int32 GraceDays = 7;   // play offline this long after the last successful revocation check

private:
	void Init();
	void Evaluate();                // key + date + revocation list + offline grace -> Info
	void LoadState();
	void SaveState();
	FString StateMac(const FString& Body) const;
	void StartRevocationCheck();    // async download of the signed list (RevUrl), then Evaluate()
	// Persisted with a checksum (license.state): last date the game ran, last successful online check, newest list serial.
	struct FState { int32 LastSeen = 0, LastCheck = 0; int64 Serial = 0; };
	FState St;
	bool bRequired = false;
	bool bCheckFailed = false;
	FString Machine, KeyPath, StatePath, CachePath, RevUrl, CurrentKey;
	kartsim::license::LicenseInfo Info;
	kartsim::license::RevocationList Revoked;
};

// Small always-on log (Shipping builds compile UE_LOG out): <Saved>/Logs/KartTester.log.
KRPKNKOFF_API void KartTesterLog(const FString& Line);
