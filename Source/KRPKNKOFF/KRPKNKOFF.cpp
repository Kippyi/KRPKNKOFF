#include "Modules/ModuleManager.h"

#include "KartSim/Json.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Parse.h"

namespace
{
	// Kart data (configs, tracks) is read through UE's file system, so it can live in the encrypted pak.
	bool ReadViaUnreal(const std::string& Path, std::string& Out)
	{
		TArray<uint8> Bytes;
		if (!FFileHelper::LoadFileToArray(Bytes, UTF8_TO_TCHAR(Path.c_str()), FILEREAD_Silent)) { return false; }
		Out.assign(reinterpret_cast<const char*>(Bytes.GetData()), Bytes.Num());
		return true;
	}
}

class FKRPKNKOFFModule : public FDefaultGameModuleImpl
{
public:
	virtual void StartupModule() override
	{
		kartsim::SetTextFileReader(&ReadViaUnreal);
#if UE_BUILD_SHIPPING
		// Tester builds: the developer test flags (-KartAutopilot, -KartTestDamage, -KartLicenseCheck, ...) are
		// removed; only these player-facing switches stay.
		static const TCHAR* Allowed[] = {TEXT("-KartOldEngineSound"), TEXT("-KartNoWheel"), TEXT("-KartGraphics=")};
		const TCHAR* Stream = FCommandLine::Get();
		FString Token, Kept;
		while (FParse::Token(Stream, Token, false))
		{
			bool bKeep = !Token.StartsWith(TEXT("-Kart"), ESearchCase::IgnoreCase);
			for (const TCHAR* A : Allowed) { bKeep = bKeep || Token.StartsWith(A, ESearchCase::IgnoreCase); }
			if (bKeep) { Kept += (Kept.IsEmpty() ? TEXT("") : TEXT(" ")) + (Token.Contains(TEXT(" ")) ? TEXT("\"") + Token + TEXT("\"") : Token); }
		}
		FCommandLine::Set(*Kept);
#endif
	}
};

IMPLEMENT_PRIMARY_GAME_MODULE(FKRPKNKOFFModule, KRPKNKOFF, "KRPKNKOFF");
