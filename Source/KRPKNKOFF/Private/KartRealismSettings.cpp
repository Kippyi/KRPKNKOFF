#include "KartRealismSettings.h"

#include "Misc/CommandLine.h"
#include "Misc/ConfigCacheIni.h"
#include "Misc/Parse.h"

namespace
{
	const TCHAR* GSection = TEXT("KartRealism");
}

FKartRealismSettings& FKartRealismSettings::Get()
{
	static FKartRealismSettings S;
	static bool bLoaded = false;
	if (!bLoaded && GConfig)
	{
		bLoaded = true;
		GConfig->GetBool(GSection, TEXT("Damage"), S.bDamage, GGameUserSettingsIni);
		GConfig->GetBool(GSection, TEXT("EngineFailures"), S.bEngineFailures, GGameUserSettingsIni);
		GConfig->GetBool(GSection, TEXT("DashOverlay"), S.bDashOverlay, GGameUserSettingsIni);
		if (FParse::Param(FCommandLine::Get(), TEXT("KartNoDamage"))) { S.bDamage = false; }
		if (FParse::Param(FCommandLine::Get(), TEXT("KartNoFailures"))) { S.bEngineFailures = false; }
	}
	return S;
}

void FKartRealismSettings::Save() const
{
	if (!GConfig) { return; }
	GConfig->SetBool(GSection, TEXT("Damage"), bDamage, GGameUserSettingsIni);
	GConfig->SetBool(GSection, TEXT("EngineFailures"), bEngineFailures, GGameUserSettingsIni);
	GConfig->SetBool(GSection, TEXT("DashOverlay"), bDashOverlay, GGameUserSettingsIni);
	GConfig->Flush(false, GGameUserSettingsIni);
}
