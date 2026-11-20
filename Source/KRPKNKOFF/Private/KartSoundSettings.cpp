#include "KartSoundSettings.h"

#include "Misc/ConfigCacheIni.h"

namespace
{
	const TCHAR* GSection = TEXT("KartSound");
}

FKartSoundSettings& FKartSoundSettings::Get()
{
	static FKartSoundSettings S;
	static bool bLoaded = false;
	if (!bLoaded && GConfig)
	{
		bLoaded = true;
		GConfig->GetFloat(GSection, TEXT("Master"), S.Master, GGameUserSettingsIni);
		GConfig->GetFloat(GSection, TEXT("Engine"), S.Engine, GGameUserSettingsIni);
		GConfig->GetFloat(GSection, TEXT("OtherKarts"), S.OtherKarts, GGameUserSettingsIni);
		GConfig->GetFloat(GSection, TEXT("Effects"), S.Effects, GGameUserSettingsIni);
		GConfig->GetFloat(GSection, TEXT("Wind"), S.Wind, GGameUserSettingsIni);
		GConfig->GetFloat(GSection, TEXT("EngineTone"), S.EngineTone, GGameUserSettingsIni);
		S.EngineTone = FMath::Clamp(S.EngineTone, 0.5f, 1.6f);
		GConfig->GetBool(GSection, TEXT("Mute"), S.bMute, GGameUserSettingsIni);
	}
	return S;
}

void FKartSoundSettings::Save() const
{
	if (!GConfig) { return; }
	GConfig->SetFloat(GSection, TEXT("Master"), Master, GGameUserSettingsIni);
	GConfig->SetFloat(GSection, TEXT("Engine"), Engine, GGameUserSettingsIni);
	GConfig->SetFloat(GSection, TEXT("OtherKarts"), OtherKarts, GGameUserSettingsIni);
	GConfig->SetFloat(GSection, TEXT("Effects"), Effects, GGameUserSettingsIni);
	GConfig->SetFloat(GSection, TEXT("Wind"), Wind, GGameUserSettingsIni);
	GConfig->SetFloat(GSection, TEXT("EngineTone"), EngineTone, GGameUserSettingsIni);
	GConfig->SetBool(GSection, TEXT("Mute"), bMute, GGameUserSettingsIni);
	GConfig->Flush(false, GGameUserSettingsIni);
}
