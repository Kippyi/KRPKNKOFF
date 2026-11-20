// Player sound settings (volumes 0..1, mute), saved in Saved/Config/<platform>/GameUserSettings.ini
// section [KartSound]. Read every frame by UKartAudioComponent; edited on the menu's Settings page.
#pragma once

#include "CoreMinimal.h"

struct KRPKNKOFF_API FKartSoundSettings
{
	float Master = 0.8f;
	float Engine = 0.8f;
	float OtherKarts = 0.55f;
	float Effects = 0.7f;   // tyres, contacts
	float Wind = 0.3f;
	float EngineTone = 1.0f; // engine sound tone corner scale: 0.5 darker .. 1.6 brighter (physical engine sound v4)
	bool bMute = false;

	static FKartSoundSettings& Get();   // loaded on first use
	void Save() const;
};
