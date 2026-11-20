// Player realism settings (Milestone 8), saved in GameUserSettings.ini section [KartRealism]. Edited on the
// menu's Settings page; read by AKartVehiclePawn every physics step.
#pragma once

#include "CoreMinimal.h"

struct KRPKNKOFF_API FKartRealismSettings
{
	bool bDamage = true;          // bent steering / axle, thrown chain, holed radiator, fairing
	bool bEngineFailures = true;  // lean / overheat seizure
	bool bDashOverlay = true;     // Mychron on the HUD in the chase camera (the cockpit view shows the real one)

	static FKartRealismSettings& Get();   // loaded on first use; -KartNoDamage / -KartNoFailures override
	void Save() const;
};
