// Kart setup: the driver-adjustable subset of KartConfig, with legal ranges, and JSON save/load.
// Values are exchanged in display units (degrees, metres, teeth).
#pragma once

#include "KartSim/KartConfig.h"

#include <string>

namespace kartsim
{
	struct SetupParam
	{
		const char* Key;        // JSON key
		const char* Label;      // UI label
		const char* Unit;       // display unit
		double Min, Max, Step;  // display units
		double (*Get)(const KartConfig&);
		void (*Set)(KartConfig&, double);
		bool (*Available)(const KartConfig&) = nullptr; // null = every class; false = hidden/ignored
		const char* Help = "";                           // one line for the setup page
		bool IsAvailable(const KartConfig& C) const { return !Available || Available(C); }
	};

	// All adjustable parameters (static table).
	KARTSIM_API const SetupParam* GetSetupParams(int& OutCount);

	// Sets one parameter by index with clamping to [Min, Max] and snapping to Step.
	KARTSIM_API void SetSetupValue(KartConfig& Config, int Index, double Value);

	// {"classId": "...", "values": {"key": value, ...}}
	KARTSIM_API std::string SerializeSetup(const KartConfig& Config);

	// Applies the values found in Text (unknown keys are reported, missing keys keep the current value).
	// Fails if the setup belongs to a different classId.
	KARTSIM_API bool ApplySetupJson(const std::string& Text, KartConfig& Config, std::string& OutError);
}
