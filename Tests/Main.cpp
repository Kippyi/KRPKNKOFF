// KartSimTests: standalone (no Unreal) test runner and scenario harness for the KartSim core.
//   KartSimTests [--config path] [filter]          run unit tests
//   KartSimTests [--config path] --scenarios dir   run vehicle-dynamics scenarios, write CSVs
//   KartSimTests --sound <KartConfig.json> <out.wav> [laps]   render the engine sound of autopilot laps
#include "TestFramework.h"
#include "TestHelpers.h"

#include <cstdlib>
#include <cstring>
#include <string>

int RunScenarios(const std::string& OutDir);
int RunManeuvers();
int RunDriverLaps();
int RunRateSweep();
int RunClassComparison();
int RenderSound(const char* ConfigFile, const char* OutPath, int Laps);
int RenderSoundSweep(const char* ConfigFile, const char* OutPath, bool bOld);

int main(int Argc, char** Argv)
{
	const char* Filter = nullptr;
	std::string ScenarioDir;
	for (int I = 1; I < Argc; ++I)
	{
		if (std::strcmp(Argv[I], "--config") == 0 && I + 1 < Argc) { kstest::ConfigPath() = Argv[++I]; }
		else if (std::strcmp(Argv[I], "--maneuvers") == 0) { return RunManeuvers(); }
		else if (std::strcmp(Argv[I], "--laps") == 0) { return RunDriverLaps(); }
		else if (std::strcmp(Argv[I], "--ratesweep") == 0) { return RunRateSweep(); }
		else if (std::strcmp(Argv[I], "--classes") == 0) { return RunClassComparison(); }
		else if ((std::strcmp(Argv[I], "--soundsweep") == 0 || std::strcmp(Argv[I], "--soundsweepold") == 0) && I + 2 < Argc)
		{
			return RenderSoundSweep(Argv[I + 1], Argv[I + 2], std::strcmp(Argv[I], "--soundsweepold") == 0);
		}
		else if (std::strcmp(Argv[I], "--sound") == 0 && I + 2 < Argc) { return RenderSound(Argv[I + 1], Argv[I + 2], I + 3 < Argc ? std::atoi(Argv[I + 3]) : 1); }
		else if (std::strcmp(Argv[I], "--scenarios") == 0 && I + 1 < Argc) { ScenarioDir = Argv[++I]; }
		else { Filter = Argv[I]; }
	}
	if (!ScenarioDir.empty())
	{
		return RunScenarios(ScenarioDir);
	}
	return kstest::RunAll(Filter) == 0 ? 0 : 1;
}
