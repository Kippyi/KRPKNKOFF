// Offline engine-sound renderer:  KartSimTests --sound <KartConfig.json> <out.wav> [laps]
// Drives the kart with the core Autopilot on the test circuit (same driver as the game's -KartAutopilot) and feeds
// the per-step engine telemetry into kartsim::EngineSoundSynth, so the sound can be listened to / analysed without
// the game. Timeline: 2.5 s idle (clutch open), the laps (standing start), 3 s lift-off coast-down.
#include "DriverBot.h"
#include "TestHelpers.h"

#include "KartSim/EngineSound.h"
#include "KartSim/KartPhysicsCore.h"
#include "KartSim/Telemetry.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>
#include <chrono>

using namespace kartsim;

namespace
{
	struct SoundKey { double T; EngineSoundInput In; };

	bool WriteWav(const char* Path, const std::vector<float>& S, int Rate)
	{
		FILE* F = std::fopen(Path, "wb");
		if (!F) { return false; }
		const uint32_t DataBytes = static_cast<uint32_t>(S.size() * 2);
		auto U32 = [F](uint32_t V) { std::fwrite(&V, 4, 1, F); };
		auto U16 = [F](uint16_t V) { std::fwrite(&V, 2, 1, F); };
		std::fwrite("RIFF", 1, 4, F); U32(36 + DataBytes); std::fwrite("WAVEfmt ", 1, 8, F);
		U32(16); U16(1); U16(1); U32(static_cast<uint32_t>(Rate)); U32(static_cast<uint32_t>(Rate * 2)); U16(2); U16(16);
		std::fwrite("data", 1, 4, F); U32(DataBytes);
		for (float V : S)
		{
			const float C = V > 1.0f ? 1.0f : (V < -1.0f ? -1.0f : V);
			const int16_t X = static_cast<int16_t>(std::lround(C * 32000.0f));
			std::fwrite(&X, 2, 1, F);
		}
		std::fclose(F);
		return true;
	}
}

int RenderSound(const char* ConfigFile, const char* OutPath, int Laps)
{
	const KartConfig C = kstest::LoadKart(ConfigFile);
	std::vector<SoundKey> Keys;
	double T = 0.0;
	// Idle at the grid: clutch open, idle control holding ~idle rpm.
	for (; T < 2.5; T += 0.01)
	{
		EngineSoundInput In;
		In.Rpm = static_cast<float>(C.Engine.IdleRpm) + 60.0f * static_cast<float>(std::sin(T * 7.0));
		In.Throttle = 0.12f; In.Load = 0.05f; In.bClutchLocked = false; In.AxleHz = 0.0f;
		Keys.push_back({T, In});
	}
	const double T0 = T;
	double Last = 0.0;
	const LapResult R = DriveLaps(C, 1.4, Laps, nullptr, nullptr, nullptr, 0.0, [&](const KartSimulation& Sim, double Time)
	{
		const TelemetryFrame& F = Sim.GetTelemetry();
		EngineSoundInput In;
		In.Rpm = static_cast<float>(F.EngineRpm);
		In.Throttle = static_cast<float>(F.ThrottleEff);
		In.Load = static_cast<float>(std::fmin(std::fmax(F.EngineTorque / 25.0, 0.0), 1.0));
		In.bRunning = F.EngineRunning > 0.5;
		In.bLimiter = F.RevLimiter > 0.5;
		In.bClutchLocked = F.ClutchLocked > 0.5;
		In.AxleHz = static_cast<float>(std::fabs(F.RearAxleRpm) / 60.0);
		Keys.push_back({T0 + Time, In});
		Last = T0 + Time;
	});
	// Lift off: overrun coast-down to idle.
	EngineSoundInput End = Keys.back().In;
	for (double Tl = 0.0; Tl < 3.0; Tl += 0.01)
	{
		EngineSoundInput In = End;
		const double K = std::exp(-Tl / 1.1);
		In.Rpm = static_cast<float>(C.Engine.IdleRpm + (End.Rpm - C.Engine.IdleRpm) * K);
		In.Throttle = 0.0f; In.Load = 0.0f;
		In.AxleHz = static_cast<float>(End.AxleHz * K);
		In.bClutchLocked = In.Rpm > 4500.0f;
		Keys.push_back({Last + Tl, In});
	}

	const std::string ClassId = C.ClassId.empty() ? std::string(ConfigFile) : C.ClassId;
	EngineSoundSynth Synth;
	const int Rate = 48000;
	Synth.SetProfile(EngineSoundProfile::ForClass(ClassId), static_cast<float>(Rate));
	std::vector<float> Audio;
	const double Duration = Keys.back().T;
	Audio.reserve(static_cast<size_t>(Duration * Rate) + 64);
	size_t K = 0;
	const int Block = 64;
	float Buf[Block];
	double Peak = 0.0, SumSq = 0.0;
	for (double Ts = 0.0; Ts < Duration; Ts += static_cast<double>(Block) / Rate)
	{
		while (K + 1 < Keys.size() && Keys[K + 1].T <= Ts) { ++K; }
		Synth.Render(Buf, Block, Keys[K].In);
		for (float V : Buf) { Audio.push_back(V); Peak = std::fmax(Peak, std::fabs(V)); SumSq += static_cast<double>(V) * V; }
	}
	if (!WriteWav(OutPath, Audio, Rate)) { std::printf("cannot write %s\n", OutPath); return 1; }
	std::printf("sound: %s (%s) %.1f s, %d laps (best %.3f s), peak %.2f, rms %.3f -> %s\n", ConfigFile, ClassId.c_str(), Duration,
	            R.LapsDone, R.BestLap, Peak, std::sqrt(SumSq / static_cast<double>(Audio.size())), OutPath);
	return 0;
}

// Scripted sweep with the rpm known at every instant (analysis): 2 s idle, 1 s blip, 8 s full-throttle ramp from
// 4,000 rpm to the rev limit (clutch locked), 3 s lift-off coast to 6,000, 2 s steady half throttle at 9,000.
// Writes <out.wav> and <out.wav>.rpm.csv (time, rpm, throttle every 10 ms). bOld = the v3 pulse synth.
int RenderSoundSweep(const char* ConfigFile, const char* OutPath, bool bOld)
{
	const KartConfig C = kstest::LoadKart(ConfigFile);
	const std::string ClassId = C.ClassId.empty() ? std::string(ConfigFile) : C.ClassId;
	EngineSoundProfile P = EngineSoundProfile::ForClass(ClassId);
	P.bPhysical = !bOld;
	EngineSoundSynth Synth;
	const int Rate = 48000;
	Synth.SetProfile(P, static_cast<float>(Rate));
	const float Idle = static_cast<float>(C.Engine.IdleRpm);
	const float Limit = static_cast<float>(C.Engine.RevLimitRpm > 0.0 ? C.Engine.RevLimitRpm : 14000.0);
	auto At = [&](double T, EngineSoundInput& In)
	{
		In = EngineSoundInput();
		In.bRunning = true;
		if (T < 2.0) { In.Rpm = Idle; In.Throttle = 0.1f; In.bClutchLocked = false; }
		else if (T < 3.0) { const double U = (T - 2.0); In.Rpm = Idle + (7000.0f - Idle) * static_cast<float>(std::sin(3.14159 * U)); In.Throttle = U < 0.4 ? 1.0f : 0.0f; In.bClutchLocked = false; }
		else if (T < 11.0) { const double U = (T - 3.0) / 8.0; In.Rpm = 4000.0f + (Limit - 4000.0f) * static_cast<float>(U); In.Throttle = 1.0f; In.Load = 1.0f; }
		else if (T < 14.0) { const double U = (T - 11.0) / 3.0; In.Rpm = Limit + (6000.0f - Limit) * static_cast<float>(1.0 - std::exp(-3.0 * U)); In.Throttle = 0.0f; }
		else { In.Rpm = 9000.0f; In.Throttle = 0.5f; In.Load = 0.5f; }
		In.AxleHz = In.bClutchLocked ? In.Rpm / 60.0f / 6.0f : 0.0f;
	};
	std::vector<float> Audio;
	const double Duration = 16.0;
	const int Block = 64;
	float Buf[Block];
	std::string Csv = std::string(OutPath) + ".rpm.csv";
	FILE* Fc = std::fopen(Csv.c_str(), "w");
	double NextCsv = 0.0;
	const auto T0 = std::chrono::steady_clock::now();
	for (double Ts = 0.0; Ts < Duration; Ts += static_cast<double>(Block) / Rate)
	{
		EngineSoundInput In;
		At(Ts, In);
		Synth.Render(Buf, Block, In);
		for (float V : Buf) { Audio.push_back(V); }
		if (Fc && Ts >= NextCsv) { std::fprintf(Fc, "%.3f,%.1f,%.2f\n", Ts, In.Rpm, In.Throttle); NextCsv += 0.01; }
	}
	const double Sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - T0).count();
	if (Fc) { std::fclose(Fc); }
	if (!WriteWav(OutPath, Audio, Rate)) { std::printf("cannot write %s\n", OutPath); return 1; }
	std::printf("sweep: %s (%s, %s) -> %s, render %.0f ms per second of audio\n", ConfigFile, ClassId.c_str(), bOld ? "v3 pulse" : "v4 physical", OutPath, 1000.0 * Sec / Duration);
	return 0;
}
