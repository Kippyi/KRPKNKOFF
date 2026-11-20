// Replays, ghost laps and the telemetry-viewer lap traces.
#include "DriverBot.h"
#include "TestFramework.h"
#include "TestHelpers.h"
#include "TestTrack.h"

#include "KartSim/KartPhysicsCore.h"
#include "KartSim/LapDelta.h"
#include "KartSim/LapTiming.h"
#include "KartSim/Replay.h"
#include "KartSim/TrackPath.h"

#include <cmath>
#include <cstdio>
#include <cstring>

using namespace kartsim;

namespace
{
	TimingLine LineAcross(const kstest::TrackLine& T, size_t I)
	{
		const Vec3 Dir = (T.P[(I + 1) % T.P.size()] - T.P[I]).Normalized();
		const Vec3 Left(-Dir.Y, Dir.X, 0.0);
		TimingLine L;
		L.A = T.P[I] + Left * 8.0;
		L.B = T.P[I] - Left * 8.0;
		L.Forward = Dir;
		return L;
	}
	double Dist(const float* A, const Vec3& B) { return std::sqrt(Sq(A[0] - B.X) + Sq(A[1] - B.Y) + Sq(A[2] - B.Z)); }

	void TruncateFile(const std::string& Path)
	{
		std::string All;
		if (FILE* In = std::fopen(Path.c_str(), "rb"))
		{
			char Buf[4096];
			size_t Got;
			while ((Got = std::fread(Buf, 1, sizeof(Buf), In)) > 0) { All.append(Buf, Got); }
			std::fclose(In);
		}
		if (FILE* Out = std::fopen(Path.c_str(), "wb"))
		{
			std::fwrite(All.data(), 1, All.size() / 2, Out);
			std::fclose(Out);
		}
	}
}

KS_TEST(Replay_RecordsAndReplaysTheRealPath)
{
	// Record autopilot laps at 30 Hz; at every 240 Hz physics step the interpolated replay must be within a
	// few cm of where the kart really was (render accuracy), and the file must round-trip exactly.
	const KartConfig C = kstest::LoadRotax();
	ReplayRecording Rec;
	Rec.Circuit = "TestCircuit.json";
	Rec.Karts.resize(1);
	Rec.Karts[0].Name = "Test";
	Rec.Karts[0].KartFile = "Rotax125MaxSenior.json";
	std::vector<std::pair<double, Vec3>> Truth;
	double Next = 0.0;
	DriveLaps(C, 1.4, 2, nullptr, nullptr, nullptr, 0.0, [&](const KartSimulation& Sim, double T)
	{
		if (T + 1e-9 >= Next) { Rec.Karts[0].Frames.push_back(MakeReplayFrame(Sim, T)); Next = T + 1.0 / 30.0; }
		Truth.push_back({T, Sim.GetChassis().Position});
	});
	double MaxErr = 0.0;
	for (const auto& P : Truth)
	{
		if (P.first > Rec.EndTime()) { break; }
		ReplayFrame F;
		KS_EXPECT(Rec.Karts[0].Sample(P.first, F));
		MaxErr = std::fmax(MaxErr, Dist(F.Pos, P.second));
	}
	const std::string Path = "Saved/SimRuns/replay_test.krpl";
	KS_EXPECT(Rec.Save(Path));
	ReplayRecording Back;
	std::string Err;
	KS_EXPECT(Back.Load(Path, Err));
	KS_LOG("%zu frames (%.0f s, %.1f KB per kart-minute); max interpolation error vs the 240 Hz path %.1f mm", Rec.FrameCount(), Rec.EndTime(),
	       Rec.FrameCount() * sizeof(ReplayFrame) / 1024.0 / (Rec.EndTime() / 60.0), MaxErr * 1000.0);
	KS_EXPECT(MaxErr < 0.03);
	KS_EXPECT(Back.Karts.size() == 1 && Back.Karts[0].Frames.size() == Rec.Karts[0].Frames.size() && Back.Circuit == Rec.Circuit);
	KS_EXPECT(Back.Karts[0].Frames.size() > 0 &&
	          std::memcmp(Back.Karts[0].Frames.data(), Rec.Karts[0].Frames.data(), Rec.Karts[0].Frames.size() * sizeof(ReplayFrame)) == 0);
	// A truncated file is rejected and leaves the target untouched.
	TruncateFile(Path);
	ReplayRecording Bad;
	KS_EXPECT(!Bad.Load(Path, Err));
	KS_EXPECT(Bad.Karts.empty());
	KS_LOG("truncated file: %s", Err.c_str());
	// Trim keeps the last N seconds.
	Rec.Trim(10.0);
	KS_EXPECT(Rec.EndTime() - Rec.StartTime() <= 10.0 + 1e-3);
}

KS_TEST(Replay_KinematicPoseMatchesTheFrame)
{
	const KartConfig C = kstest::LoadRotax();
	KartSimulation A, B;
	A.Initialize(C);
	B.Initialize(C);
	A.Reset(Vec3(), 0.3);
	B.Reset(Vec3(10, 10, 0), 0.0);
	PlaneTrack Ground;
	KartInputs In;
	In.Throttle = 1.0;
	In.SteeringColumnAngle = 0.4;
	for (int I = 0; I < 600; ++I) { A.Step(1.0 / 240.0, In, Ground); }
	const ReplayFrame F = MakeReplayFrame(A, A.GetTime());
	ApplyReplayFrame(B, F, 1.0 / 30.0);
	const double DPos = (B.GetChassis().Position - A.GetChassis().Position).Length();
	const double DSteer = std::fabs(B.GetWheel(WheelFL).SteerAngle - A.GetWheel(WheelFL).SteerAngle);
	const double DWheel = (B.GetWheel(WheelRR).CentreWorld - A.GetWheel(WheelRR).CentreWorld).Length();
	KS_LOG("posed kart vs source: CoG %.2e m, steer %.2e rad, RR wheel centre %.1f mm (frame twist is not replayed); rpm %.0f thr %.2f",
	       DPos, DSteer, DWheel * 1000.0, F.Rpm, F.Throttle);
	KS_EXPECT(DPos < 1e-5 && DSteer < 1e-5 && DWheel < 0.01);
	KS_EXPECT(F.Rpm > 1000.0f && F.Throttle > 0.5f);
}

KS_TEST(Ghost_KeepsTheFastestValidWholeLap)
{
	const KartConfig C = kstest::LoadRotax();
	const kstest::TrackLine Track = kstest::BuildTestTrack();
	TimingLine Line = LineAcross(Track, 0);
	LapTimer Timer;
	Timer.Configure(&Line, 1);
	GhostKeeper Ghost;
	size_t Seen = 0;
	std::vector<bool> NewBest;
	DriveLaps(C, 1.4, 4, nullptr, &Timer, nullptr, 0.0, [&](const KartSimulation& Sim, double T)
	{
		for (; Seen < Timer.Laps().size(); ++Seen) { NewBest.push_back(Ghost.OnLapCompleted(Timer.Laps()[Seen].LapTime, Timer.Laps()[Seen].bValid)); }
		Ghost.Step(Timer.IsTiming(), Timer.CurrentLapTime(T), Sim);
	});
	const auto& Laps = Timer.Laps();
	double Best = 1e9;
	for (const LapRecord& L : Laps) { if (L.bValid) { Best = std::fmin(Best, L.LapTime); } }
	KS_LOG("%zu timed laps, best %.3f s; ghost %.3f s with %zu frames", Laps.size(), Best, Ghost.GhostTime(), Ghost.Ghost().size());
	for (size_t I = 0; I < NewBest.size(); ++I) { KS_LOG("  lap %zu %.3f s -> %s", I + 1, Laps[I].LapTime, NewBest[I] ? "new ghost" : "-"); }
	KS_EXPECT(Ghost.HasGhost());
	KS_EXPECT_NEAR(Ghost.GhostTime(), Best, 1e-9);
	KS_EXPECT(!Ghost.Ghost().empty() && Ghost.Ghost().front().Time < 0.05 && Ghost.Ghost().back().Time > Best - 0.05);
	// The ghost's position at lap time t is where the kart was t seconds after crossing the line:
	// lap time 0 and lap time = best are both on the line.
	ReplayTrack G;
	G.Frames = Ghost.Ghost();
	ReplayFrame Start, End;
	G.Sample(0.0, Start);
	G.Sample(Best, End);
	const double Gap = std::sqrt(Sq(Start.Pos[0] - End.Pos[0]) + Sq(Start.Pos[1] - End.Pos[1]));
	KS_LOG("ghost start-to-end gap %.2f m", Gap);
	KS_EXPECT(Gap < 3.0);
	// A slower offered lap does not replace it; a faster one does.
	GhostKeeper Copy = Ghost;
	Copy.Offer(Ghost.Ghost(), Best + 1.0);
	KS_EXPECT_NEAR(Copy.GhostTime(), Best, 1e-9);
	Copy.Offer(Ghost.Ghost(), Best - 1.0);
	KS_EXPECT_NEAR(Copy.GhostTime(), Best - 1.0, 1e-9);
}

KS_TEST(LapTraces_HistoryWithChannelsForTheViewer)
{
	const KartConfig C = kstest::LoadRotax();
	const kstest::TrackLine Track = kstest::BuildTestTrack();
	TimingLine Line = LineAcross(Track, 0);
	LapTimer Timer;
	Timer.Configure(&Line, 1);
	TrackPath Path;
	Path.Build(Track.P);
	LapDelta Delta;
	Delta.Configure(&Path);
	size_t Seen = 0;
	DriveLaps(C, 1.4, 4, nullptr, &Timer, nullptr, 0.0, [&](const KartSimulation& Sim, double T)
	{
		for (; Seen < Timer.Laps().size(); ++Seen) { Delta.OnLapCompleted(Timer.Laps()[Seen].LapTime, Timer.Laps()[Seen].bValid); }
		if (!Timer.IsTiming()) { return; }
		const TelemetryFrame& F = Sim.GetTelemetry();
		TraceInputs In;
		In.SpeedKmh = F.SpeedKmh; In.Throttle = F.Throttle; In.Brake = F.Brake; In.SteerDeg = F.SteerColumnDeg;
		In.Rpm = F.EngineRpm; In.Gear = F.Gear; In.LatG = F.AccLatG; In.LongG = F.AccLongG;
		Delta.Sample(Timer.CurrentLapTime(T), Sim.GetChassis().Position, In);
	});
	const auto& H = Delta.History();
	KS_LOG("history: %zu laps, %zu stations each (%.0f m)", H.size(), Delta.SamplesPerLap(), Delta.SamplesPerLap() * LapTrace::Spacing);
	KS_EXPECT(H.size() >= 2);
	for (const LapTrace& L : H)
	{
		const size_t N = L.Time.size();
		KS_EXPECT(N == Delta.SamplesPerLap());
		KS_EXPECT(L.Speed.size() == N && L.Throttle.size() == N && L.Brake.size() == N && L.Steer.size() == N && L.Rpm.size() == N &&
		          L.Gear.size() == N && L.LatG.size() == N && L.LongG.size() == N && L.X.size() == N && L.Y.size() == N);
		double MaxThr = 0, MaxBrk = 0, MaxSteer = 0;
		for (size_t I = 0; I < N; ++I) { MaxThr = std::fmax(MaxThr, L.Throttle[I]); MaxBrk = std::fmax(MaxBrk, L.Brake[I]); MaxSteer = std::fmax(MaxSteer, std::fabs(L.Steer[I])); }
		const std::vector<double> D = DeltaTrace(L, Delta.ReferenceLap());
		KS_LOG("  lap %d %.3f s valid %d: max throttle %.2f brake %.2f steer %.0f deg, delta to best at the end %+.3f s", L.LapNumber, L.LapTime,
		       L.bValid ? 1 : 0, MaxThr, MaxBrk, MaxSteer, D.empty() ? 0.0 : D.back());
		KS_EXPECT(MaxThr > 0.9 && MaxBrk > 0.05 && MaxSteer > 5.0);
		KS_EXPECT(D.size() == N);
	}
	// The map: the traced positions close the loop.
	if (!H.empty())
	{
		const LapTrace& L = H.back();
		KS_EXPECT(std::hypot(L.X.front() - L.X.back(), L.Y.front() - L.Y.back()) < 6.0);
	}
}
