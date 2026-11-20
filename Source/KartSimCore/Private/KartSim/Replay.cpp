#include "KartSim/Replay.h"

#include "KartSim/KartPhysicsCore.h"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace kartsim
{
	ReplayFrame MakeReplayFrame(const KartSimulation& Sim, double Time)
	{
		ReplayFrame F;
		const ChassisState& C = Sim.GetChassis();
		F.Time = static_cast<float>(Time);
		F.Pos[0] = static_cast<float>(C.Position.X); F.Pos[1] = static_cast<float>(C.Position.Y); F.Pos[2] = static_cast<float>(C.Position.Z);
		F.Rot[0] = static_cast<float>(C.Orientation.W); F.Rot[1] = static_cast<float>(C.Orientation.X);
		F.Rot[2] = static_cast<float>(C.Orientation.Y); F.Rot[3] = static_cast<float>(C.Orientation.Z);
		F.Vel[0] = static_cast<float>(C.Velocity.X); F.Vel[1] = static_cast<float>(C.Velocity.Y); F.Vel[2] = static_cast<float>(C.Velocity.Z);
		F.SteerL = static_cast<float>(Sim.GetWheel(WheelFL).SteerAngle);
		F.SteerR = static_cast<float>(Sim.GetWheel(WheelFR).SteerAngle);
		F.OmegaF = static_cast<float>(Sim.GetWheel(WheelFL).Omega);
		F.OmegaR = static_cast<float>(Sim.GetWheel(WheelRL).Omega);
		const TelemetryFrame& T = Sim.GetTelemetry();
		F.Rpm = static_cast<float>(T.EngineRpm);
		F.Throttle = static_cast<float>(T.ThrottleEff);
		F.Brake = static_cast<float>(T.Brake);
		F.Gear = static_cast<int32_t>(T.Gear);
		F.Flags = T.EngineRunning > 0.5 ? 1u : 0u;
		return F;
	}

	ReplayFrame LerpFrame(const ReplayFrame& A, const ReplayFrame& B, double Alpha)
	{
		const float T = static_cast<float>(Alpha < 0.0 ? 0.0 : (Alpha > 1.0 ? 1.0 : Alpha));
		auto L = [T](float X, float Y) { return X + (Y - X) * T; };
		ReplayFrame F = T < 0.5f ? A : B; // discrete fields (gear, flags) from the nearer frame
		F.Time = L(A.Time, B.Time);
		for (int I = 0; I < 3; ++I) { F.Pos[I] = L(A.Pos[I], B.Pos[I]); F.Vel[I] = L(A.Vel[I], B.Vel[I]); }
		// Rotation: nlerp along the shorter arc (frames are 33 ms apart: nlerp ~ slerp).
		const float Dot = A.Rot[0] * B.Rot[0] + A.Rot[1] * B.Rot[1] + A.Rot[2] * B.Rot[2] + A.Rot[3] * B.Rot[3];
		const float S = Dot < 0.0f ? -1.0f : 1.0f;
		float Q[4], N = 0.0f;
		for (int I = 0; I < 4; ++I) { Q[I] = A.Rot[I] + (S * B.Rot[I] - A.Rot[I]) * T; N += Q[I] * Q[I]; }
		N = N > 1e-12f ? 1.0f / std::sqrt(N) : 1.0f;
		for (int I = 0; I < 4; ++I) { F.Rot[I] = Q[I] * N; }
		F.SteerL = L(A.SteerL, B.SteerL); F.SteerR = L(A.SteerR, B.SteerR);
		F.OmegaF = L(A.OmegaF, B.OmegaF); F.OmegaR = L(A.OmegaR, B.OmegaR);
		F.Rpm = L(A.Rpm, B.Rpm); F.Throttle = L(A.Throttle, B.Throttle); F.Brake = L(A.Brake, B.Brake);
		return F;
	}

	void ApplyReplayFrame(KartSimulation& Sim, const ReplayFrame& F, double Dt)
	{
		Sim.SetKinematicPose(Vec3(F.Pos[0], F.Pos[1], F.Pos[2]), Quat(F.Rot[0], F.Rot[1], F.Rot[2], F.Rot[3]), Vec3(F.Vel[0], F.Vel[1], F.Vel[2]),
		                     Vec3(), F.SteerL, F.SteerR, F.OmegaF, F.OmegaR, Dt);
	}

	bool ReplayTrack::Sample(double T, ReplayFrame& Out) const
	{
		if (Frames.empty()) { return false; }
		if (T <= Frames.front().Time) { Out = Frames.front(); return true; }
		if (T >= Frames.back().Time) { Out = Frames.back(); return true; }
		size_t Lo = 0, Hi = Frames.size() - 1;
		while (Hi - Lo > 1)
		{
			const size_t Mid = (Lo + Hi) / 2;
			if (Frames[Mid].Time <= T) { Lo = Mid; } else { Hi = Mid; }
		}
		const double Span = Frames[Hi].Time - Frames[Lo].Time;
		Out = LerpFrame(Frames[Lo], Frames[Hi], Span > 0.0 ? (T - Frames[Lo].Time) / Span : 0.0);
		Out.Time = static_cast<float>(T);
		return true;
	}

	double ReplayRecording::StartTime() const
	{
		double T = 1e300;
		for (const ReplayTrack& K : Karts) { if (!K.Frames.empty() && K.StartTime() < T) { T = K.StartTime(); } }
		return T == 1e300 ? 0.0 : T;
	}

	double ReplayRecording::EndTime() const
	{
		double T = 0.0;
		for (const ReplayTrack& K : Karts) { if (!K.Frames.empty() && K.EndTime() > T) { T = K.EndTime(); } }
		return T;
	}

	size_t ReplayRecording::FrameCount() const
	{
		size_t N = 0;
		for (const ReplayTrack& K : Karts) { N += K.Frames.size(); }
		return N;
	}

	void ReplayRecording::Trim(double MaxSeconds)
	{
		const double Cut = EndTime() - MaxSeconds;
		for (ReplayTrack& K : Karts)
		{
			size_t N = 0;
			while (N < K.Frames.size() && K.Frames[N].Time < Cut) { ++N; }
			if (N > 0) { K.Frames.erase(K.Frames.begin(), K.Frames.begin() + static_cast<std::ptrdiff_t>(N)); }
		}
	}

	namespace
	{
		FILE* OpenFile(const std::string& Path, const char* Mode)
		{
			FILE* F = nullptr;
#if defined(_MSC_VER)
			if (fopen_s(&F, Path.c_str(), Mode) != 0) { F = nullptr; }
#else
			F = std::fopen(Path.c_str(), Mode);
#endif
			return F;
		}
	}

	// --- File format (little-endian, as written on Windows x64) ---
	// "KRPL" u32 version | str circuit | str info | f64 lapTime | u32 karts
	// per kart: str name | str kartFile | u32 color | u32 frameSize | u64 frames | frames (raw ReplayFrame)
	namespace
	{
		constexpr uint32_t kVersion = 1;
		struct Writer
		{
			FILE* F;
			bool Ok = true;
			void Raw(const void* P, size_t N) { Ok = Ok && std::fwrite(P, 1, N, F) == N; }
			template <class T> void Pod(const T& V) { Raw(&V, sizeof(T)); }
			void Str(const std::string& S) { Pod(static_cast<uint32_t>(S.size())); Raw(S.data(), S.size()); }
		};
		struct Reader
		{
			FILE* F;
			bool Ok = true;
			void Raw(void* P, size_t N) { Ok = Ok && std::fread(P, 1, N, F) == N; }
			template <class T> void Pod(T& V) { Raw(&V, sizeof(T)); }
			void Str(std::string& S)
			{
				uint32_t N = 0;
				Pod(N);
				if (!Ok || N > (1u << 20)) { Ok = false; return; }
				S.resize(N);
				Raw(N ? &S[0] : nullptr, N);
			}
		};
	}

	bool ReplayRecording::Save(const std::string& Path) const
	{
		FILE* F = OpenFile(Path, "wb");
		if (!F) { return false; }
		Writer W{F};
		W.Raw("KRPL", 4);
		W.Pod(kVersion);
		W.Str(Circuit);
		W.Str(Info);
		W.Pod(LapTime);
		W.Pod(static_cast<uint32_t>(Karts.size()));
		for (const ReplayTrack& K : Karts)
		{
			W.Str(K.Name);
			W.Str(K.KartFile);
			W.Pod(K.Color);
			W.Pod(static_cast<uint32_t>(sizeof(ReplayFrame)));
			W.Pod(static_cast<uint64_t>(K.Frames.size()));
			if (!K.Frames.empty()) { W.Raw(K.Frames.data(), K.Frames.size() * sizeof(ReplayFrame)); }
		}
		const bool bOk = W.Ok;
		return std::fclose(F) == 0 && bOk;
	}

	bool ReplayRecording::Load(const std::string& Path, std::string& OutError)
	{
		FILE* F = OpenFile(Path, "rb");
		if (!F) { OutError = "cannot open " + Path; return false; }
		Reader R{F};
		char Magic[4] = {};
		uint32_t Version = 0, NumKarts = 0;
		R.Raw(Magic, 4);
		R.Pod(Version);
		ReplayRecording Out;
		if (!R.Ok || std::memcmp(Magic, "KRPL", 4) != 0 || Version != kVersion) { std::fclose(F); OutError = "not a replay file (or another version)"; return false; }
		R.Str(Out.Circuit);
		R.Str(Out.Info);
		R.Pod(Out.LapTime);
		R.Pod(NumKarts);
		if (NumKarts > 256) { R.Ok = false; }
		for (uint32_t I = 0; I < NumKarts && R.Ok; ++I)
		{
			ReplayTrack K;
			uint32_t FrameSize = 0;
			uint64_t N = 0;
			R.Str(K.Name);
			R.Str(K.KartFile);
			R.Pod(K.Color);
			R.Pod(FrameSize);
			R.Pod(N);
			if (!R.Ok || FrameSize != sizeof(ReplayFrame) || N > 100000000ull) { R.Ok = false; break; }
			K.Frames.resize(static_cast<size_t>(N));
			if (N) { R.Raw(K.Frames.data(), static_cast<size_t>(N) * sizeof(ReplayFrame)); }
			Out.Karts.push_back(std::move(K));
		}
		std::fclose(F);
		if (!R.Ok) { OutError = "truncated or corrupt replay file"; return false; }
		*this = std::move(Out);
		return true;
	}

	// --- Ghost ---

	void GhostKeeper::Step(bool bTiming, double LapElapsed, const KartSimulation& Sim)
	{
		if (!bTiming)
		{
			if (bWasTiming) { AbortLap(); } // kart reset mid-lap
			bWasTiming = false;
			return;
		}
		if (!bWasTiming) { AbortLap(); }
		bWasTiming = true;
		if (LapElapsed + 1e-9 >= NextSample)
		{
			Current.push_back(MakeReplayFrame(Sim, LapElapsed));
			NextSample = LapElapsed + 1.0 / Rate;
		}
	}

	bool GhostKeeper::OnLapCompleted(double LapTime, bool bValid)
	{
		// The lap timer starts the next lap at the same instant: the frames so far are this lap. Short
		// buffers (joined mid-lap, reset) are not a whole lap.
		const bool bWhole = !Current.empty() && Current.front().Time < 0.5 && Current.back().Time > LapTime - 0.5;
		bool bNew = false;
		if (bValid && bWhole && (BestTime <= 0.0 || LapTime < BestTime))
		{
			Best = Current;
			BestTime = LapTime;
			bNew = true;
		}
		Current.clear();
		NextSample = 0.0;
		return bNew;
	}

	void GhostKeeper::Offer(const std::vector<ReplayFrame>& Frames, double LapTime)
	{
		if (Frames.empty() || LapTime <= 0.0) { return; }
		if (BestTime <= 0.0 || LapTime < BestTime) { Best = Frames; BestTime = LapTime; }
	}
}
