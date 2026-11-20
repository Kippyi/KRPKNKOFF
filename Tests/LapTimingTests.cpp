// Lap timing: crossing geometry, sub-tick precision, track limits, and agreement with the driver bot.
#include "DriverBot.h"
#include "TestFramework.h"
#include "TestHelpers.h"
#include "TestTrack.h"

#include "KartSim/LapTiming.h"

using namespace kartsim;
using kstest::kTick;

namespace
{
	TimingLine LineAt(const kstest::TrackLine& T, size_t I)
	{
		const size_t M = T.P.size();
		const Vec3 Tan = (T.P[(I + 1) % M] - T.P[(I + M - 1) % M]).Normalized();
		const Vec3 N(-Tan.Y, Tan.X, 0.0);
		return TimingLine{T.P[I] + N * 6.0, T.P[I] - N * 6.0, Tan};
	}
}

KS_TEST(LapTiming_CrossingGeometryAndDirection)
{
	const TimingLine L{Vec3(10, -5, 0), Vec3(10, 5, 0), Vec3(1, 0, 0)};
	KS_EXPECT_NEAR(LapTimer::CrossingFraction(L, Vec3(9.9, 0, 0), Vec3(10.1, 0, 0)), 0.5, 1e-12);
	KS_EXPECT(LapTimer::CrossingFraction(L, Vec3(10.1, 0, 0), Vec3(9.9, 0, 0)) < 0.0);  // backwards
	KS_EXPECT(LapTimer::CrossingFraction(L, Vec3(9.9, 6, 0), Vec3(10.1, 6, 0)) < 0.0);  // beside the line
	KS_EXPECT(LapTimer::CrossingFraction(L, Vec3(9.0, 0, 0), Vec3(9.5, 0, 0)) < 0.0);   // not reached
}

KS_TEST(LapTiming_SubTickPrecisionAndTrackLimits)
{
	// Straight "track": start/finish at x = 10.05, one sector line at x = 60.05, wrap via teleport.
	TimingLine Lines[2] = {{Vec3(10.05, -5, 0), Vec3(10.05, 5, 0), Vec3(1, 0, 0)},
	                       {Vec3(60.05, -5, 0), Vec3(60.05, 5, 0), Vec3(1, 0, 0)}};
	LapTimer T;
	T.Configure(Lines, 2);
	const double V = 20.0; // m/s, exact crossing times known analytically
	auto Drive = [&](double X0, double X1, double TStart, bool bOffAt70)
	{
		double X = X0, Time = TStart;
		while (X < X1)
		{
			const double Xn = X + V * kTick;
			Time += kTick;
			T.Update(Time, kTick, Vec3(X, 0, 0), Vec3(Xn, 0, 0), bOffAt70 && X > 70.0 && X < 71.0);
			X = Xn;
		}
		return Time;
	};
	// Lap 1: x 0 -> 110, then "teleport" back to x = 0 (closed loop stand-in) for lap 2.
	double Time = Drive(0.0, 110.0, 0.0, false);
	Time = Drive(0.0, 110.0, Time, true);  // lap 2: all wheels off for 1 m -> invalid
	Time = Drive(0.0, 15.0, Time, false);  // cross start/finish to close lap 2
	(void)Time;
	const auto& Laps = T.Laps();
	KS_EXPECT(Laps.size() == 2);
	if (Laps.size() == 2)
	{
		// Lap 1: start/finish at t = 10.05/20 = 0.5025 s; next start/finish crossing after the
		// teleport. Sector 1 = 50 m / 20 m/s = 2.5 s exactly; sub-tick interpolation must hit it.
		KS_LOG("lap1 %.6f s (S1 %.6f S2 %.6f) valid %d | lap2 %.6f valid %d | best index %d",
		       Laps[0].LapTime, Laps[0].Sectors[0], Laps[0].Sectors[1], Laps[0].bValid ? 1 : 0, Laps[1].LapTime, Laps[1].bValid ? 1 : 0, T.BestLapIndex());
		KS_EXPECT_NEAR(Laps[0].Sectors[0], 2.5, 1e-9);
		KS_EXPECT(Laps[0].bValid);
		KS_EXPECT(!Laps[1].bValid);
		KS_EXPECT(T.BestLapIndex() == 0);
	}
}

KS_TEST(LapTiming_AgreesWithDriverBotOnCircuit)
{
	const kstest::TrackLine Track = kstest::BuildTestTrack();
	const size_t M = Track.P.size();
	TimingLine Lines[3] = {LineAt(Track, 0), LineAt(Track, M / 3), LineAt(Track, 2 * M / 3)};
	LapTimer Timer;
	Timer.Configure(Lines, 3);
	const KartConfig C = kstest::LoadRotax();
	const LapResult R = DriveLaps(C, 1.4, 3, nullptr, &Timer);
	const auto& Laps = Timer.Laps();
	KS_LOG("bot best (arc-length based) %.3f s; timer laps %zu, best %.3f s", R.BestLap,
	       Laps.size(), Timer.BestLapIndex() >= 0 ? Laps[Timer.BestLapIndex()].LapTime : -1.0);
	for (size_t I = 0; I < Laps.size(); ++I)
	{
		KS_LOG("  lap %zu: %.3f s  sectors %.3f / %.3f / %.3f  valid %d", I + 1, Laps[I].LapTime,
		       Laps[I].Sectors[0], Laps[I].Sectors[1], Laps[I].Sectors[2], Laps[I].bValid ? 1 : 0);
		const double SumSectors = Laps[I].Sectors[0] + Laps[I].Sectors[1] + Laps[I].Sectors[2];
		KS_EXPECT_NEAR(SumSectors, Laps[I].LapTime, 1e-9);
	}
	KS_EXPECT(Laps.size() >= 2);
	if (Timer.BestLapIndex() >= 0)
	{
		// Flying laps of a repeatable driver: the timer's best must match the bot's own lap count
		// (different reference point, same lap duration) to within a few hundredths.
		KS_EXPECT_NEAR(Laps[Timer.BestLapIndex()].LapTime, R.BestLap, 0.05);
	}
}
