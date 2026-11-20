// Milestone 5: session rules (practice / qualifying chequered flag / race distance) and the live lap
// delta. A synthetic point moves along the test-circuit centre line at a prescribed speed, so every
// expected time is known analytically.
#include "TestFramework.h"
#include "TestHelpers.h"
#include "TestTrack.h"

#include "KartSim/LapDelta.h"
#include "KartSim/LapTiming.h"
#include "KartSim/RaceDirector.h"
#include "KartSim/TrackPath.h"

#include <cmath>
#include <functional>

using namespace kartsim;
using kstest::kTick;

namespace
{
	struct Runner
	{
		kstest::TrackLine T = kstest::BuildTestTrack();
		TrackPath Path;
		LapTimer Timer;
		double S = 0.0;    // arc length travelled (unwrapped)
		double Time = 0.0;

		Runner()
		{
			Path.Build(T.P);
			const size_t M = T.P.size();
			const Vec3 Tan = (T.P[1] - T.P[M - 1]).Normalized();
			const Vec3 N(-Tan.Y, Tan.X, 0.0);
			TimingLine L{T.P[0] + N * 6.0, T.P[0] - N * 6.0, Tan};
			Timer.Configure(&L, 1);
			S = T.S[6]; // the game's grid slot: a few metres past the line
		}
		Vec3 At(double Arc) const
		{
			double A = std::fmod(Arc, T.Length);
			size_t I = 0;
			while (I + 1 < T.S.size() && T.S[I + 1] <= A) { ++I; }
			const size_t J = (I + 1) % T.P.size();
			const double Seg = (J == 0 ? T.Length : T.S[J]) - T.S[I];
			const double F = Seg > 0.0 ? (A - T.S[I]) / Seg : 0.0;
			return T.P[I] + (T.P[J] - T.P[I]) * F;
		}
		// Advances Duration seconds at Speed(m/s, may depend on lap distance); calls Each per step.
		void Run(double Duration, const std::function<double(double)>& Speed, const std::function<void()>& Each)
		{
			for (int K = 0; K < static_cast<int>(std::lround(Duration / kTick)); ++K)
			{
				const Vec3 Prev = At(S);
				S += Speed(std::fmod(S, T.Length)) * kTick;
				Time += kTick;
				Timer.Update(Time, kTick, Prev, At(S), false);
				Each();
			}
		}
	};
}

namespace
{
	std::vector<CompetitorInput> Inputs(std::vector<Runner*> Rs)
	{
		std::vector<CompetitorInput> In;
		for (Runner* R : Rs)
		{
			size_t H = 0;
			double S = 0.0, Lat = 0.0;
			R->Path.Project(R->At(R->S), S, Lat, H);
			In.push_back({&R->Timer, S});
		}
		return In;
	}
}

KS_TEST(Race_QualifyingLapInProgressCountsAfterFlag)
{
	Runner R;
	const double V = 20.0;
	const double LapT = R.T.Length / V;
	RaceDirector Q;
	SessionConfig C;
	C.Type = ESessionType::Qualifying;
	C.Duration = 2.5 * LapT; // flag falls mid-lap 3 (counting from the out-lap start)
	Q.Start(C, R.Time, R.Path.Length(), Inputs({&R}));
	bool bSawChequered = false;
	R.Run(6.0 * LapT, [&](double) { return V; }, [&] { Q.Update(R.Time, Inputs({&R})); bSawChequered |= Q.Phase() == ESessionPhase::Chequered; });
	KS_LOG("lap %.3f s, %s, phase %d", LapT, Q.Summary(0).c_str(), static_cast<int>(Q.Phase()));
	KS_EXPECT(bSawChequered);
	KS_EXPECT(Q.Phase() == ESessionPhase::Finished);
	// Out-lap (grid -> line) is not timed; timed laps: the one finished before the flag plus the one
	// in progress at the flag.
	KS_EXPECT(Q.Results()[0].Laps == 2);
	KS_EXPECT_NEAR(Q.Results()[0].BestLap, LapT, 1e-3);
}

KS_TEST(Race_StandingStartDistanceAndTotalTime)
{
	Runner R;
	R.Timer.Reset();
	const double V = 18.0;
	RaceDirector Race;
	SessionConfig C;
	C.Type = ESessionType::Race;
	C.RaceLaps = 3;
	C.StartLightsTime = 2.0;
	Race.Start(C, R.Time, R.Path.Length(), Inputs({&R}));
	R.Run(2.0, [&](double) { return 0.0; }, [&] { Race.Update(R.Time, Inputs({&R})); }); // held for the lights
	const double FromGrid = R.T.Length - R.T.S[6];
	R.Run(5.0 * R.T.Length / V, [&](double) { return V; }, [&] { Race.Update(R.Time, Inputs({&R})); });
	const double Expect = (FromGrid + 2.0 * R.T.Length) / V;
	KS_LOG("race: %s (expected total %.3f s)", Race.Summary(0).c_str(), Expect);
	KS_EXPECT(Race.Phase() == ESessionPhase::Finished);
	KS_EXPECT(Race.Results()[0].Laps == 3);
	KS_EXPECT_NEAR(Race.Results()[0].FinishTime, Expect, 0.01);
	KS_EXPECT_NEAR(Race.Results()[0].BestLap, R.T.Length / V, 1e-3);
}

KS_TEST(Race_RollingStartClassifiesFieldFromCommonGreen)
{
	// Three karts on the formation lap in a line 8 m apart; at the line they race at different speeds.
	// Race times run from the leader's green, so the gaps include the formation spacing.
	Runner A, B, Cc;
	A.S += 16.0; // the whole grid is past the line (rule): A 20.5 m, B 12.5 m, C 4.5 m after it
	B.S += 8.0;
	const double Va = 20.0, Vb = 19.0, Vc = 14.0;
	RaceDirector Race;
	SessionConfig C;
	C.Type = ESessionType::Race;
	C.RaceLaps = 3;
	C.bRollingStart = true;
	std::vector<Runner*> All = {&A, &B, &Cc};
	Race.Start(C, 0.0, A.Path.Length(), Inputs(All));
	Race.Update(0.0, Inputs(All));
	KS_EXPECT(Race.Phase() == ESessionPhase::Grid);
	KS_EXPECT(Race.Order()[0] == 0 && Race.Order()[2] == 2); // grid order before green
	const double Formation = 12.0;
	const double Dt = kTick;
	for (int K = 0; K < static_cast<int>(200.0 / Dt) && Race.Phase() != ESessionPhase::Finished; ++K)
	{
		const bool bGreen = Race.IsGreen();
		A.Run(Dt, [&](double) { return bGreen ? Va : Formation; }, [] {});
		B.Run(Dt, [&](double) { return bGreen ? Vb : Formation; }, [] {});
		Cc.Run(Dt, [&](double) { return bGreen ? Vc : Formation; }, [] {});
		Race.Update(A.Time, Inputs(All));
	}
	const auto& Rs = Race.Results();
	KS_LOG("rolling: P%d %.3f s (%d laps) | P%d %.3f s gap %.3f | P%d %d laps, %.3f s, laps down %d",
	       Rs[0].Position, Rs[0].FinishTime, Rs[0].Laps, Rs[1].Position, Rs[1].FinishTime, Rs[1].Gap, Rs[2].Position, Rs[2].Laps, Rs[2].FinishTime, Rs[2].LapsDown);
	KS_EXPECT(Race.Phase() == ESessionPhase::Finished);
	KS_EXPECT(Race.Order()[0] == 0 && Race.Order()[1] == 1 && Race.Order()[2] == 2);
	const double L = A.Path.Length();
	KS_EXPECT_NEAR(Rs[0].FinishTime, 3.0 * L / Va, 0.02);
	// At green B is 8 m short of the line and everyone accelerates to race pace: its time from the
	// common green includes those 8 m, then 3 laps at Vb.
	KS_EXPECT_NEAR(Rs[1].FinishTime, (8.0 + 3.0 * L) / Vb, 0.02);
	KS_EXPECT_NEAR(Rs[1].Gap, Rs[1].FinishTime - Rs[0].FinishTime, 1e-9);
	// C (14 m/s) has covered ~2.1 racing laps when the leader finishes: it is not lapped, so it is
	// classified last when it next crosses the line, with 3 laps.
	KS_EXPECT(Rs[2].bFinished && Rs[2].Laps == 3 && Rs[2].LapsDown == 0);
}

KS_TEST(Race_LappedKartClassifiedWithFewerLaps)
{
	Runner A, B;
	B.S -= 3.0;
	RaceDirector Race;
	SessionConfig C;
	C.Type = ESessionType::Race;
	C.RaceLaps = 4;
	C.StartLightsTime = 0.0;
	std::vector<Runner*> All = {&A, &B};
	Race.Start(C, 0.0, A.Path.Length(), Inputs(All));
	for (int K = 0; K < static_cast<int>(300.0 / kTick) && Race.Phase() != ESessionPhase::Finished; ++K)
	{
		A.Run(kTick, [](double) { return 20.0; }, [] {});
		B.Run(kTick, [](double) { return 9.0; }, [] {});
		Race.Update(A.Time, Inputs(All));
	}
	const auto& Rs = Race.Results();
	KS_LOG("lapped: winner %d laps %.3f s, P2 %d laps (%d down) %.3f s", Rs[0].Laps, Rs[0].FinishTime, Rs[1].Laps, Rs[1].LapsDown, Rs[1].FinishTime);
	KS_EXPECT(Race.Phase() == ESessionPhase::Finished);
	KS_EXPECT(Rs[0].Position == 1 && Rs[1].Position == 2);
	KS_EXPECT(Rs[1].bFinished && Rs[1].Laps < Rs[0].Laps);
	KS_EXPECT(Rs[1].LapsDown == Rs[0].Laps - Rs[1].Laps);
}

KS_TEST(LapDelta_TracksFasterAndSlowerSections)
{
	Runner R;
	LapDelta D;
	D.Configure(&R.Path);
	size_t Seen = 0;
	double MinDelta = 1e9, EndDelta = 0.0;
	bool bLap3 = false;
	// Lap 1 (first timed) at 20 m/s; lap 2: 5 % faster on the first half only.
	auto Speed = [&](double Arc) { return (R.Timer.Laps().size() == 1 && Arc < 0.5 * R.T.Length) ? 21.0 : 20.0; };
	R.Run(4.2 * R.T.Length / 20.0, Speed, [&]
	{
		if (R.Timer.Laps().size() > Seen)
		{
			const LapRecord& L = R.Timer.Laps().back();
			D.OnLapCompleted(L.LapTime, L.bValid);
			Seen = R.Timer.Laps().size();
			bLap3 = Seen == 2;
		}
		if (R.Timer.IsTiming())
		{
			D.Sample(R.Timer.CurrentLapTime(R.Time), R.At(R.S), 72.0);
			if (Seen == 1 && D.HasDelta()) { MinDelta = D.Delta() < MinDelta ? D.Delta() : MinDelta; EndDelta = D.Delta(); }
		}
	});
	const auto& Laps = R.Timer.Laps();
	KS_EXPECT(Laps.size() >= 2);
	if (Laps.size() < 2) { return; }
	const double Gain = Laps[1].LapTime - Laps[0].LapTime;
	KS_LOG("lap1 %.3f lap2 %.3f gain %.3f | delta during lap 2: min %.3f end %.3f | reference %.3f", Laps[0].LapTime, Laps[1].LapTime, Gain, MinDelta, EndDelta, D.ReferenceLap().LapTime);
	// The gain comes from the first half: the delta reaches it at half distance and holds it.
	KS_EXPECT(Gain < -0.1);
	KS_EXPECT_NEAR(EndDelta, Gain, 0.12);  // within one 2 m sample at 20 m/s
	KS_EXPECT(MinDelta < Gain + 0.12);
	KS_EXPECT_NEAR(D.ReferenceLap().LapTime, Laps[1].LapTime, 1e-9); // the faster lap became the reference
	(void)bLap3;
}

KS_TEST(TrackPath_ProjectionRoundTrip)
{
	const kstest::TrackLine T = kstest::BuildTestTrack();
	TrackPath P;
	P.Build(T.P);
	// T.Length is measured on the dense spline; the path is the 0.75 m polyline (chords are ~10 mm shorter).
	KS_EXPECT_NEAR(P.Length(), T.S.back() + (T.P.front() - T.P.back()).Length(), 1e-6);
	KS_EXPECT_NEAR(P.Length(), T.Length, 0.05);
	size_t Hint = 0;
	double MaxErr = 0.0;
	for (size_t I = 0; I < T.P.size(); I += 37)
	{
		const size_t J = (I + 1) % T.P.size();
		const Vec3 Tan = (T.P[J] - T.P[I]).Normalized();
		const Vec3 Left(-Tan.Y, Tan.X, 0.0);
		double S = 0.0, Lat = 0.0;
		P.Project(T.P[I] + Left * 2.5, S, Lat, Hint);
		MaxErr = std::fmax(MaxErr, std::fabs(Lat - 2.5));
		const double Ds = std::fabs(S - T.S[I]);
		MaxErr = std::fmax(MaxErr, std::fmin(Ds, T.Length - Ds));
	}
	KS_LOG("max projection error %.4f m", MaxErr);
	KS_EXPECT(MaxErr < 0.1);
}
