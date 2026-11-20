// Milestone 5: dynamic track conditions (rubber, water, weather). Coefficients are ASSUMPTIONS; these
// tests pin the model's STRUCTURE: where rubber goes, how water comes and goes, and that grip changes
// reach the tyres and the kart's behaviour.
#include "DriverBot.h"
#include "TestFramework.h"
#include "TestHelpers.h"
#include "TestTrack.h"

#include "KartSim/TrackConditions.h"

#include <cmath>

using namespace kartsim;

KS_TEST(TrackConditions_GripModelLimits)
{
	TrackConditions TC;
	BuildTestConditions(TC);
	KS_EXPECT(TC.IsBuilt());
	const Vec3 OnTrack = kstest::BuildTestTrack().P[100];
	const TrackConditionParams& P = TC.Params;
	TC.SetUniform(0.0, 0.0);
	KS_EXPECT_NEAR(TC.GripAt(OnTrack), 1.0, 1e-12);
	TC.SetUniform(1.0, 0.0);
	KS_EXPECT_NEAR(TC.GripAt(OnTrack), 1.0 + P.RubberGripGain, 1e-12);
	TC.SetUniform(0.0, 1.0);
	// Water: the road factor stays 1, the wetness reaches 1 (the tyre compound decides the grip loss).
	double Wetness = 0.0;
	KS_EXPECT_NEAR(TC.GripAt(OnTrack, nullptr, nullptr, &Wetness), 1.0, 1e-12);
	KS_EXPECT_NEAR(Wetness, 1.0, 1e-12);
	KS_EXPECT_NEAR(TC.GripCell(0, TC.CellsAcross() / 2, 0.60), 0.60, 1e-12); // slick view of a saturated cell
	TC.SetUniform(1.0, 1.0); // wet rubber is the slipperiest
	KS_EXPECT_NEAR(TC.GripAt(OnTrack), 1.0 - P.WetRubberPenalty, 1e-12);
	// Far off the track: unaffected.
	KS_EXPECT_NEAR(TC.GripAt(OnTrack + Vec3(0.0, 0.0, 0.0) + Vec3(500.0, 500.0, 0.0)), 1.0, 1e-12);
	// Grip is monotonic in water depth.
	double Prev = 2.0;
	for (double W = 0.0; W <= 1.0; W += 0.1)
	{
		TC.SetUniform(0.3, W);
		const double G = TC.GripAt(OnTrack);
		KS_EXPECT(G <= Prev + 1e-12);
		Prev = G;
	}
}

KS_TEST(TrackConditions_RainWetsAndSunDries)
{
	TrackConditions TC;
	BuildTestConditions(TC);
	TC.Weather.AirTemp = 18.0;
	TC.Weather.CloudCover = 1.0;
	TC.Weather.RainRate = 4.0; // mm/h, moderate rain
	for (int I = 0; I < 600; ++I) { TC.Advance(1.0); } // 10 min
	const double Wet = TC.MeanWater();
	KS_LOG("after 10 min of 4 mm/h rain: mean water %.3f mm, track %.1f C", Wet, TC.Weather.TrackTemp);
	KS_EXPECT(Wet > 0.3);
	KS_EXPECT(Wet <= TC.Params.MaxWaterDepth);

	// Rain stops: a hot sunny track dries faster than a cool overcast one.
	TrackConditions Hot = TC, Cool = TC;
	Hot.Weather = {30.0, 0.0, 0.0, 45.0};
	Cool.Weather = {12.0, 1.0, 0.0, 12.0};
	for (int I = 0; I < 600; ++I) { Hot.Advance(1.0); Cool.Advance(1.0); }
	KS_LOG("10 min later: hot/sunny %.3f mm (track %.1f C), cool/overcast %.3f mm (track %.1f C)",
	       Hot.MeanWater(), Hot.Weather.TrackTemp, Cool.MeanWater(), Cool.Weather.TrackTemp);
	KS_EXPECT(Hot.MeanWater() < Wet);
	KS_EXPECT(Cool.MeanWater() < Wet);
	KS_EXPECT(Hot.MeanWater() < Cool.MeanWater());
	// Sun heats the track above air; heavy cloud keeps it near air temperature.
	KS_EXPECT(Hot.Weather.TrackTemp > 30.0 + 5.0);
	KS_EXPECT(std::fabs(Cool.Weather.TrackTemp - 12.0) < 2.0);
}

KS_TEST(TrackConditions_RubberGoesDownOnTheRacingLine)
{
	TrackConditions TC;
	BuildTestConditions(TC);
	TC.Weather = {22.0, 0.0, 0.0, 35.0};
	const KartConfig C = kstest::LoadRotax();
	const LapResult R = DriveLaps(C, 1.4, 4, nullptr, nullptr, &TC);
	KS_EXPECT(R.LapsDone == 4);
	KS_EXPECT(R.Spins == 0);
	// Rubber is laid where the tyres ran: the most-rubbered lateral cell at each station. Cells at the
	// track edges (outside the 1.4 m-wide wheel paths around the line) stay clean.
	const double OnLine = TC.MeanRubberOnLine();
	double Edge = 0.0;
	const int Across = TC.CellsAcross();
	for (int A = 0; A < TC.CellsAlong(); ++A) { Edge += TC.RubberCell(A, 0) + TC.RubberCell(A, Across - 1); }
	Edge /= 2.0 * TC.CellsAlong();
	KS_LOG("after 4 laps: rubber on line %.4f (grip x%.4f), at track edges %.6f", OnLine, 1.0 + TC.Params.RubberGripGain * OnLine, Edge);
	KS_EXPECT(OnLine > 0.0);
	KS_EXPECT(Edge < 0.01 * OnLine);
}

KS_TEST(TrackConditions_DryLineAppearsOnWetTrack)
{
	TrackConditions TC;
	BuildTestConditions(TC);
	TC.Weather = {18.0, 1.0, 0.0, 18.0}; // rain has stopped, overcast
	TC.SetUniform(0.0, 0.6);
	const KartConfig C = kstest::LoadRotax();
	const LapResult R = DriveLaps(C, 0.8, 4, nullptr, nullptr, &TC); // cautious wet pace
	KS_EXPECT(R.LapsDone == 4);
	// Water on the line (least-wet lateral cell per station) vs the track edges.
	double Line = 0.0, Edge = 0.0;
	const int Across = TC.CellsAcross();
	for (int A = 0; A < TC.CellsAlong(); ++A)
	{
		double Mn = 1e9;
		for (int X = 0; X < Across; ++X) { Mn = TC.WaterCell(A, X) < Mn ? TC.WaterCell(A, X) : Mn; }
		Line += Mn;
		Edge += 0.5 * (TC.WaterCell(A, 0) + TC.WaterCell(A, Across - 1));
	}
	Line /= TC.CellsAlong();
	Edge /= TC.CellsAlong();
	KS_LOG("after 4 wet laps: water on line %.3f mm, at edges %.3f mm", Line, Edge);
	KS_EXPECT(Line < 0.8 * Edge);
}

KS_TEST(TrackConditions_WetGripReachesTheKart)
{
	// Same driver target on a saturated track: the kart can't hold the dry pace (spins or runs wide),
	// and a wet-appropriate target is clean but slower. This is the end-to-end check that the grip
	// multiplier reaches the tyre forces.
	const KartConfig C = kstest::LoadRotax();
	TrackConditions Wet;
	BuildTestConditions(Wet);
	Wet.Weather = {16.0, 1.0, 0.0, 16.0};
	const auto Soak = [&](TrackConditions& T) { T.SetUniform(0.0, 1.5); T.Params.SqueegeePerPass = 0.0; T.Params.DrainPerHour = 0.0; T.Params.EvaporationPerHour = 0.0; };
	Soak(Wet);
	const LapResult Dry = DriveLaps(C, 1.4, 2, nullptr);
	const LapResult WetFast = DriveLaps(C, 1.4, 2, nullptr, nullptr, &Wet);
	TrackConditions Wet2;
	BuildTestConditions(Wet2);
	Wet2.Weather = Wet.Weather;
	Soak(Wet2);
	const LapResult WetSlow = DriveLaps(C, 0.8, 2, nullptr, nullptr, &Wet2);
	KS_LOG("dry @1.4g: %.3f s spins %d off %.1f m | wet @1.4g: laps %d spins %d off %.1f m | wet @0.8g: %.3f s spins %d off %.1f m",
	       Dry.BestLap, Dry.Spins, Dry.MaxCrossTrack, WetFast.LapsDone, WetFast.Spins, WetFast.MaxCrossTrack, WetSlow.BestLap, WetSlow.Spins, WetSlow.MaxCrossTrack);
	KS_EXPECT(Dry.Spins == 0);
	KS_EXPECT(WetFast.Spins > 0 || WetFast.MaxCrossTrack > Dry.MaxCrossTrack + 1.0);
	KS_EXPECT(WetSlow.Spins == 0);
	KS_EXPECT(WetSlow.LapsDone == 2);
	KS_EXPECT(WetSlow.BestLap > Dry.BestLap);
}

KS_TEST(TrackConditions_ShowerComesAndGoes)
{
	// Weather timeline: dry, a shower, then sun. The track gets wet during the shower, the AI tyre choice
	// flips to wets, and the water goes down once it stops.
	TrackConditions TC;
	BuildTestConditions(TC);
	TC.Weather = {17.0, 0.6, 0.0, 25.0};
	TC.SetSchedule({{0.0, 0.0, 0.6}, {360.0, 0.0, 1.0}, {420.0, 8.0, 1.0}, {900.0, 8.0, 1.0}, {960.0, 0.0, 0.8}, {1500.0, 0.0, 0.3}});
	const KartConfig C = kstest::LoadRotax();
	double WetAt[4] = {};
	const double Times[4] = {300.0, 900.0, 1500.0, 4000.0};
	int Set[4] = {};
	double T = 0.0;
	for (int I = 0; I < 4; ++I)
	{
		while (T < Times[I]) { TC.Advance(1.0); T += 1.0; }
		WetAt[I] = TC.LineWetness();
		Set[I] = ChooseTyreCompound(C, WetAt[I]);
		KS_LOG("t=%4.0f s: rain %.1f mm/h, cloud %.2f, mean water %.2f mm, line wetness %.2f -> %s", T, TC.Weather.RainRate, TC.Weather.CloudCover,
		       TC.MeanWater(), WetAt[I], C.CompoundName[Set[I]].c_str());
	}
	// Without traffic the line dries only by evaporation/drainage (slow: tyres squeegee the line in practice,
	// see TrackConditions_DryLineAppearsOnWetTrack), so only the direction is pinned here.
	KS_EXPECT(Set[0] == 0 && Set[1] == 1);
	KS_EXPECT(WetAt[1] > 0.9 && WetAt[3] < WetAt[2]);
}
