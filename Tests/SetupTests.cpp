// Setup system: persistence, legality clamping, and that each adjustment changes the physics in the
// direction kart engineering predicts.
#include "TestFramework.h"
#include "TestHelpers.h"

#include "KartSim/Setup.h"

using namespace kartsim;
using kstest::kTick;

namespace
{
	int ParamIndex(const char* Key)
	{
		int N = 0;
		const SetupParam* P = GetSetupParams(N);
		for (int I = 0; I < N; ++I) { if (std::string(P[I].Key) == Key) { return I; } }
		return -1;
	}

	KartConfig WithSetup(const char* Key, double Value)
	{
		KartConfig C = kstest::LoadRotax();
		SetSetupValue(C, ParamIndex(Key), Value);
		return C;
	}
}

KS_TEST(Setup_RoundTripAndLegalClamp)
{
	KartConfig A = kstest::LoadRotax();
	SetSetupValue(A, ParamIndex("axleSprocket"), 84);
	SetSetupValue(A, ParamIndex("caster"), 16.5);
	SetSetupValue(A, ParamIndex("rearTrack"), 1.35);   // illegal: wider than the CIK limit allows
	SetSetupValue(A, ParamIndex("toe"), -0.6);          // snaps to 0.25 deg steps
	const std::string Json = SerializeSetup(A);
	KartConfig B = kstest::LoadRotax();
	std::string Err;
	KS_EXPECT(ApplySetupJson(Json, B, Err));
	KS_EXPECT(B.FinalDrive.AxleSprocket == 84);
	KS_EXPECT_NEAR(B.Steering.Caster * kRadToDeg, 16.5, 1e-9);
	KS_EXPECT_NEAR(B.Chassis.RearTrack, 1.22, 1e-9);    // clamped: 1.22 + 0.18 m tyre = 1.40 m max overall width
	KS_EXPECT_NEAR(B.Steering.StaticToe * kRadToDeg, -0.5, 1e-9);
	KartConfig K = kstest::LoadKart("KZ2_IAMEScreamer.json");
	KS_EXPECT(!ApplySetupJson(Json, K, Err));           // a Rotax setup must not load onto a KZ2
}

KS_TEST(Setup_ShorterGearingTradesTopSpeedForAcceleration)
{
	auto Launch = [](const KartConfig& C, double& T60, double& VTop)
	{
		KartSimulation Sim;
		Sim.Initialize(C);
		PlaneTrack Ground;
		kstest::Run(Sim, Ground, KartInputs(), 0.5);
		KartInputs In;
		In.Throttle = 1.0;
		T60 = -1.0;
		VTop = 0.0;
		for (int I = 0; I < static_cast<int>(30.0 / kTick); ++I)
		{
			Sim.Step(kTick, In, Ground);
			const double V = Sim.GetTelemetry().SpeedKmh;
			if (T60 < 0.0 && V >= 60.0) { T60 = (I + 1) * kTick; }
			VTop = V > VTop ? V : VTop;
		}
	};
	double T60Long, VLong, T60Short, VShort;
	Launch(WithSetup("axleSprocket", 76), T60Long, VLong);
	Launch(WithSetup("axleSprocket", 88), T60Short, VShort);
	KS_LOG("12/76: 0-60 %.2f s, top %.1f km/h | 12/88: 0-60 %.2f s, top %.1f km/h", T60Long, VLong, T60Short, VShort);
	KS_EXPECT(T60Short < T60Long);
	KS_EXPECT(VShort < VLong);
}

KS_TEST(Setup_MoreCasterUnloadsInsideRearMore)
{
	auto InsideRearDrop = [](const KartConfig& C)
	{
		KartSimulation Sim;
		Sim.Initialize(C);
		PlaneTrack Ground;
		KartInputs In;
		In.Brake = 1.0;
		kstest::Run(Sim, Ground, In, 1.5);
		const double Before = Sim.GetTelemetry().FzRL;
		In.SteeringColumnAngle = 90.0 * kDegToRad;
		kstest::Run(Sim, Ground, In, 1.5);
		return Before - Sim.GetTelemetry().FzRL;
	};
	const double D8 = InsideRearDrop(WithSetup("caster", 8)), D20 = InsideRearDrop(WithSetup("caster", 20));
	KS_LOG("inside-rear load drop at 90 deg column: caster 8 deg %.0f N, caster 20 deg %.0f N", D8, D20);
	KS_EXPECT(D20 > D8 + 20.0);
}

KS_TEST(Setup_NarrowerRearTrackTransfersMoreLoad)
{
	auto RearTransferPerG = [](const KartConfig& C)
	{
		KartSimulation Sim;
		Sim.Initialize(C);
		PlaneTrack Ground;
		kstest::Run(Sim, Ground, KartInputs(), 0.5);
		kstest::SetRolling(Sim, 12.0);
		kstest::SpeedController Ctl;
		KartInputs In;
		In.SteeringColumnAngle = 20.0 * kDegToRad;
		for (int I = 0; I < static_cast<int>(6.0 / kTick); ++I)
		{
			In.Throttle = Ctl.Throttle(12.0, Sim.ForwardSpeed(), kTick);
			Sim.Step(kTick, In, Ground);
		}
		const TelemetryFrame& F = Sim.GetTelemetry();
		return (F.FzRR - F.FzRL) / F.AccLatG; // left turn: right = outside
	};
	const double Narrow = RearTransferPerG(WithSetup("rearTrack", 1.10)), Wide = RearTransferPerG(WithSetup("rearTrack", 1.22));
	KS_LOG("rear outside-inside load difference per g: track 1.10 m %.0f N/g, 1.22 m %.0f N/g", Narrow, Wide);
	KS_EXPECT(Narrow > Wide);
}

KS_TEST(Setup_QuickerSteeringRatioTurnsMorePerColumnDegree)
{
	auto YawRate = [](const KartConfig& C)
	{
		KartSimulation Sim;
		Sim.Initialize(C);
		PlaneTrack Ground;
		kstest::Run(Sim, Ground, KartInputs(), 0.5);
		kstest::SetRolling(Sim, 10.0);
		kstest::SpeedController Ctl;
		KartInputs In;
		In.SteeringColumnAngle = 15.0 * kDegToRad;
		for (int I = 0; I < static_cast<int>(4.0 / kTick); ++I)
		{
			In.Throttle = Ctl.Throttle(10.0, Sim.ForwardSpeed(), kTick);
			Sim.Step(kTick, In, Ground);
		}
		return Sim.GetTelemetry().YawRateDeg;
	};
	const double Quick = YawRate(WithSetup("steeringRatio", 3.0)), Slow = YawRate(WithSetup("steeringRatio", 4.2));
	KS_LOG("yaw rate at 15 deg column, 36 km/h: ratio 3.0 -> %.1f deg/s, ratio 4.2 -> %.1f deg/s", Quick, Slow);
	KS_EXPECT(Quick > Slow * 1.2);
}

KS_TEST(Setup_BallastSeatAndStiffnessRoundTripExactly)
{
	const KartConfig Base = kstest::LoadRotax();
	KartConfig A = Base;
	SetSetupValue(A, ParamIndex("ballast"), 10.0);
	SetSetupValue(A, ParamIndex("seatHeight"), 0.29);
	SetSetupValue(A, ParamIndex("frontWeight"), 43.5);
	SetSetupValue(A, ParamIndex("chassisStiffness"), 115);
	// Ballast physics: +10 kg, mounted low, so the CoG drops relative to the same seat height without it.
	KS_EXPECT_NEAR(A.Chassis.Mass, Base.Chassis.Mass + 10.0, 1e-9);
	double NodeMass = 0.0;
	for (int I = 0; I < A.Chassis.NumFrameNodes; ++I) { NodeMass += A.Chassis.FrameNodeMass[I]; }
	KS_EXPECT_NEAR(NodeMass, A.Chassis.Mass, 1e-9);                       // node masses still sum to the total
	KS_EXPECT_NEAR(A.Chassis.FrameSegmentStiffness[0], Base.Chassis.FrameSegmentStiffness[0] * 1.15, 1e-6);
	KartConfig Low = Base;
	SetSetupValue(Low, ParamIndex("ballast"), 10.0);
	KS_EXPECT(Low.Chassis.CogHeight < Base.Chassis.CogHeight);
	// Exact reload (table-order application) onto a fresh class config.
	KartConfig B = Base;
	std::string Err;
	KS_EXPECT(ApplySetupJson(SerializeSetup(A), B, Err));
	KS_EXPECT_NEAR(B.Chassis.Mass, A.Chassis.Mass, 1e-9);
	KS_EXPECT_NEAR(B.Chassis.CogHeight, A.Chassis.CogHeight, 1e-9);
	KS_EXPECT_NEAR(B.Chassis.FrontWeightFraction, 0.435, 1e-9);
	KS_EXPECT_NEAR(B.Chassis.FrameSegmentStiffness[0], A.Chassis.FrameSegmentStiffness[0], 1e-6);
	// Mass-weighted node height still equals the CoG height (the frame model relies on it).
	double SumMZ = 0.0;
	for (int I = 0; I < B.Chassis.NumFrameNodes; ++I) { SumMZ += B.Chassis.FrameNodeMass[I] * B.Chassis.FrameNodeHeight[I]; }
	KS_EXPECT_NEAR(SumMZ / B.Chassis.Mass, B.Chassis.CogHeight, 1e-9);
}

KS_TEST(Setup_FrontBrakeShareOnlyForFrontBrakedClasses)
{
	int N = 0;
	const SetupParam* P = GetSetupParams(N);
	const int I = ParamIndex("frontBrakeShare");
	KartConfig R = kstest::LoadRotax();
	KS_EXPECT(!P[I].IsAvailable(R));
	SetSetupValue(R, I, 40.0);                       // ignored: a Rotax has no front brakes
	KS_EXPECT(R.Brakes.FrontMaxTorque == 0.0);
	KartConfig K = kstest::LoadKart("KZ2_IAMEScreamer.json");
	KS_EXPECT(P[I].IsAvailable(K));
	const double Total = 2.0 * K.Brakes.FrontMaxTorque + K.Brakes.RearMaxTorque;
	SetSetupValue(K, I, 45.0);
	KS_EXPECT_NEAR(2.0 * K.Brakes.FrontMaxTorque + K.Brakes.RearMaxTorque, Total, 1e-6);
	KS_EXPECT_NEAR(P[I].Get(K), 45.0, 1e-9);
}

KS_TEST(Setup_HigherSeatTransfersMoreLoad)
{
	auto RearTransferPerG = [](const KartConfig& C)
	{
		KartSimulation Sim;
		Sim.Initialize(C);
		PlaneTrack Ground;
		kstest::Run(Sim, Ground, KartInputs(), 0.5);
		kstest::SetRolling(Sim, 12.0);
		kstest::SpeedController Ctl;
		KartInputs In;
		In.SteeringColumnAngle = 20.0 * kDegToRad;
		for (int K = 0; K < static_cast<int>(6.0 / kTick); ++K)
		{
			In.Throttle = Ctl.Throttle(12.0, Sim.ForwardSpeed(), kTick);
			Sim.Step(kTick, In, Ground);
		}
		const TelemetryFrame& F = Sim.GetTelemetry();
		return (F.FzRR - F.FzRL) / F.AccLatG;
	};
	const double Low = RearTransferPerG(WithSetup("seatHeight", 0.24)), High = RearTransferPerG(WithSetup("seatHeight", 0.30));
	KS_LOG("rear load difference per g: seat 0.24 m %.0f N/g, 0.30 m %.0f N/g", Low, High);
	KS_EXPECT(High > Low);
}
