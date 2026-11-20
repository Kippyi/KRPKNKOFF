// Milestone 9: wheel mounts (rear axle / front stub as elastic cantilevers carrying the wheel assemblies) and vertical
// frame bending. Checks against beam theory, the static rest state, and the response to a bump and a corner.
#include "TestFramework.h"
#include "TestHelpers.h"

using namespace kartsim;
using kstest::kTick;

namespace
{
	// Flat ground with a short half-sine bump across the track (x0 .. x0 + Len, height H).
	struct BumpTrack final : ITrackQuery
	{
		double X0 = 5.0, Len = 0.30, H = 0.02;
		bool QueryGround(const Vec3& O, double, double, GroundHit& Out) const override
		{
			double Z = 0.0;
			if (O.X > X0 && O.X < X0 + Len) { Z = H * std::sin(kPi * (O.X - X0) / Len); }
			Out.Point = Vec3(O.X, O.Y, Z);
			Out.Normal = Vec3(0, 0, 1);
			Out.Surface = ESurfaceType::Asphalt;
			return true;
		}
	};

	bool HasFlex(const KartConfig& C) { return C.Chassis.RearMount.UnsprungMass > 0.0 && C.Chassis.FrameBendingEI > 0.0; }
}

KS_TEST(Flex_MountComplianceMatchesBeamTheory)
{
	// Plain cantilever (no hub lever): tip compliances L^3/3EI, L^2/2EI, L/EI.
	WheelMountParams P;
	P.UnsprungMass = 1.0;
	P.TubeOuterDiameter = 0.05;
	P.TubeWall = 0.002;
	P.FreeSpan = 0.2;
	const double I = kPi / 64.0 * (std::pow(0.05, 4) - std::pow(0.046, 4));
	const double EI = 2.1e11 * I;
	double Cff, Cfm, Cmm;
	KS_EXPECT(WheelMountCompliance(P, 0.6, Cff, Cfm, Cmm));
	KS_EXPECT(std::fabs(Cff - 0.008 / (3.0 * EI)) < 1e-9 * Cff + 1e-15);
	KS_EXPECT(std::fabs(Cfm - 0.04 / (2.0 * EI)) < 1e-6 * Cfm);
	KS_EXPECT(std::fabs(Cmm - 0.2 / EI) < 1e-6 * Cmm);
	// Wider rear track = hubs further out on the axle = longer free span = softer.
	const KartConfig C = kstest::LoadRotax();
	if (C.Chassis.RearMount.UnsprungMass > 0.0)
	{
		double A0, B0, D0, A1, B1, D1;
		WheelMountCompliance(C.Chassis.RearMount, 0.5 * 1.16, A0, B0, D0);
		WheelMountCompliance(C.Chassis.RearMount, 0.5 * 1.22, A1, B1, D1);
		KS_LOG("rear axle 50x%.1f mm: vertical stiffness %.2f MN/m (track 1.16) / %.2f MN/m (track 1.22), camber %.3f deg per kN vertical, %.3f deg per 100 Nm",
		       C.Chassis.RearMount.TubeWall * 1000.0, 1e-6 / A0, 1e-6 / A1, B1 * 1000.0 * kRadToDeg, D1 * 100.0 * kRadToDeg);
		KS_EXPECT(A1 > A0 && B1 > B0);
	}
}

KS_TEST(Flex_RestStateIsTheStaticDesignState)
{
	// The static geometry (wheel centres, CoG height, corner loads) is defined with the kart loaded; the mounts and the
	// frame start in their static deflection, so a kart set down at rest must not move.
	for (const char* File : {"Rotax125MaxSenior.json", "KZ2_IAMEScreamer.json", "KT100_DirectDrive.json"})
	{
		const KartConfig C = kstest::LoadKart(File);
		KartSimulation Sim;
		Sim.Initialize(C);
		PlaneTrack G;
		Sim.Reset(Vec3(0, 0, 0), 0.0);
		KartInputs In;
		double MaxDz = 0.0, MaxBend = 0.0;
		for (int I = 0; I < 240 * 2; ++I)
		{
			Sim.Step(kTick, In, G);
			for (int W = 0; W < NumWheels; ++W) { MaxDz = std::fmax(MaxDz, std::fabs(Sim.GetWheel(W).MountZ)); }
			for (int K = 0; K < Sim.GetChassisModel().NumNodes(); ++K) { MaxBend = std::fmax(MaxBend, std::fabs(Sim.GetChassisModel().NodeBendOffset(K))); }
		}
		const TelemetryFrame& F = Sim.GetTelemetry();
		const double Zcog = Sim.GetChassis().Position.Z - C.Chassis.CogHeight;
		KS_LOG("%s at rest: CoG %+.2f mm from design, mount drift max %.3f mm, bend drift max %.3f mm, static rear axle deflection %.2f mm / camber %.3f deg, frame sag %.2f mm, Fz %.0f/%.0f/%.0f/%.0f N",
		       File, Zcog * 1000.0, MaxDz * 1000.0, MaxBend * 1000.0, Sim.GetMountStaticZ(WheelRL) * 1000.0, Sim.GetMountStaticPsi(WheelRL) * kRadToDeg, F.FrameBendMm,
		       F.FzFL, F.FzFR, F.FzRL, F.FzRR);
		KS_EXPECT(std::fabs(Zcog) < 0.0005);
		KS_EXPECT(MaxDz < 2e-5 && MaxBend < 1e-4);
		const double Front = 0.5 * C.Chassis.Mass * kGravity * C.Chassis.FrontWeightFraction;
		KS_EXPECT(std::fabs(F.FzFL - Front) < 0.01 * Front && std::fabs(F.FzFR - Front) < 0.01 * Front);
	}
}

KS_TEST(Flex_BumpExcitesTheFrameAndSettles)
{
	// A 20 mm bump at 15 m/s: the frame (driver mass on the bending rails) and the wheel assemblies ring and settle;
	// no energy pumping, no runaway. Reports the frame's sag range and dominant ringing period.
	const KartConfig C = kstest::LoadRotax();
	if (!HasFlex(C)) { KS_LOG("no flex configured"); return; }
	KartSimulation Sim;
	Sim.Initialize(C);
	BumpTrack G;
	Sim.Reset(Vec3(0, 0, 0), 0.0);
	kstest::SetRolling(Sim, 15.0);
	KartInputs In;
	In.Throttle = 0.35;
	double MinSag = 1e9, MaxSag = -1e9, MaxDzR = 0.0, MaxFzR = 0.0;
	int Crossings = 0;
	double FirstCross = -1.0, LastCross = -1.0, PrevDev = 0.0;
	const double Static = [&] { KartSimulation S0; S0.Initialize(C); PlaneTrack P; S0.Reset(Vec3(), 0.0); S0.Step(kTick, KartInputs(), P); return S0.GetTelemetry().FrameBendMm; }();
	for (int I = 0; I < 240 * 2; ++I)
	{
		Sim.Step(kTick, In, G);
		const TelemetryFrame& F = Sim.GetTelemetry();
		MinSag = std::fmin(MinSag, F.FrameBendMm);
		MaxSag = std::fmax(MaxSag, F.FrameBendMm);
		MaxDzR = std::fmax(MaxDzR, std::fabs(F.MountDzRLmm));
		MaxFzR = std::fmax(MaxFzR, F.FzRL);
		const double Dev = F.FrameBendMm - Static;
		if (Sim.GetChassis().Position.X > G.X0 + 1.2 && I > 0 && (Dev > 0.0) != (PrevDev > 0.0))
		{
			++Crossings;
			if (FirstCross < 0.0) { FirstCross = F.Time; }
			LastCross = F.Time;
		}
		PrevDev = Dev;
	}
	const double Period = Crossings > 2 ? 2.0 * (LastCross - FirstCross) / (Crossings - 1) : 0.0;
	const double EndDev = Sim.GetTelemetry().FrameBendMm - Static;
	KS_LOG("20 mm bump at 15 m/s: frame sag %.2f .. %.2f mm (static %.2f), ringing ~%.1f Hz (%d zero crossings), rear mount deflection max %.2f mm, rear Fz max %.0f N, after 2 s %+.3f mm",
	       MinSag, MaxSag, Static, Period > 0 ? 1.0 / Period : 0.0, Crossings, MaxDzR, MaxFzR, EndDev);
	KS_EXPECT(std::isfinite(MaxSag) && MaxSag - MinSag > 0.1);   // the bump visibly flexes the frame
	KS_EXPECT(MaxSag - MinSag < 20.0);                           // and it stays a flex, not a collapse
	KS_EXPECT(std::fabs(EndDev) < 0.2);                          // settled
}

KS_TEST(Flex_CorneringLoadsTheOutsideRearMount)
{
	// Steady circle at ~1.3 g: the outside rear axle end carries the load and bends up (deflection and camber follow
	// beam theory for that load), the inside one unloads.
	const KartConfig C = kstest::LoadRotax();
	if (!HasFlex(C)) { KS_LOG("no flex configured"); return; }
	KartSimulation Sim;
	Sim.Initialize(C);
	PlaneTrack G;
	Sim.Reset(Vec3(0, 0, 0), 0.0);
	kstest::SetRolling(Sim, 14.0);
	kstest::SpeedController Speed;
	KartInputs In;
	In.SteeringColumnAngle = 28.0 * kDegToRad; // left turn: the right wheels are outside
	for (int I = 0; I < 240 * 4; ++I)
	{
		In.Throttle = Speed.Throttle(14.0, Sim.ForwardSpeed(), kTick);
		Sim.Step(kTick, In, G);
	}
	const TelemetryFrame& F = Sim.GetTelemetry();
	double Cff, Cfm, Cmm;
	WheelMountCompliance(C.Chassis.RearMount, 0.5 * C.Chassis.RearTrack, Cff, Cfm, Cmm);
	const double Fm = F.FzRR - C.Chassis.RearMount.UnsprungMass * kGravity * F.AccVertG;
	KS_LOG("circle %.2f g: Fz RL %.0f RR %.0f N; rear mount dz RL %.2f RR %.2f mm (beam %.2f mm for the vertical load alone), axle camber RL %+.3f RR %+.3f deg, frame sag %.2f mm, tyre camber RL %+.2f RR %+.2f deg",
	       F.AccLatG, F.FzRL, F.FzRR, F.MountDzRLmm, F.MountDzRRmm, Cff * Fm * 1000.0, F.MountCamberRLDeg, F.MountCamberRRDeg, F.FrameBendMm, F.CamberRLDeg, F.CamberRRDeg);
	KS_EXPECT(F.AccLatG > 1.0);
	KS_EXPECT(F.MountDzRRmm > F.MountDzRLmm);
	// Lateral force at the ground adds bending of its own: the deflection is the same order as the vertical-load estimate.
	KS_EXPECT(F.MountDzRRmm > 0.3 * Cff * Fm * 1000.0 && F.MountDzRRmm < 3.0 * Cff * Fm * 1000.0);
}
