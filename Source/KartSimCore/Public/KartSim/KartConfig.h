// Kart configuration: every physical parameter of a kart class. Loaded from JSONC files in
// Content/KartData/Karts. Angles are degrees in files and radians in these structs.
// See Docs/RotaxMaxBaseline.md for provenance (VERIFIED / ESTIMATE / ASSUMPTION) of each value.
#pragma once

#include "KartSim/Math.h"

#include <string>

namespace kartsim
{
	struct TyreParams
	{
		double UnloadedRadius = 0.13;      // m
		double Width = 0.15;               // m
		double SpinInertia = 0.014;        // kg m^2 (wheel + tyre about axle); rear uses axle inertia instead
		double RadialStiffness = 150000.0; // N/m
		double RadialDamping = 800.0;      // N s/m, tyre carcass (acts on wheel motion AND road profile)
		double BodyDamping = 0.0;          // N s/m on the wheel's own vertical motion only (seated driver, lumped)

		double Mu0 = 1.6;                  // peak friction coefficient at RefLoad, optimum temperature
		double RefLoad = 400.0;            // N
		double MuLoadSensitivity = 0.1;    // fractional mu drop per unit (Fz/Fz0 - 1)
		double CxCoeff = 25.0;             // longitudinal slip stiffness / Fz0 [1/unit slip]
		double CyCoeff = 30.0;             // cornering stiffness / Fz0 [1/rad]
		double StiffnessLoadExponent = 0.8;// C ~ (Fz/Fz0)^p
		double SlideMuDrop = 0.15;         // fraction of mu lost at high sliding speed
		double SlideRefSpeed = 4.0;        // m/s, sliding-speed scale of the mu drop
		double RelaxLengthX = 0.08;        // m
		double RelaxLengthY = 0.15;        // m
		double RollingResistance = 0.02;   // C_rr
		double CamberStiffnessRatio = 0.15; // camber stiffness / cornering stiffness (per rad)

		// Thermal (single lumped node)
		double OptimumTemp = 70.0;         // degC
		double TempWindow = 35.0;          // degC, half-width to MinTempGripFactor
		double MinTempGripFactor = 0.80;
		double HeatCapacity = 2500.0;      // J/K (effective tread+carcass)
		double SlideHeatFraction = 0.5;    // fraction of sliding power going into tyre
		double ConvectionBase = 4.0;       // W/K
		double ConvectionPerSpeed = 1.2;   // W/K per m/s
		double TrackConduction = 3.0;      // W/K
		double InitialTemp = 30.0;         // degC

		// Wet behaviour (compound property). Wetness w in [0,1] comes from the ground (water film / wet
		// surface): mu x (1 - (1 - WetGripFloor) * w). Water on the tread also cools it.
		double WetGripFloor = 0.60;        // fraction of mu kept on a saturated surface (slick 0.60)
		double WetCooling = 0.0;           // W/K extra convection at full wetness (toward the air temperature)

		// Wear: Wear in [0,1] is the fraction of the tread depth used.
		// dWear = WearRate * SlidePower * dt * 2^((T - OptimumTemp) / WearTempDoubling) (clamped 0.25..16):
		// a tyre run above its window wears much faster (wets on a drying track).
		// Grip: 1 - (1 - MinWearGripFactor) * Wear^2 (little loss until the tread is mostly gone).
		double TreadDepth = 3.5;           // mm, new
		double WearRate = 1.0e-9;          // 1/J of sliding energy at the optimum temperature
		double WearTempDoubling = 15.0;    // degC above the optimum per doubling of the wear rate
		double MinWearGripFactor = 0.85;   // grip with the tread fully gone

		// Inflation (Milestone 9). The gas follows the carcass temperature (lag GasTimeConstant) at constant volume, so
		// p_abs = p_abs,cold * T_gas / T_cold. Pressure scales the radial stiffness (k ~ p + CarcassPressure), and through
		// it the contact pressure that sets the friction coefficient (an effective load: mu sees Fz * k / k_ref), the
		// rolling-resistance hysteresis (~ deflection) and the relaxation lengths (~ carcass compliance^0.5).
		// ColdPressure <= 0 switches the model off (constant RadialStiffness, as before).
		double ColdPressure = 0.0;         // bar gauge, set cold in the pits (setup)
		double RefPressure = 0.85;         // bar gauge at which RadialStiffness and the grip calibration apply
		double CarcassPressure = 0.5;      // bar: structural (sidewall) share of the radial stiffness, as a pressure
		double GasTimeConstant = 25.0;     // s
		double StiffnessFactor(double PressureBar) const
		{
			return ColdPressure > 0.0 ? (PressureBar + CarcassPressure > 0.1 ? PressureBar + CarcassPressure : 0.1) / (RefPressure + CarcassPressure) : 1.0;
		}
	};

	struct EngineParams
	{
		Curve1D TorqueWOT;                 // rpm -> Nm at wide open throttle (brake torque)
		double Inertia = 0.003;            // kg m^2
		double IdleRpm = 2500.0;
		double IdleControlGain = 2.0;      // throttle per unit fractional rpm deficit
		double IdleMaxThrottle = 0.30;
		double RevLimitRpm = 14000.0;
		double RevLimitHysteresisRpm = 150.0;
		double DragTorqueConst = 0.6;      // Nm
		double DragTorquePerRadS = 0.0025; // Nm per rad/s
		double ThrottleTimeConstant = 0.025; // s
		// Running state. The engine stalls below StallRpm and fires again above FireRpm when turned
		// (onboard starter, push start, or by the wheels through a direct drive / engaged clutch).
		double StallRpm = 1200.0;
		double FireRpm = 1500.0;
		double StarterTorque = 0.0;        // Nm at the crank while the starter runs; 0 = no onboard starter
	};

	// Engine tuning, cooling and piston/seizure model (EngineTune.h). Class file block "tuning".
	enum class ECarbAdjust { MainJet, Needle };
	struct EngineTuneParams
	{
		// Carburettor. Fuel flow ~ Jet^FlowExponent (a main-jet number is a bore diameter in 1/100 mm -> area,
		// exponent 2; a needle opening in turns ~ linear, exponent 1). The best (safe best-power) setting moves
		// with the dry-air density: BestJet = BestJetRef * (rho_dry / rho_dry,ref)^JetDensityExponent.
		ECarbAdjust Adjust = ECarbAdjust::MainJet;
		double Jet = 122.0;                // fitted (setup)
		double JetMin = 110.0, JetMax = 140.0, JetStep = 1.0;
		double BestJetRef = 122.0;         // at the reference air below
		double RefAirTemp = 22.0;          // degC
		double RefPressure = 101.325;      // kPa
		double RefHumidity = 0.5;          // 0..1
		double JetDensityExponent = 0.70;
		double FlowExponent = 2.0;
		// Cooling: one lumped node (coolant + block for water-cooled engines, cylinder head for air-cooled).
		bool bWaterCooled = true;
		double Cover = 0.0;                // radiator covered fraction (setup, water-cooled only)
		double HeatCapacity = 9000.0;      // J/K
		double HeatPerPower = 1.1;         // W into the node per W of combustion (brake) power
		double IdleHeat = 600.0;           // W at idle rpm, scales with rpm
		double CoolingBase = 35.0;         // W/K at rest
		double CoolingPerSpeed = 20.5;     // W/K per m/s
		double OptimumLow = 50.0, OptimumHigh = 55.0; // degC best-power window (dash colours)
		double PowerPerDegCold = 0.0015;   // power fraction lost per degC below OptimumLow
		double PowerPerDegHot = 0.002;     // per degC above OptimumHigh
		double WarmTemp = 50.0;            // degC at reset (warmed up on the stand); <= 0 = start at air temperature
		// Piston (abstract crown temperature index) and seizure.
		double PistonRise = 200.0;         // degC above the cooling node at full power and best mixture
		double PistonLimit = 290.0;        // degC: above it the piston-to-bore clearance is consumed
		double PistonTimeConstant = 3.0;   // s
		double LeanHeat = 2.5;             // piston / heat multiplier per unit of leanness (1 - richness)
		double SeizeRate = 0.033;          // seize level per (degC above the limit * s); 1 = seized
		double ColdPerDeg = 1.5;           // limit drop per degC the cooling node is below OptimumLow - 15 (cold seizure:
		                                   // the aluminium piston expands faster than the bore when worked cold)
		double SeizedDragTorque = 80.0;    // Nm at the crank once seized
		// Exhaust gas temperature (probe in the header, as a Mychron EGT sensor).
		double EgtIdle = 250.0, EgtFull = 620.0; // degC at best mixture
		double EgtPerLean = 1200.0;        // degC per unit of leanness
		double EgtPerRich = 800.0;         // degC drop per unit of richness above 1
		double EgtTimeConstant = 1.0;      // s (probe + exhaust)
	};

	// Mechanical damage (KartDamage in KartPhysicsCore). Class file block "damage". All thresholds are
	// ASSUMPTIONS calibrated so clean laps over the kerbs cause none (tests in Tests/DamageTests.cpp).
	struct DamageParams
	{
		double ContactForce = 6000.0;      // N kart-kart contact force above which parts bend
		double FairingForce = 4500.0;      // N on the front fairing (nose) to push it back (CIK release kit)
		double SidepodForce = 5500.0;      // N side impact that knocks a sidepod loose (brackets bend)
		double BumperForce = 6500.0;       // N rear impact that pushes the rear bumper in
		double ToePerImpulse = 0.08;       // deg toe change per N s of front-corner impulse above ContactForce
		double AxleBendPerImpulse = 0.03;  // deg rear axle bend per N s of rear impulse above ContactForce
		double RadiatorLeakPerImpulse = 0.02; // coolant fraction lost per s, per N s of radiator-side impulse
		double RadiatorSide = 1.0;         // +1 = left of the seat, -1 = right, 0 = none (air-cooled)
		double WheelLoadFront = 4500.0;    // N front tyre load spike that bends a steering arm (kerb / landing)
		double WheelLoadRear = 6500.0;     // N rear tyre load spike that bends the axle
		double LoadToePerKn = 0.6;         // deg per kN above WheelLoadFront
		double LoadBendPerKn = 0.25;       // deg per kN above WheelLoadRear
		double MaxToe = 4.0;               // deg
		double MaxAxleBend = 3.0;          // deg
		double AxleDragPerDeg = 3.0;       // Nm axle friction per deg of bend
		double ChainOffImpulse = 60.0;     // N s single rear hit that throws the chain
		double ChainOffBend = 1.5;         // deg of axle bend at which the chain jumps off
		double MaxBendPerHit = 1.0;        // deg: most toe / axle bend one strike or hit can add (parts yield, then the kart moves)
	};

	enum class EClutchType { Centrifugal, Manual, DirectDrive };

	struct ClutchParams
	{
		EClutchType Type = EClutchType::Centrifugal;
		double EngageStartRpm = 3800.0;    // centrifugal only
		double FullLockRpm = 5500.0;       // centrifugal only
		double MaxTorque = 55.0;           // Nm capacity at full engagement
	};

	struct GearboxParams
	{
		static constexpr int MaxGears = 8;
		int NumGears = 1;
		double Ratios[MaxGears] = {1.0};   // engine : gearbox output
		double PrimaryRatio = 1.0;
		double ShiftCutTime = 0.0;         // s of ignition cut on upshift (KZ)
	};

	struct FinalDriveParams
	{
		int EngineSprocket = 12;
		int AxleSprocket = 80;
		double ChainEfficiency = 0.95;
		double RearAxleInertia = 0.075;    // kg m^2, entire rotating rear assembly
	};

	struct BrakeParams
	{
		double RearMaxTorque = 320.0;      // Nm at full pedal
		double FrontMaxTorque = 0.0;       // Nm per front wheel at full pedal (0 = no front brakes)
		double DiscHeatCapacity = 900.0;   // J/K
		double DiscCoolingBase = 2.0;      // W/K
		double DiscCoolingPerSpeed = 0.6;  // W/K per m/s
		double FadeStartTemp = 450.0;      // degC
		double FadePerDeg = 0.0015;        // fraction of torque lost per degC above fade start
		double InitialTemp = 30.0;         // degC
	};

	struct SteeringParams
	{
		double Ratio = 3.6;                // column angle : mean road-wheel angle
		double MaxColumnAngle = 100.0 * kDegToRad;
		double AckermannFactor = 0.7;      // 0 = parallel, 1 = ideal Ackermann
		double Caster = 14.0 * kDegToRad;
		double KingpinInclination = 10.0 * kDegToRad;
		double SpindleOffset = 0.085;      // m, kingpin pivot inboard of wheel-centre plane
		double StaticToe = 0.0;            // rad, positive = toe-in
		double StaticCamber = 0.0;         // rad, negative = top inward
	};

	// Elastic mount of one wheel assembly: a tube (rear axle) or solid bar (front stub axle) cantilevered from its
	// support, with a rigid part (hub + wheel) between the end of the free span and the wheel centre.
	//   rear:  free span = bearing -> inner end of the hub; it grows when the hubs slide out for a wider track.
	//   front: free span = FreeSpan (stub root -> effective load point of the hub bearings).
	struct WheelMountParams
	{
		double UnsprungMass = 0.0;         // kg per wheel: rim + tyre + hub (+ brake parts carried by it)
		double CamberInertia = 0.0;        // kg m^2 of that assembly about the longitudinal axis through the wheel centre
		double TubeOuterDiameter = 0.05;   // m
		double TubeWall = 0.0;             // m; 0 = solid bar
		double YoungsModulus = 2.1e11;     // Pa (steel)
		double BearingY = 0.0;             // m from the centreline to the outer axle bearing (rear); 0 = use FreeSpan
		double HubLength = 0.0;            // m (rear hub clamped on the axle, rigid)
		double HubToWheelCentre = 0.0;     // m from the hub's outer face to the wheel centre plane (rear)
		double FreeSpan = 0.06;            // m, used when BearingY = 0
		double DampingRatio = 0.03;        // structural (steel + bearings)
		double ExtraCompliance = 0.0;      // m/N added in series vertically (brackets, bolts); 0 = none
	};

	struct ChassisParams
	{
		double Mass = 162.0;               // kg, kart + driver
		double CogHeight = 0.27;           // m above ground, static loaded
		double FrontWeightFraction = 0.42;
		double Wheelbase = 1.04;           // m
		double FrontTrack = 1.10;          // m, tyre contact centres
		double RearTrack = 1.24;           // m
		Vec3 Inertia = {14.0, 24.0, 32.0}; // kg m^2 roll/pitch/yaw about CoG (roll = sum of frame node inertias)

		// Frame torsion as a chain of stations along the wheelbase (Milestone 2/3). Node 0 is the front
		// axle, the last node the rear axle. Each node carries the roll inertia of what is mounted
		// there; segments between nodes are a uniform frame with total front-to-rear stiffness
		// FrameTorsionalStiffness (segment stiffness = GJ / length).
		static constexpr int MaxFrameNodes = 8;
		int NumFrameNodes = 0;
		double FrameNodePos[MaxFrameNodes] = {};     // m behind the front axle (0 .. wheelbase)
		double FrameNodeInertia[MaxFrameNodes] = {}; // kg m^2 roll inertia carried by the node (about the CoG x-axis)
		double FrameNodeMass[MaxFrameNodes] = {};    // kg mounted at the node (sum = Mass)
		double FrameNodeHeight[MaxFrameNodes] = {};  // m above ground of that mass (mass-weighted mean = CogHeight)
		double FrameTorsionalStiffness = 18000.0;    // Nm/rad, front axle to rear axle
		double FrameTorsionalDamping = 70.0;         // Nm s/rad, front axle to rear axle (scaled like stiffness)
		// Optional explicit segment values (0 = derive from the uniform frame above).
		double FrameSegmentStiffness[MaxFrameNodes] = {};
		double FrameSegmentDamping[MaxFrameNodes] = {};
		double TwistAxisHeight = 0.10;     // m above ground
		// Vertical frame bending (Milestone 9): the frame chain's nodes also move vertically relative to the rigid
		// body (heave and pitch removed), joined by hinge springs of bending stiffness EI. Active when > 0 and every
		// node carries mass (after subtracting the wheel assemblies at the axle nodes).
		double FrameBendingEI = 0.0;       // N m^2 of the frame between the axles
		double FrameBendingDampingRatio = 0.15;
		// Wheel mounts (Milestone 9): each wheel assembly is its own mass on the elastic mount that carries it,
		// with a vertical and a camber degree of freedom. UnsprungMass <= 0 = rigidly mounted (old behaviour).
		WheelMountParams FrontMount;
		WheelMountParams RearMount;
		// Setup state (Setup.h): already folded into Mass / inertia / node masses / stiffness; stored so the
		// setup page can show and change it. Class files leave them at 0 / 1.
		double Ballast = 0.0;              // kg added (mounted low on the seat)
		double FrameStiffnessScale = 1.0;  // multiplier applied to the frame torsional stiffness/damping
		double CdA = 0.60;                 // m^2
		double AirDensity = 1.20;          // kg/m^3
	};

	struct KARTSIM_API KartConfig
	{
		std::string Name;
		std::string ClassId;
		ChassisParams Chassis;
		SteeringParams Steering;
		// The FITTED tyres (what the simulation uses). Compound sets: index 0 = dry (class file
		// "tyres"), 1 = wet (optional "wetTyres"). FitTyres() copies a set into FrontTyre/RearTyre.
		TyreParams FrontTyre;
		TyreParams RearTyre;
		static constexpr int MaxCompounds = 2;
		int NumCompounds = 1;
		int Compound = 0;                  // fitted set
		std::string CompoundName[MaxCompounds];
		TyreParams CompoundFront[MaxCompounds];
		TyreParams CompoundRear[MaxCompounds];
		bool HasWetTyres() const { return NumCompounds > 1; }
		void FitTyres(int Index);
		EngineParams Engine;
		EngineTuneParams Tuning;
		DamageParams Damage;
		ClutchParams Clutch;
		GearboxParams Gearbox;
		FinalDriveParams FinalDrive;
		BrakeParams Brakes;

		// Overall engine-to-axle ratio for a gear index (0-based). Returns 0 for invalid gear.
		double TotalRatio(int GearIndex) const;
	};

	// Parses a JSONC kart config. Every field listed in the schema is required so that
	// a missing value can never silently fall back to a default. Returns false with OutError.
	KARTSIM_API bool ParseKartConfig(const std::string& Text, KartConfig& Out, std::string& OutError);
	KARTSIM_API bool LoadKartConfigFile(const std::string& Path, KartConfig& Out, std::string& OutError);
}
