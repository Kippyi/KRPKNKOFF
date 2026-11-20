// Physically modelled two-stroke engine sound (engine sound v4). Replaces the pulse-shape synthesis of
// EngineSoundSynth v1-v3 (user: "they don't sound like the real engines").
//
// What is simulated, per audio sample (48 kHz, cylinder in 4 sub-steps):
//  * CYLINDER (0-D gas: mass, temperature, fresh-charge fraction): slider-crank volume, compression, Wiebe
//    combustion of the trapped fresh charge (cycle-to-cycle variation, misfires), wall heat loss, expansion.
//  * PORTS, uncovered by the piston: exhaust (compressible orifice flow, both directions) into the pipe; transfers
//    from the crankcase. Rotax EVO exhaust valve: lowers the exhaust port roof (later opening) below the switch rpm.
//  * CRANKCASE pumping (0-D) through the reed valve (one-way, opens with the pressure difference) and the carburettor
//    throat (throttle area) from the AIRBOX (Helmholtz volume + snorkels to the atmosphere).
//  * EXPANSION CHAMBER as a 1-D acoustic waveguide (Kelly-Lochbaum: one cylindrical section per sample of travel,
//    scattering at every area change): header, diffuser cone (returns a suction wave), belly, baffle cone (returns
//    the plugging wave), stinger. The port flow is solved against the wave arriving from the pipe at that instant,
//    so the pipe really pulls and plugs the cylinder (the "on the pipe" note and power band come from that).
//    Hot gas: the wave speed follows the exhaust gas temperature (the pipe tune shifts with EGT, as on a real kart),
//    hottest at the port and cooling along the pipe (section lengths follow the local sound speed; the impedance
//    change from the temperature gradient reflects too).
//  * SILENCER: straight-through perforated core (more waveguide) with packing that absorbs the highs, open end
//    (reflection ~ -1, weaker at high frequency) -> radiated exhaust sound = d(end flow)/dt.
//    Radiated intake sound = d(snorkel flow)/dt.
// Not modelled: nonlinear wave steepening, pipe-shell ringing, temperature gradient along the pipe.
//
// Geometry: bore / stroke VERIFIED from homologations (Rotax 54 x 54.5 mm, IAME Screamer 54 x 54.4 mm), KT100
// 52 x 46 mm (secondary source). Port timings, pipe diameters, volumes: ESTIMATES typical of these engines (the Rotax
// homologation appendix could not be fetched). Pipe LENGTHS are derived from the tuning rule (wave round-trip times
// matched to the port timing at the class's power-peak rpm), see PipeFromTuning().
#pragma once

#include "KartSim/Export.h"

#include <cstdint>
#include <vector>

namespace kartsim
{
	struct TwoStrokeGeometry
	{
		// Engine
		float Bore = 0.054f, Stroke = 0.0545f, ConRod = 0.110f;  // m
		float CompressionRatio = 12.5f;     // geometric (swept + clearance) / clearance
		float CrankcaseRatio = 1.35f;       // primary compression
		float EpoDeg = 82.0f;               // exhaust port opens, deg after TDC (valve open / no valve)
		float EpoValveClosedDeg = 0.0f;     // with the exhaust valve closed (0 = no valve)
		float TpoDeg = 116.0f;              // transfers open, deg after TDC
		float ExhaustWidth = 0.80f;         // effective exhaust port width / bore (main + auxiliaries)
		float TransferWidth = 1.30f;        // total transfer port width / bore
		float SparkDeg = 18.0f;             // ignition, deg before TDC
		float BurnDeg = 45.0f;              // combustion duration
		float CarbDiameter = 0.034f;        // m
		float ReedArea = 9.0e-4f;           // m^2 fully open
		// Exhaust (lengths from PipeFromTuning unless set)
		float TunedRpm = 11500.0f;
		float HeaderD = 0.040f, BellyD = 0.100f, StingerD = 0.022f; // m
		float HeaderL = 0.0f, DiffuserL = 0.0f, BellyL = 0.0f, BaffleL = 0.0f, StingerL = 0.20f;
		float SilencerL = 0.32f;            // m, straight-through core
		float TailD = 0.030f;               // m, core / outlet diameter
		float PackingHz = 1200.0f;          // packing absorbs above this...
		float PackingDry = 0.30f;           // ...except this fraction passing straight through
		// Intake
		float AirboxVolume = 0.0030f;       // m^3
		float SnorkelD = 0.025f, SnorkelL = 0.14f;
		int SnorkelCount = 2;
		float AirboxDamping = 0.70f;        // filter + foam baffles: heavily damped (0.25 droned at low rpm)
		// Mix (relative levels of the two radiators)
		float ExhaustGain = 1.0f, IntakeGain = 0.6f;
		// Cylinder sub-steps per audio sample (4 = full detail; 2 = cheaper, for other karts)
		int Substeps = 4;

		// Fills the pipe lengths from TunedRpm and the port timing (wave speed at a typical full-load gas temperature).
		void PipeFromTuning();
	};

	struct TwoStrokeInput
	{
		float Rpm = 0.0f;
		float Throttle = 0.0f;   // 0..1 (carb throat area)
		float FireProb = 1.0f;   // probability that a cycle fires (misfire / overrun / limiter)
		float Roughness = 0.1f;  // cycle-to-cycle burn variation
		float JitterDeg = 2.0f;  // ignition / burn timing variation
		float ValveOpen = 1.0f;  // exhaust valve position 0..1 (ignored without a valve)
	};

	struct TwoStrokeOutput
	{
		float Exhaust = 0.0f, Intake = 0.0f; // radiated (arbitrary but consistent units)
		float CrankDeg = 0.0f;               // 0 = TDC
	};

	// Diagnostics for tests (last completed cycle).
	struct TwoStrokeCycleStats
	{
		double PeakCylinderBar = 0.0;   // absolute
		double TrappedFreshMg = 0.0;    // fresh charge burnt
		double DeliveryRatio = 0.0;     // fresh mass in through the reed / (displacement x ambient density)
		double PortPeakBar = 0.0, PortMinBar = 0.0; // pipe pressure at the port (absolute)
		double PipeGasK = 0.0;          // at the port
		double WorkJ = 0.0;             // indicated work (p dV)
		double IndicatedKW = 0.0;
		int Cycles = 0;
	};

	class KARTSIM_API TwoStrokeAcoustics
	{
	public:
		void Init(const TwoStrokeGeometry& G, float SampleRateHz);
		TwoStrokeOutput Step(const TwoStrokeInput& In);
		const TwoStrokeCycleStats& LastCycle() const { return Last; }
		// Diagnostics (tests): absolute pressures in bar.
		double PortBar() const { return PortP / 1e5; }
		double CylinderBar() const;
		double CrankcaseBar() const;
		double PipeLengthM() const { return PipeLen; }

	private:
		double PistonX(double ThetaDeg) const;  // m from TDC
		double FlowMass(double Area, double PUp, double TUp, double PDn, double Gamma) const; // kg/s, >= 0
		void BuildPipe(double PortT);
		void CylinderSubstep(double Dt, double ThetaDeg, double DTheta, const TwoStrokeInput& In, double B0, double& PortMdotSum);

		TwoStrokeGeometry Geo;
		double Fs = 48000.0;
		// derived
		double Ap = 0.0, Vd = 0.0, Vc = 0.0, VccMin = 0.0, Xe = 0.0, XeClosed = 0.0, Xt = 0.0;
		double AThroat = 0.0;
		// crank
		double Theta = 0.0;
		// cylinder
		double Mc = 0.0, Tc = 300.0, Fc = 1.0;  // mass kg, temperature K, fresh fraction
		double QCycle = 0.0, BurnStart = 0.0, BurnDur = 45.0, BurnXb = 0.0;
		bool bBurning = false, bSparked = false;
		// crankcase
		double Mcc = 0.0, Tcc = 310.0;
		// airbox / snorkel
		double Pab = 0.0, Usn = 0.0;  // perturbation Pa, m^3/s (into the box)
		// silencer / tail
		double Ps = 0.0, Ut = 0.0;    // perturbation Pa, m^3/s (out of the tail)
		double PrevUt = 0.0, PrevUsn = 0.0;
		// pipe
		double PipeC = 0.0, PipeT = 600.0, PipeRho = 0.6;   // at the port (hottest section)
		double PipeLen = 0.0, BuiltT = 0.0;
		std::vector<float> F, B, NF, NB, K, Area, Csec;
		int Js = 0;                   // first silencer section
		float PackF = 0.0f, PackB = 0.0f, PackA = 0.1f;
		float PortLp = 0.0f, EndLp = 0.0f;
		double PortP = 0.0;           // last port pressure (absolute)
		double OutTempAcc = 0.0, OutMassAcc = 0.0;
		// flow table, piston position table (0.25 deg)
		std::vector<float> PsiTab[2];
		std::vector<double> XTab;
		double PistonXFast(double ThetaDeg) const;
		// cycle randomness
		uint32_t Rng = 0x1234567u;
		double CycleScale = 1.0, CycleJitter = 0.0;
		// stats
		TwoStrokeCycleStats Cur, Last;
		double ReedMassCycle = 0.0;
		int RebuildCount = 0;
		uint32_t SampleCount = 0;
		double Pab_Flow = 0.0;        // reed mass flow this sample (kg/s), into the crankcase
		float Noise();
	};
}
