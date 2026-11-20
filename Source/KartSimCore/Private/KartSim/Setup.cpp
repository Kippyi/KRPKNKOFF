#include "KartSim/Setup.h"

#include "KartSim/Json.h"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace kartsim
{
	namespace
	{
		// Ranges: rear track is capped by the CIK overall-width limit of 1400 mm (VERIFIED) minus the
		// 180 mm rear tyre; front track keeps the front axle inside the same limit. Other ranges are
		// typical adjustment spans (ASSUMPTION).
		// Ballast is fitted low on the seat (ASSUMPTION 0.15 m above ground, ~0.3 m from the CoG in plan).
		constexpr double BallastHeight = 0.15;
		constexpr double BallastRadius = 0.30;
		int HeaviestNode(const ChassisParams& Ch)
		{
			int Best = 0;
			for (int I = 1; I < Ch.NumFrameNodes; ++I) { if (Ch.FrameNodeMass[I] > Ch.FrameNodeMass[Best]) { Best = I; } }
			return Best;
		}
		void SetBallast(KartConfig& C, double V)
		{
			ChassisParams& Ch = C.Chassis;
			const double D = V - Ch.Ballast;
			const int N = HeaviestNode(Ch);
			if (Ch.NumFrameNodes > 0)
			{
				const double Mn = Ch.FrameNodeMass[N];
				Ch.FrameNodeHeight[N] = (Mn * Ch.FrameNodeHeight[N] + D * BallastHeight) / (Mn + D);
				Ch.FrameNodeMass[N] = Mn + D;
			}
			Ch.CogHeight = (Ch.Mass * Ch.CogHeight + D * BallastHeight) / (Ch.Mass + D);
			Ch.Mass += D;
			Ch.Inertia.Y += D * BallastRadius * BallastRadius;
			Ch.Inertia.Z += D * BallastRadius * BallastRadius;
			Ch.Ballast = V;
		}
		void SetSeatHeight(KartConfig& C, double V)
		{
			// The driver (and seat) move: every mass-carrying frame node moves with the CoG.
			ChassisParams& Ch = C.Chassis;
			const double D = V - Ch.CogHeight;
			for (int I = 0; I < Ch.NumFrameNodes; ++I) { if (Ch.FrameNodeMass[I] > 0.0) { Ch.FrameNodeHeight[I] += D; } }
			Ch.CogHeight = V;
		}
		void SetStiffness(KartConfig& C, double V)
		{
			ChassisParams& Ch = C.Chassis;
			const double R = V / Ch.FrameStiffnessScale;
			Ch.FrameTorsionalStiffness *= R;
			Ch.FrameTorsionalDamping *= R;
			for (int I = 0; I < ChassisParams::MaxFrameNodes; ++I) { Ch.FrameSegmentStiffness[I] *= R; Ch.FrameSegmentDamping[I] *= R; }
			Ch.FrameStiffnessScale = V;
		}

		const SetupParam GSetupParams[] = {
			{"tyreCompound", "Tyres (0 = slick, 1 = wet)", "", 0, 1, 1,
			 [](const KartConfig& C) { return static_cast<double>(C.Compound); },
			 [](KartConfig& C, double V) { C.FitTyres(static_cast<int>(V + 0.5)); },
			 [](const KartConfig& C) { return C.HasWetTyres(); },
			 "Slicks for a dry track; grooved wets for rain (they overheat and wear fast on a dry line)"},
			{"tyrePressureFront", "Front tyre pressure (cold)", "bar", 0.30, 1.20, 0.025,
			 [](const KartConfig& C) { return C.FrontTyre.ColdPressure; },
			 [](KartConfig& C, double V) { C.FrontTyre.ColdPressure = V; C.CompoundFront[C.Compound].ColdPressure = V; },
			 [](const KartConfig& C) { return C.FrontTyre.ColdPressure > 0.0; },
			 "Set cold. Rises as the tyre heats (~0.2-0.35 bar). Lower = bigger contact patch and more grip, but more heat and a lazier tyre"},
			{"tyrePressureRear", "Rear tyre pressure (cold)", "bar", 0.30, 1.20, 0.025,
			 [](const KartConfig& C) { return C.RearTyre.ColdPressure; },
			 [](KartConfig& C, double V) { C.RearTyre.ColdPressure = V; C.CompoundRear[C.Compound].ColdPressure = V; },
			 [](const KartConfig& C) { return C.RearTyre.ColdPressure > 0.0; },
			 "Set cold. Rises as the tyre heats. Higher rear pressure = less rear grip, quicker response, cooler running"},
			{"engineSprocket", "Engine sprocket", "T", 8, 24, 1,
			 [](const KartConfig& C) { return static_cast<double>(C.FinalDrive.EngineSprocket); },
			 [](KartConfig& C, double V) { C.FinalDrive.EngineSprocket = static_cast<int>(std::lround(V)); }, nullptr, "More teeth = taller gearing: higher top speed, slower acceleration"},
			{"axleSprocket", "Axle sprocket", "T", 20, 95, 1,
			 [](const KartConfig& C) { return static_cast<double>(C.FinalDrive.AxleSprocket); },
			 [](KartConfig& C, double V) { C.FinalDrive.AxleSprocket = static_cast<int>(std::lround(V)); }, nullptr, "More teeth = shorter gearing: quicker acceleration, lower top speed"},
			// Carburettor: the range / step come from the class file (main jet numbers or needle turns); the table
			// spans the widest class and SetSetupValue clamps / snaps to the class (see SetSetupValue).
			{"mainJet", "Main jet", "#", 100, 200, 1,
			 [](const KartConfig& C) { return C.Tuning.Jet; },
			 [](KartConfig& C, double V) { C.Tuning.Jet = V; },
			 [](const KartConfig& C) { return C.Tuning.Adjust == ECarbAdjust::MainJet; },
			 "Bigger jet = richer. Cold / dense air needs a bigger jet; too small (lean) = fast but hot: seizure risk"},
			{"carbNeedle", "Carb needle (turns out)", "turns", 0.5, 4.0, 0.0625,
			 [](const KartConfig& C) { return C.Tuning.Jet; },
			 [](KartConfig& C, double V) { C.Tuning.Jet = V; },
			 [](const KartConfig& C) { return C.Tuning.Adjust == ECarbAdjust::Needle; },
			 "More turns out = richer. Cold / dense air needs more; too lean = fast but hot: seizure risk"},
			{"radiatorCover", "Radiator cover", "%", 0, 90, 5,
			 [](const KartConfig& C) { return C.Tuning.Cover * 100.0; },
			 [](KartConfig& C, double V) { C.Tuning.Cover = V / 100.0; },
			 [](const KartConfig& C) { return C.Tuning.bWaterCooled; },
			 "Tape / blank the radiator to bring the water up to its window on a cold day (overheats if overdone)"},
			{"frontTrack", "Front track", "m", 0.95, 1.20, 0.01,
			 [](const KartConfig& C) { return C.Chassis.FrontTrack; },
			 [](KartConfig& C, double V) { C.Chassis.FrontTrack = V; }, nullptr, "Wider front = less weight transfer up front, usually less front grip in slow corners"},
			{"rearTrack", "Rear track", "m", 1.10, 1.22, 0.01,
			 [](const KartConfig& C) { return C.Chassis.RearTrack; },
			 [](KartConfig& C, double V) { C.Chassis.RearTrack = V; }, nullptr, "Narrower rear = more load transfer / rear grip; wider = freer, slides more"},
			{"rearAxleWall", "Rear axle wall thickness", "mm", 1.6, 3.0, 0.1,
			 [](const KartConfig& C) { return C.Chassis.RearMount.TubeWall * 1000.0; },
			 [](KartConfig& C, double V) { C.Chassis.RearMount.TubeWall = V / 1000.0; },
			 [](const KartConfig& C) { return C.Chassis.RearMount.UnsprungMass > 0.0 && C.Chassis.RearMount.TubeWall > 0.0; },
			 "Axle stiffness: a thinner wall bends more (steel 'hardness' grades barely change bending, the wall does)"},
			{"rearHubLength", "Rear hub length", "mm", 60, 160, 5,
			 [](const KartConfig& C) { return C.Chassis.RearMount.HubLength * 1000.0; },
			 [](KartConfig& C, double V) { C.Chassis.RearMount.HubLength = V / 1000.0; },
			 [](const KartConfig& C) { return C.Chassis.RearMount.UnsprungMass > 0.0 && C.Chassis.RearMount.BearingY > 0.0; },
			 "Longer hubs clamp more of the axle: stiffer rear end. A wider rear track slides the hubs out: softer"},
			{"caster", "Caster", "deg", 8, 20, 0.5,
			 [](const KartConfig& C) { return C.Steering.Caster * kRadToDeg; },
			 [](KartConfig& C, double V) { C.Steering.Caster = V * kDegToRad; }, nullptr, "More caster = more jacking (inside rear lifts), heavier steering"},
			{"toe", "Front toe (+ in)", "deg", -2, 2, 0.25,
			 [](const KartConfig& C) { return C.Steering.StaticToe * kRadToDeg; },
			 [](KartConfig& C, double V) { C.Steering.StaticToe = V * kDegToRad; }, nullptr, "Toe-in = stable on straights; toe-out = sharper turn-in"},
			{"camber", "Front camber", "deg", -3, 1, 0.25,
			 [](const KartConfig& C) { return C.Steering.StaticCamber * kRadToDeg; },
			 [](KartConfig& C, double V) { C.Steering.StaticCamber = V * kDegToRad; }, nullptr, "Negative camber = more front grip once loaded, more tyre heat"},
			{"ackermann", "Ackermann", "0-1", 0, 1, 0.05,
			 [](const KartConfig& C) { return C.Steering.AckermannFactor; },
			 [](KartConfig& C, double V) { C.Steering.AckermannFactor = V; }, nullptr, "More Ackermann = inside wheel steers more: helps turn-in, scrubs speed"},
			{"steeringRatio", "Steering ratio", ":1", 2.8, 4.5, 0.1,
			 [](const KartConfig& C) { return C.Steering.Ratio; },
			 [](KartConfig& C, double V) { C.Steering.Ratio = V; }, nullptr, "Lower ratio = quicker steering (more wheel angle per wheel turn)"},
			// Ballast BEFORE seat height: ballast lowers the CoG, seat height then sets it absolutely, so a
			// saved setup reloads exactly.
			{"ballast", "Ballast", "kg", 0, 25, 0.5,
			 [](const KartConfig& C) { return C.Chassis.Ballast; }, SetBallast, nullptr,
			 "Lead fitted low on the seat: more weight, lower CoG"},
			{"seatHeight", "Seat height (CoG)", "m", 0.22, 0.32, 0.005,
			 [](const KartConfig& C) { return C.Chassis.CogHeight; }, SetSeatHeight, nullptr,
			 "Higher seat = more weight transfer (inside rear lifts sooner)"},
			{"frontWeight", "Seat fore/aft (front weight)", "%", 38, 47, 0.5,
			 [](const KartConfig& C) { return C.Chassis.FrontWeightFraction * 100.0; },
			 [](KartConfig& C, double V) { C.Chassis.FrontWeightFraction = V / 100.0; }, nullptr,
			 "Seat forward = more front grip / turn-in, less rear traction"},
			{"chassisStiffness", "Chassis stiffness", "%", 70, 130, 5,
			 [](const KartConfig& C) { return C.Chassis.FrameStiffnessScale * 100.0; },
			 [](KartConfig& C, double V) { SetStiffness(C, V / 100.0); }, nullptr,
			 "Frame torsional stiffness (bars / frame): stiffer = more inside-rear lift"},
			{"frontBrakeShare", "Front brake share", "%", 0, 60, 2.5,
			 [](const KartConfig& C) { const double T = 2.0 * C.Brakes.FrontMaxTorque + C.Brakes.RearMaxTorque; return T > 0.0 ? 200.0 * C.Brakes.FrontMaxTorque / T : 0.0; },
			 [](KartConfig& C, double V)
			 {
				 const double T = 2.0 * C.Brakes.FrontMaxTorque + C.Brakes.RearMaxTorque;
				 C.Brakes.FrontMaxTorque = 0.5 * T * V / 100.0;
				 C.Brakes.RearMaxTorque = T * (1.0 - V / 100.0);
			 },
			 [](const KartConfig& C) { return C.Brakes.FrontMaxTorque > 0.0; },
			 "Front/rear brake balance (gearbox classes with front brakes)"},
		};
		constexpr int GNumSetupParams = static_cast<int>(sizeof(GSetupParams) / sizeof(GSetupParams[0]));
	}

	const SetupParam* GetSetupParams(int& OutCount)
	{
		OutCount = GNumSetupParams;
		return GSetupParams;
	}

	void SetSetupValue(KartConfig& Config, int Index, double Value)
	{
		if (Index < 0 || Index >= GNumSetupParams)
		{
			return;
		}
		const SetupParam& P = GSetupParams[Index];
		if (!P.IsAvailable(Config)) { return; }
		double Lo = P.Min, Hi = P.Max, Step = P.Step;
		if (std::strcmp(P.Key, "mainJet") == 0 || std::strcmp(P.Key, "carbNeedle") == 0)
		{
			// Carburettor settings: the class file gives the real range and step.
			Lo = Config.Tuning.JetMin; Hi = Config.Tuning.JetMax; Step = Config.Tuning.JetStep;
		}
		double V = Clamp(Value, Lo, Hi);
		V = Lo + std::round((V - Lo) / Step) * Step;
		P.Set(Config, Clamp(V, Lo, Hi));
	}

	std::string SerializeSetup(const KartConfig& Config)
	{
		std::string S = "{\n  \"classId\": \"" + Config.ClassId + "\",\n  \"values\": {\n";
		char Buf[160];
		for (int I = 0; I < GNumSetupParams; ++I)
		{
			std::snprintf(Buf, sizeof(Buf), "    \"%s\": %.6g%s\n", GSetupParams[I].Key, GSetupParams[I].Get(Config),
			              I + 1 < GNumSetupParams ? "," : "");
			S += Buf;
		}
		S += "  }\n}\n";
		return S;
	}

	bool ApplySetupJson(const std::string& Text, KartConfig& Config, std::string& OutError)
	{
		const JsonParseResult R = ParseJson(Text);
		if (!R.bOk) { OutError = R.Error; return false; }
		const JsonValue* Cls = R.Root.Find("classId");
		if (!Cls || Cls->StringValue != Config.ClassId)
		{
			OutError = "setup belongs to a different kart class";
			return false;
		}
		const JsonValue* Values = R.Root.Find("values");
		if (!Values || !Values->IsObject()) { OutError = "missing values"; return false; }
		OutError.clear();
		// Apply in TABLE order (ballast before seat height: see the table), whatever the file order.
		for (int I = 0; I < GNumSetupParams; ++I)
		{
			const JsonValue* V = Values->Find(GSetupParams[I].Key);
			if (V && V->IsNumber()) { SetSetupValue(Config, I, V->NumberValue); }
		}
		for (const auto& KV : Values->ObjectValue)
		{
			bool bKnown = false;
			for (int I = 0; I < GNumSetupParams; ++I) { bKnown = bKnown || KV.first == GSetupParams[I].Key; }
			if (!bKnown || !KV.second.IsNumber()) { OutError += "ignored '" + KV.first + "'; "; }
		}
		return true;
	}
}
