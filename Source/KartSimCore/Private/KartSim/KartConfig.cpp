#include "KartSim/KartConfig.h"

#include "KartSim/Json.h"

#include <cmath>
#include <cstdio>

namespace kartsim
{
	double KartConfig::TotalRatio(int GearIndex) const
	{
		if (GearIndex < 0 || GearIndex >= Gearbox.NumGears || FinalDrive.EngineSprocket <= 0)
		{
			return 0.0;
		}
		const double Chain = static_cast<double>(FinalDrive.AxleSprocket) / static_cast<double>(FinalDrive.EngineSprocket);
		return Gearbox.Ratios[GearIndex] * Gearbox.PrimaryRatio * Chain;
	}

	void KartConfig::FitTyres(int Index)
	{
		if (Index < 0 || Index >= NumCompounds) { return; }
		Compound = Index;
		FrontTyre = CompoundFront[Index];
		RearTyre = CompoundRear[Index];
	}

	namespace
	{
		// Collects every missing/invalid field so one load reports all problems at once.
		struct ConfigReader
		{
			const JsonValue& Root;
			std::string Errors;

			explicit ConfigReader(const JsonValue& InRoot) : Root(InRoot) {}

			void AddError(const char* Path, const char* What)
			{
				char Buf[256];
				std::snprintf(Buf, sizeof(Buf), "  %s: %s\n", Path, What);
				Errors += Buf;
			}

			void Num(const char* Path, double& Out, double Scale = 1.0)
			{
				const JsonValue* V = Root.FindPath(Path);
				if (!V) { AddError(Path, "missing"); return; }
				if (!V->IsNumber()) { AddError(Path, "not a number"); return; }
				Out = V->NumberValue * Scale;
			}
			void Deg(const char* Path, double& Out) { Num(Path, Out, kDegToRad); }
			// Optional key (newer model features): missing keeps the default.
			void Opt(const char* Path, double& Out) { if (Root.FindPath(Path)) { Num(Path, Out); } }
			void Int(const char* Path, int& Out)
			{
				double D = 0.0;
				const size_t Before = Errors.size();
				Num(Path, D);
				if (Errors.size() == Before) { Out = static_cast<int>(D + (D >= 0 ? 0.5 : -0.5)); }
			}
			void Str(const char* Path, std::string& Out)
			{
				const JsonValue* V = Root.FindPath(Path);
				if (!V || V->Type != JsonValue::EType::String) { AddError(Path, "missing or not a string"); return; }
				Out = V->StringValue;
			}
			void Vector(const char* Path, Vec3& Out)
			{
				const JsonValue* V = Root.FindPath(Path);
				if (!V || !V->IsArray() || V->ArrayValue.size() != 3) { AddError(Path, "expected [x, y, z]"); return; }
				Out = Vec3(V->ArrayValue[0].NumberValue, V->ArrayValue[1].NumberValue, V->ArrayValue[2].NumberValue);
			}

			void Tyre(const std::string& Prefix, TyreParams& T)
			{
				auto P = [&Prefix](const char* Field) { return Prefix + "." + Field; };
				Num(P("unloadedRadius").c_str(), T.UnloadedRadius);
				Num(P("width").c_str(), T.Width);
				Num(P("spinInertia").c_str(), T.SpinInertia);
				Num(P("radialStiffness").c_str(), T.RadialStiffness);
				Num(P("radialDamping").c_str(), T.RadialDamping);
				Num(P("bodyDamping").c_str(), T.BodyDamping);
				Num(P("mu0").c_str(), T.Mu0);
				Num(P("refLoad").c_str(), T.RefLoad);
				Num(P("muLoadSensitivity").c_str(), T.MuLoadSensitivity);
				Num(P("cxCoeff").c_str(), T.CxCoeff);
				Num(P("cyCoeff").c_str(), T.CyCoeff);
				Num(P("stiffnessLoadExponent").c_str(), T.StiffnessLoadExponent);
				Num(P("slideMuDrop").c_str(), T.SlideMuDrop);
				Num(P("slideRefSpeed").c_str(), T.SlideRefSpeed);
				Num(P("relaxLengthX").c_str(), T.RelaxLengthX);
				Num(P("relaxLengthY").c_str(), T.RelaxLengthY);
				Num(P("rollingResistance").c_str(), T.RollingResistance);
				Num(P("camberStiffnessRatio").c_str(), T.CamberStiffnessRatio);
				Num(P("thermal.optimumTemp").c_str(), T.OptimumTemp);
				Num(P("thermal.tempWindow").c_str(), T.TempWindow);
				Num(P("thermal.minTempGripFactor").c_str(), T.MinTempGripFactor);
				Num(P("thermal.heatCapacity").c_str(), T.HeatCapacity);
				Num(P("thermal.slideHeatFraction").c_str(), T.SlideHeatFraction);
				Num(P("thermal.convectionBase").c_str(), T.ConvectionBase);
				Num(P("thermal.convectionPerSpeed").c_str(), T.ConvectionPerSpeed);
				Num(P("thermal.trackConduction").c_str(), T.TrackConduction);
				Num(P("thermal.initialTemp").c_str(), T.InitialTemp);
				Num(P("wet.gripFloor").c_str(), T.WetGripFloor);
				Num(P("wet.cooling").c_str(), T.WetCooling);
				Num(P("wear.treadDepth").c_str(), T.TreadDepth);
				Num(P("wear.rate").c_str(), T.WearRate);
				Num(P("wear.tempDoubling").c_str(), T.WearTempDoubling);
				Num(P("wear.minGripFactor").c_str(), T.MinWearGripFactor);
				Opt(P("pressure.cold").c_str(), T.ColdPressure);
				Opt(P("pressure.reference").c_str(), T.RefPressure);
				Opt(P("pressure.carcassEquivalent").c_str(), T.CarcassPressure);
				Opt(P("pressure.gasTimeConstant").c_str(), T.GasTimeConstant);
			}
		};

		bool Validate(const KartConfig& C, std::string& Err)
		{
			auto Need = [&Err](bool bCond, const char* Msg) { if (!bCond) { Err += "  "; Err += Msg; Err += "\n"; } };
			Need(C.Chassis.Mass > 0.0, "chassis.mass must be > 0");
			Need(C.Chassis.FrontWeightFraction > 0.0 && C.Chassis.FrontWeightFraction < 1.0, "chassis.frontWeightFraction must be in (0,1)");
			Need(C.Chassis.Wheelbase > 0.3, "chassis.wheelbase implausible");
			Need(C.Chassis.Inertia.X > 0 && C.Chassis.Inertia.Y > 0 && C.Chassis.Inertia.Z > 0, "chassis.inertia must be positive");
			{
				const ChassisParams& Ch = C.Chassis;
				double SumJ = 0.0;
				bool bOrdered = Ch.NumFrameNodes >= 2;
				for (int I = 0; I < Ch.NumFrameNodes; ++I)
				{
					SumJ += Ch.FrameNodeInertia[I];
					if (Ch.FrameNodeInertia[I] <= 0.0) { bOrdered = false; }
					if (I > 0 && Ch.FrameNodePos[I] <= Ch.FrameNodePos[I - 1]) { bOrdered = false; }
				}
				Need(bOrdered, "chassis.frame.nodes must have ascending positions and positive inertias");
				Need(Ch.NumFrameNodes >= 2 && std::fabs(Ch.FrameNodePos[0]) < 1e-6 && std::fabs(Ch.FrameNodePos[Ch.NumFrameNodes - 1] - Ch.Wheelbase) < 1e-6,
				     "chassis.frame.nodes must start at 0 (front axle) and end at the wheelbase (rear axle)");
				Need(std::fabs(SumJ - Ch.Inertia.X) < 0.02 * Ch.Inertia.X, "sum of frame node roll inertias must equal chassis.inertia[0] (within 2 %)");
				Need(Ch.FrameTorsionalStiffness > 0.0, "chassis.frame.torsionalStiffness must be > 0");
				// The node masses must reproduce the configured mass, CoG height and weight distribution.
				double SumM = 0.0, SumMZ = 0.0, SumMX = 0.0;
				for (int I = 0; I < Ch.NumFrameNodes; ++I)
				{
					SumM += Ch.FrameNodeMass[I];
					SumMZ += Ch.FrameNodeMass[I] * Ch.FrameNodeHeight[I];
					SumMX += Ch.FrameNodeMass[I] * Ch.FrameNodePos[I];
				}
				Need(SumM > 0.0 && std::fabs(SumM - Ch.Mass) < 0.01 * Ch.Mass, "sum of frame node masses must equal chassis.mass (within 1 %)");
				Need(SumM > 0.0 && std::fabs(SumMZ / SumM - Ch.CogHeight) < 0.01, "frame node masses must reproduce chassis.cogHeight (within 10 mm)");
				Need(SumM > 0.0 && std::fabs(SumMX / SumM - Ch.Wheelbase * (1.0 - Ch.FrontWeightFraction)) < 0.02,
				     "frame node masses must reproduce chassis.frontWeightFraction (CoG position within 20 mm)");
			}
			Need(C.Engine.TorqueWOT.Count >= 2, "engine.torqueCurve needs >= 2 points");
			Need(C.Engine.Inertia > 0.0, "engine.inertia must be > 0");
			Need(C.Tuning.JetMin > 0.0 && C.Tuning.JetMin < C.Tuning.JetMax && C.Tuning.JetStep > 0.0, "tuning.jetMin/jetMax/jetStep invalid");
			Need(C.Tuning.BestJetRef > 0.0 && C.Tuning.Jet > 0.0, "tuning.jet / bestJetRef must be > 0");
			Need(C.Tuning.HeatCapacity > 0.0 && C.Tuning.CoolingBase > 0.0, "tuning.cooling.heatCapacity / base must be > 0");
			Need(C.Tuning.PistonTimeConstant > 0.0 && C.Tuning.EgtTimeConstant > 0.0, "tuning time constants must be > 0");
			Need(C.Gearbox.NumGears >= 1 && C.Gearbox.NumGears <= GearboxParams::MaxGears, "gearbox.ratios count must be 1..8");
			Need(C.FinalDrive.EngineSprocket > 0 && C.FinalDrive.AxleSprocket > 0, "sprockets must be > 0");
			Need(C.FinalDrive.ChainEfficiency > 0.5 && C.FinalDrive.ChainEfficiency <= 1.0, "chainEfficiency must be in (0.5,1]");
			for (int K = 0; K < C.NumCompounds; ++K)
			{
				for (const TyreParams* T : {&C.CompoundFront[K], &C.CompoundRear[K]})
				{
					Need(T->RelaxLengthX > 0 && T->RelaxLengthY > 0, "relaxation lengths must be > 0");
					Need(T->WetGripFloor > 0.0 && T->WetGripFloor <= 1.0, "wet.gripFloor must be in (0,1]");
					Need(T->WetCooling >= 0.0, "wet.cooling must be >= 0");
					Need(T->TreadDepth > 0.0 && T->WearTempDoubling > 0.0, "wear.treadDepth and wear.tempDoubling must be > 0");
				}
			}
			Need(C.Clutch.Type != EClutchType::Centrifugal || C.Clutch.FullLockRpm > C.Clutch.EngageStartRpm, "clutch.fullLockRpm must exceed engageStartRpm");
			for (int I = 1; I < C.Engine.TorqueWOT.Count; ++I)
			{
				Need(C.Engine.TorqueWOT.Xs[I] > C.Engine.TorqueWOT.Xs[I - 1], "engine.torqueCurve rpm must be ascending");
			}
			return Err.empty();
		}
	}

	bool ParseKartConfig(const std::string& Text, KartConfig& Out, std::string& OutError)
	{
		const JsonParseResult Parsed = ParseJson(Text);
		if (!Parsed.bOk)
		{
			OutError = Parsed.Error;
			return false;
		}

		KartConfig C;
		ConfigReader R(Parsed.Root);

		R.Str("name", C.Name);
		R.Str("classId", C.ClassId);

		ChassisParams& Ch = C.Chassis;
		R.Num("chassis.mass", Ch.Mass);
		R.Num("chassis.cogHeight", Ch.CogHeight);
		R.Num("chassis.frontWeightFraction", Ch.FrontWeightFraction);
		R.Num("chassis.wheelbase", Ch.Wheelbase);
		R.Num("chassis.frontTrack", Ch.FrontTrack);
		R.Num("chassis.rearTrack", Ch.RearTrack);
		R.Vector("chassis.inertia", Ch.Inertia);
		R.Num("chassis.frame.torsionalStiffness", Ch.FrameTorsionalStiffness);
		R.Num("chassis.frame.torsionalDamping", Ch.FrameTorsionalDamping);
		const JsonValue* Nodes = Parsed.Root.FindPath("chassis.frame.nodes");
		if (!Nodes || !Nodes->IsArray() || Nodes->ArrayValue.size() < 2)
		{
			R.AddError("chassis.frame.nodes", "need >= 2 entries of [posFromFrontAxle, rollInertia, mass, height]");
		}
		else
		{
			Ch.NumFrameNodes = 0;
			for (const JsonValue& Nd : Nodes->ArrayValue)
			{
				if (!Nd.IsArray() || Nd.ArrayValue.size() != 4 || Ch.NumFrameNodes >= ChassisParams::MaxFrameNodes)
				{
					R.AddError("chassis.frame.nodes", "entries must be [posFromFrontAxle, rollInertia, mass, height], max 8");
					break;
				}
				Ch.FrameNodePos[Ch.NumFrameNodes] = Nd.ArrayValue[0].NumberValue;
				Ch.FrameNodeInertia[Ch.NumFrameNodes] = Nd.ArrayValue[1].NumberValue;
				Ch.FrameNodeMass[Ch.NumFrameNodes] = Nd.ArrayValue[2].NumberValue;
				Ch.FrameNodeHeight[Ch.NumFrameNodes] = Nd.ArrayValue[3].NumberValue;
				++Ch.NumFrameNodes;
			}
		}
		// Optional explicit per-segment stiffness/damping (overrides the uniform-GJ distribution).
		for (int Which = 0; Which < 2; ++Which)
		{
			const char* Key = Which == 0 ? "chassis.frame.segmentStiffness" : "chassis.frame.segmentDamping";
			double* Dst = Which == 0 ? Ch.FrameSegmentStiffness : Ch.FrameSegmentDamping;
			const JsonValue* Seg = Parsed.Root.FindPath(Key);
			if (!Seg) { continue; }
			if (!Seg->IsArray() || static_cast<int>(Seg->ArrayValue.size()) != Ch.NumFrameNodes - 1)
			{
				R.AddError(Key, "must have (number of nodes - 1) entries");
				continue;
			}
			for (int I = 0; I < Ch.NumFrameNodes - 1; ++I) { Dst[I] = Seg->ArrayValue[I].NumberValue; }
		}
		R.Num("chassis.twistAxisHeight", Ch.TwistAxisHeight);
		R.Opt("chassis.frame.bendingEI", Ch.FrameBendingEI);
		R.Opt("chassis.frame.bendingDampingRatio", Ch.FrameBendingDampingRatio);
		for (int Which = 0; Which < 2; ++Which)
		{
			const std::string Base = Which == 0 ? "chassis.wheelMounts.front." : "chassis.wheelMounts.rear.";
			WheelMountParams& M = Which == 0 ? Ch.FrontMount : Ch.RearMount;
			auto P = [&Base](const char* K) { return Base + K; };
			R.Opt(P("unsprungMass").c_str(), M.UnsprungMass);
			R.Opt(P("camberInertia").c_str(), M.CamberInertia);
			R.Opt(P("tubeOuterDiameter").c_str(), M.TubeOuterDiameter);
			R.Opt(P("tubeWall").c_str(), M.TubeWall);
			R.Opt(P("youngsModulus").c_str(), M.YoungsModulus);
			R.Opt(P("bearingY").c_str(), M.BearingY);
			R.Opt(P("hubLength").c_str(), M.HubLength);
			R.Opt(P("hubToWheelCentre").c_str(), M.HubToWheelCentre);
			R.Opt(P("freeSpan").c_str(), M.FreeSpan);
			R.Opt(P("dampingRatio").c_str(), M.DampingRatio);
			R.Opt(P("extraCompliance").c_str(), M.ExtraCompliance);
		}
		R.Num("chassis.cdA", Ch.CdA);
		R.Num("chassis.airDensity", Ch.AirDensity);

		SteeringParams& St = C.Steering;
		R.Num("steering.ratio", St.Ratio);
		R.Deg("steering.maxColumnAngleDeg", St.MaxColumnAngle);
		R.Num("steering.ackermannFactor", St.AckermannFactor);
		R.Deg("steering.casterDeg", St.Caster);
		R.Deg("steering.kingpinInclinationDeg", St.KingpinInclination);
		R.Num("steering.spindleOffset", St.SpindleOffset);
		R.Deg("steering.staticToeDeg", St.StaticToe);
		R.Deg("steering.staticCamberDeg", St.StaticCamber);

		// Tyre compound sets: "tyres" (dry, required) and "wetTyres" (optional), each {name, front, rear}.
		R.Str("tyres.name", C.CompoundName[0]);
		R.Tyre("tyres.front", C.CompoundFront[0]);
		R.Tyre("tyres.rear", C.CompoundRear[0]);
		C.NumCompounds = 1;
		if (Parsed.Root.FindPath("wetTyres"))
		{
			R.Str("wetTyres.name", C.CompoundName[1]);
			R.Tyre("wetTyres.front", C.CompoundFront[1]);
			R.Tyre("wetTyres.rear", C.CompoundRear[1]);
			C.NumCompounds = 2;
		}
		C.FitTyres(0);

		EngineParams& E = C.Engine;
		const JsonValue* Curve = Parsed.Root.FindPath("engine.torqueCurve");
		if (!Curve || !Curve->IsArray())
		{
			R.AddError("engine.torqueCurve", "missing array of [rpm, Nm]");
		}
		else
		{
			for (const JsonValue& Pt : Curve->ArrayValue)
			{
				if (!Pt.IsArray() || Pt.ArrayValue.size() != 2)
				{
					R.AddError("engine.torqueCurve", "entries must be [rpm, Nm]");
					break;
				}
				E.TorqueWOT.Add(Pt.ArrayValue[0].NumberValue, Pt.ArrayValue[1].NumberValue);
			}
		}
		R.Num("engine.inertia", E.Inertia);
		R.Num("engine.idleRpm", E.IdleRpm);
		R.Num("engine.idleControlGain", E.IdleControlGain);
		R.Num("engine.idleMaxThrottle", E.IdleMaxThrottle);
		R.Num("engine.revLimitRpm", E.RevLimitRpm);
		R.Num("engine.revLimitHysteresisRpm", E.RevLimitHysteresisRpm);
		R.Num("engine.dragTorqueConst", E.DragTorqueConst);
		R.Num("engine.dragTorquePerRadS", E.DragTorquePerRadS);
		R.Num("engine.throttleTimeConstant", E.ThrottleTimeConstant);
		R.Num("engine.stallRpm", E.StallRpm);
		R.Num("engine.fireRpm", E.FireRpm);
		R.Num("engine.starterTorque", E.StarterTorque);

		EngineTuneParams& Tu = C.Tuning;
		std::string Carb;
		R.Str("tuning.carb", Carb);
		if (Carb == "mainJet") { Tu.Adjust = ECarbAdjust::MainJet; }
		else if (Carb == "needle") { Tu.Adjust = ECarbAdjust::Needle; }
		else if (!Carb.empty()) { R.AddError("tuning.carb", "must be mainJet | needle"); }
		R.Num("tuning.jet", Tu.Jet);
		R.Num("tuning.jetMin", Tu.JetMin);
		R.Num("tuning.jetMax", Tu.JetMax);
		R.Num("tuning.jetStep", Tu.JetStep);
		R.Num("tuning.bestJetRef", Tu.BestJetRef);
		R.Num("tuning.refAirTemp", Tu.RefAirTemp);
		R.Num("tuning.refPressure", Tu.RefPressure);
		R.Num("tuning.refHumidity", Tu.RefHumidity);
		R.Num("tuning.jetDensityExponent", Tu.JetDensityExponent);
		R.Num("tuning.flowExponent", Tu.FlowExponent);
		std::string Cooling;
		R.Str("tuning.cooling.type", Cooling);
		if (Cooling == "water") { Tu.bWaterCooled = true; }
		else if (Cooling == "air") { Tu.bWaterCooled = false; }
		else if (!Cooling.empty()) { R.AddError("tuning.cooling.type", "must be water | air"); }
		R.Num("tuning.cooling.cover", Tu.Cover);
		R.Num("tuning.cooling.heatCapacity", Tu.HeatCapacity);
		R.Num("tuning.cooling.heatPerPower", Tu.HeatPerPower);
		R.Num("tuning.cooling.idleHeat", Tu.IdleHeat);
		R.Num("tuning.cooling.base", Tu.CoolingBase);
		R.Num("tuning.cooling.perSpeed", Tu.CoolingPerSpeed);
		R.Num("tuning.cooling.optimumLow", Tu.OptimumLow);
		R.Num("tuning.cooling.optimumHigh", Tu.OptimumHigh);
		R.Num("tuning.cooling.powerPerDegCold", Tu.PowerPerDegCold);
		R.Num("tuning.cooling.powerPerDegHot", Tu.PowerPerDegHot);
		R.Num("tuning.cooling.warmTemp", Tu.WarmTemp);
		R.Num("tuning.piston.rise", Tu.PistonRise);
		R.Num("tuning.piston.limit", Tu.PistonLimit);
		R.Num("tuning.piston.timeConstant", Tu.PistonTimeConstant);
		R.Num("tuning.piston.leanHeat", Tu.LeanHeat);
		R.Num("tuning.piston.seizeRate", Tu.SeizeRate);
		R.Num("tuning.piston.coldPerDeg", Tu.ColdPerDeg);
		R.Num("tuning.piston.seizedDragTorque", Tu.SeizedDragTorque);
		R.Num("tuning.egt.idle", Tu.EgtIdle);
		R.Num("tuning.egt.full", Tu.EgtFull);
		R.Num("tuning.egt.perLean", Tu.EgtPerLean);
		R.Num("tuning.egt.perRich", Tu.EgtPerRich);
		R.Num("tuning.egt.timeConstant", Tu.EgtTimeConstant);

		DamageParams& Dm = C.Damage;
		R.Num("damage.contactForce", Dm.ContactForce);
		R.Num("damage.fairingForce", Dm.FairingForce);
		R.Num("damage.sidepodForce", Dm.SidepodForce);
		R.Num("damage.bumperForce", Dm.BumperForce);
		R.Num("damage.toePerImpulse", Dm.ToePerImpulse);
		R.Num("damage.axleBendPerImpulse", Dm.AxleBendPerImpulse);
		R.Num("damage.radiatorLeakPerImpulse", Dm.RadiatorLeakPerImpulse);
		R.Num("damage.radiatorSide", Dm.RadiatorSide);
		R.Num("damage.wheelLoadFront", Dm.WheelLoadFront);
		R.Num("damage.wheelLoadRear", Dm.WheelLoadRear);
		R.Num("damage.loadToePerKn", Dm.LoadToePerKn);
		R.Num("damage.loadBendPerKn", Dm.LoadBendPerKn);
		R.Num("damage.maxToe", Dm.MaxToe);
		R.Num("damage.maxAxleBend", Dm.MaxAxleBend);
		R.Num("damage.axleDragPerDeg", Dm.AxleDragPerDeg);
		R.Num("damage.chainOffImpulse", Dm.ChainOffImpulse);
		R.Num("damage.chainOffBend", Dm.ChainOffBend);
		R.Num("damage.maxBendPerHit", Dm.MaxBendPerHit);

		std::string ClutchType;
		R.Str("clutch.type", ClutchType);
		if (ClutchType == "centrifugal") { C.Clutch.Type = EClutchType::Centrifugal; }
		else if (ClutchType == "manual") { C.Clutch.Type = EClutchType::Manual; }
		else if (ClutchType == "direct") { C.Clutch.Type = EClutchType::DirectDrive; }
		else if (!ClutchType.empty()) { R.AddError("clutch.type", "must be centrifugal | manual | direct"); }
		R.Num("clutch.maxTorque", C.Clutch.MaxTorque);
		if (C.Clutch.Type == EClutchType::Centrifugal)
		{
			R.Num("clutch.engageStartRpm", C.Clutch.EngageStartRpm);
			R.Num("clutch.fullLockRpm", C.Clutch.FullLockRpm);
		}

		const JsonValue* Ratios = Parsed.Root.FindPath("gearbox.ratios");
		if (!Ratios || !Ratios->IsArray() || Ratios->ArrayValue.empty())
		{
			R.AddError("gearbox.ratios", "missing array");
		}
		else
		{
			C.Gearbox.NumGears = 0;
			for (const JsonValue& V : Ratios->ArrayValue)
			{
				if (C.Gearbox.NumGears < GearboxParams::MaxGears)
				{
					C.Gearbox.Ratios[C.Gearbox.NumGears++] = V.NumberValue;
				}
			}
		}
		R.Num("gearbox.primaryRatio", C.Gearbox.PrimaryRatio);
		R.Num("gearbox.shiftCutTime", C.Gearbox.ShiftCutTime);

		R.Int("finalDrive.engineSprocket", C.FinalDrive.EngineSprocket);
		R.Int("finalDrive.axleSprocket", C.FinalDrive.AxleSprocket);
		R.Num("finalDrive.chainEfficiency", C.FinalDrive.ChainEfficiency);
		R.Num("finalDrive.rearAxleInertia", C.FinalDrive.RearAxleInertia);

		BrakeParams& B = C.Brakes;
		R.Num("brakes.rearMaxTorque", B.RearMaxTorque);
		R.Num("brakes.frontMaxTorque", B.FrontMaxTorque);
		R.Num("brakes.discHeatCapacity", B.DiscHeatCapacity);
		R.Num("brakes.discCoolingBase", B.DiscCoolingBase);
		R.Num("brakes.discCoolingPerSpeed", B.DiscCoolingPerSpeed);
		R.Num("brakes.fadeStartTemp", B.FadeStartTemp);
		R.Num("brakes.fadePerDeg", B.FadePerDeg);
		R.Num("brakes.initialTemp", B.InitialTemp);

		std::string ValidationErrors;
		if (R.Errors.empty())
		{
			Validate(C, ValidationErrors);
		}
		if (!R.Errors.empty() || !ValidationErrors.empty())
		{
			OutError = "Kart config errors:\n" + R.Errors + ValidationErrors;
			return false;
		}
		Out = C;
		return true;
	}

	bool LoadKartConfigFile(const std::string& Path, KartConfig& Out, std::string& OutError)
	{
		std::string Text;
		if (!ReadTextFile(Path, Text))
		{
			OutError = "Cannot read kart config file: " + Path;
			return false;
		}
		return ParseKartConfig(Text, Out, OutError);
	}
}
