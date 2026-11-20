#include "KartLapAnalysis.h"

#include "KartSim/KartPhysicsCore.h"
#include "KartSim/LapTiming.h"
#include "KartUnrealConversions.h"

void FKartLapAnalysis::Configure(const TArray<FVector>& CentreLineUE)
{
	std::vector<kartsim::Vec3> P;
	P.reserve(CentreLineUE.Num());
	for (const FVector& V : CentreLineUE) { P.push_back(KartConv::FromUEPos(V)); }
	Path.Build(P);
	Delta.Configure(&Path);
	SeenLaps = 0;
	bWasTiming = false;
}

void FKartLapAnalysis::Step(double Now, const kartsim::LapTimer& Timer, const kartsim::KartSimulation& Sim)
{
	// Reference = best valid full lap of the whole run.
	for (size_t I = SeenLaps; I < Timer.Laps().size(); ++I)
	{
		const kartsim::LapRecord& L = Timer.Laps()[I];
		Delta.OnLapCompleted(L.LapTime, L.bValid);
	}
	SeenLaps = Timer.Laps().size();
	if (Timer.IsTiming())
	{
		const kartsim::TelemetryFrame& F = Sim.GetTelemetry();
		kartsim::TraceInputs In;
		In.SpeedKmh = F.SpeedKmh;
		In.Throttle = F.Throttle;
		In.Brake = F.Brake;
		In.SteerDeg = F.SteerColumnDeg;
		In.Rpm = F.EngineRpm;
		In.Gear = F.Gear;
		In.LatG = F.AccLatG;
		In.LongG = F.AccLongG;
		Delta.Sample(Timer.CurrentLapTime(Now), Sim.GetChassis().Position, In);
	}
	else if (bWasTiming)
	{
		Delta.AbortLap(); // kart reset mid-lap
	}
	bWasTiming = Timer.IsTiming();
}
