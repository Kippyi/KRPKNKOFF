// Telemetry: one fixed-layout frame per outer physics tick, a ring buffer and CSV export.
// The field list is an X-macro so the struct, CSV header and CSV rows can never drift apart.
#pragma once

#include "KartSim/Export.h"

#include <cstdio>
#include <string>
#include <vector>

namespace kartsim
{
#define KARTSIM_TELEMETRY_FIELDS(X) \
	X(Time) X(PosX) X(PosY) X(PosZ) \
	X(SpeedKmh) X(VelLong) X(VelLat) X(AccLongG) X(AccLatG) X(AccVertG) \
	X(YawRateDeg) X(RollDeg) X(PitchDeg) X(YawDeg) X(ChassisTwistDeg) X(RearTwistDeg) X(SideslipDeg) \
	X(EngineRpm) X(EngineTorque) X(Throttle) X(ThrottleEff) X(Brake) X(ClutchPedal) X(Gear) \
	X(ClutchSlipRpm) X(ClutchLocked) X(ClutchTorque) X(RevLimiter) X(EngineRunning) \
	X(SteerColumnDeg) X(SteerLeftDeg) X(SteerRightDeg) X(ColumnTorque) X(FfbOutput) X(FfbClipping) \
	X(RearAxleRpm) X(BrakeTorque) X(BrakeDiscTemp) \
	X(FzFL) X(FzFR) X(FzRL) X(FzRR) \
	X(FxFL) X(FxFR) X(FxRL) X(FxRR) \
	X(FyFL) X(FyFR) X(FyRL) X(FyRR) \
	X(MzFL) X(MzFR) X(CamberFLDeg) X(CamberFRDeg) \
	X(SlipRatioFL) X(SlipRatioFR) X(SlipRatioRL) X(SlipRatioRR) \
	X(SlipAngleFLDeg) X(SlipAngleFRDeg) X(SlipAngleRLDeg) X(SlipAngleRRDeg) \
	X(CombinedSlipFL) X(CombinedSlipFR) X(CombinedSlipRL) X(CombinedSlipRR) \
	X(MuFL) X(MuFR) X(MuRL) X(MuRR) \
	X(TempFL) X(TempFR) X(TempRL) X(TempRR) \
	X(WearFL) X(WearFR) X(WearRL) X(WearRR) \
	X(LiftFL) X(LiftFR) X(LiftRL) X(LiftRR) \
	X(SurfaceFL) X(SurfaceFR) X(SurfaceRL) X(SurfaceRR) \
	X(WaterTemp) X(PistonTemp) X(Egt) X(Lambda) X(Richness) X(SeizeLevel) X(PowerFactor) X(Coolant) X(AirDensity) \
	X(DamageToeL) X(DamageToeR) X(AxleBend) X(ChainOff) X(FairingOff) \
	X(CamberRLDeg) X(CamberRRDeg) X(MountDzFLmm) X(MountDzFRmm) X(MountDzRLmm) X(MountDzRRmm) \
	X(MountCamberRLDeg) X(MountCamberRRDeg) X(FrameBendMm) \
	X(PressFL) X(PressFR) X(PressRL) X(PressRR)

	struct TelemetryFrame
	{
#define KARTSIM_DECLARE_FIELD(Name) double Name = 0.0;
		KARTSIM_TELEMETRY_FIELDS(KARTSIM_DECLARE_FIELD)
#undef KARTSIM_DECLARE_FIELD
	};

	// Fixed-capacity ring buffer of frames (default 10 minutes at 240 Hz).
	class KARTSIM_API TelemetryBuffer
	{
	public:
		explicit TelemetryBuffer(size_t InCapacity = 240 * 600);
		void Push(const TelemetryFrame& Frame);
		void Clear();
		size_t Size() const { return Count; }
		// 0 = oldest.
		const TelemetryFrame& At(size_t Index) const;
		const TelemetryFrame* Latest() const { return Count ? &At(Count - 1) : nullptr; }

		bool WriteCsv(const std::string& Path) const;

	private:
		std::vector<TelemetryFrame> Frames;
		size_t Head = 0;
		size_t Count = 0;
	};

	// Streams frames to disk as they are produced (for long sessions).
	class KARTSIM_API CsvTelemetryWriter
	{
	public:
		~CsvTelemetryWriter() { Close(); }
		bool Open(const std::string& Path);
		void Write(const TelemetryFrame& Frame);
		void Close();
		bool IsOpen() const { return File != nullptr; }

		static void WriteHeader(FILE* F);
		static void WriteRow(FILE* F, const TelemetryFrame& Frame);

	private:
		FILE* File = nullptr;
	};
}
