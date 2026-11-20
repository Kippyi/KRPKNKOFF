// Force feedback signal processing. Input is the simulated steering-column torque (from kingpin
// moments of the real tyre forces); output is a normalised device command in [-1, 1].
// No synthetic effects: road texture, kerbs and grip loss arrive through the physics signal.
#pragma once

#include "KartSim/Export.h"

namespace kartsim
{
	struct ForceFeedbackSettings
	{
		double Gain = 1.0;               // column Nm -> wheel Nm scaling (user preference)
		double DeviceMaxTorque = 10.0;   // Nm the device can produce at 100 % (e.g. Simagic Alpha Evo ~ 15-18)
		double LowPassHz = 120.0;        // smoothing of the 240 Hz signal; <= 0 disables
		double SlewRatePerSec = 400.0;   // max change of the normalised command per second
		double SoftClipKnee = 0.85;      // above this, the command is compressed toward 1
		bool bInvert = false;
	};

	class KARTSIM_API ForceFeedbackProcessor
	{
	public:
		ForceFeedbackSettings Settings;

		// ColumnTorque: torque the tyres put on the column (positive = turning left).
		// Returns the command to send (positive = wheel pulls left).
		double Process(double ColumnTorque, double Dt);

		void Reset() { Filtered = 0.0; Output = 0.0; ClipRatio = 0.0; }
		double GetOutput() const { return Output; }
		// Fraction of recent frames whose pre-clip command exceeded 1.0 (exponential average).
		double GetClipRatio() const { return ClipRatio; }

	private:
		double Filtered = 0.0;
		double Output = 0.0;
		double ClipRatio = 0.0;
	};
}
