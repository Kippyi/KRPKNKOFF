#include "KartSim/ForceFeedback.h"

#include "KartSim/Math.h"

namespace kartsim
{
	double ForceFeedbackProcessor::Process(double ColumnTorque, double Dt)
	{
		const ForceFeedbackSettings& S = Settings;
		const double Target = (S.bInvert ? -1.0 : 1.0) * ColumnTorque * S.Gain / (S.DeviceMaxTorque > 1e-3 ? S.DeviceMaxTorque : 1e-3);

		if (S.LowPassHz > 0.0)
		{
			const double Rc = 1.0 / (2.0 * kPi * S.LowPassHz);
			Filtered += (Target - Filtered) * (Dt / (Rc + Dt));
		}
		else
		{
			Filtered = Target;
		}

		const double Abs = Filtered < 0.0 ? -Filtered : Filtered;
		ClipRatio += ((Abs > 1.0 ? 1.0 : 0.0) - ClipRatio) * (Dt / (0.5 + Dt));

		// Soft clip: linear up to the knee, then a tanh shoulder that asymptotes to 1.
		double Shaped = Abs;
		const double K = S.SoftClipKnee;
		if (Abs > K && K < 1.0)
		{
			Shaped = K + (1.0 - K) * std::tanh((Abs - K) / (1.0 - K));
		}
		Shaped *= Sign(Filtered);

		const double MaxStep = S.SlewRatePerSec * Dt;
		Output += Clamp(Shaped - Output, -MaxStep, MaxStep);
		return Output;
	}
}
