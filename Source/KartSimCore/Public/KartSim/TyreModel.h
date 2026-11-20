// Tyre model: disc-vs-plane contact, radial spring-damper, transient (relaxation-length) slip,
// combined-slip brush model with load sensitivity, sliding-speed friction drop, aligning moment,
// lumped thermal model and wear. Equations: Docs/PhysicsModel.md section 3.
#pragma once

#include "KartSim/KartConfig.h"
#include "KartSim/TrackSurface.h"

namespace kartsim
{
	struct TyreContact
	{
		bool bInContact = false;
		Vec3 WheelCentre;          // world
		Vec3 Axle;                 // world, unit (points to the kart's left)
		Vec3 ContactPoint;         // world
		Vec3 Normal = {0, 0, 1};   // world, ground normal
		Vec3 Forward = {1, 0, 0};  // world, wheel heading projected on ground
		Vec3 Lateral = {0, 1, 0};  // world, n x forward
		double Penetration = 0.0;  // m (negative = gap)
		double LoadedRadius = 0.0; // m, effective rolling radius
		double HalfLength = 0.0;   // m, contact patch half-length
		ESurfaceType Surface = ESurfaceType::Asphalt;
		double GripFactor = 1.0;   // surface grip x dynamic multiplier
		double Wetness = 0.0;      // 0 dry .. 1 saturated
		double RollingResFactor = 1.0;
	};

	struct TyreOutput
	{
		Vec3 ForceWorld;           // total force on the kart at ContactPoint
		double Fx = 0.0, Fy = 0.0, Fz = 0.0, Mz = 0.0;
		double Vx = 0.0, Vy = 0.0;
		double SlipVelX = 0.0, SlipVelY = 0.0;
		double SlipRatio = 0.0;    // lagged kappa
		double SlipAngle = 0.0;    // rad, lagged; positive when the force pushes left
		double Mu = 0.0;
		double WetFactor = 1.0;    // mu multiplier from wetness (compound-dependent)
		double CombinedSlip = 0.0; // brush lambda, >= 1 means fully sliding
		double RollingResistanceTorque = 0.0; // Nm about the axle, opposes rotation
		double SlidePower = 0.0;   // W
		double Camber = 0.0;       // rad, inclination to the ground; + = top leaning to the kart's left
		// d(Fx)/d(wheel surface speed) of the low-speed viscous term [N s/m]. The wheel-spin
		// integrator uses it implicitly: explicit treatment is unstable at standstill.
		double LongitudinalDamping = 0.0;
	};

	class KARTSIM_API TyreModel
	{
	public:
		void Init(const TyreParams& InParams);
		void Reset();

		// Effective-road-plane enveloping (7 ground samples per wheel). Default on; switchable for
		// A/B comparison and as a performance fallback (single centre sample).
		static bool bEnableEnveloping;

		// Geometry only: lowest point of the wheel disc against the ground below it.
		static bool ComputeContact(const TyreParams& Params, const Vec3& WheelCentre, const Vec3& Axle,
		                           const ITrackQuery& Track, TyreContact& Out);

		// Phase 1: computes forces from the current slip state (no state change).
		// ContactVel: world velocity of the material point of the kart at the contact point.
		// CentreVel: world velocity of the wheel centre (used for the radial damper).
		// Dt: step length, used for the penetration rate (which must include terrain motion under the
		// wheel, not just the wheel's own velocity).
		// MountVel (optional): velocity of the wheel's mount on the frame; the seated-driver BodyDamping acts on it
		// (default: the wheel centre).
		void Evaluate(const TyreContact& Contact, const Vec3& ContactVel, const Vec3& CentreVel,
		              double WheelOmega, double Dt, TyreOutput& Out, const Vec3* MountVel = nullptr);

		// Phase 2: advances slip, thermal and wear state by Dt using the wheel speed AFTER the
		// spin update of this substep (symplectic ordering: stable wheel/tyre oscillation).
		void Integrate(double WheelOmegaNew, double AmbientTemp, double TrackTemp, double Dt);

		double GetTemperature() const { return Temperature; }
		double GetWear() const { return Wear; }
		// A tyre carried over warm keeps its hot gas (pressure); the cold reference stays where it was set.
		void SetTemperature(double T) { Temperature = T; GasTemp = T; }
		// Inflation: gauge pressure (bar) and gas temperature; 0 / tyre temperature with the model off.
		double GetPressure() const;
		double GetGasTemperature() const { return GasTemp; }
		// Wear survives Reset() (a kart put back on track keeps its tyres); Init() fits a new tyre.
		void SetWear(double W) { Wear = W < 0.0 ? 0.0 : (W > 1.0 ? 1.0 : W); }
		double GetTreadRemaining() const { return P ? P->TreadDepth * (1.0 - Wear) : 0.0; } // mm
		double WetGripFactor(double Wetness) const;
		double WearTempFactor(double T) const;
		double GetSlipStateX() const { return SlipX; }
		double GetSlipStateY() const { return SlipY; }
		double GetRelaxScale() const { return SigScale; } // sigma = RelaxLength * this

		// Grip multipliers exposed for testing.
		double TemperatureGripFactor(double T) const;
		double WearGripFactor() const;

		// Pure steady-state brush evaluation (no state change), used by tests and analysis.
		// Kappa: slip ratio, TanAlphaLat: lateral slip Vy/|Vx|. Returns Fx, Fy (Fy opposes Vy).
		void SteadyStateForces(double Fz, double Kappa, double SlipY, double Mu, double& OutFx, double& OutFy) const;

	private:
		const TyreParams* P = nullptr;
		double SlipX = 0.0;        // kappa state
		double SlipY = 0.0;        // lateral slip state (~ Vy/|Vx|)
		// Relaxation-length scale the slip states currently refer to (sigma = RelaxLength * SigScale). The carcass
		// DEFLECTION (sigma * slip) is the physical state: when the load changes sigma, the slips are rescaled.
		double SigScale = 1.0;
		double Temperature = 30.0; // degC
		double Wear = 0.0;         // 0..1
		double GasTemp = 30.0;     // degC, inflation gas
		double ColdGasTemp = 30.0; // degC at which ColdPressure was set
		double PrevPenetration = 0.0;
		bool bHasPrevPenetration = false;
		double FilteredRoadRate = 0.0;  // m/s, road profile rate felt through the contact patch

		// Kinematics cached by Evaluate for Integrate.
		struct FCache
		{
			bool bInContact = false;
			double Vx = 0.0, Vy = 0.0, Re = 0.1, Speed = 0.0;
			double SlidePower = 0.0, RollPower = 0.0, Wetness = 0.0;
			// Brush normalisation used to cap the deflection states at the sliding boundary.
			double ThetaX = 0.0, ThetaY = 0.0, Dir = 1.0, CamberSlip = 0.0;
			double StiffFactor = 1.0;
			double SigScale = 1.0;
		} Cache;

		void BrushForces(double Fz, double KappaEff, double SlipLat, double Mu, double HalfLength,
		                 double& OutFx, double& OutFy, double& OutMz, double& OutLambda) const;
	};
}
