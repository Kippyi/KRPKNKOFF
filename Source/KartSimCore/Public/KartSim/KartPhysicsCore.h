// KartSimulation: the complete single-kart physics simulation. Owns its configuration and all
// sub-models; the host calls Step() at a fixed rate (240 Hz) with inputs and a track query.
#pragma once

#include "KartSim/BrakeModel.h"
#include "KartSim/ChassisModel.h"
#include "KartSim/EngineModel.h"
#include "KartSim/EngineTune.h"
#include "KartSim/KartDamage.h"
#include "KartSim/ForceFeedback.h"
#include "KartSim/KartConfig.h"
#include "KartSim/SteeringModel.h"
#include "KartSim/Telemetry.h"
#include "KartSim/TrackSurface.h"
#include "KartSim/TransmissionModel.h"
#include "KartSim/TyreModel.h"

#include <string>

// Inner rate default: 16 substeps of the 240 Hz tick. Overridable at compile time (A/B test builds).
#ifndef KARTSIM_DEFAULT_SUBSTEPS
#define KARTSIM_DEFAULT_SUBSTEPS 16
#endif

namespace kartsim
{
	struct KartInputs
	{
		double Throttle = 0.0;            // 0..1
		double Brake = 0.0;               // 0..1
		double Clutch = 0.0;              // 0..1, 1 = pedal fully pressed (disengaged); manual clutch only
		double SteeringColumnAngle = 0.0; // rad, positive = steer left
		int ShiftRequest = 0;             // +1 / -1 edge-triggered by the host
		bool bIgnitionKill = false;
		bool bStarter = false;            // onboard starter button (classes with StarterTorque > 0)
		double PushForce = 0.0;           // N, external push along the kart heading (helper push start)
	};

	struct SimulationSettings
	{
		int Substeps = KARTSIM_DEFAULT_SUBSTEPS; // inner integration steps per Step() call (240 Hz x 16 = 3840 Hz, Milestone 9)
		double AmbientTemp = 22.0;        // degC
		double TrackTemp = 30.0;          // degC
		double DragScale = 1.0;           // aerodynamic drag multiplier (slipstream behind another kart)
		double AirPressure = 101.325;     // kPa (engine power, mixture)
		double Humidity = 0.5;            // relative 0..1
		bool bDamage = true;              // mechanical damage from contacts and kerb strikes
		bool bEngineFailures = true;      // lean / overheat seizure
		AirState Air() const { AirState A; A.TempC = AmbientTemp; A.PressureKPa = AirPressure; A.Humidity = Humidity; return A; }
	};

	struct WheelState
	{
		TyreContact Contact;
		TyreOutput Tyre;
		Vec3 CentreWorld;
		Vec3 AxleWorld;
		double Omega = 0.0;               // rad/s (rear wheels share the axle)
		double SpinAngle = 0.0;           // rad, accumulated for rendering
		double SteerAngle = 0.0;          // rad about kingpin (front only)
		bool bLifted = false;
		// Wheel mount (Milestone 9): deviation from the static deflection. Z = body-z offset of the wheel centre
		// (m, + = up), Psi = rotation of the assembly about the body x axis (rad, + = right-hand about +x).
		double MountZ = 0.0, MountZRate = 0.0;
		double MountPsi = 0.0, MountPsiRate = 0.0;
		double MountForceZ = 0.0;         // N, body-z force the mount passes into the frame (static included)
	};

	// Static compliance of a wheel mount at the wheel centre (cantilever free span + rigid hub/wheel lever), for a
	// wheel at |y| = WheelY from the centreline: z per N (Cff), z per N m = camber per N (Cfm), camber per N m (Cmm).
	// Returns false when the mount is rigid (UnsprungMass <= 0 or no free span).
	KARTSIM_API bool WheelMountCompliance(const WheelMountParams& P, double WheelY, double& Cff, double& Cfm, double& Cmm);

	class KARTSIM_API KartSimulation
	{
	public:
		KartSimulation() = default;
		KartSimulation(const KartSimulation&) = delete;            // sub-models point into Config
		KartSimulation& operator=(const KartSimulation&) = delete;

		void Initialize(const KartConfig& InConfig);

		// Places the kart at rest with its CoG above GroundPoint at static height, heading Yaw (rad).
		void Reset(const Vec3& GroundPoint, double Yaw);

		// Advances the simulation by Dt (one outer tick), split into Settings.Substeps.
		void Step(double Dt, const KartInputs& Inputs, const ITrackQuery& Track);

		const KartConfig& GetConfig() const { return Config; }
		const ChassisState& GetChassis() const { return Chassis.State; }
		const ChassisModel& GetChassisModel() const { return Chassis; }
		// Static mount deflection of a wheel (z m, camber rad); 0 for a rigid mount.
		double GetMountStaticZ(int I) const { return Mounts[I].Qs[0]; }
		double GetMountStaticPsi(int I) const { return Mounts[I].Qs[1]; }
		bool HasMount(int I) const { return Mounts[I].bActive; }
		// Mechanical energy held by the wheel mounts relative to the static state (kinetic + elastic + the wheel
		// assemblies' height change), J. For energy audits.
		double MountEnergy() const;
		const SteeringModel& GetSteeringModel() const { return Steering; }
		const WheelState& GetWheel(int Index) const { return Wheels[Index]; }
		const EngineModel& GetEngine() const { return Engine; }
		const TransmissionModel& GetTransmission() const { return Transmission; }
		const BrakeModel& GetBrakes() const { return Brakes; }
		const TelemetryFrame& GetTelemetry() const { return Frame; }
		double GetTime() const { return Time; }
		double GetColumnTorque() const { return ColumnTorqueAvg; }

		ForceFeedbackProcessor& GetForceFeedback() { return Ffb; }

		// Engine condition (jetting, temperatures, seizure) and mechanical damage. Both survive Reset();
		// Repair() restores a fresh engine top end, coolant, straight parts and the chain.
		const EngineCondition& GetEngineCondition() const { return Cond; }
		EngineCondition& MutableEngineCondition() { return Cond; }
		const KartDamageState& GetDamage() const { return Damage; }
		void Repair();
		// Carburettor / radiator cover of the running kart (AI auto-tune); the setup path re-initialises instead.
		void SetCarb(double Jet, double Cover) { Config.Tuning.Jet = Jet; Config.Tuning.Cover = Cover; }
		// Kart-to-kart contact force acting on this kart during the coming Step() (KartContact calls this
		// next to SetExternalLoad); used only for damage.
		void AddContactForce(const Vec3& PointWorld, const Vec3& ForceWorld);
		// Damage by hand (tests, debug keys): toe in deg per side, axle bend in deg.
		void SetDamage(const KartDamageState& D);
		SimulationSettings Settings;

		// External force (world frame, N) acting at a world point, e.g. kart-to-kart contact. Held
		// constant over the next Step() (the lever arm is fixed in the body frame); replaced by the next
		// call, removed by ClearExternalForce(). Reported in telemetry via the chassis accelerations.
		void SetExternalForce(const Vec3& ForceWorld, const Vec3& PointWorld);
		// General form: net force (world) and net moment about the CoG (world), e.g. several contacts.
		void SetExternalLoad(const Vec3& ForceWorld, const Vec3& TorqueAboutCogWorld);
		void ClearExternalForce() { ExtForceW = Vec3(); ExtTorqueB = Vec3(); for (double& W : ExtWheelTorque) { W = 0.0; } }
		// Torques about each wheel's axle (Nm, + = spinning forward) from wheel-to-wheel friction, held over the next
		// Step() like the external load (rear wheels act on the solid axle). Replaced by the next call.
		void SetWheelTorques(const double T[NumWheels]) { for (int I = 0; I < NumWheels; ++I) { ExtWheelTorque[I] = T[I]; } }
		double GetWheelContactTorque(int I) const { return ExtWheelTorque[I]; } // non-zero = a tyre is touching another kart
		const Vec3& GetExternalForce() const { return ExtForceW; }

		// Kinematic pose for a kart simulated elsewhere (multiplayer proxy): sets the chassis state and
		// places the wheels (front steered about the kingpins), advancing wheel spin by Omega * Dt. No
		// forces, no tyre model; the kart is not stepped. Contacts against it use this state.
		void SetKinematicPose(const Vec3& Position, const Quat& Orientation, const Vec3& Velocity, const Vec3& AngularVelocityBody,
		                      double SteerLeft, double SteerRight, double OmegaFront, double OmegaRear, double Dt);

		// Forward speed (m/s) along the chassis x axis.
		double ForwardSpeed() const;

		// Allows tests to set state directly.
		ChassisState& MutableChassis() { return Chassis.State; }
		void SetRearAxleOmega(double W) { RearAxleOmega = W; Wheels[WheelRL].Omega = W; Wheels[WheelRR].Omega = W; }
		void SetFrontWheelOmega(double W) { Wheels[WheelFL].Omega = W; Wheels[WheelFR].Omega = W; }
		EngineModel& MutableEngine() { return Engine; }

		// Tyre state (wear survives Reset(); Initialize() fits new tyres). Used to carry a used tyre set
		// between sessions.
		const TyreModel& GetTyreModel(int Index) const { return Tyres[Index]; }
		void SetTyreWear(int Index, double Wear) { Tyres[Index].SetWear(Wear); }
		void SetTyreTemperature(int Index, double T) { Tyres[Index].SetTemperature(T); }

	private:
		void Substep(double Dt, const KartInputs& Inputs, const ITrackQuery& Track, double& ColumnTorqueAccum, Vec3& SpecificForceAccum);
		void FillTelemetry(const KartInputs& Inputs, const Vec3& SpecificForceBody);
		void UpdateContactDamage(double Dt);
		// Rollover / ride-over: body points (helmet, shoulders, steering wheel, sidepods, nose, bumper, seat) against
		// the ground. Adds to ForceW / TorqueB.
		void BodyGroundContacts(const ITrackQuery& Track, Vec3& ForceW, Vec3& TorqueB);
		void WheelLoadDamage(int Wheel, double Fz, double Dt);
		void AddDamageEvent(const char* What);
		void ApplyDamageEffects();

		KartConfig Config;
		ChassisModel Chassis;
		SteeringModel Steering;
		TyreModel Tyres[NumWheels];
		EngineModel Engine;
		TransmissionModel Transmission;
		BrakeModel Brakes;
		ForceFeedbackProcessor Ffb;

		WheelState Wheels[NumWheels];
		// Wheel mount models (per wheel, from Config at Initialize): mass matrix diag(M, J), stiffness K, damping C
		// (body z and rotation about x), static generalised deflection Qs.
		struct FMount
		{
			bool bActive = false;
			double M = 0.0, J = 0.0;
			double K[2][2] = {}, C[2][2] = {};
			double Qs[2] = {};
		} Mounts[NumWheels];
		Vec3 LastSpecificForceBody = {0.0, 0.0, kGravity};
		double BendAccPrev[ChassisParams::MaxFrameNodes] = {};
		void InitMounts();
		double RearAxleOmega = 0.0;
		Vec3 ExtForceW, ExtTorqueB;
		double ExtWheelTorque[NumWheels] = {};
		DrivetrainStepResult LastDrivetrain;
		double LastBrakeTorque = 0.0;
		double ColumnTorqueAvg = 0.0;
		double Time = 0.0;
		TelemetryFrame Frame;
		EngineCondition Cond;
		KartDamageState Damage;
		static constexpr int MaxContacts = 8;
		Vec3 ContactPoint[MaxContacts], ContactForce[MaxContacts];
		int NumContacts = 0;
		double WheelPeakFz[NumWheels] = {};
		// Damage inputs are low-pass filtered (a strike that bends steel lasts tens of ms; one-substep penalty-contact
		// spikes in a pile-up are numerical) and each strike / hit can add at most MaxBendPerHit.
		double WheelFzFilt[NumWheels] = {};
		double WheelStrikeDeg[NumWheels] = {};
		double ContactFilt[6] = {};        // nose (rearward), rear, left, right, left front corner, right front corner
		double EventDeg[3] = {};           // bend added in the hit in progress: left toe, right toe, axle
		bool bInitialized = false;
	};
}
