// Chassis: 6-DOF rigid body (origin at CoG) plus a torsional chain describing frame flex.
//
// The frame is a chain of N stations ("nodes") along the wheelbase: node 0 at the front axle, node
// N-1 at the rear axle, intermediate nodes where heavy items sit (driver's legs, seat/torso). Each
// node carries the roll inertia J_i of what is mounted there; consecutive nodes are joined by uniform
// frame segments (k_seg = GJ / length, GJ = k_frame * wheelbase). The rigid body's roll is the
// inertia-weighted mean of the node rolls; each node's twist phi_i is relative to it, with
// sum(J_i * phi_i) = 0 preserved exactly (internal moments cancel). No suspension: compliance is
// tyre radial stiffness + frame torsion. Docs/PhysicsModel.md section 2.
#pragma once

#include "KartSim/KartConfig.h"
#include "KartSim/SteeringModel.h"

namespace kartsim
{
	struct ChassisState
	{
		Vec3 Position;          // world, CoG
		Quat Orientation;       // body -> world
		Vec3 Velocity;          // world
		Vec3 AngularVelocity;   // body frame
		double NodeTwist[ChassisParams::MaxFrameNodes] = {};     // rad, relative to the rigid-body roll
		double NodeTwistRate[ChassisParams::MaxFrameNodes] = {}; // rad/s
		// Vertical bending (body z, m): deviation of each node from its static (loaded) bent shape, relative to the
		// rigid body's heave and pitch (sum(m_i w_i) = sum(m_i x_i w_i) = 0).
		double NodeBend[ChassisParams::MaxFrameNodes] = {};
		double NodeBendRate[ChassisParams::MaxFrameNodes] = {};
		// Convenience copies for telemetry: front-axle and rear-axle node twist.
		double Twist = 0.0;
		double RearTwist = 0.0;
	};

	class KARTSIM_API ChassisModel
	{
	public:
		void Init(const KartConfig& InConfig);

		int NumNodes() const { return Config->Chassis.NumFrameNodes; }
		int FrontNode() const { return 0; }
		int RearNode() const { return Config->Chassis.NumFrameNodes - 1; }

		// Static wheel centres in body frame (front at zero steer, before twist).
		const Vec3& StaticWheelCentre(int Wheel) const { return WheelCentres[Wheel]; }
		const Vec3& TwistAxisPoint(int Node) const { return NodePoints[Node]; }
		double DistCogToFrontAxle() const { return A; }
		double DistCogToRearAxle() const { return B; }

		// Apply a node's twist rotation to a point / direction (body frame).
		Vec3 TwistPointBody(const Vec3& P, int Node) const;
		Vec3 TwistDirBody(const Vec3& D, int Node) const;
		// Extra velocity (body frame) of a point attached to a node due to its twist rate.
		Vec3 TwistVelocityBody(const Vec3& TwistedPoint, int Node) const;

		Vec3 BodyToWorldPoint(const Vec3& P) const { return State.Position + State.Orientation.Rotate(P); }
		Vec3 BodyToWorldDir(const Vec3& D) const { return State.Orientation.Rotate(D); }
		Vec3 WorldToBodyDir(const Vec3& D) const { return State.Orientation.Unrotate(D); }
		// World velocity of a point fixed to the rigid body (body-frame coordinates).
		Vec3 PointVelocityWorld(const Vec3& BodyPoint) const
		{
			return State.Velocity + State.Orientation.Rotate(State.AngularVelocity.Cross(BodyPoint));
		}

		// Semi-implicit Euler step. ForceWorld at CoG; TorqueBody: TOTAL moment about the CoG (body
		// frame, all tyre forces included). NodeMoments: x-moment of the tyre forces attached to each
		// node about that node's axis point; the remainder of TorqueBody.X is shared by inertia.
		// NodeVertical: body-z force the wheel mounts put into each node (bending; nullptr = none).
		void Integrate(const Vec3& ForceWorld, const Vec3& TorqueBody, const double NodeMoments[], const double NodeVertical[], double Dt);

		// Vertical frame bending.
		bool HasBending() const { return bBending; }
		// Body-z offset / rate of a node from bending (0 without bending).
		double NodeBendOffset(int Node) const { return bBending ? State.NodeBend[Node] : 0.0; }
		double NodeBendVelocity(int Node) const { return bBending ? State.NodeBendRate[Node] : 0.0; }
		// Bent shape under the static load (total deflection = static + State.NodeBend), m.
		double NodeStaticBend(int Node) const { return BendStatic[Node]; }
		// Mass of a node that takes part in bending (frame node mass minus the wheel assemblies at the axles).
		double BendingMass(int Node) const { return BendMass[Node]; }
		double BendingElasticEnergy() const;
		double BendingKineticEnergy() const;

		// Elastic energy stored in the frame (tests/diagnostics).
		double FrameElasticEnergy() const;
		double FrameTwistKineticEnergy() const;
		double SegmentStiffness(int Segment) const { return SegK[Segment]; }

		ChassisState State;

	private:
		const KartConfig* Config = nullptr;
		Vec3 WheelCentres[NumWheels];
		Vec3 NodePoints[ChassisParams::MaxFrameNodes];
		double SegK[ChassisParams::MaxFrameNodes] = {};
		double SegC[ChassisParams::MaxFrameNodes] = {};
		double A = 0.6, B = 0.44;

		// Bending chain: hinge springs at the interior nodes.
		bool bBending = false;
		double BendMass[ChassisParams::MaxFrameNodes] = {};
		double BendX[ChassisParams::MaxFrameNodes] = {};      // body x of the nodes
		double HingeK[ChassisParams::MaxFrameNodes] = {};     // N m/rad
		double HingeC[ChassisParams::MaxFrameNodes] = {};     // N m s/rad
		double BendStatic[ChassisParams::MaxFrameNodes] = {};
		void BendingForces(const double W[], const double Wd[], double F[]) const;
		void RemoveRigidModes(double U[]) const;
		void SolveStaticBending(const double StaticNodeForce[]);
	};
}
