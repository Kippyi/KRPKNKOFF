// Kart-to-kart contact (Milestone 6). Each kart is a box in its own ground plane: the CIK maximum
// overall length (1820 mm, front fairing to rear bumper) and width (1400 mm), centred on the wheelbase,
// from the bumper height down. Contact is a penalty spring-damper along the minimum-penetration axis
// (2-D separating-axis test on the yaw-only footprints), with regularised Coulomb friction between the
// plastic bodywork/bumpers.
//
// Stiffness, damping and friction are ASSUMPTIONS (no measured bumper data): a 5 cm overlap gives
// 7.5 kN (~4.7 g on one kart); damping ~0.4 of critical for two 160 kg karts (mostly inelastic
// contacts, as plastic bodywork and tube bumpers behave).
//
// Milestone 8, 3-D contact on top of the bumper box:
//  * WHEEL TO WHEEL: every tyre is a disc (radius, width, spinning about its axle). Two discs that overlap
//    laterally and in their wheel planes push apart along the line between the centres, and rubber-on-rubber
//    friction acts on the SURFACE velocities (chassis motion + spin). A rear tyre of the kart ahead turns UP at
//    its back while the following kart's front tyre turns DOWN at its front: the friction throws the follower's
//    front wheel up = the kart launch. The friction torques also act on the wheels' spin.
//  * TYRE ON BODY: a tyre (sphere of its radius) against the other kart's oriented body boxes (nose, floor /
//    seat, sidepods, rear bumper, driver): a kart can ride up and over another one.
//  * The bumper box force fades out as the bodies separate vertically (a kart on top of another is carried by
//    the 3-D contacts, not pushed sideways).
// Rollover: body points (helmet, shoulders, steering wheel, sidepods, nose, bumper) against the ground are in
// KartSimulation (they need the track query). All stiffness / friction values are ASSUMPTIONS.
//
// Contacts are evaluated once per outer step for every pair and applied with
// KartSimulation::SetExternalLoad (force + moment about each CoG), equal and opposite.
#pragma once

#include "KartSim/KartPhysicsCore.h"

#include <vector>

namespace kartsim
{
	struct KartFootprint
	{
		double Front = 1.0;      // m ahead of the CoG
		double Rear = 0.82;      // m behind the CoG
		double HalfWidth = 0.70; // m
		double BumperHeight = 0.20; // m above ground where contact forces act (ASSUMPTION)
		double TopHeight = 0.60;    // m above ground: vertical extent used to reject non-overlapping heights
		// Two boxes, not one (Milestone 8): the full-width MAIN box (rear bumper, rear wheels, sidepods) ends at
		// NoseStart; ahead of it only the NOSE (CIK front fairing, narrower than the front track). The front wheels
		// outside the nose are tyre discs (wheel-to-wheel contact), so they can interlock and launch.
		double NoseStart = 0.48;     // m ahead of the CoG
		double NoseHalfWidth = 0.45; // m: front bumper bars + mean fairing width (the fairing itself tapers from +-0.57 m just
		                             // ahead of the front wheels to +-0.10 m at the tip, measured on the model: 3-D boxes)
		// Behind the rear tyres only the REAR BUMPER, narrower than the outside of the rear tyres: their outer
		// edges are exposed (where a following front wheel climbs them).
		double RearTyreBack = -0.58;   // m behind the CoG (negative)
		double BumperHalfWidth = 0.60; // m (ESTIMATE)

		// CIK maximum length 1.82 m / width 1.40 m, centred on the wheelbase (VERIFIED limits; the real
		// outline is within them, so this is the conservative envelope).
		KARTSIM_API static KartFootprint FromConfig(const KartConfig& C);
	};

	struct ContactParams
	{
		double Stiffness = 1.5e5;   // N/m (ASSUMPTION)
		double Damping = 4500.0;    // N s/m (ASSUMPTION): ~0.65 of critical for two karts; plastic bodywork barely bounces
		double DampingRamp = 0.01;  // m: damping fades in over the first 1 cm of overlap (no force jump at first touch)
		double Friction = 0.35;     // plastic on plastic/steel (ASSUMPTION)
		double FrictionVelocity = 0.3; // m/s regularisation of the friction law
		double MaxForce = 40000.0;  // N clamp (numerical safety for deep overlaps after a reset)
		// 3-D contacts (ASSUMPTIONS): tyre-tyre (two carcasses in series), tyre-body (plastic / tube).
		double TyreStiffness = 9.0e4;  // N/m
		double TyreDamping = 1500.0;   // N s/m
		double TyreFriction = 1.1;     // rubber on rubber
		// Tread engagement: two tyres pressed together interlock and deform (not modelled as geometry), so the
		// friction drags part of the follower's corner mass with it, not only the light spinning wheel. Mass (kg)
		// added to the friction's effective mass. ASSUMPTION, calibrated so a tap hops and a big closing speed
		// with overlapping wheels launches (Tests: Contact_WheelToWheelLaunchesTheFollower).
		double TyreEngageMass = 12.0;
		double BodyStiffness = 1.2e5;  // N/m
		double BodyDamping = 2000.0;   // N s/m
		double BodyFriction = 0.4;     // rubber on plastic / steel
		bool bWheelContacts = true;
	};

	// Oriented body boxes in the kart body frame (origin CoG, x forward, y left, z up), for tyre contacts.
	struct KartBodyBox { Vec3 Centre; Vec3 Half; };
	KARTSIM_API int KartBodyBoxes(const KartConfig& C, KartBodyBox* Out, int Max);

	struct KartContactInfo
	{
		bool bContact = false;
		Vec3 Point;      // world
		Vec3 Normal;     // world, unit, horizontal, from A toward B
		double Depth = 0.0;
		Vec3 ForceOnB;   // world (force on A is the negative)
	};

	// Pairwise contact between two karts (does not modify them).
	// Deepest of the box pairs (main / nose of each kart).
	KARTSIM_API KartContactInfo ComputeKartContact(const KartSimulation& A, const KartFootprint& FA, const KartSimulation& B, const KartFootprint& FB,
	                                               const ContactParams& P);

	// Evaluates all pairs and sets every kart's external load (clearing it when untouched).
	// Returns the number of touching pairs.
	// Dt: the outer step the loads are held for (limits the friction so it cannot reverse a slip within a step).
	KARTSIM_API int ApplyKartContacts(const std::vector<KartSimulation*>& Karts, const std::vector<KartFootprint>& Feet, const ContactParams& P,
	                                  std::vector<KartContactInfo>* OutContacts = nullptr, double Dt = 1.0 / 240.0);
}
