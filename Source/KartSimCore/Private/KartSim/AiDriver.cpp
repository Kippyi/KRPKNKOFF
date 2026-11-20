#include "KartSim/AiDriver.h"

#include <cmath>

namespace kartsim
{
	namespace
	{
		constexpr double KartLength = 1.82;   // CIK maximum overall length
		constexpr double LaneHalf = 0.75;     // |lateral difference| below 2 x this = same lane
		constexpr double SideClearance = 1.60; // CoG-to-CoG lateral spacing when side by side (1.40 m wide + gap)
	}

	const char* AiDriver::StateName(EAiState S)
	{
		switch (S)
		{
		case EAiState::Racing: return "racing";
		case EAiState::Following: return "following";
		case EAiState::Overtaking: return "overtaking";
		case EAiState::Formation: return "formation";
		}
		return "?";
	}

	double AiDriver::NextRandom()
	{
		// xorshift32: deterministic per driver (tests and replays reproduce).
		Rng ^= Rng << 13;
		Rng ^= Rng >> 17;
		Rng ^= Rng << 5;
		return (Rng / 4294967295.0) * 2.0 - 1.0;
	}

	void AiDriver::Configure(const RacingLine& InLine)
	{
		Line = InLine;
		Pilot.SetPath(Line.Points);
		Path.Build(Line.Points);
	}

	void AiDriver::Reset(const KartSimulation& Sim)
	{
		Pilot.Reset(Sim);
		Rng = Skill.Seed ? Skill.Seed : 1;
		LatGNow = Skill.LatG * (1.0 + Skill.Consistency * NextRandom());
		Offset = TargetOffset = 0.0;
		FollowTime = OvertakeTime = 0.0;
		OvertakeSide = 0;
		OvertakeTarget = -1;
		YieldTarget = -1;
		YieldTime = 0.0;
		PrevS = -1.0;
		Hint = Pilot.PathIndex();
		// Start from the kart's actual offset (grid slots are off the line).
		double S = 0.0, Lat = 0.0;
		if (Path.Project(Sim.GetChassis().Position, S, Lat, Hint)) { Offset = TargetOffset = Lat; }
	}

	void AiDriver::Drive(const KartSimulation& Self, const AiTraffic* Others, int NumOthers, double Dt, KartInputs& In)
	{
		if (!Path.IsBuilt()) { return; }
		const double L = Path.Length();
		const ChassisState& St = Self.GetChassis();
		const double V = Self.ForwardSpeed();
		double SSelf = 0.0, LatSelf = 0.0;
		Path.Project(St.Position, SSelf, LatSelf, Hint);
		const size_t I = Hint;

		// Lap-to-lap variation of the grip target (resampled when crossing the line's origin).
		if (PrevS >= 0.0 && PrevS > 0.75 * L && SSelf < 0.25 * L) { LatGNow = Skill.LatG * (1.0 + Skill.Consistency * NextRandom()); }
		PrevS = SSelf;

		// --- Perceive traffic relative to the racing line ---
		const Vec3 Fwd = St.Orientation.Rotate(Vec3(1, 0, 0));
		int Blocker = -1;
		double BlockGap = 1e9, BlockSpeed = 0.0, BlockLat = 0.0;
		double MinLeft = 1e9, MaxRight = -1e9; // lateral limits imposed by karts alongside
		double TargetDs = -1e9, TargetLat = 0.0, TargetSpeed = 0.0; // the kart being overtaken
		double YieldDs = -1e9, YieldSpeed = 0.0;                    // the kart we are yielding to
		for (int K = 0; K < NumOthers; ++K)
		{
			double So = 0.0, Lo = 0.0;
			size_t H = Hint;
			if (!Path.Project(Others[K].Position, So, Lo, H)) { continue; }
			double Ds = So - SSelf;
			if (Ds > 0.5 * L) { Ds -= L; }
			if (Ds < -0.5 * L) { Ds += L; }
			const double Vo = Others[K].Velocity.Dot(Fwd);
			if (K == OvertakeTarget) { TargetDs = Ds; TargetLat = Lo; TargetSpeed = Vo; }
			if (K == YieldTarget) { YieldDs = Ds; YieldSpeed = Vo; }
			// Alongside (overlapping lengthwise): keep a lateral clearance from it.
			if (std::fabs(Ds) < KartLength + 0.3)
			{
				if (Lo > LatSelf) { MinLeft = std::fmin(MinLeft, Lo - SideClearance); }
				else { MaxRight = std::fmax(MaxRight, Lo + SideClearance); }
				continue;
			}
			// Ahead in the lane we are steering for.
			if (Ds > 0.0 && Ds < 45.0 && std::fabs(Lo - LatSelf) < 2.0 * LaneHalf)
			{
				const double Gap = Ds - KartLength;
				if (Gap < BlockGap) { BlockGap = Gap; BlockSpeed = Vo; BlockLat = Lo; Blocker = K; }
			}
		}

		// --- Decide ---
		double SpeedCap = ExternalSpeedCap;
		// Committed to a pass: close right up (a driver dives into the slipstream / braking zone instead of holding
		// the following gap, which kept a 1.4 g kart stuck 4-9 m behind a 1.1 g one once the Rotax curve gained
		// low-rpm torque from the datasheet - the slower kart's exits got stronger).
		const double DesiredGap = OvertakeSide != 0 ? 0.6 + 0.05 * V : (1.2 - 0.6 * Skill.Aggression) + 0.20 * V; // m
		StateNow = ExternalSpeedCap < 1e8 ? EAiState::Formation : EAiState::Racing;
		if (Blocker >= 0 && BlockGap < 25.0)
		{
			// Speed matching: approach to the desired gap, never close faster than the gap allows.
			const double Cap = BlockSpeed + 0.8 * (BlockGap - DesiredGap);
			if (Cap < V + 3.0) { SpeedCap = std::fmin(SpeedCap, std::fmax(Cap, 0.0)); }
			if (BlockGap < DesiredGap + 4.0) { FollowTime += Dt; }
			if (StateNow != EAiState::Formation) { StateNow = EAiState::Following; }
		}
		else
		{
			FollowTime = 0.0;
		}

		// Room for a lateral offset: the minimum over the next ~15 m, because the Autopilot applies the
		// offset at its lookahead point and the line may swap sides before then.
		double RoomL = 1e9, RoomR = 1e9;
		{
			const size_t M = Line.Points.size();
			const double Spacing = L / static_cast<double>(M);
			const size_t Ahead = static_cast<size_t>(15.0 / Spacing) + 1;
			for (size_t K = 0; K <= Ahead; ++K)
			{
				RoomL = std::fmin(RoomL, Line.RoomLeft[(I + K) % M]);
				RoomR = std::fmin(RoomR, Line.RoomRight[(I + K) % M]);
			}
		}
		const double PatienceTime = 2.0 - 1.5 * Skill.Aggression;
		if (OvertakeSide == 0 && Blocker >= 0 && FollowTime > PatienceTime && StateNow != EAiState::Formation)
		{
			// Prefer the INSIDE of the next corner (the classic pass: alongside on the inside into the
			// braking zone); otherwise the side with more room.
			const double SpaceLeft = RoomL - BlockLat, SpaceRight = BlockLat + RoomR;
			double CurvAhead = 0.0;
			{
				const size_t M = Line.Points.size();
				const size_t N60 = static_cast<size_t>(60.0 / (L / static_cast<double>(M)));
				for (size_t K = 0; K < N60; ++K) { CurvAhead += Pilot.CurvatureAt((I + K) % M); }
			}
			const int Inside = CurvAhead > 0.0 ? 1 : -1;
			const double InsideSpace = Inside > 0 ? SpaceLeft : SpaceRight;
			if (std::fmax(SpaceLeft, SpaceRight) >= SideClearance)
			{
				OvertakeSide = InsideSpace >= SideClearance ? Inside : (SpaceLeft >= SpaceRight ? 1 : -1);
				OvertakeTarget = Blocker;
				OvertakeTime = 0.0;
				TargetDs = BlockGap + KartLength;
				TargetLat = BlockLat;
				TargetSpeed = BlockSpeed;
			}
		}
		if (OvertakeSide != 0)
		{
			OvertakeTime += Dt;
			StateNow = EAiState::Overtaking;
			// Until alongside, switch sides if the chosen one runs out of room (the line moves across the
			// track); once alongside, hold the lane beside the kart until it is clearly behind (or give up).
			if (TargetDs > KartLength)
			{
				const double SpaceLeft = RoomL - TargetLat, SpaceRight = TargetLat + RoomR;
				const double Chosen = OvertakeSide > 0 ? SpaceLeft : SpaceRight, Other = OvertakeSide > 0 ? SpaceRight : SpaceLeft;
				if (Chosen < SideClearance && Other >= SideClearance + 0.3) { OvertakeSide = -OvertakeSide; }
			}
			TargetOffset = TargetLat + OvertakeSide * SideClearance;
			// Out of road on our side before we are ahead (the line swings across, or around the outside
			// of a corner): abort and drop back behind rather than squeeze into the other kart.
			const double ChosenSpace = OvertakeSide > 0 ? RoomL - TargetLat : TargetLat + RoomR;
			if (ChosenSpace < SideClearance - 0.2 && TargetDs > -0.5 * KartLength && TargetDs < KartLength + 1.0)
			{
				YieldTarget = OvertakeTarget;
				YieldTime = 2.0;
				YieldDs = TargetDs;
				YieldSpeed = TargetSpeed;
				OvertakeSide = 0;
				OvertakeTarget = -1;
				FollowTime = 0.0;
				TargetOffset = 0.0;
			}
			const bool bPassed = TargetDs < -(KartLength + 1.0);
			const bool bLost = TargetDs > 40.0 || TargetDs < -0.5 * L + 1.0;
			if (OvertakeSide != 0 && (bPassed || bLost || OvertakeTime > 10.0))
			{
				OvertakeSide = 0;
				OvertakeTarget = -1;
			}
			else if (OvertakeSide != 0 && std::fabs(Offset - TargetLat) > 2.0 * LaneHalf && Blocker == OvertakeTarget)
			{
				// Already beside its lane: it no longer blocks us, drop its speed cap.
				SpeedCap = ExternalSpeedCap;
			}
			(void)TargetSpeed;
		}
		else
		{
			TargetOffset = 0.0; // back to the line
		}

		// Yielding: stay behind the kart we backed off from until it is clearly ahead.
		if (YieldTime > 0.0)
		{
			YieldTime -= Dt;
			if (YieldTarget >= 0 && YieldDs > -KartLength && YieldDs < KartLength + 4.0)
			{
				SpeedCap = std::fmin(SpeedCap, std::fmax(YieldSpeed - 4.0, 0.0));
			}
			if (YieldTime <= 0.0 || YieldDs > KartLength + 4.0) { YieldTarget = -1; YieldTime = 0.0; }
		}

		// Track limits, then karts alongside: never steer into a kart beside us, even if that means using
		// the kerb (up to 0.8 m beyond the room limit is still on the 1 m kerb).
		constexpr double KerbUse = 0.8;
		TargetOffset = Clamp(TargetOffset, -RoomR, RoomL);
		if (MinLeft < 1e8) { TargetOffset = std::fmin(TargetOffset, MinLeft); }
		if (MaxRight > -1e8) { TargetOffset = std::fmax(TargetOffset, MaxRight); }
		TargetOffset = Clamp(TargetOffset, -RoomR - KerbUse, RoomL + KerbUse);

		// Smooth lateral move (a driver changes line progressively).
		const double Rate = 1.5 * Dt;
		Offset += Clamp(TargetOffset - Offset, -Rate, Rate);

		Pilot.TargetLatG = LatGNow * GripScale;
		Pilot.BrakeDecelG = Skill.BrakeG;
		Pilot.LateralOffset = Offset;
		Pilot.SpeedCap = SpeedCap;
		Pilot.Drive(Self, Dt, In);
		// Following very close and still closing: brake harder than the speed loop alone would.
		if (Blocker >= 0 && BlockGap < 0.8 && V > BlockSpeed + 0.5 && std::fabs(BlockLat - LatSelf) < 2.0 * LaneHalf)
		{
			In.Throttle = 0.0;
			In.Brake = std::fmax(In.Brake, 0.35);
		}
	}
}
