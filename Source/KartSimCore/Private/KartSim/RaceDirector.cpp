#include "KartSim/RaceDirector.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace kartsim
{
	const char* RaceDirector::TypeName(ESessionType T)
	{
		switch (T)
		{
		case ESessionType::Practice: return "Practice";
		case ESessionType::Qualifying: return "Qualifying";
		case ESessionType::Race: return "Race";
		}
		return "?";
	}

	void RaceDirector::Start(const SessionConfig& Config, double Now, double TrackLength, const std::vector<CompetitorInput>& In)
	{
		Cfg = Config;
		bActive = true;
		StartTime = Now;
		Length = TrackLength > 1.0 ? TrackLength : 1.0;
		LeaderFinishTime = -1.0;
		EndTime = 0.0;
		Tr.assign(In.size(), Track());
		Res.assign(In.size(), CompetitorResult());
		for (size_t I = 0; I < In.size(); ++I)
		{
			Tr[I].BaseCrossings = In[I].Timer->StartFinishCrossings();
			Tr[I].SeenLaps = In[I].Timer->Laps().size();
		}
		if (Cfg.Type == ESessionType::Race)
		{
			PhaseNow = ESessionPhase::Grid;
			GreenTime = Cfg.bRollingStart ? 0.0 : Now + Cfg.StartLightsTime;
		}
		else
		{
			PhaseNow = ESessionPhase::Running;
			GreenTime = Now;
		}
		OrderNow.resize(In.size());
		for (size_t I = 0; I < In.size(); ++I) { OrderNow[I] = static_cast<int>(I); }
	}

	double RaceDirector::LightsRemaining(double Now) const
	{
		if (PhaseNow != ESessionPhase::Grid || Cfg.bRollingStart) { return 0.0; }
		return GreenTime - Now > 0.0 ? GreenTime - Now : 0.0;
	}

	double RaceDirector::TimeRemaining(double Now) const
	{
		if (Cfg.Type == ESessionType::Race || Cfg.Duration <= 0.0) { return 0.0; }
		const double R = GreenTime + Cfg.Duration - Now;
		return R > 0.0 ? R : 0.0;
	}

	double RaceDirector::Elapsed(double Now) const
	{
		if (PhaseNow == ESessionPhase::Grid) { return 0.0; }
		if (Cfg.Type == ESessionType::Race && LeaderFinishTime >= 0.0) { return LeaderFinishTime - GreenTime; }
		return Now - GreenTime;
	}

	int RaceDirector::LeaderLaps() const
	{
		return OrderNow.empty() ? 0 : Res[OrderNow[0]].Laps;
	}

	void RaceDirector::Update(double Now, const std::vector<CompetitorInput>& In)
	{
		if (!bActive || PhaseNow == ESessionPhase::Finished || In.size() != Res.size()) { return; }
		const size_t N = In.size();
		const bool bRace = Cfg.Type == ESessionType::Race;

		if (PhaseNow == ESessionPhase::Grid)
		{
			// Before green: running order = position on the track (grid / formation order).
			for (size_t I = 0; I < N; ++I) { Res[I].Progress = In[I].LapDistance - Length; }
			Classify();
			if (Cfg.bRollingStart)
			{
				int First = -1;
				for (size_t I = 0; I < N; ++I)
				{
					if (In[I].Timer->StartFinishCrossings() > Tr[I].BaseCrossings &&
					    (First < 0 || In[I].Timer->LastStartFinishTime() < In[First].Timer->LastStartFinishTime()))
					{
						First = static_cast<int>(I);
					}
				}
				if (First < 0) { return; }
				GreenTime = In[First].Timer->LastStartFinishTime();
			}
			else if (Now < GreenTime)
			{
				return;
			}
			PhaseNow = ESessionPhase::Running;
			for (size_t I = 0; I < N; ++I)
			{
				const LapTimer& T = *In[I].Timer;
				// Rolling: karts that crossed at/after green started then; their crossing is the start.
				const bool bCrossedAtGreen = Cfg.bRollingStart && T.StartFinishCrossings() > Tr[I].BaseCrossings;
				Res[I].bStarted = !Cfg.bRollingStart || bCrossedAtGreen;
				Tr[I].BaseCrossings = T.StartFinishCrossings();
				Tr[I].SeenLaps = T.Laps().size();
			}
		}

		for (size_t I = 0; I < N; ++I)
		{
			const LapTimer& T = *In[I].Timer;
			CompetitorResult& R = Res[I];
			if (R.bFinished) { continue; }
			// Lap records (practice/qualifying laps; best/last/invalid for every session).
			while (Tr[I].SeenLaps < T.Laps().size())
			{
				const LapRecord& L = T.Laps()[Tr[I].SeenLaps++];
				if (!bRace) { ++R.Laps; }
				R.LastLap = L.LapTime;
				if (!L.bValid) { ++R.InvalidLaps; }
				else if (R.BestLap == 0.0 || L.LapTime < R.BestLap) { R.BestLap = L.LapTime; }
			}
			if (bRace)
			{
				if (!R.bStarted && T.StartFinishCrossings() > Tr[I].BaseCrossings)
				{
					R.bStarted = true; // rolling start: this crossing is the start
					Tr[I].BaseCrossings = T.StartFinishCrossings();
				}
				if (R.bStarted) { R.Laps = T.StartFinishCrossings() - Tr[I].BaseCrossings; }
				R.Progress = R.bStarted ? R.Laps * Length + In[I].LapDistance : In[I].LapDistance - Length;
				// Chequered flag: the leader at the race distance, everyone else at the next crossing.
				const bool bLeaderDone = LeaderFinishTime >= 0.0;
				const bool bCrossedSinceFlag = Tr[I].FlagCrossings >= 0 && T.StartFinishCrossings() > Tr[I].FlagCrossings;
				if ((R.bStarted && R.Laps >= Cfg.RaceLaps) || (bLeaderDone && bCrossedSinceFlag))
				{
					R.bFinished = true;
					R.FinishTime = T.LastStartFinishTime() - GreenTime;
					if (!bLeaderDone)
					{
						LeaderFinishTime = T.LastStartFinishTime();
						PhaseNow = ESessionPhase::Chequered;
						for (size_t J = 0; J < N; ++J) { Tr[J].FlagCrossings = In[J].Timer->StartFinishCrossings(); }
					}
				}
			}
			else if (PhaseNow == ESessionPhase::Chequered && T.StartFinishCrossings() > Tr[I].FlagCrossings)
			{
				R.bFinished = true; // lap in progress at the flag completed
			}
		}

		if (!bRace && Cfg.Duration > 0.0 && PhaseNow == ESessionPhase::Running && Now >= GreenTime + Cfg.Duration)
		{
			PhaseNow = ESessionPhase::Chequered;
			EndTime = Now;
			for (size_t J = 0; J < N; ++J) { Tr[J].FlagCrossings = In[J].Timer->StartFinishCrossings(); }
		}
		Classify();

		bool bAll = true;
		for (const CompetitorResult& R : Res) { bAll = bAll && R.bFinished; }
		const double FlagTime = bRace ? LeaderFinishTime : EndTime;
		if (PhaseNow == ESessionPhase::Chequered && (bAll || (FlagTime >= 0.0 && Now > FlagTime + 90.0)))
		{
			PhaseNow = ESessionPhase::Finished;
		}
	}

	void RaceDirector::Classify()
	{
		const size_t N = Res.size();
		std::vector<int> O(N);
		for (size_t I = 0; I < N; ++I) { O[I] = static_cast<int>(I); }
		if (Cfg.Type == ESessionType::Race)
		{
			std::stable_sort(O.begin(), O.end(), [&](int A, int B)
			{
				const CompetitorResult &Ra = Res[A], &Rb = Res[B];
				if (Ra.bFinished != Rb.bFinished)
				{
					// A finisher is ahead of a runner unless the runner has more laps (cannot happen).
					return Ra.bFinished ? Ra.Laps >= Rb.Laps : Ra.Laps > Rb.Laps;
				}
				if (Ra.bFinished) { return Ra.Laps != Rb.Laps ? Ra.Laps > Rb.Laps : Ra.FinishTime < Rb.FinishTime; }
				return Ra.Progress > Rb.Progress;
			});
		}
		else
		{
			std::stable_sort(O.begin(), O.end(), [&](int A, int B)
			{
				const double Ba = Res[A].BestLap > 0.0 ? Res[A].BestLap : 1e9, Bb = Res[B].BestLap > 0.0 ? Res[B].BestLap : 1e9;
				return Ba != Bb ? Ba < Bb : Res[A].Laps > Res[B].Laps;
			});
		}
		OrderNow = O;
		const CompetitorResult& Lead = Res[O[0]];
		// Race gap estimate while running: distance behind at the leader's average speed so far.
		const double LeadSpeed = (Lead.Progress > 0.0 && Lead.Laps > 0 && Lead.LastLap > 0.0) ? Length / Lead.LastLap : 20.0;
		for (size_t P = 0; P < N; ++P)
		{
			CompetitorResult& R = Res[O[P]];
			R.Position = static_cast<int>(P) + 1;
			if (Cfg.Type == ESessionType::Race)
			{
				// Finished karts: laps short of the winner; running karts: whole laps of distance behind.
				const int Down = (R.bFinished && Lead.bFinished) ? Lead.Laps - R.Laps : static_cast<int>(std::floor((Lead.Progress - R.Progress) / Length));
				R.LapsDown = Down > 0 ? Down : 0;
				if (Lead.bFinished && R.bFinished && R.Laps == Lead.Laps) { R.Gap = R.FinishTime - Lead.FinishTime; }
				else { R.Gap = (Lead.Progress - R.Progress) / LeadSpeed; }
			}
			else
			{
				R.LapsDown = 0;
				R.Gap = (R.BestLap > 0.0 && Lead.BestLap > 0.0) ? R.BestLap - Lead.BestLap : 0.0;
			}
		}
	}

	std::string RaceDirector::Summary(int C) const
	{
		if (C < 0 || C >= static_cast<int>(Res.size())) { return std::string(); }
		const CompetitorResult& R = Res[C];
		char Buf[256];
		if (Cfg.Type == ESessionType::Race)
		{
			std::snprintf(Buf, sizeof(Buf), "Race %d laps: P%d of %d, %d laps, %.3f s, fastest lap %.3f s, track-limit laps %d", Cfg.RaceLaps, R.Position,
			              static_cast<int>(Res.size()), R.Laps, R.FinishTime, R.BestLap, R.InvalidLaps);
		}
		else
		{
			std::snprintf(Buf, sizeof(Buf), "%s: P%d of %d, %d laps, best %.3f s, invalid %d", TypeName(Cfg.Type), R.Position, static_cast<int>(Res.size()), R.Laps,
			              R.BestLap, R.InvalidLaps);
		}
		return Buf;
	}
}
