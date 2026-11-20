#include "KartSim/TwoStrokeAcoustics.h"

#include <algorithm>
#include <cmath>

namespace kartsim
{
	namespace
	{
		constexpr double Pi = 3.14159265358979;
		constexpr double P0 = 101325.0;   // Pa
		constexpr double T0 = 300.0;      // K, ambient
		constexpr double Rg = 287.0;      // J/(kg K)
		constexpr double GammaHot = 1.35; // cylinder / exhaust gas
		constexpr double GammaAir = 1.40; // crankcase / intake
		constexpr double QFresh = 2.7e6;  // J released per kg of fresh mixture (LHV 44 MJ/kg fuel / 14 kg mixture x 0.85)
		constexpr double WallK = 450.0;   // cylinder wall temperature
		constexpr double WallH = 500.0;   // W/(m^2 K), heat transfer (ESTIMATE)
		constexpr double Cd = 0.75;       // port discharge coefficient
		constexpr int PsiN = 1024;

		double Clampd(double X, double A, double B) { return X < A ? A : (X > B ? B : X); }
	}

	void TwoStrokeGeometry::PipeFromTuning()
	{
		// Mean wave speed over the tuned part of the pipe with the BuildPipe temperature profile at full load
		// (port ~800 K, cooling along the pipe -> ~640 K mean over the first 0.7 m).
		const double C = std::sqrt(GammaHot * Rg * 640.0);
		const double TDeg = 1.0 / (6.0 * TunedRpm); // s per crank degree
		const double Ed = 360.0 - 2.0 * EpoDeg;     // exhaust open duration
		// The blowdown pulse peaks ~12 deg after the port opens. Its suction reflection from the diffuser should be at
		// the port while the transfers flow (~20 deg before BDC); the plugging reflection from the baffle cone just
		// before the port closes (~10 deg before EPC). Lengths are port -> middle of each cone.
		const double Ld = C * (160.0 - EpoDeg - 12.0) * TDeg * 0.5;
		const double Lt = C * (Ed - 22.0) * TDeg * 0.5;
		if (HeaderL <= 0.0f) { HeaderL = static_cast<float>(std::max(0.08, 0.45 * Ld)); }
		if (DiffuserL <= 0.0f) { DiffuserL = static_cast<float>(std::max(0.10, 2.0 * (Ld - HeaderL))); }
		const double Rem = std::max(0.10, Lt - HeaderL - DiffuserL);
		if (BellyL <= 0.0f) { BellyL = static_cast<float>(0.35 * Rem); }
		if (BaffleL <= 0.0f) { BaffleL = static_cast<float>(1.30 * Rem); }
	}

	float TwoStrokeAcoustics::Noise()
	{
		Rng ^= Rng << 13; Rng ^= Rng >> 17; Rng ^= Rng << 5;
		return static_cast<float>(Rng) / 2147483648.0f - 1.0f;
	}

	double TwoStrokeAcoustics::PistonX(double ThetaDeg) const
	{
		const double R = 0.5 * Geo.Stroke, L = Geo.ConRod, T = ThetaDeg * Pi / 180.0;
		const double S = std::sin(T);
		return R * (1.0 - std::cos(T)) + L - std::sqrt(std::max(L * L - R * R * S * S, 1e-12));
	}

	double TwoStrokeAcoustics::PistonXFast(double ThetaDeg) const
	{
		double T = std::fmod(ThetaDeg, 360.0);
		if (T < 0.0) { T += 360.0; }
		const double X = T * 4.0;
		const int I = static_cast<int>(X);
		const double W = X - I;
		return XTab[I] + (XTab[I + 1] - XTab[I]) * W;
	}

	double TwoStrokeAcoustics::FlowMass(double A, double PUp, double TUp, double PDn, double Gamma) const
	{
		if (A <= 0.0 || PUp <= PDn) { return 0.0; }
		const std::vector<float>& Tab = PsiTab[Gamma < 1.375 ? 0 : 1];
		const double X = Clampd(PDn / PUp, 0.0, 1.0) * (PsiN - 1);
		const int I = std::min(static_cast<int>(X), PsiN - 2);
		const double Psi = Tab[I] + (Tab[I + 1] - Tab[I]) * (X - I);
		return Cd * A * PUp * Psi / std::sqrt(Rg * std::max(TUp, 100.0));
	}

	void TwoStrokeAcoustics::Init(const TwoStrokeGeometry& G, float SampleRateHz)
	{
		Geo = G;
		Geo.Substeps = std::max(1, std::min(8, Geo.Substeps));
		Geo.PipeFromTuning();
		Fs = SampleRateHz;
		Ap = Pi * 0.25 * Geo.Bore * Geo.Bore;
		Vd = Ap * Geo.Stroke;
		Vc = Vd / (Geo.CompressionRatio - 1.0);
		VccMin = Vd / (Geo.CrankcaseRatio - 1.0);
		XTab.resize(1441);
		for (int I = 0; I <= 1440; ++I) { XTab[I] = PistonX(I * 0.25); }
		Xe = PistonX(Geo.EpoDeg);
		XeClosed = Geo.EpoValveClosedDeg > 0.0f ? PistonX(Geo.EpoValveClosedDeg) : Xe;
		Xt = PistonX(Geo.TpoDeg);
		AThroat = Pi * 0.25 * Geo.CarbDiameter * Geo.CarbDiameter;
		for (int K2 = 0; K2 < 2; ++K2)
		{
			const double Gm = K2 == 0 ? GammaHot : GammaAir;
			const double Crit = std::pow(2.0 / (Gm + 1.0), Gm / (Gm - 1.0));
			PsiTab[K2].resize(PsiN);
			for (int I = 0; I < PsiN; ++I)
			{
				const double Pr = std::max(static_cast<double>(I) / (PsiN - 1), Crit);
				PsiTab[K2][I] = static_cast<float>(std::sqrt(std::max(0.0, 2.0 * Gm / (Gm - 1.0) * (std::pow(Pr, 2.0 / Gm) - std::pow(Pr, (Gm + 1.0) / Gm)))));
			}
		}
		Theta = 0.0;
		Tc = T0; Fc = 1.0;
		Mc = P0 * (Vc + Ap * PistonX(0.0)) / (Rg * Tc);
		Tcc = T0;
		Mcc = P0 * (VccMin + Ap * Geo.Stroke) / (Rg * Tcc);
		Pab = Usn = Ps = Ut = PrevUt = PrevUsn = 0.0;
		bBurning = bSparked = false;
		PipeT = 500.0;
		PipeC = 0.0;
		F.clear(); B.clear();
		BuildPipe(PipeT);
		PortP = P0;
		Cur = Last = TwoStrokeCycleStats();
	}

	double TwoStrokeAcoustics::CylinderBar() const { return Mc * Rg * Tc / (Vc + Ap * PistonX(Theta)) / 1e5; }
	double TwoStrokeAcoustics::CrankcaseBar() const { return Mcc * Rg * Tcc / (VccMin + Ap * (Geo.Stroke - PistonX(Theta))) / 1e5; }

	void TwoStrokeAcoustics::BuildPipe(double PortT)
	{
		// Gas temperature along the pipe: hottest at the port, cooling towards the outlet (ESTIMATE: e-folding 0.5 m,
		// outlet at ~45 % of the port's excess over ambient). One section per sample of LOCAL wave travel.
		const double L[6] = {Geo.HeaderL, Geo.DiffuserL, Geo.BellyL, Geo.BaffleL, Geo.StingerL, Geo.SilencerL};
		const double Total = L[0] + L[1] + L[2] + L[3] + L[4] + L[5];
		const double TEnd = T0 + 0.45 * (PortT - T0);
		auto TempAt = [&](double X) { return TEnd + (PortT - TEnd) * std::exp(-X / 0.5); };
		auto DiaAt = [&](double X)
		{
			if (X < L[0]) { return static_cast<double>(Geo.HeaderD); }
			X -= L[0];
			if (X < L[1]) { return Geo.HeaderD + (Geo.BellyD - Geo.HeaderD) * X / L[1]; }
			X -= L[1];
			if (X < L[2]) { return static_cast<double>(Geo.BellyD); }
			X -= L[2];
			if (X < L[3]) { return Geo.BellyD + (Geo.StingerD - Geo.BellyD) * X / L[3]; }
			X -= L[3];
			if (X < L[4]) { return static_cast<double>(Geo.StingerD); }
			return static_cast<double>(Geo.TailD);
		};
		std::vector<double> Xs;
		for (double X = 0.0; X < Total && Xs.size() < 4000;) { Xs.push_back(X); X += std::sqrt(GammaHot * Rg * TempAt(X)) / Fs; }
		const int N = std::max(8, static_cast<int>(Xs.size()));
		std::vector<float> OldF = F, OldB = B;
		Area.assign(N, 0.0f);
		Csec.assign(N, 0.0f);
		Js = N - 2;
		for (int I = 0; I < N; ++I)
		{
			const double X = I < static_cast<int>(Xs.size()) ? Xs[I] : Total;
			const double Xm = X + 0.5 * std::sqrt(GammaHot * Rg * TempAt(X)) / Fs;
			const double D = DiaAt(Xm);
			Area[I] = static_cast<float>(Pi * 0.25 * D * D);
			Csec[I] = static_cast<float>(std::sqrt(GammaHot * Rg * TempAt(Xm)));
			if (Xm >= Total - L[5] && Js == N - 2 && I > 0) { Js = I; }
		}
		// Pressure reflection at each junction from the impedance step Z = rho c / A ~ 1 / (c A) (area AND temperature).
		K.assign(N, 0.0f);
		for (int I = 0; I + 1 < N; ++I)
		{
			const double Z1 = 1.0 / (static_cast<double>(Csec[I]) * Area[I]), Z2 = 1.0 / (static_cast<double>(Csec[I + 1]) * Area[I + 1]);
			K[I] = static_cast<float>((Z2 - Z1) / (Z2 + Z1));
		}
		// Resample the travelling waves onto the new grid (the gas temperature changes slowly).
		F.assign(N, 0.0f); B.assign(N, 0.0f);
		if (!OldF.empty())
		{
			const int No = static_cast<int>(OldF.size());
			for (int I = 0; I < N; ++I)
			{
				const double X = (I + 0.5) / N * No - 0.5;
				const int J = static_cast<int>(Clampd(std::floor(X), 0.0, No - 2.0));
				const double W = Clampd(X - J, 0.0, 1.0);
				F[I] = static_cast<float>(OldF[J] * (1.0 - W) + OldF[J + 1] * W);
				B[I] = static_cast<float>(OldB[J] * (1.0 - W) + OldB[J + 1] * W);
			}
		}
		NF.assign(N, 0.0f); NB.assign(N, 0.0f);
		PipeLen = Total;
		BuiltT = PortT;
		PackA = static_cast<float>(1.0 - std::exp(-2.0 * Pi * Geo.PackingHz / Fs));
		PipeC = Csec[0];
		PipeRho = P0 / (Rg * PortT);
		++RebuildCount;
	}

	void TwoStrokeAcoustics::CylinderSubstep(double Dt, double Th, double DTh, const TwoStrokeInput& In, double B0, double& PortMdotSum)
	{
		const double X0 = PistonXFast(Th), X1 = PistonXFast(Th + DTh);
		const double V0 = Vc + Ap * X0, V1 = Vc + Ap * X1;
		const double Vcc0 = VccMin + Ap * (Geo.Stroke - X0), Vcc1 = VccMin + Ap * (Geo.Stroke - X1);
		const double Pc = Mc * Rg * Tc / V0;
		const double Pcc = Mcc * Rg * Tcc / Vcc0;
		const double CvH = Rg / (GammaHot - 1.0), CpH = CvH * GammaHot;
		const double CvA = Rg / (GammaAir - 1.0), CpA = CvA * GammaAir;

		// Ignition: spark crossing (crank angle in [0, 360)).
		const double Spark = 360.0 - Geo.SparkDeg + CycleJitter;
		if (!bSparked && Th < Spark && Th + DTh >= Spark)
		{
			bSparked = true;
			const double Fresh = Fc * Mc;
			const bool bFire = 0.5f * (Noise() + 1.0f) < In.FireProb && Fresh > 2e-6;
			if (bFire)
			{
				QCycle = Fresh * QFresh * CycleScale;
				BurnDur = Geo.BurnDeg * (1.0 + 0.15 * Noise());
				BurnXb = 0.0;
				BurnStart = 0.0;
				bBurning = true;
				Cur.TrappedFreshMg = Fresh * 1e6;
			}
		}
		double DQ = 0.0;
		if (bBurning)
		{
			BurnStart += DTh; // degrees since the spark
			const double U = std::max(0.0, BurnStart / BurnDur);
			const double Xb = 1.0 - std::exp(-5.0 * U * U * U); // Wiebe a = 5, m = 2
			const double DXb = Xb - BurnXb;
			DQ = QCycle * DXb;
			if (BurnXb < 0.999) { Fc *= std::max(0.0, 1.0 - DXb / (1.0 - BurnXb)); }
			BurnXb = Xb;
			if (U > 2.0) { bBurning = false; Fc = 0.0; }
		}

		// Exhaust port: flow solved against the pipe (port pressure = p0 + incoming wave x 2 + c/A x mdot).
		const double XeNow = XeClosed + (Xe - XeClosed) * Clampd(In.ValveOpen, 0.0, 1.0);
		const double Ae = Geo.ExhaustWidth * Geo.Bore * std::max(0.0, X0 - XeNow);
		const double Kp = PipeC / Area[0];
		double Mex = 0.0;
		if (Ae > 0.0)
		{
			auto Flow = [&](double M)
			{
				const double Pp = std::max(0.25 * P0, P0 + 2.0 * B0 + Kp * M);
				return Pc >= Pp ? FlowMass(Ae, Pc, Tc, Pp, GammaHot) : -FlowMass(Ae, Pp, PipeT, Pc, GammaHot);
			};
			const double M0 = Flow(0.0);
			double Lo = std::min(0.0, M0), Hi = std::max(0.0, M0);
			for (int It = 0; It < 8; ++It)
			{
				const double Mid = 0.5 * (Lo + Hi);
				if (Mid - Flow(Mid) > 0.0) { Hi = Mid; } else { Lo = Mid; }
			}
			Mex = 0.5 * (Lo + Hi);
			PortP = std::max(0.25 * P0, P0 + 2.0 * B0 + Kp * Mex);
		}
		else { PortP = P0 + 2.0 * B0; }
		// Transfers (crankcase <-> cylinder).
		const double At = Geo.TransferWidth * Geo.Bore * std::max(0.0, X0 - Xt);
		const double Mtr = Pcc >= Pc ? FlowMass(At, Pcc, Tcc, Pc, GammaAir) : -FlowMass(At, Pc, Tc, Pcc, GammaHot);
		// Reed + carburettor (airbox -> crankcase, one way).
		const double Pbox = P0 + Pab;
		double Mreed = 0.0;
		if (Pbox > Pcc)
		{
			const double Ar = Geo.ReedArea * Clampd((Pbox - Pcc) / 12000.0, 0.02, 1.0);
			const double Ath = AThroat * (0.035 + 0.965 * Clampd(In.Throttle, 0.0, 1.0));
			const double Aeff = 1.0 / std::sqrt(1.0 / (Ar * Ar) + 1.0 / (Ath * Ath));
			Mreed = FlowMass(Aeff, Pbox, T0, Pcc, GammaAir);
		}

		const double DmEx = Mex * Dt, DmTr = Mtr * Dt, DmReed = Mreed * Dt;
		// Cylinder energy / mass / fresh fraction.
		const double Qloss = WallH * (2.0 * Ap + Pi * Geo.Bore * X0) * (Tc - WallK) * Dt;
		double U = Mc * CvH * Tc;
		U += -Pc * (V1 - V0) + DQ - Qloss;
		Cur.WorkJ += Pc * (V1 - V0);
		U -= DmEx > 0.0 ? CpH * Tc * DmEx : CpH * PipeT * DmEx;
		U += DmTr > 0.0 ? CpA * Tcc * DmTr : CpH * Tc * DmTr;
		// Scavenging between perfect mixing and displacement: the gas leaving is fresh with probability Fc^2
		// (fresh charge short-circuits less while the cylinder is still mostly burnt gas). ESTIMATE.
		double Fresh = Fc * Mc + (DmTr > 0.0 ? DmTr : Fc * DmTr) - (DmEx > 0.0 ? Fc * Fc * DmEx : 0.0);
		Mc = std::max(1e-7, Mc + DmTr - DmEx);
		Tc = Clampd(U / (Mc * CvH), 200.0, 3500.0);
		Fc = Clampd(Fresh / Mc, 0.0, 1.0);
		// Crankcase.
		double Ucc = Mcc * CvA * Tcc;
		Ucc += -Pcc * (Vcc1 - Vcc0) + CpA * T0 * DmReed - (DmTr > 0.0 ? CpA * Tcc * DmTr : CpH * Tc * DmTr);
		Mcc = std::max(1e-7, Mcc + DmReed - DmTr);
		Tcc = Clampd(Ucc / (Mcc * CvA), 200.0, 900.0);

		PortMdotSum += Mex / Geo.Substeps;
		ReedMassCycle += DmReed;
		Pab_Flow += Mreed / Geo.Substeps;
		if (Mex > 0.0) { OutTempAcc += Tc * Mex; OutMassAcc += Mex; }
		Cur.PeakCylinderBar = std::max(Cur.PeakCylinderBar, Pc / 1e5);
		Cur.PortPeakBar = std::max(Cur.PortPeakBar, PortP / 1e5);
		Cur.PortMinBar = Cur.PortMinBar <= 0.0 ? PortP / 1e5 : std::min(Cur.PortMinBar, PortP / 1e5);
	}

	TwoStrokeOutput TwoStrokeAcoustics::Step(const TwoStrokeInput& In)
	{
		TwoStrokeOutput Out;
		const double Dt = 1.0 / Fs;
		const double Rpm = std::max(0.0, static_cast<double>(In.Rpm));
		// Wave arriving at the port (port-end losses: one-pole ~6 kHz).
		PortLp += (B[0] - PortLp) * 0.55f;
		const double B0 = PortLp;
		double Mdot = 0.0;
		Pab_Flow = 0.0;
		if (Rpm > 30.0)
		{
			const double DTh = 6.0 * Rpm * Dt / Geo.Substeps;
			for (int S = 0; S < Geo.Substeps; ++S)
			{
				CylinderSubstep(Dt / Geo.Substeps, Theta, DTh, In, B0, Mdot);
				Theta += DTh;
				if (Theta >= 360.0)
				{
					Theta -= 360.0;
					// Cycle boundary (TDC): stats, next cycle's randomness.
					Cur.DeliveryRatio = ReedMassCycle / (Vd * P0 / (Rg * T0));
					Cur.PipeGasK = PipeT;
					Cur.IndicatedKW = Cur.WorkJ * Rpm / 60.0 / 1000.0;
					Cur.Cycles = Last.Cycles + 1;
					Last = Cur;
					Cur = TwoStrokeCycleStats();
					ReedMassCycle = 0.0;
					bSparked = false;
					CycleScale = std::max(0.2, 1.0 + In.Roughness * Noise());
					CycleJitter = In.JitterDeg * Noise();
				}
			}
		}
		// Expansion chamber: Kelly-Lochbaum scattering, one section per sample.
		const int N = static_cast<int>(F.size());
		const float G = static_cast<float>(std::exp(-0.15 * PipeLen / F.size())); // wall friction / heat transfer / non-linear losses per section (0.15 /m: ESTIMATE, broadens the tune)
		const double Z0 = PipeRho * PipeC / Area[0];
		NF[0] = static_cast<float>(B0 + Z0 * (Mdot / PipeRho));
		for (int J = 0; J + 1 < N; ++J)
		{
			const float Kj = K[J];
			NF[J + 1] = G * ((1.0f + Kj) * F[J] - Kj * B[J + 1]);
			NB[J] = G * (Kj * F[J] + (1.0f - Kj) * B[J + 1]);
			if (J + 1 == Js)
			{
				// Silencer packing: waves crossing into / out of the silencer lose their highs.
				const float Dry = Geo.PackingDry;
				PackF += (NF[J + 1] - PackF) * PackA;
				NF[J + 1] = PackF + Dry * (NF[J + 1] - PackF);
				PackB += (NB[J] - PackB) * PackA;
				NB[J] = PackB + Dry * (NB[J] - PackB);
			}
		}
		// Open outlet: reflection ~ -0.95, weaker at high frequency (radiation, ka -> 1 near 5 kHz).
		const float FEnd = F[N - 1];
		const double Zn = (P0 * GammaHot / Csec[N - 1]) / Area[N - 1]; // rho c / A with rho c = gamma p / c
		EndLp += (FEnd - EndLp) * 0.41f;
		const float Refl = -0.95f * EndLp;
		NB[N - 1] = G * Refl;
		Ut = (FEnd - Refl) / Zn; // volume flow out of the outlet
		F.swap(NF);
		B.swap(NB);
		// Airbox + snorkels (flow into the box +), reed flow out.
		{
			const double Rho = P0 / (Rg * T0), C0 = std::sqrt(GammaAir * Rg * T0);
			const double As = Geo.SnorkelCount * Pi * 0.25 * Geo.SnorkelD * Geo.SnorkelD, Leff = Geo.SnorkelL + 0.6 * Geo.SnorkelD;
			const double W = C0 * std::sqrt(As / (Geo.AirboxVolume * Leff));
			const double Ureed = Pab_Flow / Rho;
			Pab += (GammaAir * P0 / Geo.AirboxVolume) * (Usn - Ureed) * Dt;
			Usn += ((As / (Rho * Leff)) * (-Pab) - 2.0 * Geo.AirboxDamping * W * Usn) * Dt;
		}
		// Radiation: far-field pressure ~ d(volume flow)/dt of each opening.
		Out.Exhaust = static_cast<float>((Ut - PrevUt) * Fs * Geo.ExhaustGain);
		Out.Intake = static_cast<float>((Usn - PrevUsn) * Fs * Geo.IntakeGain);
		PrevUt = Ut;
		PrevUsn = Usn;
		Out.CrankDeg = static_cast<float>(Theta);

		// Pipe gas temperature follows the exhaust gas (wave speed -> pipe tune), every 1024 samples.
		if (++SampleCount % 1024 == 0)
		{
			if (OutMassAcc > 0.0)
			{
				const double Tout = OutTempAcc / OutMassAcc;
				const double Target = T0 + 0.8 * (Tout - T0); // gas at the port (header), ESTIMATE
				PipeT += (Target - PipeT) * 0.03;             // ~0.7 s
			}
			else { PipeT += (T0 + 50.0 - PipeT) * 0.01; }
			OutTempAcc = OutMassAcc = 0.0;
			if (std::fabs(PipeT - BuiltT) > 0.03 * BuiltT) { BuildPipe(PipeT); }
		}
		// Safety: a numerical blow-up resets the model (silence for a moment rather than noise).
		if (!std::isfinite(Ps) || !std::isfinite(Ut) || !std::isfinite(Tc) || !std::isfinite(F[N / 2]))
		{
			const TwoStrokeGeometry Gm = Geo;
			Init(Gm, static_cast<float>(Fs));
			Out.Exhaust = Out.Intake = 0.0f;
		}
		return Out;
	}
}
