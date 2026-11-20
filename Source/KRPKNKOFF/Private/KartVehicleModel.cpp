// Procedural kart + driver model (visual only: the physics never reads any of this).
//
// Layout is in the "kart frame": metres forward of the rear axle (x), left (y), up from the ground (z). Axle,
// track and kingpin positions come from the live config, so the model follows the physics geometry and setup.
//
// Bodywork: a modern CIK-style four-piece kit (front fairing / spoiler, nassau (front) panel, two sidepods,
// plastic rear bumper) shaped after current kits of the Eurostar Dynamica type. Proportions are estimates from
// CIK bodywork rules and typical kits, not measured from a scan; no logos or brand graphics. Livery: the kart
// colour (BodyColor) + pearl white + black, painted per vertex.
//
// v3 proportions (user reference photo of a CIK senior kart, 2026-11-18): about 2.0 m overall (ESTIMATE, typical
// senior kart with CIK fairing and rear protection), front fairing ~1.12 m wide and no higher than the top of the
// front tyres, sidepods up to the plane through the tyre tops (both VERIFIED as limits: Motorsport UK NCR, which
// follows the FIA karting bodywork rules), steering wheel centre ~0.53 m. Driver: adult proportions, slightly
// reclined, legs reaching pedals just ahead of the front axle.
#include "KartVehiclePawn.h"

#include "Camera/CameraComponent.h"
#include "Misc/Crc.h"
#include "KartProceduralVisuals.h"
#include "KartUnrealConversions.h"
#include "ProceduralMeshComponent.h"

namespace
{
	// Two-bone IK: the middle joint for a limb Root -> End with segment lengths A, B, bent towards Hint.
	FVector SolveJoint(const FVector& Root, const FVector& End, float A, float B, const FVector& Hint)
	{
		const FVector D = End - Root;
		const float Dist = FMath::Clamp(static_cast<float>(D.Size()), 1e-3f, A + B - 1e-3f);
		const FVector Dir = D.GetSafeNormal();
		const float Along = (A * A - B * B + Dist * Dist) / (2.0f * Dist);
		const float Off = FMath::Sqrt(FMath::Max(A * A - Along * Along, 0.0f));
		const FVector Perp = (Hint - Dir * FVector::DotProduct(Hint, Dir)).GetSafeNormal();
		return Root + Dir * Along + Perp * Off;
	}
}

void AKartVehiclePawn::BuildVisuals()
{
	const kartsim::KartConfig& C = Sim->GetConfig();
	const double H = C.Chassis.CogHeight, L = C.Chassis.Wheelbase;
	const double B = L * C.Chassis.FrontWeightFraction;            // CoG -> rear axle (as ChassisModel)
	const double Yk = 0.5 * C.Chassis.FrontTrack - C.Steering.SpindleOffset; // kingpin lateral position
	const double RearR = C.RearTyre.UnloadedRadius;
	const FString ClassId = UTF8_TO_TCHAR(C.ClassId.c_str());
	const bool bKZ = ClassId.Contains(TEXT("kz"));
	const bool bAirCooled = ClassId.Contains(TEXT("kt100"));

	// Kart frame (m) -> component space (cm), and back.
	auto K = [H, B](double X, double Y, double Z) { return KartConv::ToUEPos(kartsim::Vec3(X - B, Y, Z - H)); };
	auto KI = [H, B](const FVector& P) { const kartsim::Vec3 Q = KartConv::FromUEPos(P); return FVector(Q.X + B, Q.Y, Q.Z + H); };
	const FVector Xa(1, 0, 0), Ya(0, -1, 0), Za(0, 0, 1); // kart forward / left / up in component space
	auto RingYZ = [&](double X, double Yc, double Zc, double HalfY, double HalfZ, float Ex, int32 Pts)
	{ return FKartMeshBuilder::SuperEllipse(K(X, Yc, Zc), Ya, Za, static_cast<float>(HalfY * 100.0), static_cast<float>(HalfZ * 100.0), Ex, Pts); };
	auto RingXZ = [&](double Y, double Xc, double Zc, double HalfX, double HalfZ, float Ex, int32 Pts)
	{ return FKartMeshBuilder::SuperEllipse(K(Xc, Y, Zc), Xa, Za, static_cast<float>(HalfX * 100.0), static_cast<float>(HalfZ * 100.0), Ex, Pts); };
	auto RingXY = [&](double Z, double Xc, double Yc, double HalfX, double HalfY, float Ex, int32 Pts)
	{ return FKartMeshBuilder::SuperEllipse(K(Xc, Yc, Z), Xa, Ya, static_cast<float>(HalfX * 100.0), static_cast<float>(HalfY * 100.0), Ex, Pts); };
	// Closed polygon -> N points evenly spaced along its perimeter (dense rings keep livery edges sharp).
	auto Resample = [](const TArray<FVector>& Ring, int32 Count)
	{
		TArray<float> Acc;
		float Total = 0.0f;
		for (int32 I = 0; I < Ring.Num(); ++I) { Acc.Add(Total); Total += static_cast<float>(FVector::Dist(Ring[I], Ring[(I + 1) % Ring.Num()])); }
		TArray<FVector> Out;
		int32 Seg = 0;
		for (int32 K = 0; K < Count; ++K)
		{
			const float D = Total * K / Count;
			while (Seg + 1 < Ring.Num() && Acc[Seg + 1] <= D) { ++Seg; }
			const float Len = (Seg + 1 < Ring.Num() ? Acc[Seg + 1] : Total) - Acc[Seg];
			const float T = Len > 1e-4f ? (D - Acc[Seg]) / Len : 0.0f;
			Out.Add(FMath::Lerp(Ring[Seg], Ring[(Seg + 1) % Ring.Num()], T));
		}
		return Out;
	};
	auto Scaled = [](TArray<FVector> Ring, float S)
	{
		FVector Cn = FVector::ZeroVector;
		for (const FVector& P : Ring) { Cn += P; }
		Cn /= Ring.Num();
		for (FVector& P : Ring) { P = Cn + (P - Cn) * S; }
		return Ring;
	};

	// --- Materials (instances of /Game/KartVisuals/M_KartSurface) ---
	auto Surf = [this](FLinearColor A, FLinearColor Bc, float R, float RV, float Met, bool bVc = false) -> UMaterialInterface*
	{
		UMaterialInterface* M = KartVisuals::MakeSurface(this, A, Bc, R, RV, Met, 0.004f, 0.5f, 0.3f, bVc);
		return M ? M : BaseMaterial.Get();
	};
	const FLinearColor White(1, 1, 1);
	auto SurfSmooth = [this](FLinearColor A, FLinearColor Bc, float R, float RV, float Met, bool bVc = false) -> UMaterialInterface*
	{
		UMaterialInterface* M = KartVisuals::MakeSurface(this, A, Bc, R, RV, Met, 0.004f, 0.5f, 0.06f, bVc);
		return M ? M : BaseMaterial.Get();
	};
	UMaterialInterface* Chrome = SurfSmooth(FLinearColor(0.62f, 0.63f, 0.66f), FLinearColor(0.68f, 0.69f, 0.72f), 0.18f, 0.04f, 1.0f);
	UMaterialInterface* GlossVC = Surf(White, White, 0.22f, 0.04f, 0.0f, true);        // painted / gloss plastic
	UMaterialInterface* MatteVC = Surf(White, White, 0.60f, 0.10f, 0.0f, true);        // matte plastic, rubber
	UMaterialInterface* EngineMetal = SurfSmooth(FLinearColor(0.52f, 0.53f, 0.55f), FLinearColor(0.58f, 0.59f, 0.61f), 0.32f, 0.05f, 0.9f);
	UMaterialInterface* Cast = SurfSmooth(FLinearColor(0.16f, 0.16f, 0.17f), FLinearColor(0.20f, 0.20f, 0.21f), 0.45f, 0.06f, 0.7f);
	UMaterialInterface* SuitVC = Surf(White, White, 0.85f, 0.10f, 0.0f, true);
	UMaterialInterface* Visor = Surf(FLinearColor(0.01f, 0.012f, 0.016f), FLinearColor(0.02f, 0.025f, 0.03f), 0.05f, 0.02f, 0.3f);
	UMaterialInterface* Rubber = Surf(FLinearColor(0.016f, 0.016f, 0.017f), FLinearColor(0.03f, 0.03f, 0.03f), 0.90f, 0.10f, 0.0f, true);
	UMaterialInterface* Alloy = Surf(FLinearColor(0.55f, 0.55f, 0.57f), FLinearColor(0.62f, 0.62f, 0.64f), 0.35f, 0.10f, 1.0f, true);

	// Livery.
	const FLinearColor Prim = BodyColor;
	const FLinearColor Pearl(0.80f, 0.80f, 0.82f);
	const FLinearColor Blk(0.022f, 0.022f, 0.025f);
	const FLinearColor Graphite(0.06f, 0.06f, 0.065f);

	// ============================== Frame (32 mm main rails) ==============================
	FKartMeshBuilder Fr;
	const float Rail = 1.6f;
	for (double S : {1.0, -1.0})
	{
		Fr.AddTubePath({K(-0.06, 0.24 * S, 0.05), K(0.30, 0.25 * S, 0.045), K(0.62, 0.21 * S, 0.045), K(0.88, 0.16 * S, 0.05), K(L + 0.06, 0.16 * S, 0.06)}, Rail);
		Fr.AddTube(K(L - 0.03, 0.16 * S, 0.06), K(L, Yk * S, 0.10), Rail);          // kingpin bracket arms
		Fr.AddTube(K(L + 0.06, 0.16 * S, 0.06), K(L, Yk * S, 0.10), Rail);
		const double Cs = std::tan(C.Steering.Caster) * 0.07;                       // kingpin tilted by caster
		Fr.AddTube(K(L + Cs, Yk * S, 0.05), K(L - Cs, Yk * S, 0.19), 1.2f);
		Fr.AddTubePath({K(0.15, 0.25 * S, 0.07), K(0.18, 0.56 * S, 0.11), K(0.82, 0.54 * S, 0.11), K(0.86, 0.18 * S, 0.07)}, 1.4f); // side bars (pod mounts)
		Fr.AddTube(K(0.10, 0.20 * S, 0.05), K(0.22, 0.17 * S, 0.42), 1.0f);         // seat stays
		Fr.AddTube(K(-0.06, 0.22 * S, 0.06), K(-0.26, 0.30 * S, 0.13), 1.4f);       // rear bumper supports
		Fr.AddTube(K(L + 0.06, 0.16 * S, 0.06), K(L + 0.40, 0.14 * S, 0.09), 1.4f); // front bumper (inside the fairing)
		Fr.AddTube(K(L - 0.04, 0.02 * S, 0.08), K(L - 0.01, (Yk - 0.05) * S, 0.11), 0.7f); // tie rods
		Fr.AddBox(K(L + 0.10, 0.09 * S, 0.13), FVector(1.2f, 3.0f, 7.0f), FQuat(FRotator(-25.0f, 0.0f, 0.0f))); // pedals
		Fr.AddBox(K(0.0, 0.32 * S, RearR), FVector(4.0f, 2.0f, 5.0f));              // axle bearing cassettes
	}
	Fr.AddTube(K(-0.06, 0.24, 0.05), K(-0.06, -0.24, 0.05), Rail);
	Fr.AddTube(K(0.62, 0.21, 0.045), K(0.62, -0.21, 0.045), Rail);
	Fr.AddTube(K(L + 0.06, 0.16, 0.06), K(L + 0.06, -0.16, 0.06), Rail);
	Fr.AddTube(K(L + 0.40, 0.14, 0.09), K(L + 0.40, -0.14, 0.09), 1.4f);
	Fr.AddTube(K(-0.26, 0.62, 0.13), K(-0.26, -0.62, 0.13), 1.6f);
	const FVector ColBase = K(L - 0.05, 0.0, 0.07), ColTop = K(L - 0.36, 0.0, 0.53);
	Fr.AddTube(ColBase, ColTop, 1.0f);                                             // steering column
	Fr.AddTube(K(L - 0.02, 0.0, 0.07), K(L + 0.08, 0.0, 0.20), 0.8f);              // nassau panel bracket
	FrameMesh->ClearAllMeshSections();
	Fr.ToSection(FrameMesh, 0, Chrome);

	// ============================== Bodywork ==============================
	FKartMeshBuilder Bw;   // gloss painted plastic (vertex livery)
	FKartMeshBuilder Mt;   // matte black plastic: floor tray, chain guard, airbox
	FKartMeshBuilder En;   // engine metal
	FKartMeshBuilder Cs;   // cast / dark metal (brake disc, radiator core, gearbox)

	// --- Front fairing (spoiler): a wedge across the front, ahead of the front tyres; raised, swept-back
	//     "side ends" in front of each tyre. Lofted across the kart from right to left. ---
	{
		const double X1 = L + C.FrontTyre.UnloadedRadius + 0.04; // back edge, clear of the front tyres
		const double Span = 0.56;
		TArray<TArray<FVector>> Rings;
		const int32 NS = 121;
		for (int32 I = 0; I < NS; ++I)
		{
			const double Y = -Span + 2.0 * Span * I / (NS - 1);
			const double Ay = FMath::Abs(Y) / Span;
			const double Side = FMath::SmoothStep(0.62, 0.95, Ay);   // 0 centre .. 1 side ends
			const double Dip = 0.012 * (1.0 - FMath::SmoothStep(0.0, 0.25, Ay)); // shallow centre recess (panel mount)
			const double X0 = L + 0.60 - 0.09 * Side;                // nose line sweeps back at the ends
				const double Top = 0.225 + 0.03 * Side - Dip;            // <= top of the front tyres (CIK limit)
			// Closed section (x, z): bottom from the back to the nose, over the top, down the back face.
			const TArray<FVector2D> Sec = {
				{X1, 0.068}, {X1 + 0.08, 0.056}, {(X1 + X0) * 0.5, 0.054}, {X0 - 0.05, 0.058}, {X0 - 0.01, 0.072},
				{X0, 0.095}, {X0 - 0.012, 0.122}, {X0 - 0.07, 0.150 + 0.02 * Side}, {X0 - 0.17, 0.180 + 0.03 * Side},
				{X1 + 0.12, 0.205 + 0.03 * Side - Dip}, {X1 + 0.05, Top - 0.008}, {X1 + 0.012, Top}, {X1 - 0.004, Top - 0.03},
				{X1 - 0.01, 0.15}, {X1 - 0.008, 0.11}, {X1 - 0.004, 0.08}};
			TArray<FVector> Ring;
			for (const FVector2D& Q : Sec) { Ring.Add(K(Q.X, Y, Q.Y)); }
			Ring = Resample(Ring, 160);
			const float EndScale = static_cast<float>(FMath::Lerp(0.55, 1.0, FMath::SmoothStep(0.0, 1.0, (1.0 - Ay) / 0.06)));
			Rings.Add(Scaled(Ring, EndScale));
		}
		Bw.AddLoft(Rings, true, true, [&](int32, int32, const FVector& P)
		{
			// Relative height within the local section, so the bands follow the raised side ends cleanly.
			const FVector Q = KI(P);
			// Bands on smooth absolute heights (the sections shrink at the ends, so relative heights tore the edges).
			const double Side = FMath::SmoothStep(0.55, 1.0, FMath::Abs(Q.Y) / 0.56);
			if (Q.Z < 0.075) { return Blk; }                          // lower lip
			if (Q.Z > 0.178 + 0.03 * Side) { return Pearl; }          // top deck
			return Prim;
		});
	}

	// --- Nassau (front) panel: curved shield ahead of the steering column, top just below the wheel rim. ---
	{
		const FVector2D Bot(L + 0.17, 0.13), Top(L - 0.13, 0.50);    // (x, z) of the panel's mid-plane
		const FVector2D Up = (Top - Bot).GetSafeNormal();
		const FVector2D Nf(Up.Y, -Up.X);                              // forward-facing normal (x, z)
		TArray<TArray<FVector>> Rings;
		const int32 NH = 31, NW = 41;
		for (int32 I = 0; I < NH; ++I)
		{
			const double T = double(I) / (NH - 1);
			const FVector2D Mid = Bot + (Top - Bot) * T;
			const double W = FMath::Lerp(0.225, 0.18, T);
			const double Bulge = 0.045;
			TArray<FVector> Ring;
			for (int32 Pass = 0; Pass < 2; ++Pass)                     // front face L->R, back face R->L
			{
				for (int32 J = 0; J < NW; ++J)
				{
					const double U = Pass == 0 ? -1.0 + 2.0 * J / (NW - 1) : 1.0 - 2.0 * J / (NW - 1);
					const double Off = Bulge * (1.0 - U * U) + (Pass == 0 ? 0.012 : -0.012);
					Ring.Add(K(Mid.X + Nf.X * Off, U * W, Mid.Y + Nf.Y * Off));
				}
			}
			const double Edge = FMath::Min(T, 1.0 - T);
			Rings.Add(Scaled(Ring, static_cast<float>(FMath::Lerp(0.82, 1.0, FMath::SmoothStep(0.0, 0.12, Edge)))));
		}
		Bw.AddLoft(Rings, true, true, [&](int32 Ri, int32, const FVector& P)
		{
			const FVector Q = KI(P);
			const double T = double(Ri) / (NH - 1);
			if (T < 0.22) { return Prim; }                             // lower band in the kart colour
			if (T < 0.30) { return Blk; }
			return FMath::Abs(Q.Y) > 0.15 ? Prim : Pearl;               // pearl number field, coloured edges
		});
	}

	// --- Sidepods: long rounded pods on the side bars between the wheels, raised at the rear end. ---
	for (double S : {1.0, -1.0})
	{
		const double Xr = RearR + 0.03, Xf = L - C.FrontTyre.UnloadedRadius - 0.035;
		TArray<TArray<FVector>> Rings;
		const int32 NX = 120;
		for (int32 I = 0; I < NX; ++I)
		{
			const double T = double(I) / (NX - 1);                  // 0 rear .. 1 front
			const double X = FMath::Lerp(Xr, Xf, T);
			const double Yo = 0.675 - 0.06 * T, Yi = 0.42 - 0.08 * T;
				const double Zb = 0.055, Zt = 0.20 + 0.075 * FMath::Pow(1.0 - T, 1.3); // rear 0.275 .. front 0.20 m
			TArray<FVector> Ring = RingYZ(X, S * 0.5 * (Yo + Yi), 0.5 * (Zb + Zt), 0.5 * (Yo - Yi), 0.5 * (Zt - Zb), 3.4f, 160);
			const double End = FMath::Min(T, 1.0 - T) * (Xf - Xr);  // metres from the nearer end
			Rings.Add(Scaled(Ring, static_cast<float>(FMath::Lerp(T > 0.5 ? 0.45 : 0.75, 1.0, FMath::SmoothStep(0.0, 0.07, End)))));
		}
		Bw.AddLoft(Rings, true, true, [&](int32, int32, const FVector& P)
		{
			const FVector Q = KI(P);
			const double T = FMath::Clamp((Q.X - Xr) / (Xf - Xr), 0.0, 1.0);
			const double Zt = 0.20 + 0.075 * FMath::Pow(1.0 - T, 1.3);
				const double Rel = (Q.Z - 0.055) / (Zt - 0.055);           // 0 bottom .. 1 top
			if (Rel < 0.16) { return Blk; }
			if (Rel > 0.82) { return Pearl; }                          // top deck
			// Diagonal pearl stripe on the outer face, rising towards the rear.
			const double Band = Rel - (0.62 - 0.45 * T);
			if (FMath::Abs(Q.Y) > 0.47 && Band > 0.0 && Band < 0.16) { return Pearl; }
			return Prim;
		});
	}

	// --- Rear bumper: full-width plastic guard behind the rear tyres, ends wrapped slightly forward. ---
	{
		const double Span = 0.5 * C.Chassis.RearTrack + 0.5 * C.RearTyre.Width - 0.005;
		TArray<TArray<FVector>> Rings;
		const int32 NS = 81;
		for (int32 I = 0; I < NS; ++I)
		{
			const double Y = -Span + 2.0 * Span * I / (NS - 1);
			const double Ay = FMath::Abs(Y) / Span;
			const double Wrap = 0.03 * FMath::SmoothStep(0.7, 1.0, Ay);
			const double Hz = 0.09 - 0.014 * FMath::SmoothStep(0.8, 1.0, Ay);
			const double Mid = 0.012 * (1.0 - FMath::SmoothStep(0.0, 0.3, Ay)); // lowered centre (number plate)
			TArray<FVector> Ring = RingXZ(Y, -RearR - 0.15 + Wrap, 0.165 - Mid, 0.095, Hz, 4.0f, 96);
			Rings.Add(Scaled(Ring, static_cast<float>(FMath::Lerp(0.6, 1.0, FMath::SmoothStep(0.0, 1.0, (1.0 - Ay) / 0.05)))));
		}
		Bw.AddLoft(Rings, true, true, [&](int32, int32, const FVector& P)
		{
			const FVector Q = KI(P);
			if (Q.Z > 0.225) { return Prim; }
				if (Q.Z > 0.213) { return Pearl; }
			return Blk;
		});
	}

	// --- Floor tray ---
	Mt.AddLoft({RingXY(0.028, L - 0.25, 0.0, 0.29, 0.20, 6.0f, 28), RingXY(0.036, L - 0.25, 0.0, 0.29, 0.20, 6.0f, 28)}, true, true, nullptr, Blk);

	// --- Seat: fibreglass bucket (U-shaped shell) reclined behind the driver, with a seat pan. ---
	{
		const FVector2D Base(0.42, 0.06), TopP(0.15, 0.61);            // reclined ~26 deg from vertical            // (x, z) along the seat's centre
		const FVector2D Ax = (TopP - Base).GetSafeNormal();
		const FVector2D Fw(Ax.Y, -Ax.X);                                  // forward, perpendicular to the back
		TArray<TArray<FVector>> Rings;
		const int32 NU = 10;
		for (int32 I = 0; I < 9; ++I)
		{
			const double T = I / 8.0;
			const FVector2D Cn = Base + (TopP - Base) * T;
			const double W = FMath::Lerp(0.175, 0.20, T), D = FMath::Lerp(0.15, 0.07, T), Th = 0.012;
			TArray<FVector> Ring;
			for (int32 Pass = 0; Pass < 2; ++Pass)
			{
				for (int32 J = 0; J <= NU; ++J)
				{
					const double A = PI * (Pass == 0 ? double(J) / NU : 1.0 - double(J) / NU);
					const double Ww = Pass == 0 ? W : W - Th, Dd = Pass == 0 ? D : D - Th;
					const double U = Ww * FMath::Cos(A), V = -Dd * FMath::Pow(FMath::Sin(A), 0.7);
					Ring.Add(K(Cn.X + Fw.X * V, U, Cn.Y + Fw.Y * V));
				}
			}
			Rings.Add(Ring);
		}
		Bw.AddLoft(Rings, true, true, nullptr, Graphite);
		Bw.AddLoft({RingXY(0.045, 0.37, 0.0, 0.14, 0.18, 3.0f, 24), RingXY(0.075, 0.37, 0.0, 0.14, 0.18, 3.0f, 24)}, true, true, nullptr, Graphite);
	}

	// --- Fuel tank between the driver's legs (translucent white plastic look) ---
	Bw.AddEllipsoid(K(0.84, 0.0, 0.10), FVector(11.0f, 6.5f, 6.5f), FQuat::Identity, 8, 14, [&](int32, int32, const FVector&) { return FLinearColor(0.70f, 0.70f, 0.66f); });

	// --- Engine bay (right side, beside the seat): crankcase on its mount, cylinder + head, clutch drum, carburettor
	//     and airbox, exhaust (header, expansion chamber held by springs, silencer), radiator (left), chain drive,
	//     brake system, hubs, cables and hoses. Shapes follow typical CIK direct-drive / KZ kart layouts (ESTIMATE,
	//     generic, no brands). ---
	const double Ey = -0.335, Ex = 0.20;
	FKartMeshBuilder Hs;   // hoses, cables, chain (matte, vertex colour)
	const FLinearColor HoseC(0.05f, 0.05f, 0.06f), ChainC(0.12f, 0.12f, 0.13f), Yellow(0.95f, 0.78f, 0.05f);
	// Crankcase, engine mount plate and rail clamps.
	En.AddRoundBox(K(Ex, Ey, 0.165), FVector(bKZ ? 15.0f : 12.5f, 8.5f, 8.0f), FQuat::Identity, 3.2f);
	Cs.AddRoundBox(K(Ex, -0.245, 0.06), FVector(11.0f, 7.0f, 0.8f), FQuat::Identity, 0.5f);
	for (double Dx : {-0.07, 0.07}) { Cs.AddRoundBox(K(Ex + Dx, -0.245, 0.045), FVector(2.2f, 2.6f, 2.6f), FQuat::Identity, 0.6f); }
	En.AddCylinder(K(Ex, -0.245, 0.15), K(Ex, -0.228, 0.15), 6.0f, 32);      // clutch drum (inboard, on the chain line)
	Mt.AddCylinder(K(Ex + 0.07, Ey + 0.05, 0.095), K(Ex - 0.03, Ey + 0.05, 0.095), 3.0f, 16, Blk); // starter motor
	if (bAirCooled)
	{
		// KT100: air-cooled, finned barrel and head.
		En.AddTaperTube(K(Ex + 0.01, Ey, 0.22), K(Ex + 0.03, Ey, 0.37), 5.0f, 5.0f, 18);
		for (int32 F = 0; F < 9; ++F)
		{
			const double Z = 0.24 + 0.016 * F;
			En.AddLoft({RingXY(Z, Ex + 0.012 + 0.02 * (Z - 0.22) / 0.15, Ey, 0.085, 0.085, 2.6f, 28),
			            RingXY(Z + 0.004, Ex + 0.012 + 0.02 * (Z - 0.22) / 0.15, Ey, 0.085, 0.085, 2.6f, 28)}, true, true);
		}
		Mt.AddCylinder(K(Ex + 0.05, Ey, 0.39), K(Ex + 0.07, Ey, 0.43), 1.3f, 12, Blk); // plug cap
	}
	else
	{
		const FQuat Lean(FRotator(-8.0f, 0.0f, 0.0f));
		En.AddRoundBox(K(Ex + 0.015, Ey, 0.27), FVector(6.6f, 6.6f, 6.0f), Lean, 1.6f);       // cylinder block
		Cs.AddRoundBox(K(Ex + 0.028, Ey, 0.343), FVector(7.2f, 7.2f, 2.0f), Lean, 1.8f);      // head
		for (double Bx : {-0.045, 0.045}) { for (double By : {-0.045, 0.045}) { En.AddCylinder(K(Ex + 0.028 + Bx, Ey + By, 0.36), K(Ex + 0.028 + Bx, Ey + By, 0.372), 0.8f, 8); } }
		Mt.AddCylinder(K(Ex + 0.035, Ey, 0.365), K(Ex + 0.055, Ey, 0.415), 1.3f, 12, Blk);    // plug cap
		Hs.AddTubePath({K(Ex + 0.055, Ey, 0.415), K(Ex + 0.0, Ey + 0.035, 0.44), K(Ex - 0.08, Ey + 0.06, 0.40), K(Ex - 0.10, Ey + 0.06, 0.31)}, 0.45f, 6, HoseC);
		Mt.AddRoundBox(K(Ex - 0.11, Ey + 0.06, 0.27), FVector(3.0f, 2.4f, 4.0f), FQuat::Identity, 0.8f, Blk); // ignition coil
		En.AddCylinder(K(Ex + 0.0, Ey, 0.355), K(Ex - 0.03, Ey + 0.02, 0.385), 1.4f, 12);   // water outlet
		// Radiator on the left beside the seat: core with fins, side frame, top/bottom tanks, shutter, stays, hoses.
		const double Rx = 0.23, Ry = 0.31, Rz = 0.36;
		Cs.AddBox(K(Rx, Ry, Rz), FVector(1.4f, 10.5f, 14.0f));
		for (int32 F = 0; F < 26; ++F) { En.AddBox(K(Rx + 0.016, Ry - 0.10 + 0.2 * F / 25.0, Rz), FVector(0.25f, 0.08f, 13.6f)); } // fins
		Mt.AddRoundBox(K(Rx, Ry + 0.115, Rz), FVector(2.2f, 1.2f, 15.5f), FQuat::Identity, 0.6f, Blk);
		Mt.AddRoundBox(K(Rx, Ry - 0.115, Rz), FVector(2.2f, 1.2f, 15.5f), FQuat::Identity, 0.6f, Blk);
		En.AddRoundBox(K(Rx, Ry, Rz + 0.155), FVector(2.4f, 11.5f, 1.6f), FQuat::Identity, 0.8f);
		En.AddRoundBox(K(Rx, Ry, Rz - 0.155), FVector(2.4f, 11.5f, 1.6f), FQuat::Identity, 0.8f);
		Mt.AddBox(K(Rx + 0.05, Ry, Rz + 0.14), FVector(0.25f, 10.5f, 5.0f), FQuat(FRotator(-55.0f, 0.0f, 0.0f)), Blk); // shutter
		Fr.AddTube(K(Rx, Ry - 0.09, Rz - 0.17), K(0.20, 0.53, 0.11), 0.7f);           // stays to the side bar
		Fr.AddTube(K(Rx, Ry + 0.09, Rz - 0.17), K(0.24, 0.55, 0.11), 0.7f);
		Fr.AddTube(K(Rx - 0.02, Ry - 0.11, Rz + 0.12), K(0.20, 0.19, 0.50), 0.6f);   // stay to the seat
		Hs.AddTubePath({K(Rx - 0.02, 0.24, Rz - 0.15), K(0.13, 0.12, 0.20), K(0.13, -0.20, 0.20), K(Ex - 0.04, Ey + 0.05, 0.22)}, 1.6f, 12, HoseC);
		Hs.AddTubePath({K(Rx - 0.02, 0.24, Rz + 0.15), K(0.15, 0.10, 0.48), K(0.15, -0.20, 0.43), K(Ex - 0.03, Ey + 0.02, 0.385)}, 1.6f, 12, HoseC);
	}
	if (bKZ)
	{
		Cs.AddRoundBox(K(Ex - 0.06, Ey + 0.03, 0.14), FVector(10.0f, 7.0f, 7.0f), FQuat::Identity, 2.5f); // gearbox housing
		Fr.AddTube(K(0.56, -0.25, 0.14), K(0.63, -0.29, 0.42), 0.9f);                                    // gear lever
		Fr.AddSphere(K(0.63, -0.29, 0.43), 2.0f);
	}
	// Carburettor (rearward, behind the cylinder) and the intake silencer behind the seat with two snorkels.
	En.AddCylinder(K(Ex - 0.07, Ey + 0.04, 0.24), K(Ex - 0.12, Ey + 0.04, 0.24), 2.8f, 18);
	Mt.AddTaperTube(K(Ex - 0.12, Ey + 0.04, 0.24), K(0.03, -0.15, 0.27), 2.6f, 3.0f, 14, Blk);   // intake rubber
	Mt.AddRoundBox(K(0.02, -0.12, 0.31), FVector(6.0f, 13.5f, 9.0f), FQuat::Identity, 3.0f, Blk);     // intake silencer
	for (double Sy : {-0.05, -0.19}) { Mt.AddTaperTube(K(0.03, Sy, 0.39), K(0.06, Sy, 0.45), 2.0f, 2.3f, 12, Blk); }
	// Exhaust: header loops forward and out, expansion chamber runs back outboard of the engine (two springs),
	// silencer ends above the bumper.
	En.AddTubePath({K(Ex + 0.06, Ey - 0.02, 0.30), K(Ex + 0.15, Ey - 0.07, 0.28), K(Ex + 0.14, Ey - 0.13, 0.25), K(Ex + 0.04, Ey - 0.15, 0.25)}, 2.3f, 16);
	{
		const FVector EA = K(Ex + 0.04, Ey - 0.15, 0.25), EB = K(-0.10, Ey - 0.10, 0.33);
		const FVector Ax = (EB - EA).GetSafeNormal();
		const FVector U = FVector::CrossProduct(Ax, Za).GetSafeNormal(), W = FVector::CrossProduct(Ax, U);
		struct FPt { float T, R; };
		const FPt Prof[] = {{0.0f, 2.4f}, {0.12f, 3.6f}, {0.3f, 5.6f}, {0.45f, 6.2f}, {0.6f, 6.2f}, {0.78f, 4.6f}, {0.92f, 2.6f}, {1.0f, 2.2f}};
		TArray<TArray<FVector>> Rings;
		for (const FPt& P : Prof) { Rings.Add(FKartMeshBuilder::SuperEllipse(FMath::Lerp(EA, EB, P.T), U, W, P.R, P.R, 2.0f, 28)); }
		En.AddLoft(Rings, false, false);
		for (float T : {0.04f, 0.97f}) { Fr.AddTorus(FMath::Lerp(EA, EB, T), Ax, 2.9f, 0.35f, 16, 6); } // springs at the joints
		Mt.AddCylinder(EB, K(-0.22, Ey + 0.05, 0.31), 4.6f, 24, Graphite);   // silencer
		En.AddCylinder(K(-0.22, Ey + 0.05, 0.31), K(-0.25, Ey + 0.08, 0.305), 1.4f, 12); // outlet
		Fr.AddTube(K(-0.16, Ey + 0.0, 0.29), K(-0.10, -0.24, 0.10), 0.6f);  // silencer stay
	}
	// Chain drive on the chain line (y = -0.236): axle sprocket with teeth, engine sprocket, chain, upper guard.
	{
		const double Cy = -0.236, Rs = 0.10, Re = 0.026, Ez = 0.15;
		Cs.AddCylinder(K(0.0, Cy - 0.003, RearR), K(0.0, Cy + 0.003, RearR), Rs * 100.0f - 0.4f, 48);
		En.AddCylinder(K(0.0, Cy - 0.008, RearR), K(0.0, Cy + 0.008, RearR), 4.5f, 24);   // sprocket carrier
		for (int32 Tth = 0; Tth < 48; ++Tth)
		{
			const double A = 2.0 * PI * Tth / 48;
			Cs.AddBox(K(Rs * FMath::Cos(A), Cy, RearR + Rs * FMath::Sin(A)), FVector(0.35f, 0.3f, 0.45f), FQuat(Ya, -static_cast<float>(A)));
		}
		Cs.AddCylinder(K(Ex, Cy - 0.004, Ez), K(Ex, Cy + 0.004, Ez), Re * 100.0f, 16);
		TArray<FVector> Chain;
		for (int32 I = 0; I <= 24; ++I) { const double A = 0.5 * PI + PI * I / 24; Chain.Add(K(Rs * FMath::Cos(A) + 0.0, Cy, RearR + Rs * FMath::Sin(A))); } // rear half of the axle sprocket
		for (int32 I = 0; I <= 12; ++I) { const double A = -0.5 * PI + PI * I / 12; Chain.Add(K(Ex + Re * FMath::Cos(A), Cy, Ez + Re * FMath::Sin(A))); } // front half of the engine sprocket
		const FVector ChainStart = Chain[0]; // copy: Add() must not take a reference into the array it grows
		Chain.Add(ChainStart);
		Hs.AddTubePath(Chain, 0.42f, 6, ChainC);
		Mt.AddRoundBox(K(0.5 * Ex, Cy - 0.012, 0.215), FVector(0.5f * Ex * 100.0f + 6.0f, 0.4f, 3.5f), FQuat(FRotator(-6.0f, 0.0f, 0.0f)), 0.3f, Blk); // guard
	}
	// Brake: ventilated disc (two plates and vanes) on the left of the axle, caliper, hose to the master cylinder.
	{
		const double Dy = 0.172, Rd = 0.10;
		Cs.AddCylinder(K(0.0, Dy - 0.009, RearR), K(0.0, Dy - 0.005, RearR), Rd * 100.0f, 40);
		Cs.AddCylinder(K(0.0, Dy + 0.005, RearR), K(0.0, Dy + 0.009, RearR), Rd * 100.0f, 40);
		for (int32 Vn = 0; Vn < 24; ++Vn)
		{
			const double A = 2.0 * PI * Vn / 24;
			Cs.AddBox(K(0.08 * FMath::Cos(A), Dy, RearR + 0.08 * FMath::Sin(A)), FVector(1.6f, 0.45f, 0.25f), FQuat(Ya, -static_cast<float>(A)));
		}
		En.AddCylinder(K(0.0, Dy - 0.012, RearR), K(0.0, Dy + 0.012, RearR), 5.0f, 24);   // disc carrier
		Mt.AddRoundBox(K(0.07, Dy, RearR + 0.07), FVector(3.6f, 2.9f, 3.4f), FQuat(Ya, FMath::DegreesToRadians(-45.0f)), 1.0f, Prim); // caliper
		Fr.AddTube(K(0.07, Dy, RearR + 0.035), K(0.10, 0.20, 0.055), 0.8f);                // caliper bracket
		Hs.AddTubePath({K(0.09, Dy + 0.02, RearR + 0.10), K(0.25, 0.21, 0.12), K(0.70, 0.18, 0.07), K(L - 0.06, 0.08, 0.09)}, 0.4f, 6, HoseC);
		En.AddCylinder(K(L - 0.06, 0.08, 0.09), K(L - 0.14, 0.08, 0.10), 1.4f, 12);        // master cylinder
	}
	// Rear hubs (inboard of each rear wheel) with bolts.
	for (double S : {1.0, -1.0})
	{
		const double Hy = S * (0.5 * C.Chassis.RearTrack - 0.5 * C.RearTyre.Width - 0.03);
		En.AddCylinder(K(0.0, Hy - 0.04 * S, RearR), K(0.0, Hy + 0.02 * S, RearR), 4.2f, 24);
		for (int32 Bt = 0; Bt < 3; ++Bt)
		{
			const double A = 2.0 * PI * Bt / 3;
			Fr.AddCylinder(K(0.028 * FMath::Cos(A), Hy - 0.045 * S, RearR + 0.028 * FMath::Sin(A)), K(0.028 * FMath::Cos(A), Hy - 0.05 * S, RearR + 0.028 * FMath::Sin(A)), 0.7f, 6);
		}
	}
	// Seat struts, tie-rod ends, steering column support, battery, fuel tank cap and line, throttle cable.
	for (double S : {1.0, -1.0})
	{
		Fr.AddTube(K(0.30, 0.19 * S, 0.33), K(0.47, 0.54 * S, 0.11), 0.9f);
		Fr.AddTube(K(0.22, 0.18 * S, 0.47), K(0.03, 0.30 * S, 0.17), 0.9f);
		Fr.AddSphere(K(L - 0.04, 0.02 * S, 0.08), 1.2f, 6, 10);
		Fr.AddSphere(K(L - 0.01, (Yk - 0.05) * S, 0.11), 1.2f, 6, 10);
	}
	{
		const FVector ColMid = FMath::Lerp(ColBase, ColTop, 0.5f);
		Cs.AddRoundBox(ColMid, FVector(2.2f, 3.2f, 2.2f), FRotationMatrix::MakeFromZ((ColTop - ColBase).GetSafeNormal()).ToQuat(), 0.6f);
		Fr.AddTube(ColMid, K(L - 0.10, 0.12, 0.06), 0.8f);
		Fr.AddTube(ColMid, K(L - 0.10, -0.12, 0.06), 0.8f);
	}
	if (!bKZ && !bAirCooled)
	{
		Mt.AddRoundBox(K(0.50, 0.21, 0.14), FVector(6.5f, 4.0f, 4.5f), FQuat::Identity, 0.8f, Blk);   // battery
		Mt.AddCylinder(K(0.53, 0.21, 0.185), K(0.53, 0.21, 0.195), 0.8f, 8, FLinearColor(0.7f, 0.05f, 0.04f)); // terminal cover
	}
	Mt.AddCylinder(K(0.92, 0.0, 0.16), K(0.92, 0.0, 0.18), 2.0f, 16, Blk);                      // fuel cap
	Hs.AddTubePath({K(0.80, 0.0, 0.07), K(0.60, -0.10, 0.07), K(0.35, -0.24, 0.10), K(Ex - 0.08, Ey + 0.04, 0.22)}, 0.4f, 6, FLinearColor(0.6f, 0.6f, 0.55f));
	Hs.AddTubePath({K(L + 0.06, -0.09, 0.14), K(L - 0.10, -0.15, 0.07), K(0.45, -0.24, 0.07), K(Ex - 0.08, Ey + 0.04, 0.26)}, 0.3f, 6, HoseC); // throttle cable

	// --- Race numbers: yellow plates with black digits on the front panel, the rear bumper and both sidepods ---
	{
		const int32 Num = static_cast<int32>(FCrc::StrCrc32(*DriverName) % 97u) + 2;
		const int32 D0 = Num / 10, D1 = Num % 10;
		// Seven-segment digit, H tall; Rt / Up span the plate, Out = plate normal.
		auto Digit = [&](const FVector& Origin, const FVector& Rt, const FVector& Up, const FVector& Out, float H, int32 D)
		{
			static const uint8 Segs[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};
			const float W = 0.56f * H, T = 0.15f * H;
			struct FS { float R, U, HR, HU; };
			const FS S[7] = {{W / 2, H - T / 2, W / 2, T / 2}, {W - T / 2, 0.75f * H, T / 2, H / 4}, {W - T / 2, 0.25f * H, T / 2, H / 4},
			                 {W / 2, T / 2, W / 2, T / 2}, {T / 2, 0.25f * H, T / 2, H / 4}, {T / 2, 0.75f * H, T / 2, H / 4}, {W / 2, H / 2, W / 2, T / 2}};
			const FQuat Q(FMatrix(Rt, Up, FVector::CrossProduct(Rt, Up), FVector::ZeroVector));
			for (int32 I = 0; I < 7; ++I)
			{
				if (!((Segs[D] >> I) & 1)) { continue; }
				Mt.AddBox(Origin + Rt * S[I].R + Up * S[I].U + Out * 0.12f, FVector(S[I].HR, S[I].HU, 0.1f), Q, Blk);
			}
		};
		auto Plate = [&](const FVector& Centre, const FVector& Out, const FVector& Up, float W, float H)
		{
			const FVector Rt = FVector::CrossProduct(Out, Up).GetSafeNormal();      // viewer's right, looking at the plate
			const FQuat Q(FMatrix(Rt, Up, FVector::CrossProduct(Rt, Up), FVector::ZeroVector));
			Mt.AddRoundBox(Centre, FVector(0.5f * W, 0.5f * H, 0.15f), Q, 0.14f, Yellow);
			const float Dh = 0.72f * H, Dw = 0.56f * Dh, Gap = 0.18f * Dh;
			const FVector O = Centre + Out * 0.15f - Rt * (Dw + 0.5f * Gap) - Up * (0.5f * Dh);
			Digit(O, Rt, Up, Out, Dh, D0);
			Digit(O + Rt * (Dw + Gap), Rt, Up, Out, Dh, D1);
		};
		// Front panel (on its bulged front face, near the top).
		{
			const FVector2D Bot(L + 0.17, 0.13), Top(L - 0.13, 0.50);
			const FVector2D Up2 = (Top - Bot).GetSafeNormal(), Nf2(Up2.Y, -Up2.X);
			const FVector2D Mid = Bot + (Top - Bot) * 0.62 + Nf2 * 0.059;
			const FVector Up(Up2.X, 0.0, Up2.Y), Out(Nf2.X, 0.0, Nf2.Y);
			Plate(K(Mid.X, 0.0, Mid.Y), Out, Up, 17.0f, 13.0f);
		}
		// Rear bumper (centre, back face).
		Plate(K(-RearR - 0.15 - 0.097, 0.0, 0.168), -Xa, Za, 16.0f, 11.0f);
		// Sidepods (outer faces, towards the rear).
		for (double S : {1.0, -1.0})
		{
			const double Xs = RearR + 0.03 + 0.14, Ys = 0.675 - 0.06 * 0.2 + 0.009; // just proud of the curved pod face
			Plate(K(Xs, Ys * S, 0.165), Ya * static_cast<float>(S), Za, 15.0f, 10.0f);
		}
	}
	FrameMesh->ClearAllMeshSections();
	Fr.ToSection(FrameMesh, 0, Chrome); // the frame gained struts, springs, rod ends and bolts above
	BodyMesh->ClearAllMeshSections();
	Bw.ToSection(BodyMesh, 0, GlossVC);
	Mt.ToSection(BodyMesh, 1, MatteVC);
	En.ToSection(BodyMesh, 2, EngineMetal);
	Cs.ToSection(BodyMesh, 3, Cast);
	Hs.ToSection(BodyMesh, 4, MatteVC);

	// ============================== Driver ==============================
	// Reclined ~25 deg with the seat (spine axis), head held level looking forward, gloved hands on the rim at 9 and 3,
	// knees splayed round the wheel, boots on the pedals. Animated parts (AnimateDriver): the torso pivots at the
	// pelvis, the helmet at the neck, and the arm segments / hands are separate components placed every frame.
	FKartMeshBuilder Dr, Gl, Tr, TrG, Hm;
	const FLinearColor Seam(0.035f, 0.035f, 0.04f);
	const double Recline = FMath::DegreesToRadians(25.0);
	const FVector SpineDir(-FMath::Sin(Recline), 0.0, FMath::Cos(Recline));     // spine direction (component space: x fwd, z up)
	const FVector Fd(FMath::Cos(Recline), 0.0, FMath::Sin(Recline));      // chest-forward direction, normal to the spine
	const FVector Pelvis = K(0.37, 0.0, 0.12);
	const float SpineLen = 52.0f;                                          // pelvis -> neck base (cm): 50th-percentile adult, seated
	const FVector Chest = Pelvis + SpineDir * SpineLen;
	const FVector NeckTop = Chest + SpineDir * 7.0f;
	auto SuitColour = [&](const FVector& Q) -> FLinearColor
	{
		const double Ay = FMath::Abs(Q.Y);
		if (Q.Z < 0.30 && Q.X > 0.62) { return Blk; }                             // lower legs
		if (Q.Z > 0.52 && Ay > 0.085) { return Pearl; }                           // shoulder yoke
		if (Ay > 0.13 && Q.Z > 0.28 && Q.Z < 0.37) { return Pearl; }              // side band
		return Prim;
	};
	auto ColFn = [&](int32, int32, const FVector& P) { return SuitColour(KI(P)); };

	// --- Legs, pelvis, boots (static: DriverMesh) ---
	Dr.AddEllipsoid(Pelvis, FVector(12.0f, 15.5f, 9.0f), FQuat(FVector::RightVector, -static_cast<float>(Recline)), 12, 24, ColFn);
	for (double S : {1.0, -1.0})
	{
		const FVector Hip = K(0.38, 0.09 * S, 0.115);
		const FVector Ankle = K(1.215, 0.15 * S, 0.12);   // heel on the floor tray, sole on the pedal (imported kart: pedals x 1.23-1.32 m, y +-0.15 m)
		const FVector Knee = SolveJoint(Hip, Ankle, 46.0f, 45.0f, Ya * (0.5f * S) + Za * 0.87f);
		{ const int32 Si = S > 0.0 ? 0 : 1; DRig.Hip[Si] = Hip; DRig.Knee[Si] = Knee; DRig.Ankle[Si] = Ankle; }
		Dr.AddTaperTube(Hip, Knee, 6.6f, 5.0f, 24, Prim);
		Dr.AddTaperTube(Knee, Ankle, 4.6f, 3.6f, 24, Prim);
		const FVector KneeOut = (Knee - FMath::Lerp(Hip, Ankle, 0.5f)).GetSafeNormal();
		Dr.AddEllipsoid(Knee + KneeOut * 0.8f, FVector(5.2f, 5.2f, 4.2f), FRotationMatrix::MakeFromZ(KneeOut).ToQuat(), 10, 18, [&](int32, int32, const FVector&) { return Prim; });
		const FQuat FootRot(FRotator(40.0f, 0.0f, 0.0f));  // foot tilted up against the pedal face
		Gl.AddEllipsoid(Ankle + FVector(6.0f, 0.0f, 1.5f), FVector(12.0f, 4.3f, 4.6f), FootRot, 12, 20, [&](int32, int32, const FVector&) { return Blk; });
		Gl.AddRoundBox(Ankle + FVector(6.0f, 0.0f, -2.4f), FVector(12.0f, 4.2f, 0.7f), FootRot, 0.5f, Graphite);
	}

	// --- Torso (TorsoMesh, pivots at the pelvis): rings normal to the reclined spine, rib protector, straps, zip, neck ---
	{
		struct FSec { float T, W, D; };
		const FSec Secs[] = {{0.0f, 15.0f, 10.5f}, {0.12f, 14.2f, 10.0f}, {0.25f, 13.4f, 9.6f}, {0.38f, 13.2f, 9.6f}, {0.5f, 14.2f, 10.2f},
		                     {0.62f, 15.4f, 10.8f}, {0.74f, 16.3f, 10.8f}, {0.84f, 16.5f, 10.3f}, {0.92f, 15.5f, 9.4f}, {0.97f, 12.5f, 7.8f}, {1.0f, 8.5f, 6.0f}};
		TArray<TArray<FVector>> Rings, Rib;
		for (const FSec& Sc : Secs)
		{
			const FVector Pc = Pelvis + SpineDir * (SpineLen * Sc.T);
			Rings.Add(FKartMeshBuilder::SuperEllipse(Pc, Fd, Ya, Sc.D, Sc.W, 2.4f, 64));
			if (Sc.T >= 0.38f && Sc.T <= 0.62f) { Rib.Add(FKartMeshBuilder::SuperEllipse(Pc, Fd, Ya, Sc.D * 1.05f, Sc.W * 1.05f, 2.4f, 64)); }
		}
		Tr.AddLoft(Rings, true, true, ColFn);
		TrG.AddLoft(Rib, true, true, nullptr, Graphite);
		for (double S : {1.0, -1.0})
		{
			const FVector Lat = Ya * static_cast<float>(S);
			TrG.AddTubePath({Pelvis + SpineDir * 19.0f + Fd * 9.8f + Lat * 9.0f, Pelvis + SpineDir * 36.0f + Fd * 9.4f + Lat * 10.5f, Chest + Fd * 1.5f + Lat * 9.5f,
			                 Chest - Fd * 5.0f + Lat * 8.0f, Pelvis + SpineDir * 38.0f - Fd * 9.5f + Lat * 8.0f}, 0.8f, 8, Graphite);
		}
		TrG.AddTubePath({Pelvis + SpineDir * 8.0f + Fd * 10.4f, Pelvis + SpineDir * 47.0f + Fd * 10.6f}, 0.22f, 6, Seam);        // zip
		Tr.AddTaperTube(Chest - SpineDir * 1.0f, NeckTop, 5.4f, 5.0f, 20, Prim);                                          // neck
		TrG.AddTorus(Chest + SpineDir * 2.0f, SpineDir, 8.2f, 2.4f, 32, 12, Graphite);                                         // neck brace
	}

	// --- Helmet (HelmetMesh, pivots at the neck): held level, looking forward ---
	const FVector HelmetC = NeckTop + Xa * 3.0f + Za * 9.5f;
	{
		const FVector HQ = KI(HelmetC);
		TArray<TArray<FVector>> Rings;
		const int32 NT = 40;
		for (int32 I = 0; I < NT; ++I)
		{
			const double T = 0.01 + 0.98 * I / (NT - 1);
			const double U = 2.0 * T - 1.0;
			const double E = U < 0.0 ? 2.4 : 3.4;
			const double EndF = FMath::Pow(FMath::Max(0.0, 1.0 - FMath::Pow(FMath::Abs(U), E)), 0.5);
			const double Top = 0.142 * FMath::Sqrt(FMath::Max(0.0, 1.0 - FMath::Square((U - 0.08) / 1.08)));
			const double Bot = -(0.108 + 0.05 * FMath::SmoothStep(0.45, 0.85, T));
			const double Zc = 0.5 * (Top + Bot), Hz = 0.5 * (Top - Bot) * FMath::Pow(EndF, 0.55);
			Rings.Add(RingYZ(HQ.X - 0.165 + 0.31 * T, 0.0, HQ.Z + Zc, 0.128 * EndF, FMath::Max(Hz, 0.004), 2.25f, 72));
		}
		Hm.AddLoft(Rings, true, true, [&](int32, int32, const FVector& P)
		{
			const FVector Q = KI(P);
			const double Rx = Q.X - HQ.X, Rz = Q.Z - HQ.Z, Ay = FMath::Abs(Q.Y);
			if (Rx > 0.055 && Rz > -0.048 && Rz < 0.043 && Ay < 0.105) { return FLinearColor(0.012f, 0.016f, 0.026f); }
			if (Rx > 0.09 && Rz < -0.085 && Ay < 0.035) { return Blk; }
			if (Rz < -0.10) { return Blk; }
			if (Ay < 0.024 && Rz > 0.055) { return Prim; }
			if (Rz > -0.012 && Rz < 0.012 && Rx < 0.04) { return Prim; }
			return Pearl;
		});
		for (double S : {1.0, -1.0})
		{
			Hm.AddRoundBox(K(HQ.X + 0.03, 0.045 * S, HQ.Z + 0.138), FVector(2.4f, 1.2f, 0.7f), FQuat::Identity, 0.5f, Blk);
			Hm.AddCylinder(K(HQ.X + 0.035, 0.124 * S, HQ.Z), K(HQ.X + 0.035, 0.131 * S, HQ.Z), 1.6f, 14, Graphite);
		}
		Hm.AddRoundBox(K(HQ.X - 0.152, 0.0, HQ.Z + 0.065), FVector(1.4f, 7.5f, 1.0f), FQuat(FRotator(-25.0f, 0.0f, 0.0f)), 0.6f, Prim);
		// Eye just behind the visor (cockpit camera; moved with the head by AnimateDriver).
		DRig.Eye = HelmetC + Xa * 9.5f + Za * 6.5f;
	}

	// --- Arms and hands: built in their own local frames (bone along +X; hand in its grip frame) ---
	const FVector Ax3 = (ColTop - ColBase).GetSafeNormal();
	const FVector WheelUp = (Za - Ax3 * FVector::DotProduct(Za, Ax3)).GetSafeNormal();
	DRig.UpperLen = 31.0f;
	DRig.ForeLen = 28.0f;
	FKartMeshBuilder ArmB[6];
	for (int32 Side = 0; Side < 2; ++Side)
	{
		const float S = Side == 0 ? 1.0f : -1.0f;                                    // 0 = left
		DRig.Shoulder[Side] = Chest - SpineDir * 4.0f + Ya * (16.5f * S);
		DRig.Grip[Side] = ColTop + Ya * (15.5f * S) + Ax3 * 0.5f;
		DRig.Wrist[Side] = DRig.Grip[Side] - Ax3 * 5.5f - Ya * (1.0f * S);
		DRig.ElbowHint[Side] = Ya * (0.6f * S) - Za * 0.8f;
		DRig.GripRot[Side] = FRotationMatrix::MakeFromXZ(Ax3, WheelUp).ToQuat();
		const FLinearColor PrimC = Prim;
		// Upper arm: shoulder cap at the origin, sleeve to the elbow.
		ArmB[Side].AddEllipsoid(FVector::ZeroVector, FVector(5.8f, 5.6f, 5.4f), FQuat::Identity, 12, 20, [&](int32, int32, const FVector&) { return Pearl; });
		ArmB[Side].AddTaperTube(FVector::ZeroVector, FVector(DRig.UpperLen, 0, 0), 4.9f, 4.0f, 24, PrimC);
		// Forearm: elbow to wrist, glove gauntlet at the end.
		ArmB[2 + Side].AddSphere(FVector::ZeroVector, 4.1f, 10, 20, PrimC);
		ArmB[2 + Side].AddTaperTube(FVector::ZeroVector, FVector(DRig.ForeLen, 0, 0), 4.0f, 3.2f, 24, PrimC);
		ArmB[2 + Side].AddTaperTube(FVector(DRig.ForeLen - 4.0f, 0, 0), FVector(DRig.ForeLen + 1.0f, 0, 0), 3.9f, 3.5f, 20, Blk);
		// Hand, built at its rest place, then moved into the grip frame (origin on the rim).
		FKartMeshBuilder& Hb = ArmB[4 + Side];
		const FVector Grip = DRig.Grip[Side];
		Hb.AddEllipsoid(Grip - Ax3 * 2.4f - Ya * (0.5f * S), FVector(4.2f, 3.0f, 4.6f), FRotationMatrix::MakeFromZX(WheelUp, Ax3).ToQuat(), 8, 14,
		               [&](int32, int32, const FVector&) { return Blk; });
		for (float Off : {-2.7f, -0.9f, 0.9f, 2.7f}) { Hb.AddTorus(Grip + WheelUp * Off, WheelUp, 2.2f, 0.85f, 16, 8, Blk); }
		Hb.AddTaperTube(Grip + WheelUp * 4.6f - Ax3 * 2.0f, Grip + WheelUp * 3.8f + Ax3 * 1.5f, 1.2f, 1.0f, 8, Blk);
		Hb.AddTaperTube(DRig.Wrist[Side], Grip - Ax3 * 2.0f, 3.6f, 3.2f, 16, Blk);       // back of the glove to the wrist
		const FQuat Inv = DRig.GripRot[Side].Inverse();
		for (int32 V = 0; V < Hb.V.Num(); ++V) { Hb.V[V] = Inv.RotateVector(Hb.V[V] - Grip); Hb.N[V] = Inv.RotateVector(Hb.N[V]); }
	}
	DRig.Pelvis = Pelvis;
	DRig.HelmetC = HelmetC;
	DRig.Neck = Chest + SpineDir * 2.0f;
	DRig.ColTop = ColTop;
	DRig.ColAxis = Ax3;
	DRig.bValid = true;
	AnimRoll = AnimPitch = AnimYaw = 0.0f;
	bAnimHasPrev = false;

	DriverMesh->ClearAllMeshSections();
	Dr.ToSection(DriverMesh, 0, SuitVC);
	Gl.ToSection(DriverMesh, 1, MatteVC);
	TorsoMesh->ClearAllMeshSections();
	Tr.ToSection(TorsoMesh, 0, SuitVC);
	TrG.ToSection(TorsoMesh, 1, MatteVC);
	HelmetMesh->ClearAllMeshSections();
	Hm.ToSection(HelmetMesh, 0, GlossVC);
	for (int32 I = 0; I < 6 && I < ArmMeshes.Num(); ++I)
	{
		ArmMeshes[I]->ClearAllMeshSections();
		ArmB[I].ToSection(ArmMeshes[I], 0, I >= 4 ? MatteVC : SuitVC);
	}

	// ============================== Steering wheel ==============================
	// Built around local Z (column axis), local +X = top of the wheel. Flat three-spoke aluminium wheel, contoured
	// rubber grips at 9 and 3, a data-logger display on the hub (generic, no brand).
	FKartMeshBuilder Sw, Sg, Sd;
	{
		const float RimR = 14.0f;
		TArray<FVector> Ring;
		for (int32 I = 0; I <= 48; ++I)
		{
			const float A = 2.0f * PI * I / 48;
			Ring.Add(FVector(RimR * FMath::Cos(A), RimR * FMath::Sin(A), 0.0f));
		}
		Sw.AddTubePath(Ring, 1.0f, 10);
		for (float Mid : {0.5f * PI, 1.5f * PI})
		{
			TArray<FVector> Grip;
			for (int32 I = -6; I <= 6; ++I) { const float A = Mid + 0.075f * I; Grip.Add(FVector(RimR * FMath::Cos(A), RimR * FMath::Sin(A), 0.0f)); }
			for (int32 I = 0; I + 1 < Grip.Num(); ++I)
			{
				const float W = 1.0f - FMath::Abs(I - 5.5f) / 6.0f;          // thickest in the middle of the grip
				Sg.AddTaperTube(Grip[I], Grip[I + 1], 1.7f + 0.5f * W, 1.7f + 0.5f * W, 12);
			}
		}
		// Spokes: flat plates at 9, 3 and 6 o'clock, joined by a centre plate.
		for (float A : {0.5f * PI, 1.5f * PI, PI})
		{
			const FVector Dir(FMath::Cos(A), FMath::Sin(A), 0.0f);
			Sw.AddBox(Dir * (0.5f * RimR), FVector(0.5f * RimR * FMath::Abs(Dir.X) + 1.6f * FMath::Abs(Dir.Y), 0.5f * RimR * FMath::Abs(Dir.Y) + 1.6f * FMath::Abs(Dir.X), 0.35f));
		}
		Sw.AddCylinder(FVector(0, 0, -0.4f), FVector(0, 0, 0.4f), 5.5f, 24);       // centre plate
		Sw.AddCylinder(FVector(0, 0, -4.5f), FVector(0, 0, -0.4f), 2.6f, 16);      // boss
		for (int32 Bi = 0; Bi < 3; ++Bi) { const float A = 2.0f * PI * Bi / 3; Sw.AddCylinder(FVector(3.6f * FMath::Cos(A), 3.6f * FMath::Sin(A), 0.4f), FVector(3.6f * FMath::Cos(A), 3.6f * FMath::Sin(A), 0.9f), 0.5f, 8); }
		// Data display on the top of the hub: housing, dark screen, shift-light LEDs.
		Sd.AddRoundBox(FVector(4.2f, 0.0f, 1.8f), FVector(3.0f, 6.0f, 1.3f), FQuat::Identity, 0.8f, Graphite);
		Sd.AddBox(FVector(4.4f, 0.0f, 3.12f), FVector(2.0f, 4.6f, 0.05f), FQuat::Identity, FLinearColor(0.01f, 0.02f, 0.03f));
		for (int32 Ld = 0; Ld < 6; ++Ld)
		{
			const FLinearColor LedC = Ld < 2 ? FLinearColor(0.1f, 0.8f, 0.2f) : (Ld < 4 ? FLinearColor(0.9f, 0.6f, 0.05f) : FLinearColor(0.9f, 0.1f, 0.05f));
			Sd.AddCylinder(FVector(6.6f, -3.0f + 1.2f * Ld, 2.6f), FVector(6.6f, -3.0f + 1.2f * Ld, 3.2f), 0.35f, 8, LedC);
		}
	}
	SteeringWheelMesh->ClearAllMeshSections();
	Sw.ToSection(SteeringWheelMesh, 0, Chrome);
	Sg.ToSection(SteeringWheelMesh, 1, Rubber);
	Sd.ToSection(SteeringWheelMesh, 2, MatteVC);
	SteeringAxisLocal = (ColTop - ColBase).GetSafeNormal();
	SteeringAlign = FRotationMatrix::MakeFromZX(SteeringAxisLocal, FVector::ForwardVector).ToQuat();
	SteeringWheelMesh->SetRelativeLocation(ColTop);
	SteeringWheelMesh->SetRelativeRotation(SteeringAlign);

	// ============================== Wheels and rear axle ==============================
	for (int32 I = 0; I < kartsim::NumWheels; ++I)
	{
		const kartsim::TyreParams& T = I < 2 ? C.FrontTyre : C.RearTyre;
		const bool bRight = I == kartsim::WheelFR || I == kartsim::WheelRR;
		KartVisuals::BuildWheel(WheelMeshes[I], static_cast<float>(T.UnloadedRadius), static_cast<float>(T.Width), 0.0635f, Rubber, Alloy, bRight); // 5 in rims
		WheelMeshes[I]->SetWorldScale3D(FVector::OneVector);
	}
	FKartMeshBuilder AxB;
	AxB.AddTube(FVector(0, 0, -50.0f), FVector(0, 0, 50.0f), 2.5f, 12); // unit length 1 m, scaled per frame
	AxleMesh->ClearAllMeshSections();
	AxB.ToSection(AxleMesh, 0, Chrome);

	// Driver's eye: inside the helmet's front, looking over the steering wheel (pitch / FOV adjustable in game:
	// Page Up/Down, Home/End; saved).
	CockpitCamera->SetRelativeLocation(DRig.Eye);
	CockpitCamera->SetRelativeRotation(FRotator(CockpitPitch, 0.0f, 0.0f));
	CockpitCamera->FieldOfView = CockpitFov;

	ApplyVisualRig();
	if (FParse::Param(FCommandLine::Get(), TEXT("KartDumpDriverRig"))) { DumpDriverRig(); }
}
