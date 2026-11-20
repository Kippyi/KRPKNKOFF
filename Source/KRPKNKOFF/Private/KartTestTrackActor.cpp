#include "KartTestTrackActor.h"

#include "HAL/FileManager.h"
#include "KartSim/Json.h"
#include "KartSim/RacingLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

#include "Components/InstancedStaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "KartProceduralVisuals.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "ProceduralMeshComponent.h"
#include "UObject/ConstructorHelpers.h"

namespace
{
	constexpr float Cm = 100.0f;
	constexpr float AsphaltZ = 0.005f; // asphalt 5 mm above the grass plane: a real edge lip

	struct FMeshBuffers
	{
		TArray<FVector> V;
		TArray<int32> Tri;
		TArray<FVector> N;
		TArray<FVector2D> UV;
		TArray<FLinearColor> Col;
		TArray<FProcMeshTangent> Tan;

		int32 AddVertex(const FVector& P, const FVector2D& InUV)
		{
			V.Add(P);
			N.Add(FVector::UpVector);
			UV.Add(InUV);
			return V.Num() - 1;
		}
		// Adds a triangle facing up (+Z) regardless of the order the points were given in.
		void AddTri(int32 A, int32 B, int32 C)
		{
			const FVector AB = V[B] - V[A], AC = V[C] - V[A];
			const double Cross = AB.X * AC.Y - AB.Y * AC.X;
			if (Cross > 0.0) { Swap(B, C); }
			Tri.Add(A); Tri.Add(B); Tri.Add(C);
		}
		void AddQuad(int32 A, int32 B, int32 C, int32 D) { AddTri(A, B, C); AddTri(A, C, D); }
	};

	FVector2D CatmullRom(const FVector2D& P0, const FVector2D& P1, const FVector2D& P2, const FVector2D& P3, float T)
	{
		const float T2 = T * T, T3 = T2 * T;
		return 0.5f * ((2.0f * P1) + (-P0 + P2) * T + (2.0f * P0 - 5.0f * P1 + 4.0f * P2 - P3) * T2 + (-P0 + 3.0f * P1 - 3.0f * P2 + P3) * T3);
	}

	void ConfigureCollision(UProceduralMeshComponent* M)
	{
		M->bUseComplexAsSimpleCollision = true;
		M->SetCollisionObjectType(ECC_WorldStatic);
		M->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
		M->SetCollisionResponseToAllChannels(ECR_Block);
	}
}

AKartTestTrackActor::AKartTestTrackActor()
{
	PrimaryActorTick.bCanEverTick = false;

	GrassMesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("Grass"));
	SetRootComponent(GrassMesh);
	AsphaltMesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("Asphalt"));
	AsphaltMesh->SetupAttachment(GrassMesh);
	KerbMesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("Kerbs"));
	KerbMesh->SetupAttachment(GrassMesh);

	GrassMesh->ComponentTags.Add(TEXT("KartSurface.Grass"));
	// (Procedural-mesh complex collision returns no physical material in UE 5.8, so procedural surfaces use
	// tags; the PM_Kart_<Surface> convention for imported static meshes is verified by -KartSurfaceSelfTest.)
	AsphaltMesh->ComponentTags.Add(TEXT("KartSurface.Asphalt"));
	KerbMesh->ComponentTags.Add(TEXT("KartSurface.Kerb"));
	ConfigureCollision(GrassMesh);
	ConfigureCollision(AsphaltMesh);
	ConfigureCollision(KerbMesh);

	static ConstructorHelpers::FObjectFinder<UMaterialInterface> MatFinder(TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	BaseMaterial = MatFinder.Object;
	static ConstructorHelpers::FObjectFinder<UStaticMesh> CylFinder(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
	CylinderMesh = CylFinder.Object;
	BarrierTyres = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("BarrierTyres"));
	BarrierTyres->SetupAttachment(GrassMesh);
	BarrierTyres->SetCollisionEnabled(ECollisionEnabled::NoCollision); // visual only (see BuildTrack)
	BarrierTyres->SetStaticMesh(CylinderMesh);

	// Default layout (metres): ~520 m with a hairpin, a fast sweeper, an esses section and a long straight.
	ControlPoints = {
		{0, 0}, {60, 0}, {95, 8}, {110, 30}, {105, 55}, {80, 65}, {55, 58}, {40, 40},
		{20, 35}, {0, 45}, {-15, 70}, {-40, 78}, {-65, 65}, {-70, 40}, {-55, 20}, {-30, 5}};
}

void AKartTestTrackActor::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
	BuildTrack();
}

void AKartTestTrackActor::BeginPlay()
{
	Super::BeginPlay();
	if (Samples.Num() == 0)
	{
		BuildTrack();
	}
}

FTransform AKartTestTrackActor::GetStartTransform() const
{
	if (Samples.Num() < 2)
	{
		return GetActorTransform();
	}
	const int32 I = FMath::Min(6, Samples.Num() - 1); // a few metres after the line
	const FVector Local(Samples[I].X * Cm, Samples[I].Y * Cm, (SampleElev.IsValidIndex(I) ? SampleElev[I] * Cm : 0.0f) + 30.0f);
	const FRotator Rot(0.0f, FMath::RadiansToDegrees(FMath::Atan2(Tangents[I].Y, Tangents[I].X)), 0.0f);
	return FTransform(Rot, Local) * GetActorTransform();
}

void AKartTestTrackActor::GetTimingLines(TArray<FVector>& OutA, TArray<FVector>& OutB, TArray<FVector>& OutForward) const
{
	OutA.Reset(); OutB.Reset(); OutForward.Reset();
	const int32 N = Samples.Num();
	if (N < 3) { return; }
	const FTransform& X = GetActorTransform();
	for (int32 K = 0; K < 3; ++K)
	{
		const int32 I = (K * N) / 3;
		const float HalfSpan = (SampleHalfW.IsValidIndex(I) ? SampleHalfW[I] : 0.5f * TrackWidth) + KerbWidth + 2.0f; // track + kerbs + run-off
		const FVector2D L(-Tangents[I].Y, Tangents[I].X);
		const float Z = SampleElev.IsValidIndex(I) ? SampleElev[I] * Cm : 0.0f;
		OutA.Add(X.TransformPosition(FVector((Samples[I] + L * HalfSpan) * Cm, Z)));
		OutB.Add(X.TransformPosition(FVector((Samples[I] - L * HalfSpan) * Cm, Z)));
		OutForward.Add(X.TransformVectorNoScale(FVector(Tangents[I], 0.0f)));
	}
}

void AKartTestTrackActor::GetCentreLine(TArray<FVector>& OutPoints) const
{
	OutPoints.Reset();
	for (const FVector2D& P : Samples)
	{
		const int32 I = OutPoints.Num();
		OutPoints.Add(GetActorTransform().TransformPosition(FVector(P * Cm, SampleElev.IsValidIndex(I) ? SampleElev[I] * Cm : 0.0f)));
	}
}

void AKartTestTrackActor::BuildTrack()
{
	const int32 NCtl = ControlPoints.Num();
	if (NCtl < 4)
	{
		return;
	}

	// --- Sample the closed centre line at ~SampleSpacing ---
	TArray<FVector2D> Dense;
	for (int32 I = 0; I < NCtl; ++I)
	{
		const FVector2D& P0 = ControlPoints[(I - 1 + NCtl) % NCtl];
		const FVector2D& P1 = ControlPoints[I];
		const FVector2D& P2 = ControlPoints[(I + 1) % NCtl];
		const FVector2D& P3 = ControlPoints[(I + 2) % NCtl];
		for (int32 K = 0; K < 200; ++K)
		{
			Dense.Add(CatmullRom(P0, P1, P2, P3, K / 200.0f));
		}
	}
	Samples.Reset();
	Samples.Add(Dense[0]);
	TArray<int32> SampleDense; // dense index of each sample (-> control segment and parameter)
	SampleDense.Add(0);
	float Acc = 0.0f;
	TrackLength = 0.0f;
	for (int32 I = 1; I <= Dense.Num(); ++I)
	{
		const FVector2D& P = Dense[I % Dense.Num()];
		const float D = FVector2D::Distance(P, Dense[I - 1]);
		Acc += D;
		TrackLength += D;
		if (Acc >= SampleSpacing && I < Dense.Num())
		{
			Samples.Add(P);
			SampleDense.Add(I);
			Acc = 0.0f;
		}
	}
	const int32 N = Samples.Num();
	Tangents.SetNum(N);
	TArray<float> Curvature;
	Curvature.SetNum(N);
	for (int32 I = 0; I < N; ++I)
	{
		Tangents[I] = (Samples[(I + 1) % N] - Samples[(I - 1 + N) % N]).GetSafeNormal();
	}
	for (int32 I = 0; I < N; ++I)
	{
		const FVector2D& T0 = Tangents[(I - 1 + N) % N];
		const FVector2D& T1 = Tangents[(I + 1) % N];
		const float Ds = FVector2D::Distance(Samples[(I + 1) % N], Samples[(I - 1 + N) % N]);
		Curvature[I] = (T0.X * T1.Y - T0.Y * T1.X) / FMath::Max(Ds, 0.01f); // signed, +: turning toward left normal
	}

	auto Left = [this](int32 I) { return FVector2D(-Tangents[I].Y, Tangents[I].X); };
	auto Make3 = [](const FVector2D& P, float Z) { return FVector(P.X * Cm, P.Y * Cm, Z * Cm); };

	// --- Camber: signed bank per sample (rad, + raises the LEFT edge), blended between control points.
	//     Layout camber: + = banked (outside of the corner raised), - = off-camber. The lower edge stays at
	//     the asphalt lip height; the raised edge gets a grass verge down to the ground. ---
	const int32 NCtlPts = ControlPoints.Num();
	TArray<float> CtlBank;
	CtlBank.SetNum(NCtlPts);
	for (int32 C = 0; C < NCtlPts; ++C)
	{
		const FVector2D A = ControlPoints[C] - ControlPoints[(C - 1 + NCtlPts) % NCtlPts];
		const FVector2D B = ControlPoints[(C + 1) % NCtlPts] - ControlPoints[C];
		const float Turn = A.X * B.Y - A.Y * B.X; // + = turning toward the left normal
		const float Cam = ControlCamber.IsValidIndex(C) ? ControlCamber[C] : 0.0f;
		CtlBank[C] = -FMath::DegreesToRadians(Cam) * (Turn >= 0.0f ? 1.0f : -1.0f);
	}
	TArray<float> Bank;
	TArray<int32> NearCtl;
	Bank.SetNum(N);
	NearCtl.SetNum(N);
	for (int32 I = 0; I < N; ++I)
	{
		const int32 Seg = (SampleDense[I] / 200) % NCtlPts;
		const float T = (SampleDense[I] % 200) / 200.0f;
		Bank[I] = FMath::Lerp(CtlBank[Seg], CtlBank[(Seg + 1) % NCtlPts], T * T * (3.0f - 2.0f * T));
		NearCtl[I] = T < 0.5f ? Seg : (Seg + 1) % NCtlPts;
	}
	// Elevation: Catmull-Rom through the control-point heights (C1-smooth crests and dips).
	SampleElev.SetNum(N);
	auto CtlE = [this, NCtlPts](int32 C) { const int32 K = (C % NCtlPts + NCtlPts) % NCtlPts; return ControlElev.IsValidIndex(K) ? ControlElev[K] : 0.0f; };
	for (int32 I = 0; I < N; ++I)
	{
		const int32 Seg = (SampleDense[I] / 200) % NCtlPts;
		const float T = (SampleDense[I] % 200) / 200.0f;
		const float P0 = CtlE(Seg - 1), P1 = CtlE(Seg), P2 = CtlE(Seg + 1), P3 = CtlE(Seg + 2);
		SampleElev[I] = 0.5f * ((2.0f * P1) + (-P0 + P2) * T + (2.0f * P0 - 5.0f * P1 + 4.0f * P2 - P3) * T * T + (-P0 + 3.0f * P1 - 3.0f * P2 + P3) * T * T * T);
	}
	// Width: Catmull-Rom through the per-control-point widths (measured layouts vary 8-10.5 m); else TrackWidth.
	SampleHalfW.SetNum(N);
	auto CtlW = [this, NCtlPts](int32 C) { const int32 K = (C % NCtlPts + NCtlPts) % NCtlPts; return 0.5f * (ControlWidth.IsValidIndex(K) ? ControlWidth[K] : TrackWidth); };
	float HalfWMax = 0.0f;
	for (int32 I = 0; I < N; ++I)
	{
		const int32 Seg = (SampleDense[I] / 200) % NCtlPts;
		const float T = (SampleDense[I] % 200) / 200.0f;
		const float P0 = CtlW(Seg - 1), P1 = CtlW(Seg), P2 = CtlW(Seg + 1), P3 = CtlW(Seg + 2);
		SampleHalfW[I] = FMath::Max(1.0f, 0.5f * ((2.0f * P1) + (-P0 + P2) * T + (2.0f * P0 - 5.0f * P1 + 4.0f * P2 - P3) * T * T + (-P0 + 3.0f * P1 - 3.0f * P2 + P3) * T * T * T));
		HalfWMax = FMath::Max(HalfWMax, SampleHalfW[I]);
	}
	auto EdgeZ = [&](int32 I, float SideSign)
	{
		const float Tb = FMath::Tan(Bank[I]);
		return AsphaltZ + SampleElev[I] + SampleHalfW[I] * FMath::Abs(Tb) + SideSign * SampleHalfW[I] * Tb;
	};

	// --- Asphalt surface: rows every SampleSpacing/4 along the track, 8 strips across, each vertex computed
	//     exactly from the interpolated bank and elevation. (One quad per 0.75 m sample with a changing bank is
	//     not planar: its diagonal folds by a few mm, which the tyres felt as a ~25 Hz rumble everywhere on
	//     banked sections - user report + wobble telemetry, Docs/Milestone7MenusSetupSound.md.) ---
	// --- Surface wear masks for the asphalt material (visual only, M_KartAsphalt; Tools/CreateTrackMaterials.py).
	//     Vertex colour: R = racing-line lateral position in UV.x space (0 = right edge, 1 = left edge),
	//     G = half width / 10 m, B = corner / braking-zone intensity (tyre marks), A = 1.
	//     The line is the same minimum-curvature line the AI drives (ComputeRacingLine, 0.9 m margin); the sign of
	//     its offset is matched to this actor's left normal by correlating with the curvature (apex = inside). ---
	TArray<float> LineU, WearB;
	{
		std::vector<kartsim::Vec3> C;
		C.reserve(N);
		for (int32 I = 0; I < N; ++I) { C.emplace_back(Samples[I].X, Samples[I].Y, SampleElev[I]); }
		const kartsim::RacingLine RL = kartsim::ComputeRacingLine(C, 0.5 * TrackWidth, 0.9);
		double Corr = 0.0;
		for (int32 I = 0; I < N && I < static_cast<int32>(RL.Offset.size()); ++I) { Corr += RL.Offset[I] * Curvature[I]; }
		const float Sign = Corr >= 0.0 ? 1.0f : -1.0f;
		LineU.SetNum(N);
		for (int32 I = 0; I < N; ++I)
		{
			const float Off = I < static_cast<int32>(RL.Offset.size()) ? Sign * static_cast<float>(RL.Offset[I]) : 0.0f;
			LineU[I] = FMath::Clamp((Off + SampleHalfW[I]) / (2.0f * SampleHalfW[I]), 0.0f, 1.0f);
		}
		// Rubber and tyre marks gather in braking zones (up to ~22 m before a corner) and through it.
		const int32 Before = FMath::CeilToInt(22.0f / SampleSpacing), After = FMath::CeilToInt(8.0f / SampleSpacing);
		WearB.SetNum(N);
		for (int32 I = 0; I < N; ++I)
		{
			float M = 0.0f;
			for (int32 K = -After; K <= Before; ++K) { M = FMath::Max(M, FMath::Abs(Curvature[(I + K + N) % N])); }
			WearB[I] = FMath::Clamp(M * 14.0f, 0.0f, 1.0f); // 1 at a ~14 m radius or tighter
		}
	}

	FMeshBuffers Asphalt;
	{
		constexpr int32 SubRows = 4;
		constexpr int32 Strips = 8;
		const int32 Rows = N * SubRows;
		for (int32 Rw = 0; Rw < Rows; ++Rw)
		{
			const int32 I = Rw / SubRows;
			const int32 J = (I + 1) % N;
			const float A = static_cast<float>(Rw % SubRows) / SubRows;
			const FVector2D C = FMath::Lerp(Samples[I], Samples[J], A);
			const FVector2D Dir = FMath::Lerp(Left(I), Left(J), A).GetSafeNormal();
			const float Tb = FMath::Tan(FMath::Lerp(Bank[I], Bank[J], A));
			const float Hw = FMath::Lerp(SampleHalfW[I], SampleHalfW[J], A);
			const float Base = AsphaltZ + FMath::Lerp(SampleElev[I], SampleElev[J], A) + Hw * FMath::Abs(Tb);
			const FLinearColor Wear(FMath::Lerp(LineU[I], LineU[J], A), Hw / 10.0f, FMath::Lerp(WearB[I], WearB[J], A), 1.0f);
			for (int32 K = 0; K <= Strips; ++K)
			{
				const float L = -Hw + 2.0f * Hw * K / Strips; // lateral, + = left
				Asphalt.AddVertex(Make3(C + Dir * L, Base + L * Tb), FVector2D(static_cast<float>(K) / Strips, static_cast<float>(Rw) / SubRows));
				Asphalt.Col.Add(Wear);
			}
		}
		for (int32 Rw = 0; Rw < Rows; ++Rw)
		{
			const int32 R0 = Rw * (Strips + 1), R1 = ((Rw + 1) % Rows) * (Strips + 1);
			for (int32 K = 0; K < Strips; ++K) { Asphalt.AddQuad(R0 + K, R0 + K + 1, R1 + K + 1, R1 + K); }
		}
	}
	AsphaltMesh->CreateMeshSection_LinearColor(0, Asphalt.V, Asphalt.Tri, Asphalt.N, Asphalt.UV, Asphalt.Col, Asphalt.Tan, true);

	// --- Start/finish line (visual only, sits 1 mm above asphalt; no collision) ---
	FMeshBuffers Line;
	{
		const FVector2D C = Samples[0], T = Tangents[0], Lf = Left(0);
		const float Hw0 = SampleHalfW[0];
		const FVector2D A = C + Lf * Hw0 - T * 0.15f, B = C - Lf * Hw0 - T * 0.15f;
		const FVector2D Cc = C - Lf * Hw0 + T * 0.15f, D = C + Lf * Hw0 + T * 0.15f;
		const int32 Ia = Line.AddVertex(Make3(A, EdgeZ(0, 1.0f) + 0.001f), FVector2D(0, 0));
		const int32 Ib = Line.AddVertex(Make3(B, EdgeZ(0, -1.0f) + 0.001f), FVector2D(1, 0));
		const int32 Ic = Line.AddVertex(Make3(Cc, EdgeZ(0, -1.0f) + 0.001f), FVector2D(1, 1));
		const int32 Id = Line.AddVertex(Make3(D, EdgeZ(0, 1.0f) + 0.001f), FVector2D(0, 1));
		Line.AddQuad(Ia, Ib, Ic, Id);
	}
	AsphaltMesh->CreateMeshSection_LinearColor(1, Line.V, Line.Tri, Line.N, Line.UV, Line.Col, Line.Tan, false);

	// --- Kerbs: a type per sample and side (0 = left, 1 = right), then tapered runs. Types (layout file):
	//     flat 25 mm rideable / rumble saw-tooth ridges 0.25 m apart / high: a rounded 60 mm hump you can jump
	//     to straight-line a chicane (the kart goes light or airborne). Without explicit kerbs: automatic flat
	//     kerbs on the inside of tight corners (gaps up to 3 m merged). Every run tapers from flush to full
	//     height over ~2 m: abrupt kerb ends launched karts (user telemetry, Docs/Milestone7MenusSetupSound.md). ---
	TArray<EKerbType> KType[2];
	KType[0].Init(EKerbType::None, N);
	KType[1].Init(EKerbType::None, N);
	const float Threshold = 1.0f / FMath::Max(KerbRadiusThreshold, 1.0f);
	if (KerbSpecs.Num() == 0)
	{
		TArray<int8> AutoSide;
		AutoSide.SetNumZeroed(N);
		for (int32 I = 0; I < N; ++I) { if (FMath::Abs(Curvature[I]) >= Threshold) { AutoSide[I] = Curvature[I] > 0.0f ? 1 : -1; } }
		const int32 MaxGap = FMath::Max(1, FMath::RoundToInt(3.0f / SampleSpacing));
		for (int32 I = 0; I < N; ++I)
		{
			if (AutoSide[I] == 0 || AutoSide[(I + 1) % N] != 0) { continue; }
			for (int32 G = 2; G <= MaxGap + 1; ++G)
			{
				const int32 K = (I + G) % N;
				if (AutoSide[K] == AutoSide[I]) { for (int32 F = 1; F < G; ++F) { AutoSide[(I + F) % N] = AutoSide[I]; } break; }
				if (AutoSide[K] != 0) { break; }
			}
		}
		for (int32 I = 0; I < N; ++I) { if (AutoSide[I] != 0) { KType[AutoSide[I] > 0 ? 0 : 1][I] = EKerbType::Flat; } }
	}
	else
	{
		for (const FKerbSpec& Sp : KerbSpecs)
		{
			auto InRange = [&Sp](int32 C) { return Sp.From <= Sp.To ? (C >= Sp.From && C <= Sp.To) : (C >= Sp.From || C <= Sp.To); };
			float CurvSum = 0.0f;
			for (int32 I = 0; I < N; ++I) { if (InRange(NearCtl[I])) { CurvSum += Curvature[I]; } }
			const int32 Inside = CurvSum > 0.0f ? 0 : 1; // + curvature turns toward the left
			for (int32 I = 0; I < N; ++I)
			{
				if (!InRange(NearCtl[I])) { continue; }
				if (Sp.Side == 3) { KType[1][I] = Sp.Type; continue; }   // driver's left
				if (Sp.Side == 4) { KType[0][I] = Sp.Type; continue; }   // driver's right
				if (Sp.Side != 1) { KType[Inside][I] = Sp.Type; }
				if (Sp.Side != 0) { KType[1 - Inside][I] = Sp.Type; }
			}
		}
	}
	// Taper: samples to the nearest end of a same-type run (circular).
	TArray<int32> RunDist[2];
	for (int32 Sd = 0; Sd < 2; ++Sd)
	{
		RunDist[Sd].Init(0, N);
		for (int32 I = 0; I < N; ++I) { RunDist[Sd][I] = KType[Sd][I] != EKerbType::None ? N : 0; }
		for (int32 Pass = 0; Pass < 4; ++Pass)
		{
			const bool bFwd = (Pass % 2) == 0;
			for (int32 K = 0; K < N; ++K)
			{
				const int32 I = bFwd ? K : N - 1 - K;
				const int32 Prev = bFwd ? (I + N - 1) % N : (I + 1) % N;
				if (KType[Sd][I] != EKerbType::None)
				{
					RunDist[Sd][I] = FMath::Min(RunDist[Sd][I], KType[Sd][Prev] == KType[Sd][I] ? RunDist[Sd][Prev] + 1 : 0);
				}
			}
		}
	}
	const float RampSamples = FMath::Max(1.0f, 2.0f / SampleSpacing);
	auto Scale = [&](int32 Sd, int32 I) { return FMath::Clamp(RunDist[Sd][I] / RampSamples, 0.0f, 1.0f); };
	auto KerbWidthOf = [this](EKerbType T) { return T == EKerbType::None ? 0.0f : KerbWidth; };
	auto Profile = [this](EKerbType T, TArray<FVector2f>& Out)
	{
		Out.Reset();
		if (T == EKerbType::High)
		{
			// Rounded 40 mm hump, 1.0 m wide (60 mm / 0.8 m was too violent at speed - user feedback).
			Out = {FVector2f(0.0f, 0.0f), FVector2f(0.15f, 0.012f), FVector2f(0.3f, 0.03f), FVector2f(0.5f, 0.04f),
			       FVector2f(0.7f, 0.03f), FVector2f(0.85f, 0.012f), FVector2f(1.0f, 0.0f)};
		}
		else if (T == EKerbType::Rumble)
		{
			Out = {FVector2f(0.0f, 0.0f), FVector2f(0.15f, 0.02f), FVector2f(KerbWidth - 0.15f, 0.02f), FVector2f(KerbWidth, 0.0f)};
		}
		else
		{
			Out = {FVector2f(0.0f, 0.0f), FVector2f(0.3f, KerbHeight), FVector2f(KerbWidth - 0.25f, KerbHeight), FVector2f(KerbWidth, 0.0f)};
		}
	};
	FMeshBuffers KerbRed, KerbWhite;
	bHasKerbProbe = false;
	TArray<FVector2f> Prof;
	for (int32 Sd = 0; Sd < 2; ++Sd)
	{
		const float SideSign = Sd == 0 ? 1.0f : -1.0f;
		for (int32 I = 0; I < N; ++I)
		{
			const int32 J = (I + 1) % N;
			const EKerbType T = KType[Sd][I];
			if (T == EKerbType::None || KType[Sd][J] != T) { continue; }
			Profile(T, Prof);
			FMeshBuffers& B = ((I / 2) % 2 == 0) ? KerbRed : KerbWhite;
			const int32 Steps = T == EKerbType::Rumble ? 6 : 1; // rumble teeth need ~0.12 m resolution along the track
			TArray<int32> PrevRow, Row;
			for (int32 St = 0; St <= Steps; ++St)
			{
				const float A = static_cast<float>(St) / Steps;
				const FVector2D C = FMath::Lerp(Samples[I], Samples[J], A);
				const FVector2D Dir = FMath::Lerp(Left(I), Left(J), A).GetSafeNormal() * SideSign;
				const float Hk = FMath::Lerp(SampleHalfW[I], SampleHalfW[J], A);
				const float Ez = FMath::Lerp(EdgeZ(I, SideSign), EdgeZ(J, SideSign), A);
				const float Sc = FMath::Lerp(Scale(Sd, I), Scale(Sd, J), A);
				const float SAlong = (I + A) * SampleSpacing;
				Row.Reset();
				for (int32 P = 0; P < Prof.Num(); ++P)
				{
					float H = Prof[P].Y;
					if (T == EKerbType::Rumble && P > 0 && P + 1 < Prof.Num())
					{
						const float Ph = FMath::Frac(SAlong / 0.25f);
						H = 0.008f + 0.016f * (Ph < 0.5f ? Ph * 2.0f : 2.0f - Ph * 2.0f); // ridges 0.25 m apart
					}
					Row.Add(B.AddVertex(Make3(C + Dir * (Hk + Prof[P].X), Ez + H * Sc), FVector2D(Prof[P].X, SAlong))); // UV: m from the track edge, m along (M_KartKerb)
				}
				if (PrevRow.Num() == Row.Num())
				{
					for (int32 P = 0; P + 1 < Row.Num(); ++P) { B.AddQuad(PrevRow[P], PrevRow[P + 1], Row[P + 1], Row[P]); }
				}
				PrevRow = Row;
				if (!bHasKerbProbe && Sc >= 1.0f && T == EKerbType::Flat)
				{
					// Self-test probe: the middle of a full-height flat kerb.
					const FVector2D Mid = C + Dir * (Hk + 0.5f * KerbWidth);
					KerbProbe = GetActorTransform().TransformPosition(FVector(Mid.X * Cm, Mid.Y * Cm, (Ez + KerbHeight) * Cm));
					bHasKerbProbe = true;
				}
			}
		}
	}
	KerbMesh->ClearAllMeshSections();
	if (KerbRed.V.Num() > 0) { KerbMesh->CreateMeshSection_LinearColor(0, KerbRed.V, KerbRed.Tri, KerbRed.N, KerbRed.UV, KerbRed.Col, KerbRed.Tan, true); }
	if (KerbWhite.V.Num() > 0) { KerbMesh->CreateMeshSection_LinearColor(1, KerbWhite.V, KerbWhite.Tri, KerbWhite.N, KerbWhite.UV, KerbWhite.Col, KerbWhite.Tan, true); }

	// --- Shoulders: a grass strip (ShoulderWidth) on both sides from the asphalt edge (or the kerb's outer
	//     edge) at the edge height, so the surface is flush (no lip) and the terrain blends from there. ---
	FMeshBuffers Verge;
	for (int32 Sd = 0; Sd < 2; ++Sd)
	{
		const float SideSign = Sd == 0 ? 1.0f : -1.0f;
		auto Ring = [&](int32 K, int32& OutIn, int32& OutOut)
		{
			const float Ez = EdgeZ(K, SideSign);
			const float Start = SampleHalfW[K] + (KType[Sd][K] != EKerbType::None ? KerbWidthOf(KType[Sd][K]) : 0.0f);
			const FVector2D Dir = Left(K) * SideSign;
			OutIn = Verge.AddVertex(Make3(Samples[K] + Dir * Start, Ez), FVector2D(0, K));
			OutOut = Verge.AddVertex(Make3(Samples[K] + Dir * (SampleHalfW[K] + 1.0f + ShoulderWidth), Ez - 0.15f), FVector2D(1, K)); // meets the terrain
		};
		for (int32 I = 0; I < N; ++I)
		{
			const int32 J = (I + 1) % N;
			int32 Ai, Ao, Bi, Bo;
			Ring(I, Ai, Ao);
			Ring(J, Bi, Bo);
			Verge.AddQuad(Ai, Ao, Bo, Bi);
		}
	}
	// --- Terrain (grass): a 2.5 m height grid. Under the track and its shoulders it sits 8 cm below the
	//     surface (hidden); beyond the shoulders it blends over 25 m from the track-edge height into gentle
	//     hills around the track's mean elevation. ---
	FMeshBuffers Grass;
	const int32 Cells = FMath::Max(8, FMath::RoundToInt(GroundSize / (Terrain.bValid ? Terrain.MeshCell : 2.5f)));
	const float Half = 0.5f * GroundSize;
	FVector2D Centre(0, 0);
	for (const FVector2D& P : ControlPoints) { Centre += P; }
	Centre /= NCtl;
	float MeanElev = 0.0f;
	for (float E : SampleElev) { MeanElev += E; }
	MeanElev /= FMath::Max(1, SampleElev.Num());
	auto TerrainHeight = [&](const FVector2D& P) -> float
	{
		// Nearest centre-line sample: coarse pass every 8th sample, then refine.
		int32 Best = 0;
		float BestD = TNumericLimits<float>::Max();
		for (int32 I = 0; I < N; I += 8) { const float D = FVector2D::DistSquared(P, Samples[I]); if (D < BestD) { BestD = D; Best = I; } }
		for (int32 K = -8; K <= 8; ++K) { const int32 I = (Best + K + N) % N; const float D = FVector2D::DistSquared(P, Samples[I]); if (D < BestD) { BestD = D; Best = I; } }
		const float Lat = FVector2D::DotProduct(P - Samples[Best], Left(Best));
		const float Dist = FMath::Sqrt(BestD);
		// Under the track: the ACTUAL road height at this lateral position (a banked road is lower in the
		// middle than at its raised edge - using the edge height let the grass poke through), 15 cm below.
		const float Tb = FMath::Tan(Bank[Best]);
		const float Hb = SampleHalfW[Best];
		const float ShoulderOuter = Hb + 1.0f + ShoulderWidth; // widest kerb + shoulder
		const float Road = AsphaltZ + SampleElev[Best] + Hb * FMath::Abs(Tb) + FMath::Clamp(Lat, -Hb, Hb) * Tb;
		constexpr float Margin = 0.15f;
		if (Dist <= ShoulderOuter) { return Road - Margin; }
		// Beyond the shoulder (which slopes down by the same margin): blend into gentle hills, starting 1 m out
		// so no grid cell next to the shoulder can rise above it.
		// Measured layouts: the LiDAR terrain itself (embankments, mounds, drains), blended in over a few metres.
		const float Far = Terrain.bValid ? Terrain.Sample(P)
		                                 : MeanElev + 1.6f * FMath::Sin(P.X / 41.0f) * FMath::Cos(P.Y / 33.0f) + 0.7f * FMath::Sin((P.X + P.Y) / 17.0f);
		const float W = FMath::SmoothStep(0.0f, 1.0f, (Dist - ShoulderOuter - 1.0f) / (Terrain.bValid ? Terrain.Blend : 25.0f));
		return FMath::Lerp(Road - Margin, Far, W);
	};
	TArray<float> H;
	H.SetNum((Cells + 1) * (Cells + 1));
	for (int32 Y = 0; Y <= Cells; ++Y)
	{
		for (int32 X = 0; X <= Cells; ++X)
		{
			const FVector2D P = Centre + FVector2D(-Half + GroundSize * X / Cells, -Half + GroundSize * Y / Cells);
			H[Y * (Cells + 1) + X] = TerrainHeight(P);
			Grass.AddVertex(Make3(P, H[Y * (Cells + 1) + X]), FVector2D(X, Y));
		}
	}
	const float CellM = GroundSize / Cells;
	for (int32 Y = 0; Y <= Cells; ++Y)
	{
		for (int32 X = 0; X <= Cells; ++X)
		{
			// Smooth lighting on the hills: normal from the height gradient.
			const float Hx0 = H[Y * (Cells + 1) + FMath::Max(X - 1, 0)], Hx1 = H[Y * (Cells + 1) + FMath::Min(X + 1, Cells)];
			const float Hy0 = H[FMath::Max(Y - 1, 0) * (Cells + 1) + X], Hy1 = H[FMath::Min(Y + 1, Cells) * (Cells + 1) + X];
			Grass.N[Y * (Cells + 1) + X] = FVector(-(Hx1 - Hx0) / (2.0f * CellM), -(Hy1 - Hy0) / (2.0f * CellM), 1.0f).GetSafeNormal();
		}
	}
	for (int32 Y = 0; Y < Cells; ++Y)
	{
		for (int32 X = 0; X < Cells; ++X)
		{
			const int32 A = Y * (Cells + 1) + X;
			Grass.AddQuad(A, A + 1, A + Cells + 2, A + Cells + 1);
		}
	}
	GrassMesh->CreateMeshSection_LinearColor(0, Grass.V, Grass.Tri, Grass.N, Grass.UV, Grass.Col, Grass.Tan, true);
	if (Verge.V.Num() > 0) { GrassMesh->CreateMeshSection_LinearColor(1, Verge.V, Verge.Tri, Verge.N, Verge.UV, Verge.Col, Verge.Tan, true); }
	else { GrassMesh->ClearMeshSection(1); }

	// --- Tyre-wall barriers on the outside of the corners (VISUAL ONLY: kart-barrier collisions are
	//     not simulated; the walls sit beyond 7 m of grass run-off) ---
	if (BarrierTyres) { BarrierTyres->ClearInstances(); }
	UStaticMesh* TyreMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Game/KartVisuals/Track/SM_BarrierTyre.SM_BarrierTyre"));
	if (BarrierTyres && (TyreMesh || CylinderMesh) && SceneryFile.IsEmpty())
	{
		BarrierTyres->ClearInstances();
		// Realism pass 4: real used road tyres (Tools/MakeBarrierTyre.py, 0.62 m x 0.20 m, open centre), stacked 3
		// high in two staggered rows, each a little rotated / offset; per stack a painted top tyre now and then.
		// Custom data: [0] paint (0 bare, 1 white, 2 red), [1] random (fade / dust). Deterministic (multiplayer).
		const bool bReal = TyreMesh != nullptr;
		if (bReal) { BarrierTyres->SetStaticMesh(TyreMesh); BarrierTyres->NumCustomDataFloats = 2; }
		FRandomStream Rng(4711);
		float SinceLast = 0.0f;
		int32 Stack = 0;
		for (int32 I = 0; I < N; ++I)
		{
			SinceLast += SampleSpacing;
			if (FMath::Abs(Curvature[I]) < 0.5f * Threshold || SinceLast < 0.62f) { continue; }
			SinceLast = 0.0f;
			++Stack;
			const float Outside = Curvature[I] > 0.0f ? -1.0f : 1.0f;
			for (int32 RowIdx = 0; RowIdx < (bReal ? 2 : 1); ++RowIdx)
			{
				const float Out = SampleHalfW[I] + KerbWidth + 7.0f + RowIdx * 0.58f;
				const FVector2D P = Samples[I] + Left(I) * (Outside * Out) + Tangents[I] * (RowIdx * 0.31f);
				const float PaintTop = (Stack % 2 == 0) ? 1.0f : 2.0f;     // alternating white / red tops on the front row
				for (int32 Level = 0; Level < 3; ++Level)
				{
					if (!bReal)
					{
						// Engine cylinder: 100 cm diameter x 100 cm tall -> 60 cm tyre, 20 cm high.
						BarrierTyres->AddInstance(FTransform(FRotator::ZeroRotator, FVector(P.X * Cm, P.Y * Cm, TerrainHeight(P) * Cm + 10.0f + 20.0f * Level), FVector(0.6f, 0.6f, 0.2f)));
						continue;
					}
					const FVector2D J(Rng.FRandRange(-0.025f, 0.025f), Rng.FRandRange(-0.025f, 0.025f));
					const FRotator R(Rng.FRandRange(-1.5f, 1.5f), Rng.FRandRange(0.0f, 360.0f), Rng.FRandRange(-1.5f, 1.5f));
					const int32 Idx = BarrierTyres->AddInstance(FTransform(R, FVector((P.X + J.X) * Cm, (P.Y + J.Y) * Cm, TerrainHeight(P) * Cm + 10.0f + 20.0f * Level)));
					const float Paint = (RowIdx == 0 && Level == 2) ? PaintTop : 0.0f;
					BarrierTyres->SetCustomDataValue(Idx, 0, Paint, false);
					BarrierTyres->SetCustomDataValue(Idx, 1, Rng.FRand(), false);
				}
			}
		}
		BarrierTyres->MarkRenderStateDirty();
	}

	// --- Trackside scenery (fencing, pit / paddock, trees): KartScenery.cpp ---
	BuildScenery(TerrainHeight, HalfWMax);

	// --- Materials: instances of /Game/KartVisuals/M_KartSurface (fallback: flat engine material) ---
	auto Flat = [this](const FLinearColor& C) -> UMaterialInterface*
	{
		UMaterialInstanceDynamic* M = BaseMaterial ? UMaterialInstanceDynamic::Create(BaseMaterial, this) : nullptr;
		if (M) { M->SetVectorParameterValue(TEXT("Color"), C); }
		return M;
	};
	WetMaterials.Reset();
	AppliedWetness = -1.0f;
	auto Surf = [this, &Flat](FLinearColor A, FLinearColor B, float R, float RV, float Macro, float Micro, float MacroAmt) -> UMaterialInterface*
	{
		UMaterialInstanceDynamic* M = KartVisuals::MakeSurface(this, A, B, R, RV, 0.0f, Macro, Micro, MacroAmt);
		return M ? static_cast<UMaterialInterface*>(M) : Flat(A);
	};
	auto Paved = [this, &Surf](FLinearColor A, FLinearColor B, float R, float RV, float Macro, float Micro, float MacroAmt) -> UMaterialInterface*
	{
		UMaterialInterface* M = Surf(A, B, R, RV, Macro, Micro, MacroAmt);
		if (UMaterialInstanceDynamic* Mid = Cast<UMaterialInstanceDynamic>(M)) { WetMaterials.Add({Mid, A, B, R}); }
		return M;
	};
	// Photo-textured ground (CC0 ambientCG sets, Tools/CreateGroundMaterials.py): MI_Ground_<Surface> instances of
	// M_KartGround, world-projected, with a Wetness parameter. Falls back to the procedural surfaces when missing.
	auto Ground = [this](const TCHAR* Instance, const FLinearColor& Tint, bool bWet) -> UMaterialInterface*
	{
		UMaterialInterface* Parent = LoadObject<UMaterialInterface>(nullptr, *FString::Printf(TEXT("/Game/KartVisuals/%s.%s"), Instance, Instance));
		if (!Parent) { return nullptr; }
		UMaterialInstanceDynamic* M = UMaterialInstanceDynamic::Create(Parent, this);
		M->SetVectorParameterValue(TEXT("Tint"), Tint);
		if (bWet) { FWetMaterial W; W.Mid = M; W.A = Tint; W.B = Tint; W.Roughness = 0.0f; W.bGround = true; WetMaterials.Add(W); }
		return M;
	};
	// Realism pass 4 (Tools/CreateTrackMaterials.py): M_KartAsphalt reads the wear masks in the asphalt vertex colours
	// (rubbered racing line, tyre marks, dusty edges); M_KartGrass breaks up the lawn with hue / wear variation.
	// Their tints are baked into the instances (Tint = white here). Older content falls back to MI_Ground_*.
	UMaterialInterface* TrackGrass = Ground(TEXT("MI_Track_Grass"), FLinearColor::White, true);
	UMaterialInterface* TrackAsphalt = Ground(TEXT("MI_Track_Asphalt"), FLinearColor::White, true);
	if (UMaterialInterface* GroundGrass = TrackGrass ? TrackGrass : Ground(TEXT("MI_Ground_Grass"), FLinearColor(0.50f, 0.52f, 0.40f), false))
	{
		GrassMesh->SetMaterial(0, GroundGrass);
		GrassMesh->SetMaterial(1, GroundGrass);
		AsphaltMesh->SetMaterial(0, TrackAsphalt ? TrackAsphalt : Ground(TEXT("MI_Ground_Asphalt"), FLinearColor(0.46f, 0.47f, 0.52f), true));
		AsphaltMesh->SetMaterial(1, Ground(TEXT("MI_Ground_Concrete"), FLinearColor(0.95f, 0.95f, 0.95f), true));  // painted lines
		if (UMaterialInterface* VergeMat = TrackGrass ? Ground(TEXT("MI_Track_Verge"), FLinearColor::White, true) : nullptr) { GrassMesh->SetMaterial(1, VergeMat); }
		const bool bKerbMat = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/KartVisuals/MI_Track_Kerb.MI_Track_Kerb")) != nullptr;
		const TCHAR* KerbInst = bKerbMat ? TEXT("MI_Track_Kerb") : TEXT("MI_Ground_Concrete");
		KerbMesh->SetMaterial(0, Ground(KerbInst, bKerbMat ? FLinearColor(0.62f, 0.035f, 0.025f) : FLinearColor(1.15f, 0.07f, 0.05f), true)); // red paint
		KerbMesh->SetMaterial(1, Ground(KerbInst, bKerbMat ? FLinearColor(0.70f, 0.70f, 0.68f) : FLinearColor(0.98f, 0.98f, 0.98f), true));   // white paint
		UMaterialInterface* TyreMat = BarrierTyres && BarrierTyres->GetStaticMesh() && BarrierTyres->GetStaticMesh()->GetName() == TEXT("SM_BarrierTyre") ? Ground(TEXT("M_BarrierTyre"), FLinearColor::White, true) : nullptr;
		if (BarrierTyres) { BarrierTyres->SetMaterial(0, TyreMat ? TyreMat : Surf(FLinearColor(0.015f, 0.015f, 0.016f), FLinearColor(0.035f, 0.035f, 0.035f), 0.9f, 0.1f, 0.01f, 0.4f, 0.4f)); }
		return;
	}
	// Asphalt: dark aggregate with lighter large-scale patches; grass: mottled; kerbs: painted concrete.
	UMaterialInterface* GrassMat = Surf(FLinearColor(0.025f, 0.085f, 0.015f), FLinearColor(0.07f, 0.17f, 0.03f), 0.95f, 0.05f, 0.0025f, 0.08f, 0.65f);
	GrassMesh->SetMaterial(0, GrassMat);
	GrassMesh->SetMaterial(1, GrassMat);
	AsphaltMesh->SetMaterial(0, Paved(FLinearColor(0.028f, 0.028f, 0.031f), FLinearColor(0.085f, 0.085f, 0.09f), 0.80f, 0.25f, 0.0018f, 0.9f, 0.35f));
	AsphaltMesh->SetMaterial(1, Paved(FLinearColor(0.75f, 0.75f, 0.75f), FLinearColor(0.85f, 0.85f, 0.85f), 0.6f, 0.1f, 0.01f, 0.5f, 0.3f));
	KerbMesh->SetMaterial(0, Paved(FLinearColor(0.55f, 0.03f, 0.02f), FLinearColor(0.68f, 0.05f, 0.03f), 0.55f, 0.15f, 0.01f, 0.6f, 0.3f));
	KerbMesh->SetMaterial(1, Paved(FLinearColor(0.70f, 0.70f, 0.70f), FLinearColor(0.82f, 0.82f, 0.82f), 0.55f, 0.15f, 0.01f, 0.6f, 0.3f));
	if (BarrierTyres) { BarrierTyres->SetMaterial(0, Surf(FLinearColor(0.015f, 0.015f, 0.016f), FLinearColor(0.035f, 0.035f, 0.035f), 0.9f, 0.1f, 0.01f, 0.4f, 0.4f)); }
}

void AKartTestTrackActor::SetWetness(float Wet01)
{
	const float W = FMath::Clamp(Wet01, 0.0f, 1.0f);
	if (FMath::Abs(W - AppliedWetness) < 0.01f) { return; }
	AppliedWetness = W;
	// Wet porous surfaces read darker (water fills the texture) and much smoother (specular film).
	const float Dark = 1.0f - 0.45f * W;
	for (const FWetMaterial& M : WetMaterials)
	{
		if (M.bGround)
		{
			if (UMaterialInstanceDynamic* GroundMid = M.Mid.Get()) { GroundMid->SetScalarParameterValue(TEXT("Wetness"), W); }
		}
		else if (UMaterialInstanceDynamic* Mid = M.Mid.Get())
		{
			Mid->SetVectorParameterValue(TEXT("ColorA"), M.A * Dark);
			Mid->SetVectorParameterValue(TEXT("ColorB"), M.B * Dark);
			Mid->SetScalarParameterValue(TEXT("Roughness"), FMath::Lerp(M.Roughness, 0.12f, W));
		}
	}
}

TMulticastDelegate<void()> AKartTestTrackActor::OnLayoutChanged;

bool AKartTestTrackActor::LoadLayout(const FString& File)
{
	const FString Path = FPaths::ProjectContentDir() / TEXT("KartData/Tracks") / File;
	FString Text;
	if (!FFileHelper::LoadFileToString(Text, *Path))
	{
		UE_LOG(LogTemp, Warning, TEXT("Track layout %s not found"), *Path);
		return false;
	}
	const kartsim::JsonParseResult R = kartsim::ParseJson(TCHAR_TO_UTF8(*Text));
	const kartsim::JsonValue* Pts = R.bOk ? R.Root.Find("controlPoints") : nullptr;
	if (!Pts || !Pts->IsArray() || Pts->ArrayValue.size() < 4)
	{
		UE_LOG(LogTemp, Warning, TEXT("Track layout %s invalid (%s)"), *File, UTF8_TO_TCHAR(R.Error.c_str()));
		return false;
	}
	TArray<FVector2D> NewPoints;
	TArray<float> NewCamber;
	TArray<float> NewElev;
	TArray<float> NewWidth;
	FBox2D Bounds(ForceInit);
	for (const kartsim::JsonValue& P : Pts->ArrayValue)
	{
		if (!P.IsArray() || P.ArrayValue.size() < 2) { return false; }
		const FVector2D V(P.ArrayValue[0].NumberValue, P.ArrayValue[1].NumberValue);
		NewPoints.Add(V);
		NewCamber.Add(P.ArrayValue.size() > 2 ? static_cast<float>(P.ArrayValue[2].NumberValue) : 0.0f);
		NewElev.Add(P.ArrayValue.size() > 3 ? static_cast<float>(P.ArrayValue[3].NumberValue) : 0.0f);
		if (P.ArrayValue.size() > 4) { NewWidth.Add(static_cast<float>(P.ArrayValue[4].NumberValue)); }
		Bounds += V;
	}
	ControlPoints = NewPoints;
	ControlCamber = NewCamber;
	ControlElev = NewElev;
	ControlWidth = NewWidth.Num() == NewPoints.Num() ? NewWidth : TArray<float>();
	KerbSpecs.Reset();
	if (const kartsim::JsonValue* Ks = R.Root.Find("kerbs"); Ks && Ks->IsArray())
	{
		for (const kartsim::JsonValue& K : Ks->ArrayValue)
		{
			FKerbSpec Spec;
			if (const kartsim::JsonValue* F = K.Find("from")) { Spec.From = static_cast<int32>(F->NumberValue); }
			if (const kartsim::JsonValue* T = K.Find("to")) { Spec.To = static_cast<int32>(T->NumberValue); }
			const kartsim::JsonValue* Sd = K.Find("side");
			const std::string Side = Sd ? Sd->StringValue : "inside";
			Spec.Side = Side == "outside" ? 1 : (Side == "both" ? 2 : (Side == "left" ? 3 : (Side == "right" ? 4 : 0)));
			const kartsim::JsonValue* Ty = K.Find("type");
			const std::string Type = Ty ? Ty->StringValue : "flat";
			Spec.Type = Type == "rumble" ? EKerbType::Rumble : (Type == "high" ? EKerbType::High : EKerbType::Flat);
			KerbSpecs.Add(Spec);
		}
	}
	if (const kartsim::JsonValue* W = R.Root.Find("width")) { TrackWidth = static_cast<float>(W->NumberValue); }
	if (const kartsim::JsonValue* K = R.Root.Find("kerbRadius")) { KerbRadiusThreshold = static_cast<float>(K->NumberValue); }
	const kartsim::JsonValue* ScV = R.Root.Find("scenery");
	SceneryFile = ScV ? UTF8_TO_TCHAR(ScV->StringValue.c_str()) : TEXT("");
	const kartsim::JsonValue* KwV = R.Root.Find("kerbWidth");
	KerbWidth = KwV ? static_cast<float>(KwV->NumberValue) : 1.0f;
	// Measured terrain: { "file", "originX", "originY", "cell", "nx", "ny", "meshCell", "blend" } (float32, little endian).
	Terrain = FTerrainGrid();
	if (const kartsim::JsonValue* Tv = R.Root.Find("terrain"))
	{
		auto Num = [Tv](const char* Key, double Def) { const kartsim::JsonValue* V = Tv->Find(Key); return V ? V->NumberValue : Def; };
		const kartsim::JsonValue* Fv = Tv->Find("file");
		TArray<uint8> Bytes;
		const FString TPath = FPaths::ProjectContentDir() / TEXT("KartData/Tracks") / (Fv ? UTF8_TO_TCHAR(Fv->StringValue.c_str()) : TEXT(""));
		const int32 NX = static_cast<int32>(Num("nx", 0)), NY = static_cast<int32>(Num("ny", 0));
		if (Fv && NX > 1 && NY > 1 && FFileHelper::LoadFileToArray(Bytes, *TPath) && Bytes.Num() == NX * NY * 4)
		{
			Terrain.H.SetNumUninitialized(NX * NY);
			FMemory::Memcpy(Terrain.H.GetData(), Bytes.GetData(), Bytes.Num());
			Terrain.NX = NX; Terrain.NY = NY;
			Terrain.Origin = FVector2D(Num("originX", 0.0), Num("originY", 0.0));
			Terrain.Cell = static_cast<float>(Num("cell", 1.0));
			Terrain.MeshCell = static_cast<float>(Num("meshCell", 2.5));
			Terrain.Blend = static_cast<float>(Num("blend", 25.0));
			Terrain.bValid = true;
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("Track layout %s: terrain %s missing or wrong size (procedural hills used)"), *File, *TPath);
		}
	}
	DisplayName = File;
	if (const kartsim::JsonValue* N = R.Root.Find("name")) { DisplayName = UTF8_TO_TCHAR(N->StringValue.c_str()); }
	LayoutFile = File;
	// Ground plane: covers the layout with at least 150 m of margin (measured terrain: its own extent).
	GroundSize = Terrain.bValid ? FMath::Min(Terrain.NX, Terrain.NY) * Terrain.Cell - 10.0f
	                            : FMath::Max(700.0f, static_cast<float>(FMath::Max(Bounds.GetSize().X, Bounds.GetSize().Y)) + 300.0f);
	BuildTrack();
	UE_LOG(LogTemp, Log, TEXT("Track layout: %s (%s), %.0f m"), *DisplayName, *File, TrackLength);
	OnLayoutChanged.Broadcast();
	return true;
}

TArray<TPair<FString, FString>> AKartTestTrackActor::ListLayouts()
{
	TArray<TPair<FString, FString>> Out;
	const FString Dir = FPaths::ProjectContentDir() / TEXT("KartData/Tracks");
	TArray<FString> Files;
	IFileManager::Get().FindFiles(Files, *(Dir / TEXT("*.json")), true, false);
	for (const FString& F : Files)
	{
		FString Text, Name = F;
		if (FFileHelper::LoadFileToString(Text, *(Dir / F)))
		{
			const kartsim::JsonParseResult R = kartsim::ParseJson(TCHAR_TO_UTF8(*Text));
			if (!R.bOk || !R.Root.Find("controlPoints")) { continue; } // data files next to layouts (scenery, ...)
			if (const kartsim::JsonValue* N = R.Root.Find("name")) { Name = UTF8_TO_TCHAR(N->StringValue.c_str()); }
		}
		Out.Add({F, Name});
	}
	Out.Sort([](const TPair<FString, FString>& A, const TPair<FString, FString>& B) { return A.Value < B.Value; });
	return Out;
}
