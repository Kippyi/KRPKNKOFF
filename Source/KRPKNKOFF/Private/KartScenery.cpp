// Trackside scenery (realism pass 2): catch fencing, a pit / paddock area on the start straight, trees and shrubs.
// Built procedurally around whatever layout is loaded, from CC0 Poly Haven models (Tools/FetchPolyHaven.py +
// Tools/ImportScenery.py -> /Game/Scenery) and engine cubes for the buildings. VISUAL ONLY: no collision (karts
// cannot reach it: the run-off ends at the tyre walls 7 m out, the fence is 11 m out).
// Every placement is deterministic (fixed seed), so all machines in a multiplayer session see the same scenery.
#include "KartTestTrackActor.h"

#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "KartProceduralVisuals.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "ProceduralMeshComponent.h"
#include "KartSim/Json.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

namespace
{
	constexpr float Cm = 100.0f;

	struct FSceneryMesh
	{
		UStaticMesh* Mesh = nullptr;
		FVector Pivot = FVector::ZeroVector; // bounds centre XY, base Z (cm): placed so this point lands on the target
	};

	FSceneryMesh LoadScenery(const TCHAR* Path)
	{
		FSceneryMesh R;
		R.Mesh = LoadObject<UStaticMesh>(nullptr, Path);
		if (R.Mesh)
		{
			const FBox B = R.Mesh->GetBoundingBox();
			R.Pivot = FVector(B.GetCenter().X, B.GetCenter().Y, B.Min.Z);
		}
		return R;
	}
}

UHierarchicalInstancedStaticMeshComponent* AKartTestTrackActor::SceneryComponent(UStaticMesh* Mesh, UMaterialInterface* Override)
{
	UHierarchicalInstancedStaticMeshComponent* C = NewObject<UHierarchicalInstancedStaticMeshComponent>(this);
	C->SetStaticMesh(Mesh);
	C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	C->SetupAttachment(GrassMesh);
	C->SetMobility(EComponentMobility::Movable); // the track root is movable: a static child cannot attach (and would leak on a layout change)
	if (Override)
	{
		for (int32 S = 0; S < Mesh->GetStaticMaterials().Num(); ++S) { C->SetMaterial(S, Override); }
	}
	C->RegisterComponent();
	SceneryComps.Add(C);
	return C;
}

void AKartTestTrackActor::BuildScenery(TFunctionRef<float(const FVector2D&)> Height, float HalfW)
{
	for (UHierarchicalInstancedStaticMeshComponent* C : SceneryComps) { if (C) { C->DestroyComponent(); } }
	SceneryComps.Reset();
	const int32 N = Samples.Num();
	if (N < 8 || FParse::Param(FCommandLine::Get(), TEXT("KartNoScenery"))) { return; }
	if (!SceneryFile.IsEmpty()) { BuildMeasuredScenery(Height); return; }

	// Distance (m) from a point to the centre line (coarse pass, then refine).
	auto TrackDist = [&](const FVector2D& P)
	{
		int32 Best = 0;
		float BestD = TNumericLimits<float>::Max();
		for (int32 I = 0; I < N; I += 6) { const float D = FVector2D::DistSquared(P, Samples[I]); if (D < BestD) { BestD = D; Best = I; } }
		for (int32 K = -6; K <= 6; ++K) { const float D = FVector2D::DistSquared(P, Samples[(Best + K + N) % N]); BestD = FMath::Min(BestD, D); }
		return FMath::Sqrt(BestD);
	};
	auto LeftN = [&](int32 I) { return FVector2D(-Tangents[I].Y, Tangents[I].X); };
	auto At = [&](const FVector2D& P, float Up = 0.0f) { return FVector(P.X * Cm, P.Y * Cm, (Height(P) + Up) * Cm); };
	// Instance so the mesh's base centre lands at Pos (cm), yawed, scaled.
	auto Place = [](UHierarchicalInstancedStaticMeshComponent* C, const FSceneryMesh& M, const FVector& Pos, float YawDeg, const FVector& Scale)
	{
		const FQuat Q(FVector::UpVector, FMath::DegreesToRadians(YawDeg));
		C->AddInstance(FTransform(Q, Pos - Q.RotateVector(M.Pivot * Scale), Scale));
	};
	FRandomStream Rng(0x6B617274); // "kart": deterministic across machines
	const float Edge = HalfW + KerbWidth;  // asphalt + kerb (m)
	const float FenceOff = Edge + 11.0f;

	// ---------------- Pit / paddock on the start straight ----------------
	// Side: the one with more free space 35 m out from the start line.
	const FVector2D S0 = Samples[0], T0 = Tangents[0], L0 = LeftN(0);
	const float Side = TrackDist(S0 + L0 * 35.0f) >= TrackDist(S0 - L0 * 35.0f) ? 1.0f : -1.0f;
	const FVector2D Out = L0 * Side;                         // away from the track
	const float Yaw0 = FMath::RadiansToDegrees(FMath::Atan2(T0.Y, T0.X));
	auto Pit = [&](float Along, float Lateral) { return S0 + T0 * Along + Out * (Edge + Lateral); };
	// Free if every corner of a footprint keeps Clear metres from the centre line.
	auto FootprintClear = [&](float A0, float A1, float L0m, float L1m, float Clear)
	{
		for (float A : {A0, 0.5f * (A0 + A1), A1}) { for (float L : {L0m, L1m}) { if (TrackDist(Pit(A, L)) < Edge + Clear - 0.01f) { return false; } } }
		return true;
	};
	const bool bPit = FootprintClear(-30.0f, 30.0f, 14.0f, 24.0f, 12.0f);
	auto InPitZone = [&](const FVector2D& P)
	{
		const FVector2D D = P - S0;
		const float A = FVector2D::DotProduct(D, T0), L = FVector2D::DotProduct(D, Out);
		return bPit && A > -60.0f && A < 60.0f && L > 0.0f && L < Edge + 40.0f;
	};

	UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
	if (bPit && Cube)
	{
		const FSceneryMesh CubeM{Cube, FVector(0, 0, -50.0f)};
		auto Mat = [this](FLinearColor A, FLinearColor B, float R, float Met) { return KartVisuals::MakeSurface(this, A, B, R, 0.08f, Met, 0.004f, 0.6f, 0.25f); };
		UHierarchicalInstancedStaticMeshComponent* Plaster = SceneryComponent(Cube, Mat(FLinearColor(0.68f, 0.67f, 0.63f), FLinearColor(0.74f, 0.73f, 0.69f), 0.85f, 0.0f));
		UHierarchicalInstancedStaticMeshComponent* Metal = SceneryComponent(Cube, Mat(FLinearColor(0.38f, 0.39f, 0.41f), FLinearColor(0.44f, 0.45f, 0.47f), 0.5f, 0.6f));
		UHierarchicalInstancedStaticMeshComponent* Shutter = SceneryComponent(Cube, Mat(FLinearColor(0.55f, 0.56f, 0.58f), FLinearColor(0.6f, 0.61f, 0.63f), 0.45f, 0.5f));
		UHierarchicalInstancedStaticMeshComponent* Seats = SceneryComponent(Cube, Mat(FLinearColor(0.05f, 0.12f, 0.35f), FLinearColor(0.06f, 0.14f, 0.4f), 0.5f, 0.0f));
		auto Box = [&](UHierarchicalInstancedStaticMeshComponent* C, float Along, float Lat, float Up, float SizeA, float SizeL, float SizeZ)
		{
			const FVector2D P = Pit(Along, Lat);
			const float Base = Height(Pit(Along, Lat)) + Up;
			Place(C, CubeM, FVector(P.X * Cm, P.Y * Cm, Base * Cm), Yaw0, FVector(SizeA, SizeL, SizeZ));
		};
		// Pit building: 36 x 9 x 4.2 m, roller doors facing the track, roof slab with a 2 m overhang.
		const float BLat = 19.5f;
		Box(Plaster, 0.0f, BLat, -0.5f, 36.0f, 9.0f, 4.7f);
		Box(Metal, 0.0f, BLat - 1.0f, 4.2f, 38.0f, 13.0f, 0.35f);
		for (int32 D = 0; D < 8; ++D) { Box(Shutter, -15.75f + 4.5f * D, BLat - 4.55f, 0.0f, 3.6f, 0.12f, 3.0f); }
		// Grandstand behind the pit wall's far end: 6 stepped rows, roof on posts.
		const float GA = 42.0f, GLat = 16.0f;
		for (int32 R = 0; R < 6; ++R)
		{
			Box(Plaster, GA, GLat + 1.0f * R, -0.3f, 22.0f, 1.0f, 0.75f + 0.5f * R);
			Box(Seats, GA, GLat + 1.0f * R - 0.2f, 0.45f + 0.5f * R, 21.6f, 0.45f, 0.12f);
		}
		Box(Metal, GA, GLat + 2.5f, 6.2f, 23.0f, 8.0f, 0.25f);
		for (int32 P = 0; P < 4; ++P) { Box(Metal, GA - 10.5f + 7.0f * P, GLat + 6.2f, 0.0f, 0.25f, 0.25f, 6.3f); }

		// Pit wall: concrete Jersey barriers 4 m from the asphalt edge.
		const FSceneryMesh Jersey = LoadScenery(TEXT("/Game/Scenery/concrete_road_barrier/concrete_road_barrier_2k/StaticMeshes/concrete_road_barrier_2k.concrete_road_barrier_2k"));
		if (Jersey.Mesh)
		{
			UHierarchicalInstancedStaticMeshComponent* C = SceneryComponent(Jersey.Mesh);
			for (float A = -55.0f; A <= 55.0f; A += 1.56f)
			{
				const FVector2D P = Pit(A, 4.0f);
				if (TrackDist(P) > Edge + 3.0f) { Place(C, Jersey, At(P), Yaw0, FVector(1.0f)); }
			}
		}
		// Paddock props behind the building.
		const struct { const TCHAR* Path; float Along, Lat, Yaw; int32 Count; float Step; } Props[] = {
			{TEXT("/Game/Scenery/street_lamp_01/street_lamp_01_2k/StaticMeshes/street_lamp_01_2k.street_lamp_01_2k"), -30.0f, 27.0f, 90.0f, 5, 15.0f},
			{TEXT("/Game/Scenery/utility_box_01/utility_box_01_2k/StaticMeshes/utility_box_01_2k.utility_box_01_2k"), 20.0f, 25.0f, 90.0f, 2, 1.2f},
			{TEXT("/Game/Scenery/painted_wooden_bench/painted_wooden_bench_2k/StaticMeshes/painted_wooden_bench_2k.painted_wooden_bench_2k"), -12.0f, 26.0f, 0.0f, 3, 8.0f},
			{TEXT("/Game/Scenery/Barrel_01/Barrel_01_2k/StaticMeshes/Barrel_01_2k.Barrel_01_2k"), 24.0f, 14.5f, 0.0f, 4, 0.7f},
		};
		for (const auto& Pr : Props)
		{
			const FSceneryMesh M = LoadScenery(Pr.Path);
			if (!M.Mesh) { continue; }
			UHierarchicalInstancedStaticMeshComponent* C = SceneryComponent(M.Mesh);
			for (int32 I = 0; I < Pr.Count; ++I) { Place(C, M, At(Pit(Pr.Along + Pr.Step * I, Pr.Lat)), Yaw0 + Pr.Yaw + Rng.FRandRange(-4.0f, 4.0f), FVector(1.0f)); }
		}
		// Floodlights on the pit roof edge.
		const FSceneryMesh Flood = LoadScenery(TEXT("/Game/Scenery/flood_lights/flood_lights_2k/StaticMeshes/flood_lights_standing_single_light.flood_lights_standing_single_light"));
		if (Flood.Mesh)
		{
			UHierarchicalInstancedStaticMeshComponent* C = SceneryComponent(Flood.Mesh);
			for (int32 I = 0; I < 6; ++I) { const FVector2D P = Pit(-15.0f + 6.0f * I, BLat - 6.5f); Place(C, Flood, At(P, 4.0f), Yaw0 - 90.0f, FVector(2.0f)); }
		}
	}

	// ---------------- Catch fencing ----------------
	// Posts (Poly Haven) every ~2 m with galvanised top / bottom rails and horizontal tension cables (engine cylinders).
	// (The Poly Haven chain-link panel's wire quad rendered as solid black slabs in the packaged game whatever the
	// mask / shadow setup, so the panel mesh is not used.)
	const FSceneryMesh Post = LoadScenery(TEXT("/Game/Scenery/modular_chainlink_fence/modular_chainlink_fence_2k/StaticMeshes/modular_chainlink_fence_post.modular_chainlink_fence_post"));
	UStaticMesh* CylMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
	int32 FenceCount = 0;
	if (Post.Mesh && CylMesh)
	{
		UHierarchicalInstancedStaticMeshComponent* PostC = SceneryComponent(Post.Mesh);
		UMaterialInterface* Galv = KartVisuals::MakeSurface(this, FLinearColor(0.50f, 0.51f, 0.53f), FLinearColor(0.58f, 0.59f, 0.61f), 0.38f, 0.1f, 1.0f, 0.004f, 0.6f, 0.2f);
		UHierarchicalInstancedStaticMeshComponent* RailC = SceneryComponent(CylMesh, Galv);
		UHierarchicalInstancedStaticMeshComponent* CableC = SceneryComponent(CylMesh, Galv);
		CableC->SetCastShadow(false); // sub-centimetre cables: shadows would only alias
		constexpr float Span = 2.0f;  // m between posts
		struct FLine { float Z, Dia; bool bRail; };
		const FLine Lines[] = {{0.12f, 4.2f, true}, {2.45f, 4.2f, true}, {0.6f, 0.7f, false}, {1.05f, 0.7f, false}, {1.5f, 0.7f, false}, {1.95f, 0.7f, false}};
		for (float S : {1.0f, -1.0f})
		{
			FVector2D Last = Samples[0] + LeftN(0) * (S * FenceOff);
			bool bLastOk = false;
			for (int32 I = 1; I <= N; ++I)
			{
				const FVector2D P = Samples[I % N] + LeftN(I % N) * (S * FenceOff);
				const bool bOk = TrackDist(P) > FenceOff - 0.75f && !InPitZone(P);
				const float D = FVector2D::Distance(P, Last);
				if (!bOk) { Last = P; bLastOk = false; continue; }
				if (D < Span) { continue; }
				if (bLastOk && D < 2.0f * Span)
				{
					const FVector2D Mid = 0.5f * (P + Last), Dir = (P - Last) / D;
					const float Yaw = FMath::RadiansToDegrees(FMath::Atan2(Dir.Y, Dir.X));
					Place(PostC, Post, At(P, -0.05f), Yaw, FVector(1.0f));
					// Engine cylinder: 100 cm long along Z, 100 cm diameter -> lay it along the segment.
					const FQuat Q = FRotator(-90.0f, Yaw, 0.0f).Quaternion();
					const float Base = 0.5f * (Height(P) + Height(Last));
					for (const FLine& L : Lines)
					{
						const FVector Pos(Mid.X * Cm, Mid.Y * Cm, (Base + L.Z) * Cm);
						(L.bRail ? RailC : CableC)->AddInstance(FTransform(Q, Pos, FVector(L.Dia / 100.0f, L.Dia / 100.0f, D)));
					}
					++FenceCount;
				}
				Last = P;
				bLastOk = true;
			}
		}
	}

	// ---------------- Trees and shrubs ----------------
	const FSceneryMesh Trees[] = {
		LoadScenery(TEXT("/Game/Scenery/jacaranda_tree/jacaranda_tree_2k/StaticMeshes/jacaranda_tree_2k.jacaranda_tree_2k")),
		LoadScenery(TEXT("/Game/Scenery/fir_sapling_medium/fir_sapling_medium_2k/StaticMeshes/fir_sapling_medium_a_LOD0.fir_sapling_medium_a_LOD0")),
		LoadScenery(TEXT("/Game/Scenery/fir_sapling_medium/fir_sapling_medium_2k/StaticMeshes/fir_sapling_medium_b_LOD0.fir_sapling_medium_b_LOD0")),
		LoadScenery(TEXT("/Game/Scenery/fir_sapling_medium/fir_sapling_medium_2k/StaticMeshes/fir_sapling_medium_c_LOD0.fir_sapling_medium_c_LOD0")),
		LoadScenery(TEXT("/Game/Scenery/tree_small_02/tree_small_02_2k/StaticMeshes/tree_small_02_2k.tree_small_02_2k")),
	};
	const float TreeWeight[] = {0.2f, 0.2f, 0.15f, 0.15f, 0.3f};
	const float TreeClear[] = {13.0f, 6.0f, 5.0f, 4.0f, 4.0f};      // canopy radius (m): keep it off the fence line
	const FSceneryMesh Shrubs[] = {
		LoadScenery(TEXT("/Game/Scenery/shrub_02/shrub_02_2k/StaticMeshes/shrub_02_a.shrub_02_a")),
		LoadScenery(TEXT("/Game/Scenery/shrub_02/shrub_02_2k/StaticMeshes/shrub_02_b.shrub_02_b")),
		LoadScenery(TEXT("/Game/Scenery/shrub_02/shrub_02_2k/StaticMeshes/shrub_02_c.shrub_02_c")),
		LoadScenery(TEXT("/Game/Scenery/shrub_02/shrub_02_2k/StaticMeshes/shrub_02_d.shrub_02_d")),
	};
	UHierarchicalInstancedStaticMeshComponent* TreeC[UE_ARRAY_COUNT(Trees)] = {};
	UHierarchicalInstancedStaticMeshComponent* ShrubC[UE_ARRAY_COUNT(Shrubs)] = {};
	for (int32 I = 0; I < UE_ARRAY_COUNT(Trees); ++I) { if (Trees[I].Mesh) { TreeC[I] = SceneryComponent(Trees[I].Mesh); } }
	for (int32 I = 0; I < UE_ARRAY_COUNT(Shrubs); ++I) { if (Shrubs[I].Mesh) { ShrubC[I] = SceneryComponent(Shrubs[I].Mesh); } }
	FVector2D Centre(0, 0);
	for (const FVector2D& P : ControlPoints) { Centre += P; }
	Centre /= FMath::Max(1, ControlPoints.Num());
	const float Half = 0.5f * GroundSize - 15.0f;
	int32 TreeCount = 0, ShrubCount = 0;
	TArray<FVector2D> Placed;
	for (int32 Try = 0; Try < 6000 && TreeCount < 420; ++Try)
	{
		const FVector2D P = Centre + FVector2D(Rng.FRandRange(-Half, Half), Rng.FRandRange(-Half, Half));
		float Pick = Rng.FRand(), Acc = 0.0f;
		int32 K = 0;
		for (; K < UE_ARRAY_COUNT(Trees) - 1; ++K) { Acc += TreeWeight[K]; if (Pick < Acc) { break; } }
		if (!TreeC[K] || InPitZone(P)) { continue; }
		const float Scale = Rng.FRandRange(0.75f, 1.2f);
		if (TrackDist(P) < FenceOff + 2.0f + TreeClear[K] * Scale) { continue; }
		bool bCrowded = false;
		for (const FVector2D& Q : Placed) { if (FVector2D::DistSquared(P, Q) < 36.0f) { bCrowded = true; break; } }
		if (bCrowded) { continue; }
		Placed.Add(P);
		Place(TreeC[K], Trees[K], At(P, -0.1f), Rng.FRandRange(0.0f, 360.0f), FVector(Scale));
		++TreeCount;
	}
	// Shrub clusters just behind the fence.
	for (int32 Try = 0; Try < 400 && Shrubs[0].Mesh; ++Try)
	{
		const int32 I = Rng.RandRange(0, N - 1);
		const float S = Rng.FRand() < 0.5f ? 1.0f : -1.0f;
		const FVector2D Base = Samples[I] + LeftN(I) * (S * (FenceOff + Rng.FRandRange(2.5f, 6.0f)));
		if (TrackDist(Base) < FenceOff + 2.0f || InPitZone(Base)) { continue; }
		for (int32 C = 0; C < 3; ++C)
		{
			const int32 K = Rng.RandRange(0, UE_ARRAY_COUNT(Shrubs) - 1);
			if (!ShrubC[K]) { continue; }
			const FVector2D P = Base + FVector2D(Rng.FRandRange(-2.0f, 2.0f), Rng.FRandRange(-2.0f, 2.0f));
			Place(ShrubC[K], Shrubs[K], At(P, -0.05f), Rng.FRandRange(0.0f, 360.0f), FVector(Rng.FRandRange(0.8f, 1.3f)));
			++ShrubCount;
		}
	}
	UE_LOG(LogTemp, Log, TEXT("KartScenery: pit %s (side %+.0f), fence spans %d, trees %d, shrubs %d"), bPit ? TEXT("yes") : TEXT("no space"), Side, FenceCount, TreeCount, ShrubCount);
}


void AKartTestTrackActor::BuildMeasuredScenery(TFunctionRef<float(const FVector2D&)> Height)
{
	// Layout "scenery" file (Tools/IpswichKC/extract_scenery.py): { "buildings": [ { "footprint": [[x, y], ...],
	// "height": m, "roofOnly": bool, "name": "" } ], "trees": [[x, y, crownRadius], ...] }, track frame metres.
	FString Text;
	const FString Path = FPaths::ProjectContentDir() / TEXT("KartData/Tracks") / SceneryFile;
	if (!FFileHelper::LoadFileToString(Text, *Path))
	{
		UE_LOG(LogTemp, Warning, TEXT("KartScenery: %s not found"), *Path);
		return;
	}
	const kartsim::JsonParseResult R = kartsim::ParseJson(TCHAR_TO_UTF8(*Text));
	if (!R.bOk) { UE_LOG(LogTemp, Warning, TEXT("KartScenery: %s: %s"), *SceneryFile, UTF8_TO_TCHAR(R.Error.c_str())); return; }

	// ---- Buildings: oriented box per footprint (walls) + roof slab; shelters: corner posts + roof ----
	int32 NumBuildings = 0;
	if (UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube")))
	{
		auto Mat = [this](FLinearColor A, FLinearColor B, float Rg, float Met) { return KartVisuals::MakeSurface(this, A, B, Rg, 0.08f, Met, 0.004f, 0.6f, 0.25f); };
		UHierarchicalInstancedStaticMeshComponent* Walls = SceneryComponent(Cube, Mat(FLinearColor(0.78f, 0.77f, 0.73f), FLinearColor(0.84f, 0.83f, 0.79f), 0.7f, 0.0f));
		UHierarchicalInstancedStaticMeshComponent* Roofs = SceneryComponent(Cube, Mat(FLinearColor(0.50f, 0.52f, 0.54f), FLinearColor(0.56f, 0.58f, 0.60f), 0.45f, 0.6f));
		UHierarchicalInstancedStaticMeshComponent* Posts = SceneryComponent(Cube, Mat(FLinearColor(0.35f, 0.36f, 0.38f), FLinearColor(0.4f, 0.41f, 0.43f), 0.5f, 0.8f));
		const kartsim::JsonValue* Bs = R.Root.Find("buildings");
		for (const kartsim::JsonValue& Bv : (Bs && Bs->IsArray()) ? Bs->ArrayValue : std::vector<kartsim::JsonValue>())
		{
			const kartsim::JsonValue* Fp = Bv.Find("footprint");
			if (!Fp || !Fp->IsArray() || Fp->ArrayValue.size() < 3) { continue; }
			TArray<FVector2D> P;
			for (const kartsim::JsonValue& Q : Fp->ArrayValue) { P.Add(FVector2D(Q.ArrayValue[0].NumberValue, Q.ArrayValue[1].NumberValue)); }
			// Orientation = longest edge; extents = footprint projected on that frame (minimum box for rectangles).
			FVector2D Dir(1, 0);
			double Best = 0.0;
			for (int32 I = 0; I < P.Num(); ++I) { const FVector2D E = P[(I + 1) % P.Num()] - P[I]; if (E.Size() > Best) { Best = E.Size(); Dir = E.GetSafeNormal(); } }
			const FVector2D Perp(-Dir.Y, Dir.X);
			double A0 = 1e9, A1 = -1e9, B0 = 1e9, B1 = -1e9;
			float Ground = TNumericLimits<float>::Max();
			for (const FVector2D& Q : P)
			{
				A0 = FMath::Min(A0, FVector2D::DotProduct(Q, Dir)); A1 = FMath::Max(A1, FVector2D::DotProduct(Q, Dir));
				B0 = FMath::Min(B0, FVector2D::DotProduct(Q, Perp)); B1 = FMath::Max(B1, FVector2D::DotProduct(Q, Perp));
				Ground = FMath::Min(Ground, Height(Q));
			}
			const FVector2D C = Dir * (0.5 * (A0 + A1)) + Perp * (0.5 * (B0 + B1));
			const float Len = static_cast<float>(A1 - A0), Wid = static_cast<float>(B1 - B0);
			const kartsim::JsonValue* Hv = Bv.Find("height");
			const float Hgt = Hv ? static_cast<float>(Hv->NumberValue) : 3.5f;
			const kartsim::JsonValue* RoofV = Bv.Find("roofOnly");
			const bool bRoofOnly = RoofV && RoofV->BoolValue;
			const float Yaw = FMath::RadiansToDegrees(FMath::Atan2(Dir.Y, Dir.X));
			const FQuat Q(FVector::UpVector, FMath::DegreesToRadians(Yaw));
			auto Box = [&](UHierarchicalInstancedStaticMeshComponent* Cm_, const FVector2D& At, float Z0, float SX, float SY, float SZ)
			{
				Cm_->AddInstance(FTransform(Q, FVector(At.X * Cm, At.Y * Cm, (Z0 + 0.5f * SZ) * Cm), FVector(SX, SY, SZ)));
			};
			if (!bRoofOnly) { Box(Walls, C, Ground - 0.3f, Len, Wid, Hgt + 0.3f); }
			else
			{
				for (float Sa : {-1.0f, 1.0f}) { for (float Sb : {-1.0f, 1.0f})
				{
					Box(Posts, C + Dir * (Sa * (0.5f * Len - 0.15f)) + Perp * (Sb * (0.5f * Wid - 0.15f)), Ground - 0.2f, 0.12f, 0.12f, Hgt + 0.2f);
				} }
			}
			Box(Roofs, C, Ground + Hgt, Len + 0.5f, Wid + 0.5f, 0.18f);
			++NumBuildings;
		}
	}

	// ---- Trees: measured crown positions; species by crown size, scaled to the crown radius ----
	struct FSpecies { FSceneryMesh M; float CrownR; float MaxScale; };
	const FSpecies Big{LoadScenery(TEXT("/Game/Scenery/jacaranda_tree/jacaranda_tree_2k/StaticMeshes/jacaranda_tree_2k.jacaranda_tree_2k")), 11.5f, 0.6f};
	const FSpecies Small{LoadScenery(TEXT("/Game/Scenery/tree_small_02/tree_small_02_2k/StaticMeshes/tree_small_02_2k.tree_small_02_2k")), 2.2f, 2.4f};
	UHierarchicalInstancedStaticMeshComponent* BigC = Big.M.Mesh ? SceneryComponent(Big.M.Mesh) : nullptr;
	UHierarchicalInstancedStaticMeshComponent* SmallC = Small.M.Mesh ? SceneryComponent(Small.M.Mesh) : nullptr;
	FRandomStream Rng(0x494B43); // "IKC": deterministic yaw / small jitter, identical on every machine
	int32 NumTrees = 0;
	if (const kartsim::JsonValue* Ts = R.Root.Find("trees"); Ts && Ts->IsArray() && BigC && SmallC)
	{
		for (const kartsim::JsonValue& Tv : Ts->ArrayValue)
		{
			if (Tv.ArrayValue.size() < 3) { continue; }
			const FVector2D P(Tv.ArrayValue[0].NumberValue, Tv.ArrayValue[1].NumberValue);
			const float Crown = static_cast<float>(Tv.ArrayValue[2].NumberValue);
			const bool bBig = Crown >= 3.2f;
			const FSpecies& Sp = bBig ? Big : Small;
			const float S = FMath::Clamp(Crown / Sp.CrownR, 0.25f, Sp.MaxScale) * Rng.FRandRange(0.92f, 1.08f);
			const FQuat Q(FVector::UpVector, FMath::DegreesToRadians(Rng.FRandRange(0.0f, 360.0f)));
			const FVector Pos(P.X * Cm, P.Y * Cm, (Height(P) - 0.1f) * Cm);
			(bBig ? BigC : SmallC)->AddInstance(FTransform(Q, Pos - Q.RotateVector(Sp.M.Pivot * S), FVector(S)));
			++NumTrees;
		}
	}
	UE_LOG(LogTemp, Log, TEXT("KartScenery (measured %s): %d buildings, %d trees"), *SceneryFile, NumBuildings, NumTrees);
}
