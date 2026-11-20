// Procedural Milestone-1 test circuit: asphalt ribbon (closed Catmull-Rom loop), red/white kerbs
// on high-curvature sections, grass run-off. Surfaces are identified by component tags read by
// FKartUnrealTrackQuery. Geometry is real collision, so kerbs are physical bumps, not effects.
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "KartTestTrackActor.generated.h"

class UProceduralMeshComponent;
class UMaterialInterface;

UCLASS()
class KRPKNKOFF_API AKartTestTrackActor : public AActor
{
	GENERATED_BODY()

public:
	AKartTestTrackActor();
	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void BeginPlay() override;

	/** Track centre-line control points in metres (closed loop, local XY). */
	UPROPERTY(EditAnywhere, Category = "Track")
	TArray<FVector2D> ControlPoints;

	UPROPERTY(EditAnywhere, Category = "Track", meta = (ClampMin = "4", ClampMax = "20"))
	float TrackWidth = 8.0f;

	UPROPERTY(EditAnywhere, Category = "Track")
	float SampleSpacing = 0.75f;

	/** Kerbs are placed where the centre-line radius is below this (metres). */
	UPROPERTY(EditAnywhere, Category = "Track")
	float KerbRadiusThreshold = 40.0f;

	UPROPERTY(EditAnywhere, Category = "Track")
	float KerbHeight = 0.025f;

	UPROPERTY(EditAnywhere, Category = "Track")
	float KerbWidth = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Track")
	float GroundSize = 700.0f;

	/** Grass shoulder on both sides beyond the asphalt / kerb, flush with the edge (m). */
	UPROPERTY(EditAnywhere, Category = "Track")
	float ShoulderWidth = 2.5f;

	/** Loads a layout from Content/KartData/Tracks/<File> (name, width, kerbRadius, controlPoints) and
	 *  rebuilds the track; broadcasts OnLayoutChanged. Returns false (and keeps the current layout) on error. */
	bool LoadLayout(const FString& File);
	const FString& GetLayoutFile() const { return LayoutFile; }
	const FString& GetDisplayName() const { return DisplayName; }
	/** Available layouts: (file name, display name), sorted by display name. */
	static TArray<TPair<FString, FString>> ListLayouts();
	/** Fired after LoadLayout rebuilt the track (lap timing, conditions and the race world re-read it). */
	static TMulticastDelegate<void()> OnLayoutChanged;

	/** Start/finish pose (world), first centre-line sample heading along the track. */
	FTransform GetStartTransform() const;

	float GetTrackLength() const { return TrackLength; }

	/** Timing lines (world space, UE units): [0] start/finish, [1..2] sector lines. Each line is
	 *  A-B across the full track width plus run-off, with the racing direction Forward. */
	void GetTimingLines(TArray<FVector>& OutA, TArray<FVector>& OutB, TArray<FVector>& OutForward) const;

	/** Centre-line samples (world space, UE units), closed loop in driving order. */
	void GetCentreLine(TArray<FVector>& OutPoints) const;

	/** A point on the middle of the first kerb (world, UE units); false if the track has no kerbs. */
	bool GetKerbProbe(FVector& OutPoint) const { OutPoint = KerbProbe; return bHasKerbProbe; }

	// Track-wide wet look (Milestone 5): 0 = dry, 1 = standing water. Darkens the paved surfaces and
	// lowers their roughness. Visual only; grip comes from AKartConditionsActor's per-cell model.
	void SetWetness(float Wet01);

private:
	void BuildTrack();

	UPROPERTY(VisibleAnywhere, Category = "Track")
	TObjectPtr<UProceduralMeshComponent> GrassMesh;

	UPROPERTY(VisibleAnywhere, Category = "Track")
	TObjectPtr<UProceduralMeshComponent> AsphaltMesh;

	UPROPERTY(VisibleAnywhere, Category = "Track")
	TObjectPtr<UProceduralMeshComponent> KerbMesh;

	UPROPERTY(VisibleAnywhere, Category = "Track")
	TObjectPtr<class UInstancedStaticMeshComponent> BarrierTyres;

	UPROPERTY(Transient)
	TObjectPtr<class UStaticMesh> CylinderMesh;

	/** Trackside scenery (KartScenery.cpp): fencing, pit / paddock, trees. Rebuilt with the layout; -KartNoScenery skips it. */
	UPROPERTY(Transient)
	TArray<TObjectPtr<class UHierarchicalInstancedStaticMeshComponent>> SceneryComps;
	void BuildScenery(TFunctionRef<float(const FVector2D&)> Height, float HalfW);
	// Measured layouts: real buildings (footprints) and trees (crown positions / radii) from the layout's "scenery"
	// file instead of the procedural pit, fences and woods. Also disables the procedural tyre walls.
	void BuildMeasuredScenery(TFunctionRef<float(const FVector2D&)> Height);
	FString SceneryFile;
	class UHierarchicalInstancedStaticMeshComponent* SceneryComponent(class UStaticMesh* Mesh, UMaterialInterface* Override = nullptr);

	UPROPERTY(Transient)
	TObjectPtr<UMaterialInterface> BaseMaterial;

	struct FWetMaterial { TWeakObjectPtr<class UMaterialInstanceDynamic> Mid; FLinearColor A, B; float Roughness; bool bGround = false; /* M_KartGround: "Wetness" param */ };
	TArray<FWetMaterial> WetMaterials;
	float AppliedWetness = -1.0f;

	// Layout detail (Milestone 7): camber per control point (deg, + banked / - off-camber) and explicit kerbs.
	enum class EKerbType : uint8 { None = 0, Flat, Rumble, High };
	struct FKerbSpec { int32 From = 0, To = 0; int8 Side = 0; /* 0 inside, 1 outside, 2 both, 3 driver's left, 4 driver's right */ EKerbType Type = EKerbType::Flat; };
	TArray<float> ControlCamber;   // per control point, degrees
	TArray<float> ControlElev;     // per control point, metres (smooth Catmull-Rom between points)
	TArray<float> ControlWidth;    // per control point, metres (optional 5th value; default TrackWidth)
	TArray<float> SampleHalfW;     // per centre-line sample, metres (half the asphalt width)
	// Optional measured terrain (layout "terrain": LiDAR DTM resampled to the track frame, float32 grid,
	// heights relative to the layout's elevation datum). Replaces the procedural hills beyond the shoulders.
	struct FTerrainGrid
	{
		bool bValid = false;
		TArray<float> H;
		FVector2D Origin = FVector2D::ZeroVector;
		float Cell = 1.0f, MeshCell = 2.5f, Blend = 25.0f;
		int32 NX = 0, NY = 0;
		float Sample(const FVector2D& P) const
		{
			const float X = FMath::Clamp(static_cast<float>((P.X - Origin.X) / Cell), 0.0f, NX - 1.001f);
			const float Y = FMath::Clamp(static_cast<float>((P.Y - Origin.Y) / Cell), 0.0f, NY - 1.001f);
			const int32 X0 = static_cast<int32>(X), Y0 = static_cast<int32>(Y);
			const float Fx = X - X0, Fy = Y - Y0;
			const float A = FMath::Lerp(H[Y0 * NX + X0], H[Y0 * NX + X0 + 1], Fx);
			const float B = FMath::Lerp(H[(Y0 + 1) * NX + X0], H[(Y0 + 1) * NX + X0 + 1], Fx);
			return FMath::Lerp(A, B, Fy);
		}
	} Terrain;
	TArray<float> SampleElev;      // per centre-line sample, metres
	TArray<FKerbSpec> KerbSpecs;   // empty = automatic flat kerbs on the inside of tight corners
	FString LayoutFile = TEXT("TestCircuit.json");
	FString DisplayName = TEXT("Test Circuit");

	TArray<FVector2D> Samples;   // metres
	TArray<FVector2D> Tangents;  // unit
	float TrackLength = 0.0f;
	FVector KerbProbe = FVector::ZeroVector;
	bool bHasKerbProbe = false;
};
