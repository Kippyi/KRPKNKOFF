// Procedural geometry for the kart and track placeholders (Milestone 4). Purely visual: nothing here
// is read by the physics. Real assets replace these through the visual rig (see KartVisualRig).
#pragma once

#include "CoreMinimal.h"
#include "ProceduralMeshComponent.h"

class UMaterialInterface;
class UMaterialInstanceDynamic;

// Mesh buffers for one procedural mesh section.
struct KRPKNKOFF_API FKartMeshBuilder
{
	TArray<FVector> V;
	TArray<int32> Tri;
	TArray<FVector> N;
	TArray<FVector2D> UV;
	TArray<FLinearColor> Col;
	TArray<FProcMeshTangent> Tan;

	int32 AddVertex(const FVector& P, const FVector& Normal, const FLinearColor& C = FLinearColor::White);
	void AddTri(int32 A, int32 B, int32 C) { Tri.Add(A); Tri.Add(B); Tri.Add(C); }

	// Tube (open cylinder) from P0 to P1, radius R (cm).
	void AddTube(const FVector& P0, const FVector& P1, float R, int32 Sides = 10, const FLinearColor& C = FLinearColor::White);
	// Tube along a polyline with spherical joints.
	void AddTubePath(const TArray<FVector>& Points, float R, int32 Sides = 10, const FLinearColor& C = FLinearColor::White);
	void AddSphere(const FVector& Centre, float R, int32 Rings = 8, int32 Segs = 12, const FLinearColor& C = FLinearColor::White);
	// Oriented box: centre, half extents, rotation.
	void AddBox(const FVector& Centre, const FVector& HalfExtent, const FQuat& Rot = FQuat::Identity, const FLinearColor& C = FLinearColor::White);
	// Surface of revolution about local Z: Profile = (radius, z) pairs (cm), Segments around.
	// SectorColor: optional per-angular-sector colour (e.g. a marker to make wheel spin visible).
	void AddRevolve(const TArray<FVector2D>& Profile, int32 Segments, const FLinearColor& C = FLinearColor::White,
	                int32 MarkerSector = -1, const FLinearColor& MarkerColor = FLinearColor::White);

	// Smooth surface through closed rings (all with the same point count), e.g. bodywork panels. Normals point
	// away from each ring's centroid; ends optionally capped. Color (optional) paints a livery per vertex.
	using FColorFn = TFunction<FLinearColor(int32 Ring, int32 Point, const FVector& P)>;
	void AddLoft(const TArray<TArray<FVector>>& Rings, bool bCapStart, bool bCapEnd, const FColorFn& Color = nullptr,
	             const FLinearColor& C = FLinearColor::White);
	// Closed rounded-rectangle ring (superellipse |u/a|^n + |v/b|^n = 1) in the plane spanned by AxisU / AxisV.
	static TArray<FVector> SuperEllipse(const FVector& Centre, const FVector& AxisU, const FVector& AxisV, float HalfU, float HalfV,
	                                    float Exponent, int32 Points);
	// Tube with different end radii (limbs), rounded ends.
	void AddTaperTube(const FVector& P0, const FVector& P1, float R0, float R1, int32 Sides = 12, const FLinearColor& C = FLinearColor::White);
	// Ellipsoid (or a patch of one: polar Phi / azimuth Theta ranges, radians) with radii along the rotated axes.
	void AddEllipsoid(const FVector& Centre, const FVector& Radii, const FQuat& Rot, int32 Rings, int32 Segs, const FColorFn& Color = nullptr,
	                  float PhiMin = 0.0f, float PhiMax = PI, float ThMin = 0.0f, float ThMax = 2.0f * PI);

	// Capped cylinder (disc when short) from P0 to P1.
	void AddCylinder(const FVector& P0, const FVector& P1, float R, int32 Sides = 16, const FLinearColor& C = FLinearColor::White);
	// Torus around Axis (major radius R, tube radius Rt).
	void AddTorus(const FVector& Centre, const FVector& Axis, float R, float Rt, int32 Segs = 24, int32 Sides = 8, const FLinearColor& C = FLinearColor::White);
	// Box with rounded edges (radius Round, cm), oriented.
	void AddRoundBox(const FVector& Centre, const FVector& Half, const FQuat& Rot, float Round, const FLinearColor& C = FLinearColor::White);
	// Hard colour edges: every triangle (from FirstTri on) whose corners carry different vertex colours gets its own
	// vertices in its majority colour, so livery bands stay crisp instead of blending across the triangle.
	void SharpenColors(int32 FirstTri = 0);

	void ToSection(UProceduralMeshComponent* Mesh, int32 Section, UMaterialInterface* Material, bool bCollision = false) const;
};

namespace KartVisuals
{
	// Material instance of /Game/KartVisuals/M_KartSurface with the given look (nullptr if missing).
	KRPKNKOFF_API UMaterialInstanceDynamic* MakeSurface(UObject* Outer, const FLinearColor& A, const FLinearColor& B,
	                                                    float Roughness, float RoughnessVar, float Metallic,
	                                                    float MacroScale = 0.004f, float MicroScale = 0.35f, float MacroAmount = 0.5f,
	                                                    bool bUseVertexColor = false);

	// Tyre + rim for a wheel of unloaded radius / width (metres), axle along local Z, centred.
	// Section 0 = tyre (rubber), section 1 = rim (alloy).
	// bMirror: rim face on local -Z (right-hand wheels, whose axle still points to the kart's left).
	KRPKNKOFF_API void BuildWheel(UProceduralMeshComponent* Mesh, float RadiusM, float WidthM, float RimRadiusM,
	                              UMaterialInterface* Rubber, UMaterialInterface* Alloy, bool bMirror = false);
}
