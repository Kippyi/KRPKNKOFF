#include "KartProceduralVisuals.h"

#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "Algo/Reverse.h"
#include "UObject/UObjectGlobals.h"

int32 FKartMeshBuilder::AddVertex(const FVector& P, const FVector& Normal, const FLinearColor& C)
{
	V.Add(P);
	N.Add(Normal);
	UV.Add(FVector2D(P.X * 0.01f, P.Y * 0.01f));
	Col.Add(C);
	return V.Num() - 1;
}

void FKartMeshBuilder::AddTube(const FVector& P0, const FVector& P1, float R, int32 Sides, const FLinearColor& C)
{
	const FVector Axis = (P1 - P0).GetSafeNormal();
	if (Axis.IsNearlyZero()) { return; }
	FVector U = FVector::CrossProduct(Axis, FMath::Abs(Axis.Z) < 0.9f ? FVector::UpVector : FVector::ForwardVector).GetSafeNormal();
	const FVector W = FVector::CrossProduct(Axis, U);
	const int32 Base = V.Num();
	for (int32 I = 0; I <= Sides; ++I)
	{
		const float A = 2.0f * PI * I / Sides;
		const FVector Dir = U * FMath::Cos(A) + W * FMath::Sin(A);
		AddVertex(P0 + Dir * R, Dir, C);
		AddVertex(P1 + Dir * R, Dir, C);
	}
	for (int32 I = 0; I < Sides; ++I)
	{
		const int32 A0 = Base + 2 * I, A1 = A0 + 1, B0 = A0 + 2, B1 = A0 + 3;
		AddTri(A0, B0, A1);
		AddTri(A1, B0, B1);
	}
}

void FKartMeshBuilder::AddTubePath(const TArray<FVector>& Points, float R, int32 Sides, const FLinearColor& C)
{
	for (int32 I = 0; I + 1 < Points.Num(); ++I)
	{
		AddTube(Points[I], Points[I + 1], R, Sides, C);
		if (I > 0) { AddSphere(Points[I], R, 4, Sides, C); }
	}
}

void FKartMeshBuilder::AddSphere(const FVector& Centre, float R, int32 Rings, int32 Segs, const FLinearColor& C)
{
	const int32 Base = V.Num();
	for (int32 Ri = 0; Ri <= Rings; ++Ri)
	{
		const float Phi = PI * Ri / Rings;
		for (int32 S = 0; S <= Segs; ++S)
		{
			const float Th = 2.0f * PI * S / Segs;
			const FVector Dir(FMath::Sin(Phi) * FMath::Cos(Th), FMath::Sin(Phi) * FMath::Sin(Th), FMath::Cos(Phi));
			AddVertex(Centre + Dir * R, Dir, C);
		}
	}
	for (int32 Ri = 0; Ri < Rings; ++Ri)
	{
		for (int32 S = 0; S < Segs; ++S)
		{
			const int32 A = Base + Ri * (Segs + 1) + S, B = A + Segs + 1;
			AddTri(A, A + 1, B);
			AddTri(A + 1, B + 1, B);
		}
	}
}

void FKartMeshBuilder::AddBox(const FVector& Centre, const FVector& H, const FQuat& Rot, const FLinearColor& C)
{
	static const FVector Normals[6] = {FVector(1, 0, 0), FVector(-1, 0, 0), FVector(0, 1, 0), FVector(0, -1, 0), FVector(0, 0, 1), FVector(0, 0, -1)};
	for (const FVector& Nl : Normals)
	{
		// Two axes spanning the face.
		const FVector T1 = FMath::Abs(Nl.Z) > 0.5f ? FVector(1, 0, 0) : FVector(0, 0, 1);
		const FVector T2 = FVector::CrossProduct(Nl, T1);
		const FVector Fc = Nl * H;
		const FVector E1 = T1 * H, E2 = T2 * H;
		const FVector Nw = Rot.RotateVector(Nl);
		const int32 A = AddVertex(Centre + Rot.RotateVector(Fc - E1 - E2), Nw, C);
		const int32 B = AddVertex(Centre + Rot.RotateVector(Fc + E1 - E2), Nw, C);
		const int32 Cc = AddVertex(Centre + Rot.RotateVector(Fc + E1 + E2), Nw, C);
		const int32 D = AddVertex(Centre + Rot.RotateVector(Fc - E1 + E2), Nw, C);
		AddTri(A, Cc, B);
		AddTri(A, D, Cc);
	}
}

void FKartMeshBuilder::AddRevolve(const TArray<FVector2D>& Profile, int32 Segments, const FLinearColor& C,
                                  int32 MarkerSector, const FLinearColor& MarkerColor)
{
	const int32 P = Profile.Num();
	if (P < 2) { return; }
	// Unshared vertices per sector so a marker sector can carry its own colour.
	for (int32 S = 0; S < Segments; ++S)
	{
		const FLinearColor Cs = S == MarkerSector ? MarkerColor : C;
		const float A0 = 2.0f * PI * S / Segments, A1 = 2.0f * PI * (S + 1) / Segments;
		for (int32 K = 0; K + 1 < P; ++K)
		{
			const FVector2D& Q0 = Profile[K];
			const FVector2D& Q1 = Profile[K + 1];
			// Profile normal in (r, z): perpendicular to the segment, pointing outward.
			const FVector2D Seg = (Q1 - Q0).GetSafeNormal();
			const FVector2D Nrz(Seg.Y, -Seg.X);
			auto Pt = [](const FVector2D& Q, float A) { return FVector(Q.X * FMath::Cos(A), Q.X * FMath::Sin(A), Q.Y); };
			auto Nm = [&Nrz](float A) { return FVector(Nrz.X * FMath::Cos(A), Nrz.X * FMath::Sin(A), Nrz.Y).GetSafeNormal(); };
			const int32 I0 = AddVertex(Pt(Q0, A0), Nm(A0), Cs);
			const int32 I1 = AddVertex(Pt(Q1, A0), Nm(A0), Cs);
			const int32 I2 = AddVertex(Pt(Q0, A1), Nm(A1), Cs);
			const int32 I3 = AddVertex(Pt(Q1, A1), Nm(A1), Cs);
			AddTri(I0, I2, I1);
			AddTri(I1, I2, I3);
		}
	}
}

void FKartMeshBuilder::AddLoft(const TArray<TArray<FVector>>& Rings, bool bCapStart, bool bCapEnd, const FColorFn& Color, const FLinearColor& C)
{
	const int32 R = Rings.Num();
	if (R < 2) { return; }
	const int32 M = Rings[0].Num();
	if (M < 3) { return; }
	for (const TArray<FVector>& Ring : Rings) { if (Ring.Num() != M) { return; } }
	TArray<FVector> Centroid;
	for (const TArray<FVector>& Ring : Rings)
	{
		FVector Cn = FVector::ZeroVector;
		for (const FVector& P : Ring) { Cn += P; }
		Centroid.Add(Cn / M);
	}
	const int32 Base = V.Num();
	for (int32 I = 0; I < R; ++I)
	{
		for (int32 J = 0; J < M; ++J)
		{
			const FVector& P = Rings[I][J];
			const FVector Tj = Rings[I][(J + 1) % M] - Rings[I][(J + M - 1) % M];
			const FVector Ti = Rings[FMath::Min(I + 1, R - 1)][J] - Rings[FMath::Max(I - 1, 0)][J];
			FVector Nrm = FVector::CrossProduct(Tj, Ti).GetSafeNormal();
			const FVector Out = P - Centroid[I];
			if (Nrm.IsNearlyZero()) { Nrm = Out.GetSafeNormal(); }
			else if (FVector::DotProduct(Nrm, Out) < 0.0f) { Nrm = -Nrm; }
			AddVertex(P, Nrm, Color ? Color(I, J, P) : C);
		}
	}
	// Winding follows AddBox: (C - A) x (B - A) along the outward normal. The kart material is two-sided, the
	// normals carry the lighting.
	auto Tri3 = [this](int32 A, int32 B, int32 Cc)
	{
		const FVector Nf = FVector::CrossProduct(V[Cc] - V[A], V[B] - V[A]);
		if (FVector::DotProduct(Nf, N[A] + N[B] + N[Cc]) >= 0.0f) { AddTri(A, B, Cc); } else { AddTri(A, Cc, B); }
	};
	for (int32 I = 0; I + 1 < R; ++I)
	{
		for (int32 J = 0; J < M; ++J)
		{
			const int32 A = Base + I * M + J, B = Base + I * M + (J + 1) % M;
			const int32 Cc = Base + (I + 1) * M + J, D = Base + (I + 1) * M + (J + 1) % M;
			Tri3(A, B, D);
			Tri3(A, D, Cc);
		}
	}
	auto Cap = [&](int32 I, bool bStart)
	{
		const FVector Axis = (bStart ? Centroid[0] - Centroid[1] : Centroid[R - 1] - Centroid[R - 2]).GetSafeNormal();
		const int32 Ctr = AddVertex(Centroid[I], Axis, Color ? Color(I, 0, Centroid[I]) : C);
		TArray<int32> Ring;
		for (int32 J = 0; J < M; ++J) { Ring.Add(AddVertex(Rings[I][J], Axis, Color ? Color(I, J, Rings[I][J]) : C)); }
		for (int32 J = 0; J < M; ++J) { Tri3(Ctr, Ring[J], Ring[(J + 1) % M]); }
	};
	if (bCapStart) { Cap(0, true); }
	if (bCapEnd) { Cap(R - 1, false); }
}

TArray<FVector> FKartMeshBuilder::SuperEllipse(const FVector& Centre, const FVector& AxisU, const FVector& AxisV, float HalfU, float HalfV,
                                               float Exponent, int32 Points)
{
	TArray<FVector> Out;
	for (int32 I = 0; I < Points; ++I)
	{
		const float A = 2.0f * PI * I / Points;
		const float Ca = FMath::Cos(A), Sa = FMath::Sin(A);
		const float U = FMath::Sign(Ca) * FMath::Pow(FMath::Abs(Ca), 2.0f / Exponent);
		const float W = FMath::Sign(Sa) * FMath::Pow(FMath::Abs(Sa), 2.0f / Exponent);
		Out.Add(Centre + AxisU * (U * HalfU) + AxisV * (W * HalfV));
	}
	return Out;
}

void FKartMeshBuilder::AddTaperTube(const FVector& P0, const FVector& P1, float R0, float R1, int32 Sides, const FLinearColor& C)
{
	const FVector Axis = (P1 - P0).GetSafeNormal();
	if (Axis.IsNearlyZero()) { return; }
	const FVector U = FVector::CrossProduct(Axis, FMath::Abs(Axis.Z) < 0.9f ? FVector::UpVector : FVector::ForwardVector).GetSafeNormal();
	const FVector W = FVector::CrossProduct(Axis, U);
	TArray<TArray<FVector>> Rings;
	for (int32 K = 0; K <= 4; ++K)
	{
		const float T = K / 4.0f;
		Rings.Add(SuperEllipse(FMath::Lerp(P0, P1, T), U, W, FMath::Lerp(R0, R1, T), FMath::Lerp(R0, R1, T), 2.0f, Sides));
	}
	AddLoft(Rings, false, false, nullptr, C);
	AddSphere(P0, R0, 6, Sides, C);
	AddSphere(P1, R1, 6, Sides, C);
}

void FKartMeshBuilder::AddEllipsoid(const FVector& Centre, const FVector& Radii, const FQuat& Rot, int32 Rings, int32 Segs, const FColorFn& Color,
                                    float PhiMin, float PhiMax, float ThMin, float ThMax)
{
	const bool bFull = ThMax - ThMin > 2.0f * PI - 1e-3f;
	const int32 Base = V.Num();
	const int32 Cols = Segs + 1;
	for (int32 Ri = 0; Ri <= Rings; ++Ri)
	{
		const float Phi = FMath::Lerp(PhiMin, PhiMax, float(Ri) / Rings);
		for (int32 S = 0; S <= Segs; ++S)
		{
			const float Th = FMath::Lerp(ThMin, ThMax, float(S) / Segs);
			const FVector Unit(FMath::Sin(Phi) * FMath::Cos(Th), FMath::Sin(Phi) * FMath::Sin(Th), FMath::Cos(Phi));
			const FVector Local = Unit * Radii;
			const FVector Nl = FVector(Unit.X / Radii.X, Unit.Y / Radii.Y, Unit.Z / Radii.Z).GetSafeNormal();
			const FVector P = Centre + Rot.RotateVector(Local);
			AddVertex(P, Rot.RotateVector(Nl), Color ? Color(Ri, S, P) : FLinearColor::White);
		}
	}
	(void)bFull;
	for (int32 Ri = 0; Ri < Rings; ++Ri)
	{
		for (int32 S = 0; S < Segs; ++S)
		{
			const int32 A = Base + Ri * Cols + S, B = A + Cols;
			// Same orientation as AddSphere.
			AddTri(A, A + 1, B);
			AddTri(A + 1, B + 1, B);
		}
	}
}

void FKartMeshBuilder::AddCylinder(const FVector& P0, const FVector& P1, float R, int32 Sides, const FLinearColor& C)
{
	const FVector Axis = (P1 - P0).GetSafeNormal();
	if (Axis.IsNearlyZero()) { return; }
	AddTube(P0, P1, R, Sides, C);
	const FVector U = FVector::CrossProduct(Axis, FMath::Abs(Axis.Z) < 0.9f ? FVector::UpVector : FVector::ForwardVector).GetSafeNormal();
	const FVector W = FVector::CrossProduct(Axis, U);
	for (int32 End = 0; End < 2; ++End)
	{
		const FVector Ctr = End ? P1 : P0;
		const FVector Nm = End ? Axis : -Axis;
		const int32 Mid = AddVertex(Ctr, Nm, C);
		const int32 Base = V.Num();
		for (int32 I = 0; I < Sides; ++I)
		{
			const float A = 2.0f * PI * I / Sides;
			AddVertex(Ctr + (U * FMath::Cos(A) + W * FMath::Sin(A)) * R, Nm, C);
		}
		for (int32 I = 0; I < Sides; ++I)
		{
			const int32 A = Base + I, B = Base + (I + 1) % Sides;
			const FVector Nf = FVector::CrossProduct(V[B] - V[Mid], V[A] - V[Mid]);
			if (FVector::DotProduct(Nf, Nm) >= 0.0f) { AddTri(Mid, A, B); } else { AddTri(Mid, B, A); }
		}
	}
}

void FKartMeshBuilder::AddTorus(const FVector& Centre, const FVector& Axis, float R, float Rt, int32 Segs, int32 Sides, const FLinearColor& C)
{
	const FVector Ax = Axis.GetSafeNormal();
	const FVector U = FVector::CrossProduct(Ax, FMath::Abs(Ax.Z) < 0.9f ? FVector::UpVector : FVector::ForwardVector).GetSafeNormal();
	const FVector W = FVector::CrossProduct(Ax, U);
	TArray<TArray<FVector>> Rings;
	for (int32 S = 0; S <= Segs; ++S)
	{
		const float A = 2.0f * PI * S / Segs;
		const FVector Dir = U * FMath::Cos(A) + W * FMath::Sin(A);
		Rings.Add(SuperEllipse(Centre + Dir * R, Dir, Ax, Rt, Rt, 2.0f, Sides));
	}
	AddLoft(Rings, false, false, nullptr, C);
}

void FKartMeshBuilder::AddRoundBox(const FVector& Centre, const FVector& H, const FQuat& Rot, float Round, const FLinearColor& C)
{
	// Rings of a rounded rectangle (plan) stacked in local Z; the top and bottom edges are rounded by a quarter circle.
	const float Rd = FMath::Clamp(Round, 0.05f, FMath::Min3(H.X, H.Y, H.Z) * 0.95f);
	TArray<TArray<FVector>> Rings;
	const int32 Steps = 4;
	TArray<float> Zs, Ins;
	for (int32 K = 0; K <= Steps; ++K) { const float A = 0.5f * PI * K / Steps; Zs.Add(-H.Z + Rd - Rd * FMath::Cos(A)); Ins.Add(Rd - Rd * FMath::Sin(A)); }
	for (int32 K = Steps; K >= 0; --K) { const float A = 0.5f * PI * K / Steps; Zs.Add(H.Z - Rd + Rd * FMath::Cos(A)); Ins.Add(Rd - Rd * FMath::Sin(A)); }
	const float Ex = 2.0f + 10.0f * (1.0f - Rd / FMath::Min(H.X, H.Y)); // corner sharpness in plan follows the radius
	for (int32 I = 0; I < Zs.Num(); ++I)
	{
		TArray<FVector> Ring = SuperEllipse(FVector(0, 0, Zs[I]), FVector(1, 0, 0), FVector(0, 1, 0), FMath::Max(H.X - Ins[I], 0.05f), FMath::Max(H.Y - Ins[I], 0.05f), Ex, 32);
		for (FVector& P : Ring) { P = Centre + Rot.RotateVector(P); }
		Rings.Add(Ring);
	}
	AddLoft(Rings, true, true, nullptr, C);
}

void FKartMeshBuilder::SharpenColors(int32 FirstTri)
{
	const int32 NumTri = Tri.Num() / 3;
	for (int32 T = FirstTri; T < NumTri; ++T)
	{
		const int32 I0 = Tri[3 * T], I1 = Tri[3 * T + 1], I2 = Tri[3 * T + 2];
		const FLinearColor& C0 = Col[I0];
		const FLinearColor& C1 = Col[I1];
		const FLinearColor& C2 = Col[I2];
		if (C0.Equals(C1, 1e-4f) && C1.Equals(C2, 1e-4f)) { continue; }
		const FLinearColor Pick = C1.Equals(C2, 1e-4f) ? C1 : C0; // majority (or the first corner)
		const int32 Src[3] = {I0, I1, I2};
		for (int32 K = 0; K < 3; ++K)
		{
			const int32 NewI = V.Num();
			// Copies first: Add() must not take a reference into the array it grows.
			const FVector Pv = V[Src[K]], Nv = N[Src[K]];
			const FVector2D Uv = UV[Src[K]];
			V.Add(Pv); N.Add(Nv); UV.Add(Uv); Col.Add(Pick);
			Tri[3 * T + K] = NewI;
		}
	}
}

void FKartMeshBuilder::ToSection(UProceduralMeshComponent* Mesh, int32 Section, UMaterialInterface* Material, bool bCollision) const
{
	if (!Mesh || V.Num() == 0) { return; }
	Mesh->CreateMeshSection_LinearColor(Section, V, Tri, N, UV, Col, Tan, bCollision);
	if (Material) { Mesh->SetMaterial(Section, Material); }
}

namespace KartVisuals
{
	UMaterialInstanceDynamic* MakeSurface(UObject* Outer, const FLinearColor& A, const FLinearColor& B, float Roughness, float RoughnessVar,
	                                      float Metallic, float MacroScale, float MicroScale, float MacroAmount, bool bUseVertexColor)
	{
		UMaterialInterface* Base = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/KartVisuals/M_KartSurface.M_KartSurface"));
		if (!Base) { return nullptr; }
		UMaterialInstanceDynamic* M = UMaterialInstanceDynamic::Create(Base, Outer);
		M->SetVectorParameterValue(TEXT("ColorA"), A);
		M->SetVectorParameterValue(TEXT("ColorB"), B);
		M->SetScalarParameterValue(TEXT("Roughness"), Roughness);
		M->SetScalarParameterValue(TEXT("RoughnessVar"), RoughnessVar);
		M->SetScalarParameterValue(TEXT("Metallic"), Metallic);
		M->SetScalarParameterValue(TEXT("MacroScale"), MacroScale);
		M->SetScalarParameterValue(TEXT("MicroScale"), MicroScale);
		M->SetScalarParameterValue(TEXT("MacroAmount"), MacroAmount);
		M->SetScalarParameterValue(TEXT("UseVertexColor"), bUseVertexColor ? 1.0f : 0.0f);
		return M;
	}

	static void MirrorProfile(TArray<FVector2D>& Profile)
	{
		// Negate z and reverse the order so the revolved normals still point outward.
		for (FVector2D& P : Profile) { P.Y = -P.Y; }
		Algo::Reverse(Profile);
	}

	void BuildWheel(UProceduralMeshComponent* Mesh, float RadiusM, float WidthM, float RimRadiusM, UMaterialInterface* Rubber, UMaterialInterface* Alloy, bool bMirror)
	{
		Mesh->ClearAllMeshSections();
		const float R = RadiusM * 100.0f, W = WidthM * 100.0f, Rr = RimRadiusM * 100.0f;
		const float Sh = FMath::Min(2.2f, 0.17f * W);   // shoulder radius
		const float Hw = 0.5f * W;
		// Tyre cross-section (r, z): bead on the rim flange, sidewall bulging slightly past the tread width,
		// rounded shoulder, flat tread. Symmetric.
		TArray<FVector2D> Half = {
			{Rr + 0.2f, -Hw + 0.9f}, {Rr + 0.9f, -Hw + 0.25f}, {Rr + 0.42f * (R - Rr), -Hw - 0.35f}, {Rr + 0.72f * (R - Rr), -Hw - 0.15f},
			{R - Sh * 1.1f, -Hw + 0.15f}, {R - Sh * 0.55f, -Hw + 0.45f * Sh}, {R - Sh * 0.15f, -Hw + 0.85f * Sh}, {R, -Hw + 1.35f * Sh}};
		TArray<FVector2D> Tyre = Half;
		for (int32 I = Half.Num() - 1; I >= 0; --I) { Tyre.Add(FVector2D(Half[I].X, -Half[I].Y)); }
		if (bMirror) { MirrorProfile(Tyre); }
		FKartMeshBuilder TyreB;
		TyreB.AddRevolve(Tyre, 72, FLinearColor::White, 0, FLinearColor(0.10f, 0.10f, 0.10f)); // a faint sector shows rotation
		TyreB.ToSection(Mesh, 0, Rubber);

		// Rim: inner flange, barrel, outer flange lip, dished face, hub boss.
		TArray<FVector2D> Rim = {
			{Rr + 0.45f, -Hw + 0.1f}, {Rr - 0.35f, -Hw + 0.5f}, {Rr - 0.35f, Hw - 0.6f}, {Rr + 0.45f, Hw - 0.1f}, {Rr + 0.45f, Hw + 0.25f},
			{Rr - 0.5f, Hw + 0.35f}, {Rr * 0.82f, Hw - 0.9f}, {Rr * 0.42f, Hw - 2.3f}, {Rr * 0.34f, Hw - 0.7f}, {Rr * 0.30f, Hw - 0.45f}, {0.4f, Hw - 0.45f}};
		if (bMirror) { MirrorProfile(Rim); }
		FKartMeshBuilder RimB;
		RimB.AddRevolve(Rim, 48, FLinearColor::White);
		// Face details (on the outer side): six vent holes in the dish, three hub nuts, valve stem.
		const float Zs = bMirror ? -1.0f : 1.0f;
		const FLinearColor Hole(0.03f, 0.03f, 0.03f), Nut(0.75f, 0.75f, 0.78f);
		for (int32 K = 0; K < 6; ++K)
		{
			const float A = 2.0f * PI * (K + 0.5f) / 6.0f, Rh = Rr * 0.62f;
			const float Zf = Hw - 1.6f;   // on the dished face
			RimB.AddCylinder(FVector(Rh * FMath::Cos(A), Rh * FMath::Sin(A), Zs * (Zf - 0.05f)), FVector(Rh * FMath::Cos(A), Rh * FMath::Sin(A), Zs * (Zf + 0.12f)), 0.85f, 10, Hole);
		}
		for (int32 K = 0; K < 3; ++K)
		{
			const float A = 2.0f * PI * K / 3.0f, Rn = Rr * 0.22f;
			RimB.AddCylinder(FVector(Rn * FMath::Cos(A), Rn * FMath::Sin(A), Zs * (Hw - 0.5f)), FVector(Rn * FMath::Cos(A), Rn * FMath::Sin(A), Zs * (Hw + 0.4f)), 0.55f, 6, Nut);
		}
		RimB.AddCylinder(FVector(Rr * 0.92f, 0.0f, Zs * (Hw - 0.6f)), FVector(Rr * 0.92f, 0.0f, Zs * (Hw + 1.0f)), 0.35f, 8, Hole);   // valve stem
		RimB.ToSection(Mesh, 1, Alloy);
	}

}
