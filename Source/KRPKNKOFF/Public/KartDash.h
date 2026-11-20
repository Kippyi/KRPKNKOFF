// UKartDashComponent: a Mychron-style data logger screen on the steering wheel (Milestone 8).
// A procedural quad on the rig's "display" part (the Mychron body of the imported kart) shows a canvas render
// target drawn from the kart's telemetry: shift LEDs, rpm, lap / best / live delta, water (or CHT) and EGT,
// speed, gear, and alarms (temperatures, seizing engine, damage). The same drawing code paints the HUD
// overlay in the chase camera (DrawMychron).
#pragma once

#include "CoreMinimal.h"
#include "ProceduralMeshComponent.h"
#include "KartDash.generated.h"

class AKartVehiclePawn;
class UCanvas;
class UCanvasRenderTarget2D;
class UMaterialInstanceDynamic;

UCLASS()
class KRPKNKOFF_API UKartDashComponent : public UProceduralMeshComponent
{
	GENERATED_BODY()

public:
	// Builds the screen quad (component space, cm): Centre, unit Up (screen top) and Right (driver's right).
	void Init(AKartVehiclePawn* InKart, const FVector& Centre, const FVector& Up, const FVector& Right, float WidthCm, float HeightCm);
	void Refresh();   // redraws the render target

	// Draws the dash into any canvas (render target or HUD) at X, Y with size W x H (2:1 looks right).
	static void DrawMychron(UCanvas* Canvas, float X, float Y, float W, float H, const AKartVehiclePawn& Kart);

private:
	UFUNCTION()
	void OnDraw(UCanvas* Canvas, int32 Width, int32 Height);

	UPROPERTY(Transient) TObjectPtr<UCanvasRenderTarget2D> Target;
	UPROPERTY(Transient) TObjectPtr<UMaterialInstanceDynamic> ScreenMat;
	TWeakObjectPtr<AKartVehiclePawn> Kart;
};
