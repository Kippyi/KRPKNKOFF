// Telemetry overlay drawn with the Canvas (no UMG assets required for Milestone 1).
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "KartHUD.generated.h"

class AKartVehiclePawn;

UCLASS()
class KRPKNKOFF_API AKartHUD : public AHUD
{
	GENERATED_BODY()

public:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	virtual void DrawHUD() override;

	// Main / pause menu (Slate, see KartMenu.h). bInGame: pause menu (Resume, Main menu) instead of main menu.
	void ShowMenu(bool bInGame);
	void HideMenu();
	bool IsMenuOpen() const { return Menu.IsValid(); }

private:
	void DrawPanel(const AKartVehiclePawn& Kart, float X, float Y);
	void DrawWheelGrid(const AKartVehiclePawn& Kart, float X, float Y);
	void DrawSession(const AKartVehiclePawn& Kart, float X, float Y);
	void DrawConditions(const AKartVehiclePawn& Kart, float X, float Y);
	void DrawSpeedTrace(const AKartVehiclePawn& Kart, float X, float Y, float W, float H);
	void DrawGG(const AKartVehiclePawn& Kart, float CX, float CY, float Radius);
	void DrawReplay(const class AKartRaceWorld& World);
	void DrawAnalysis(const AKartVehiclePawn& Kart);
	void DrawBar(float X, float Y, float W, float H, float Fraction, const FLinearColor& Color, const FString& Label);
	void Text(const FString& S, float X, float Y, const FLinearColor& Color = FLinearColor::White, float Scale = 1.0f);

	TSharedPtr<class SKartMenu> Menu;
	TSharedPtr<class SWidget> MenuHost;
	TSharedPtr<class SWidget> LicenseHost;   // tester activation screen (blocks everything until a valid key)
	TSharedPtr<class SWidget> LicenseScreen; // keeps the screen alive (SWeakWidget holds it weakly)
	void ShowLicenseScreen();

	// Rolling history for the g-g trace.
	TArray<FVector2D> GGHistory;
	int32 GGHead = 0;
};
