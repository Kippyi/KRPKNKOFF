// SKartMenu: main menu, pause menu, session setup, multiplayer, garage (full setup page) and controls.
// Built in Slate from code (the project has no UI assets). Owned by AKartHUD.
#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"

class AKartHUD;
class AKartVehiclePawn;
class SBox;

class SKartMenu : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SKartMenu) {}
		SLATE_ARGUMENT(TWeakObjectPtr<AKartHUD>, Owner)
		SLATE_ARGUMENT(bool, bInGame)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);
	virtual bool SupportsKeyboardFocus() const override { return true; }
	virtual FReply OnKeyDown(const FGeometry& Geometry, const FKeyEvent& Key) override;
	virtual void Tick(const FGeometry& AllottedGeometry, const double InCurrentTime, const float InDeltaTime) override;

private:
	enum class EPage { Main, Practice, RaceWeekend, Multiplayer, Garage, Controls, Settings, Wheel };
	void Show(EPage Page);
	TSharedRef<SWidget> BuildMain();
	TSharedRef<SWidget> BuildSession(bool bRaceWeekend);
	TSharedRef<SWidget> BuildMultiplayer();
	TSharedRef<SWidget> BuildGarage();
	TSharedRef<SWidget> BuildControls();
	TSharedRef<SWidget> BuildSettings();
	TSharedRef<SWidget> BuildWheel();
	TSharedRef<SWidget> BuildSessionOptions(bool bRaceWeekend);
	// A content card: title, scrolling body and an optional footer pinned below it (action buttons).
	TSharedRef<SWidget> Page(const FText& Title, TSharedRef<SWidget> Body, TSharedPtr<SWidget> Footer = nullptr);

	AKartVehiclePawn* Kart() const;
	UWorld* World() const;
	void SelectClass(int32 Index);

	TWeakObjectPtr<AKartHUD> Owner;
	bool bInGame = false;
	EPage Current = EPage::Main;
	TSharedPtr<SBox> Content;
	// Multiplayer page: the server list is rebuilt when the game instance reports new search results.
	TSharedPtr<class SVerticalBox> ServerList;
	int32 SeenServersVersion = -1;
	void RebuildServerList();
};
