#include "KartMenu.h"

#include "Brushes/SlateColorBrush.h"
#include "Brushes/SlateRoundedBoxBrush.h"
#include "Framework/Application/SlateApplication.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Kismet/KismetSystemLibrary.h"
#include "KartGameInstance.h"
#include "KartHUD.h"
#include "KartSim/Setup.h"
#include "KartRealismSettings.h"
#include "KartGraphicsSettings.h"
#include "KartSoundSettings.h"
#include "KartConditionsActor.h"
#include "KartSim/EngineTune.h"
#include "KartTestTrackActor.h"
#include "KartWheelInput.h"
#include "KartVehiclePawn.h"
#include "Styling/CoreStyle.h"
#include "Widgets/Images/SImage.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/Input/SSpinBox.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Layout/SSpacer.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/SOverlay.h"
#include "Widgets/Text/STextBlock.h"

#define LOCTEXT_NAMESPACE "KartMenu"

// Visual design: a dark navigation rail on the left (logo, pages with one-line descriptions, the current
// page marked with an accent bar), content on a rounded card to the right, and the 3D scene visible behind
// (in the front end the camera slowly orbits your kart; in game the paused race is dimmed).
namespace
{
	const TCHAR* GClassFiles[] = {TEXT("Rotax125MaxSenior.json"), TEXT("KZ2_IAMEScreamer.json"), TEXT("KT100_DirectDrive.json")};
	const TCHAR* GClassNames[] = {TEXT("125cc Rotax Max Senior"), TEXT("125cc KZ2 shifter"), TEXT("100cc KT100 direct drive")};
	const TCHAR* GClassShort[] = {TEXT("Rotax Max"), TEXT("KZ2"), TEXT("KT100")};
	const TCHAR* GWeatherNames[] = {TEXT("Dry"), TEXT("Hot"), TEXT("Overcast"), TEXT("Rain"), TEXT("Drying"), TEXT("Shower"), TEXT("Cold")};

	// Palette
	const FLinearColor Accent(1.0f, 0.55f, 0.08f);
	const FLinearColor TextMain(0.94f, 0.95f, 0.97f);
	const FLinearColor TextDim(0.62f, 0.65f, 0.70f);
	const FLinearColor TextFaint(0.45f, 0.48f, 0.53f);
	const FLinearColor Good(0.55f, 0.95f, 0.6f);
	constexpr float NavWidth = 470.0f;

	int32 ClassIndex(const FString& File)
	{
		for (int32 K = 0; K < UE_ARRAY_COUNT(GClassFiles); ++K) { if (File == GClassFiles[K]) { return K; } }
		return 0;
	}

	FSlateFontInfo Font(int32 Size, bool bBold = false, int32 Spacing = 0)
	{
		FSlateFontInfo F = FCoreStyle::GetDefaultFontStyle(bBold ? "Bold" : "Regular", Size);
		F.LetterSpacing = Spacing;
		return F;
	}
	FSlateFontInfo LightFont(int32 Size) { return FCoreStyle::GetDefaultFontStyle("Light", Size); }

	// --- Brushes and button styles (static: Slate keeps pointers to brushes) ---
	const FSlateBrush* NavBrush() { static FSlateColorBrush B(FLinearColor(0.012f, 0.014f, 0.02f, 0.9f)); return &B; }
	const FSlateBrush* DimBrush() { static FSlateColorBrush B(FLinearColor(0.0f, 0.0f, 0.0f, 0.55f)); return &B; }
	const FSlateBrush* AccentBrush() { static FSlateColorBrush B(Accent); return &B; }
	const FSlateBrush* CardBrush() { static FSlateRoundedBoxBrush B(FLinearColor(0.025f, 0.03f, 0.042f, 0.93f), 16.0f); return &B; }
	const FSlateBrush* PillBrush() { static FSlateRoundedBoxBrush B(FLinearColor(1.0f, 1.0f, 1.0f, 0.06f), 8.0f); return &B; }
	const FSlateBrush* RuleBrush() { static FSlateColorBrush B(FLinearColor(1.0f, 1.0f, 1.0f, 0.13f)); return &B; }

	FButtonStyle MakeStyle(const FLinearColor& Normal, const FLinearColor& Hovered, const FLinearColor& Pressed, float Radius)
	{
		FButtonStyle S;
		S.SetNormal(FSlateRoundedBoxBrush(Normal, Radius));
		S.SetHovered(FSlateRoundedBoxBrush(Hovered, Radius));
		S.SetPressed(FSlateRoundedBoxBrush(Pressed, Radius));
		S.SetDisabled(FSlateRoundedBoxBrush(Normal * 0.5f, Radius));
		S.SetNormalPadding(FMargin(0.0f));
		S.SetPressedPadding(FMargin(0.0f));
		return S;
	}
	const FButtonStyle* NavStyle()
	{
		static const FButtonStyle S = MakeStyle(FLinearColor(0, 0, 0, 0), FLinearColor(1, 1, 1, 0.06f), FLinearColor(Accent.R, Accent.G, Accent.B, 0.25f), 8.0f);
		return &S;
	}
	const FButtonStyle* PrimaryStyle()
	{
		static const FButtonStyle S = MakeStyle(Accent, FLinearColor(1.0f, 0.66f, 0.25f), FLinearColor(0.85f, 0.42f, 0.04f), 10.0f);
		return &S;
	}
	const FButtonStyle* SecondaryStyle()
	{
		static const FButtonStyle S = MakeStyle(FLinearColor(1, 1, 1, 0.07f), FLinearColor(1, 1, 1, 0.14f), FLinearColor(Accent.R, Accent.G, Accent.B, 0.4f), 10.0f);
		return &S;
	}

	TSharedRef<SWidget> Label(const TAttribute<FText>& T, int32 Size = 14, FLinearColor C = TextMain)
	{
		return SNew(STextBlock).Text(T).Font(Font(Size)).ColorAndOpacity(C).AutoWrapText(true);
	}
	TSharedRef<SWidget> Caption(const FText& T) // small spaced capitals above a group
	{
		return SNew(STextBlock).Text(T).Font(Font(11, true, 220)).ColorAndOpacity(Accent);
	}
	TSharedRef<SWidget> Rule() { return SNew(SBox).HeightOverride(1.0f)[ SNew(SImage).Image(RuleBrush()) ]; }
	// A solid accent bar of a fixed size (left-aligned in its slot).
	TSharedRef<SWidget> AccentBar(float W, float H)
	{
		return SNew(SHorizontalBox) + SHorizontalBox::Slot().AutoWidth()[ SNew(SBox).WidthOverride(W).HeightOverride(H)[ SNew(SBorder).BorderImage(AccentBrush()) ] ];
	}

	TSharedRef<SWidget> StyledButton(const FText& T, TFunction<void()> OnClick, float Width, const FButtonStyle* Style, const FLinearColor& TextColor, int32 Size = 15)
	{
		// Wrapped in an auto-width row: a button keeps its width even in a stretching column.
		return SNew(SHorizontalBox) + SHorizontalBox::Slot().AutoWidth()
		[
			SNew(SBox).WidthOverride(Width).HeightOverride(48.0f).Padding(FMargin(0, 3))
			[
				SNew(SButton).ButtonStyle(Style).HAlign(HAlign_Center).VAlign(VAlign_Center)
				.OnClicked_Lambda([OnClick]() { OnClick(); return FReply::Handled(); })
				[ SNew(STextBlock).Text(T).Font(Font(Size, true, 60)).ColorAndOpacity(TextColor) ]
			]
		];
	}
	TSharedRef<SWidget> Button(const FText& T, TFunction<void()> OnClick, float Width = 300.0f)
	{
		return StyledButton(T, OnClick, Width, SecondaryStyle(), TextMain);
	}
	TSharedRef<SWidget> PrimaryButton(const FText& T, TFunction<void()> OnClick, float Width = 300.0f)
	{
		return StyledButton(T, OnClick, Width, PrimaryStyle(), FLinearColor(0.05f, 0.04f, 0.03f), 16);
	}

	// "Name            [<]  value  [>]" row.
	TSharedRef<SWidget> Cycler(const FText& Name, TAttribute<FText> Value, TFunction<void(int32)> Step)
	{
		auto Arrow = [Step](const TCHAR* Glyph, int32 Dir)
		{
			return SNew(SBox).WidthOverride(40.0f).HeightOverride(40.0f)
			[
				SNew(SButton).ButtonStyle(SecondaryStyle()).HAlign(HAlign_Center).VAlign(VAlign_Center)
				.OnClicked_Lambda([Step, Dir]() { Step(Dir); return FReply::Handled(); })
				[ SNew(STextBlock).Text(FText::FromString(Glyph)).Font(Font(16, true)).ColorAndOpacity(TextMain) ]
			];
		};
		return SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center).Padding(0, 0, 16, 0)[ Label(Name, 15, TextDim) ]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)[ Arrow(TEXT("‹"), -1) ]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(6, 0)
			[
				SNew(SBox).WidthOverride(250.0f).HeightOverride(40.0f)
				[
					SNew(SBorder).BorderImage(PillBrush()).HAlign(HAlign_Center).VAlign(VAlign_Center)
					[ SNew(STextBlock).Text(Value).Font(Font(15, true)).ColorAndOpacity(Accent) ]
				]
			]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)[ Arrow(TEXT("›"), 1) ];
	}

	FString TrackName(const FString& File)
	{
		for (const TPair<FString, FString>& T : AKartTestTrackActor::ListLayouts()) { if (T.Key == File) { return T.Value; } }
		return FPaths::GetBaseFilename(File);
	}
}

void SKartMenu::Construct(const FArguments& InArgs)
{
	Owner = InArgs._Owner;
	bInGame = InArgs._bInGame;

	TSharedRef<SVerticalBox> Nav = SNew(SVerticalBox);
	// Logo
	Nav->AddSlot().AutoHeight()[ SNew(STextBlock).Text(FText::FromString(TEXT("KRPKNKOFF"))).Font(Font(46, true, 120)).ColorAndOpacity(TextMain) ];
	Nav->AddSlot().AutoHeight().Padding(0, 6, 0, 10)[ AccentBar(96.0f, 4.0f) ];
	Nav->AddSlot().AutoHeight().Padding(0, 0, 0, 36)[ SNew(STextBlock).Text(LOCTEXT("Tag", "KART RACING SIMULATOR")).Font(Font(11, true, 400)).ColorAndOpacity(TextDim) ];

	auto Item = [this, &Nav](const FText& Title, const FText& Sub, EPage Page, TFunction<void()> Action)
	{
		Nav->AddSlot().AutoHeight().Padding(0, 2)
		[
			SNew(SButton).ButtonStyle(NavStyle()).OnClicked_Lambda([Action]() { Action(); return FReply::Handled(); })
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth()
				[
					SNew(SBox).WidthOverride(4.0f)
					[
						SNew(SImage).Image(AccentBrush())
						.Visibility_Lambda([this, Page]() { return Current == Page && Page != EPage::Main ? EVisibility::Visible : EVisibility::Hidden; })
					]
				]
				+ SHorizontalBox::Slot().FillWidth(1.0f).Padding(16, 9, 8, 9)
				[
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight()
					[
						SNew(STextBlock).Text(Title).Font(Font(18, true, 40))
						.ColorAndOpacity_Lambda([this, Page]() { return FSlateColor(Current == Page && Page != EPage::Main ? Accent : TextMain); })
					]
					+ SVerticalBox::Slot().AutoHeight()[ SNew(STextBlock).Text(Sub).Font(LightFont(12)).ColorAndOpacity(TextDim) ]
				]
			]
		];
	};
	auto Go = [this](EPage P) { return [this, P]() { Show(P); }; };
	if (bInGame)
	{
		Item(LOCTEXT("Resume", "Resume"), LOCTEXT("ResumeSub", "Back to the track"), EPage::Main, [this]() { if (Owner.IsValid()) { Owner->HideMenu(); } });
	}
	else
	{
		Item(LOCTEXT("Practice", "Practice"), LOCTEXT("PracticeSub", "Free running, with or without AI"), EPage::Practice, Go(EPage::Practice));
		Item(LOCTEXT("Weekend", "Race weekend"), LOCTEXT("WeekendSub", "Practice, qualifying and the race"), EPage::RaceWeekend, Go(EPage::RaceWeekend));
		Item(LOCTEXT("Multi", "Multiplayer"), LOCTEXT("MultiSub", "Host a session or join one"), EPage::Multiplayer, Go(EPage::Multiplayer));
	}
	Item(LOCTEXT("Garage", "Garage"), LOCTEXT("GarageSub", "Kart class, setup and tyres"), EPage::Garage, [this]() { if (AKartVehiclePawn* K = Kart()) { K->GarageBegin(); } Show(EPage::Garage); });
	Item(LOCTEXT("WheelBtn", "Wheel & pedals"), LOCTEXT("WheelSub", "Bind controls, force feedback"), EPage::Wheel, Go(EPage::Wheel));
	Item(LOCTEXT("Settings", "Settings"), LOCTEXT("SettingsSub", "Graphics quality, sound, realism"), EPage::Settings, Go(EPage::Settings));
	Item(LOCTEXT("Controls", "Controls"), LOCTEXT("ControlsSub", "Keyboard and gamepad keys"), EPage::Controls, Go(EPage::Controls));
	Nav->AddSlot().AutoHeight().Padding(0, 14, 0, 14)[ Rule() ];
	if (bInGame)
	{
		Item(LOCTEXT("ToMenu", "Main menu"), LOCTEXT("ToMenuSub", "Leave this session"), EPage::Main, [this]()
		{
			if (UKartGameInstance* GI = UKartGameInstance::Get(World())) { GI->bLaunchedFromMenu = false; GI->LeaveOnlineSession(); }
			UGameplayStatics::OpenLevel(World(), FName(TEXT("/Engine/Maps/Entry")));
		});
	}
	Item(LOCTEXT("Quit", "Quit"), LOCTEXT("QuitSub", "Close the game"), EPage::Main, [this]() { UKismetSystemLibrary::QuitGame(World(), nullptr, EQuitPreference::Quit, false); });
	Nav->AddSlot().FillHeight(1.0f)[ SNew(SSpacer) ];
	Nav->AddSlot().AutoHeight()[ SNew(STextBlock).Text(LOCTEXT("Footer", "Esc  back      Mouse / Tab  select")).Font(Font(11, false, 120)).ColorAndOpacity(TextFaint) ];

	ChildSlot
	[
		SNew(SOverlay)
		// In game: dim the paused race. In the front end the scene stays bright behind the rail.
		+ SOverlay::Slot()[ SNew(SImage).Image(DimBrush()).Visibility(bInGame ? EVisibility::HitTestInvisible : EVisibility::Collapsed) ]
		+ SOverlay::Slot().HAlign(HAlign_Left)
		[
			SNew(SBox).WidthOverride(NavWidth)
			[
				SNew(SBorder).BorderImage(NavBrush()).Padding(FMargin(44.0f, 56.0f, 28.0f, 36.0f))[ Nav ]
			]
		]
		+ SOverlay::Slot().HAlign(HAlign_Left).Padding(FMargin(NavWidth, 0.0f, 0.0f, 0.0f))
		[
			SNew(SBox).WidthOverride(2.0f)[ SNew(SImage).Image(AccentBrush()).ColorAndOpacity(FLinearColor(1, 1, 1, 0.6f)) ]
		]
		+ SOverlay::Slot().Padding(FMargin(NavWidth + 56.0f, 56.0f, 56.0f, 56.0f))[ SAssignNew(Content, SBox) ]
	];
	// Tests / screenshots: -KartMenuPage=<0..7> (main, practice, weekend, multiplayer, garage, controls, sound, wheel).
	int32 StartPage = 0;
	FParse::Value(FCommandLine::Get(), TEXT("KartMenuPage="), StartPage);
	Show(static_cast<EPage>(FMath::Clamp(StartPage, 0, static_cast<int32>(EPage::Wheel))));
}

AKartVehiclePawn* SKartMenu::Kart() const
{
	return Owner.IsValid() ? Cast<AKartVehiclePawn>(Owner->GetOwningPawn()) : nullptr;
}

UWorld* SKartMenu::World() const
{
	return Owner.IsValid() ? Owner->GetWorld() : nullptr;
}

FReply SKartMenu::OnKeyDown(const FGeometry& Geometry, const FKeyEvent& Key)
{
	if (Key.GetKey() == EKeys::Escape)
	{
		if (Current != EPage::Main) { Show(EPage::Main); }
		else if (bInGame && Owner.IsValid()) { Owner->HideMenu(); }
		return FReply::Handled();
	}
	return FReply::Unhandled();
}

void SKartMenu::Show(EPage P)
{
	Current = P;
	TSharedRef<SWidget> W = BuildMain();
	switch (P)
	{
	case EPage::Practice: W = BuildSession(false); break;
	case EPage::RaceWeekend: W = BuildSession(true); break;
	case EPage::Multiplayer: W = BuildMultiplayer(); break;
	case EPage::Garage: W = BuildGarage(); break;
	case EPage::Controls: W = BuildControls(); break;
	case EPage::Settings: W = BuildSettings(); break;
	case EPage::Wheel: W = BuildWheel(); break;
	default: break;
	}
	Content->SetContent(W);
	FSlateApplication::Get().SetKeyboardFocus(SharedThis(this));
}

TSharedRef<SWidget> SKartMenu::Page(const FText& Title, TSharedRef<SWidget> Body, TSharedPtr<SWidget> Footer)
{
	TSharedRef<SWidget> Foot = Footer.IsValid()
		? StaticCastSharedRef<SWidget>(SNew(SVerticalBox) + SVerticalBox::Slot().AutoHeight().Padding(0, 14, 0, 14)[ Rule() ] + SVerticalBox::Slot().AutoHeight()[ Footer.ToSharedRef() ])
		: StaticCastSharedRef<SWidget>(SNew(SSpacer));
	return SNew(SBox).MaxDesiredWidth(1180.0f).HAlign(HAlign_Fill).VAlign(VAlign_Top)
	[
		SNew(SBorder).BorderImage(CardBrush()).Padding(FMargin(40.0f, 32.0f, 40.0f, 32.0f))
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight()[ SNew(STextBlock).Text(Title).Font(Font(30, true, 40)).ColorAndOpacity(TextMain) ]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 8, 0, 22)[ AccentBar(56.0f, 3.0f) ]
			+ SVerticalBox::Slot().FillHeight(1.0f)[ SNew(SScrollBox) + SScrollBox::Slot()[ Body ] ]
			+ SVerticalBox::Slot().AutoHeight()[ Foot ] // pinned: always visible below the scrolling body
		]
	];
}

void SKartMenu::SelectClass(int32 Index)
{
	UKartGameInstance* GI = UKartGameInstance::Get(World());
	const int32 N = UE_ARRAY_COUNT(GClassFiles);
	const int32 I = (Index % N + N) % N;
	if (GI) { GI->Launch.KartClass = GClassFiles[I]; }
	if (AKartVehiclePawn* K = Kart())
	{
		const int32 Avail = K->AvailableKarts.IndexOfByKey(FString(GClassFiles[I]));
		if (Avail != INDEX_NONE) { K->SwitchKart(Avail); K->GarageBegin(); }
	}
}

TSharedRef<SWidget> SKartMenu::BuildMain()
{
	// Main page: the scene is the picture; a compact card bottom-right says what you will drive.
	UKartGameInstance* GI = UKartGameInstance::Get(World());
	TSharedRef<SVerticalBox> Card = SNew(SVerticalBox);
	if (bInGame)
	{
		Card->AddSlot().AutoHeight()[ Caption(LOCTEXT("Paused", "PAUSED")) ];
		Card->AddSlot().AutoHeight().Padding(0, 6, 0, 4)[ SNew(STextBlock).Text_Lambda([this]()
		{
			const AKartVehiclePawn* K = Kart();
			return FText::FromString(K && K->GetSim() ? FString(UTF8_TO_TCHAR(K->GetSim()->GetConfig().Name.c_str())) : FString(TEXT("Kart")));
		}).Font(Font(24, true)).ColorAndOpacity(TextMain) ];
		Card->AddSlot().AutoHeight().Padding(0, 0, 0, 16)[ Label(LOCTEXT("PausedHelp", "F5 replay  ·  F6 ghost  ·  L lap analysis  ·  Tab quick setup"), 13, TextDim) ];
		Card->AddSlot().AutoHeight()[ PrimaryButton(LOCTEXT("ResumeBtn", "RESUME"), [this]() { if (Owner.IsValid()) { Owner->HideMenu(); } }, 260.0f) ];
	}
	else
	{
		Card->AddSlot().AutoHeight()[ Caption(LOCTEXT("Ready", "READY TO RACE")) ];
		Card->AddSlot().AutoHeight().Padding(0, 8, 0, 2)[ SNew(STextBlock).Text_Lambda([GI]()
		{
			return FText::FromString(GI ? GClassNames[ClassIndex(GI->Launch.KartClass)] : TEXT(""));
		}).Font(Font(26, true)).ColorAndOpacity(TextMain) ];
		Card->AddSlot().AutoHeight().Padding(0, 0, 0, 16)[ SNew(STextBlock).Text_Lambda([GI]()
		{
			if (!GI) { return FText::GetEmpty(); }
			const FKartLaunchSettings& L = GI->Launch;
			return FText::FromString(FString::Printf(TEXT("%s   ·   %s   ·   %d AI"), *TrackName(L.Circuit),
				GWeatherNames[FMath::Clamp(L.Weather, 0, static_cast<int32>(UE_ARRAY_COUNT(GWeatherNames)) - 1)], L.NumAI));
		}).Font(Font(15)).ColorAndOpacity(TextDim) ];
		TSharedRef<SHorizontalBox> Btns = SNew(SHorizontalBox);
		Btns->AddSlot().AutoWidth().Padding(0, 0, 10, 0)[ PrimaryButton(LOCTEXT("QuickStart", "DRIVE"), [GI, this]()
		{
			if (GI) { GI->Launch.Session = 0; GI->StartSession(World(), false); }
		}, 200.0f) ];
		Btns->AddSlot().AutoWidth()[ Button(LOCTEXT("Change", "Change..."), [this]() { Show(EPage::Practice); }, 180.0f) ];
		Card->AddSlot().AutoHeight()[ Btns ];
	}
	return SNew(SBox).HAlign(HAlign_Right).VAlign(VAlign_Bottom)
	[
		SNew(SBox).WidthOverride(560.0f)
		[
			SNew(SBorder).BorderImage(CardBrush()).Padding(FMargin(32.0f, 26.0f))[ Card ]
		]
	];
}

TSharedRef<SWidget> SKartMenu::BuildSessionOptions(bool bRaceWeekend)
{
	UKartGameInstance* GI = UKartGameInstance::Get(World());
	TSharedRef<SVerticalBox> B = SNew(SVerticalBox);
	if (!GI) { B->AddSlot().AutoHeight()[ Label(LOCTEXT("NoGI", "Game instance missing (check DefaultEngine.ini)"), 14, FLinearColor::Red) ]; return B; }
	FKartLaunchSettings* L = &GI->Launch;
	auto Row = [&B](TSharedRef<SWidget> W) { B->AddSlot().AutoHeight().Padding(0, 5)[ W ]; };
	B->AddSlot().AutoHeight().Padding(0, 0, 0, 6)[ Caption(LOCTEXT("TrackKart", "TRACK AND KART")) ];
	{
		const TArray<TPair<FString, FString>> Tracks = AKartTestTrackActor::ListLayouts();
		auto TrackIndex = [Tracks, L]() { for (int32 I = 0; I < Tracks.Num(); ++I) { if (Tracks[I].Key == L->Circuit) { return I; } } return 0; };
		Row(Cycler(LOCTEXT("Track", "Track"),
		           TAttribute<FText>::CreateLambda([Tracks, TrackIndex]() { return Tracks.Num() ? FText::FromString(Tracks[TrackIndex()].Value) : FText::FromString(TEXT("(none)")); }),
		           [Tracks, TrackIndex, L](int32 D) { if (Tracks.Num()) { L->Circuit = Tracks[(TrackIndex() + D + Tracks.Num()) % Tracks.Num()].Key; } }));
	}
	Row(Cycler(LOCTEXT("Class", "Kart class"),
	           TAttribute<FText>::CreateLambda([L]() { return FText::FromString(GClassShort[ClassIndex(L->KartClass)]); }),
	           [this, L](int32 D) { SelectClass(ClassIndex(L->KartClass) + D); }));
	B->AddSlot().AutoHeight().Padding(0, 16, 0, 6)[ Caption(LOCTEXT("Field", "FIELD")) ];
	Row(Cycler(LOCTEXT("AI", "AI opponents"), TAttribute<FText>::CreateLambda([L]() { return L->NumAI == 0 ? FText::FromString(TEXT("None (solo)")) : FText::AsNumber(L->NumAI); }),
	           [L](int32 D) { L->NumAI = FMath::Clamp(L->NumAI + D, 0, 11); }));
	Row(Cycler(LOCTEXT("Str", "AI strength"), TAttribute<FText>::CreateLambda([L]() { return FText::FromString(FString::Printf(TEXT("%.0f %%"), L->AIStrength * 100.0f)); }),
	           [L](int32 D) { L->AIStrength = FMath::Clamp(L->AIStrength + 0.05f * D, 0.6f, 1.05f); }));
	B->AddSlot().AutoHeight().Padding(0, 16, 0, 6)[ Caption(LOCTEXT("Conditions", "CONDITIONS")) ];
	Row(Cycler(LOCTEXT("Wx", "Weather"), TAttribute<FText>::CreateLambda([L]() { return FText::FromString(GWeatherNames[FMath::Clamp(L->Weather, 0, static_cast<int32>(UE_ARRAY_COUNT(GWeatherNames)) - 1)]); }),
	           [L](int32 D) { const int32 N = UE_ARRAY_COUNT(GWeatherNames); L->Weather = (L->Weather + D + N) % N; }));
	Row(Cycler(LOCTEXT("Rub", "Track surface"), TAttribute<FText>::CreateLambda([L]() { return FText::FromString(L->bRubbered ? TEXT("Rubbered in") : TEXT("Green")); }),
	           [L](int32) { L->bRubbered = !L->bRubbered; }));
	if (bRaceWeekend)
	{
		B->AddSlot().AutoHeight().Padding(0, 16, 0, 6)[ Caption(LOCTEXT("Format", "FORMAT")) ];
		Row(Cycler(LOCTEXT("Q", "Qualifying length"), TAttribute<FText>::CreateLambda([L]() { return FText::FromString(FString::Printf(TEXT("%.0f min"), L->QualiMinutes)); }),
		           [L](int32 D) { L->QualiMinutes = FMath::Clamp(L->QualiMinutes + D, 1.0f, 30.0f); }));
		Row(Cycler(LOCTEXT("Laps", "Race laps"), TAttribute<FText>::CreateLambda([L]() { return FText::AsNumber(L->RaceLaps); }),
		           [L](int32 D) { L->RaceLaps = FMath::Clamp(L->RaceLaps + D, 1, 50); }));
	}
	return B;
}

TSharedRef<SWidget> SKartMenu::BuildSession(bool bRaceWeekend)
{
	TSharedRef<SVerticalBox> B = SNew(SVerticalBox);
	B->AddSlot().AutoHeight().Padding(0, 0, 0, 20)[ Label(bRaceWeekend
		? LOCTEXT("WkHelp", "Starts with practice. Press Y on track for qualifying, then Y again for the race (the grid is the qualifying order).")
		: LOCTEXT("PrHelp", "Free practice. Tab = quick setup, L = lap analysis, F5 = replay, Esc = menu."), 14, TextDim) ];
	B->AddSlot().AutoHeight()[ BuildSessionOptions(bRaceWeekend) ];
	auto Go = [this](int32 Session)
	{
		if (UKartGameInstance* GI = UKartGameInstance::Get(World())) { GI->Launch.Session = Session; GI->StartSession(World(), false); }
	};
	TSharedRef<SHorizontalBox> Btns = SNew(SHorizontalBox);
	Btns->AddSlot().AutoWidth().Padding(0, 0, 10, 0)[ PrimaryButton(bRaceWeekend ? LOCTEXT("StartWk", "START WEEKEND") : LOCTEXT("Start", "START"), [Go]() { Go(0); }, 260.0f) ];
	if (bRaceWeekend) { Btns->AddSlot().AutoWidth()[ Button(LOCTEXT("StraightRace", "Straight to the race"), [Go]() { Go(2); }, 260.0f) ]; }
	B->AddSlot().AutoHeight().Padding(0, 24, 0, 0)[ Btns ];
	return Page(bRaceWeekend ? LOCTEXT("WkTitle", "Race weekend") : LOCTEXT("PrTitle", "Practice"), B);
}

TSharedRef<SWidget> SKartMenu::BuildMultiplayer()
{
	UKartGameInstance* GI = UKartGameInstance::Get(World());
	TSharedRef<SVerticalBox> B = SNew(SVerticalBox);
	if (!GI) { B->AddSlot().AutoHeight()[ Label(LOCTEXT("NoGIm", "Game instance missing"), 14, FLinearColor::Red) ]; return Page(LOCTEXT("MpTitle", "Multiplayer"), B); }
	if (GI->ServerName.IsEmpty()) { GI->ServerName = FString::Printf(TEXT("%s's server"), *GI->GetLocalPlayerName()); }
	// Status: Steam signed in (server list + relay, no port forwarding) or not running (direct IP only).
	B->AddSlot().AutoHeight().Padding(0, 0, 0, 14)
	[
		SNew(SBorder).BorderImage(PillBrush()).Padding(FMargin(14.0f, 10.0f))
		[
			SNew(STextBlock).Font(Font(13, true)).AutoWrapText(true)
			.Text_Lambda([GI]() { return FText::FromString(GI->GetOnlineStatus()); })
			.ColorAndOpacity_Lambda([GI]() { return FSlateColor(GI->IsSteamOnline() ? Good : Accent); })
		]
	];

	// --- Server list ---
	B->AddSlot().AutoHeight().Padding(0, 0, 0, 8)
	[
		SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center)[ Caption(LOCTEXT("ListCap", "SERVER LIST")) ]
		+ SHorizontalBox::Slot().AutoWidth()[ Button(LOCTEXT("Refresh", "Refresh"), [GI]() { GI->RefreshServers(); }, 160.0f) ]
	];
	B->AddSlot().AutoHeight().Padding(0, 0, 0, 4)
	[
		SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().FillWidth(0.34f)[ SNew(STextBlock).Text(LOCTEXT("ColName", "Server")).Font(Font(11, true, 120)).ColorAndOpacity(TextFaint) ]
		+ SHorizontalBox::Slot().FillWidth(0.2f)[ SNew(STextBlock).Text(LOCTEXT("ColTrack", "Track")).Font(Font(11, true, 120)).ColorAndOpacity(TextFaint) ]
		+ SHorizontalBox::Slot().FillWidth(0.14f)[ SNew(STextBlock).Text(LOCTEXT("ColClass", "Class")).Font(Font(11, true, 120)).ColorAndOpacity(TextFaint) ]
		+ SHorizontalBox::Slot().FillWidth(0.12f)[ SNew(STextBlock).Text(LOCTEXT("ColPlayers", "Drivers")).Font(Font(11, true, 120)).ColorAndOpacity(TextFaint) ]
		+ SHorizontalBox::Slot().FillWidth(0.2f)[ SNew(SSpacer) ]
	];
	B->AddSlot().AutoHeight()[ SAssignNew(ServerList, SVerticalBox) ];
	SeenServersVersion = -1; // rebuilt on the next tick
	if (!GI->IsSearching()) { GI->RefreshServers(); }

	// --- Host ---
	B->AddSlot().AutoHeight().Padding(0, 22, 0, 22)[ Rule() ];
	B->AddSlot().AutoHeight().Padding(0, 0, 0, 6)[ Caption(LOCTEXT("Host", "HOST A SERVER")) ];
	B->AddSlot().AutoHeight().Padding(0, 0, 0, 8)
	[
		SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 12, 0)[ Label(LOCTEXT("SrvName", "Server name"), 15, TextDim) ]
		+ SHorizontalBox::Slot().FillWidth(1.0f)
		[
			SNew(SBox).HeightOverride(42.0f)
			[
				SNew(SEditableTextBox).Font(Font(15)).Text(FText::FromString(GI->ServerName))
				.OnTextChanged_Lambda([GI](const FText& T) { GI->ServerName = T.ToString(); })
			]
		]
	];
	B->AddSlot().AutoHeight()[ BuildSessionOptions(true) ];
	B->AddSlot().AutoHeight().Padding(0, 14, 0, 0)[ PrimaryButton(LOCTEXT("HostGo", "HOST"), [this, GI]() { GI->Launch.Session = 0; GI->HostOnline(World()); }, 220.0f) ];
	B->AddSlot().AutoHeight().Padding(0, 6)[ Label(LOCTEXT("HostNote", "Your server appears in everyone's list. Connections go through Steam's relays: no port forwarding, no VPN."), 12, TextFaint) ];

	// --- Direct IP (fallback) ---
	B->AddSlot().AutoHeight().Padding(0, 22, 0, 22)[ Rule() ];
	B->AddSlot().AutoHeight().Padding(0, 0, 0, 8)[ Caption(LOCTEXT("Join", "DIRECT CONNECT (IP)")) ];
	B->AddSlot().AutoHeight()
	[
		SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center)
		[
			SNew(SBox).HeightOverride(42.0f)
			[
				SNew(SEditableTextBox).Font(Font(15)).HintText(LOCTEXT("JoinHint", "Host IP address")).Text(FText::FromString(GI->JoinAddress))
				.OnTextChanged_Lambda([GI](const FText& T) { GI->JoinAddress = T.ToString().TrimStartAndEnd(); })
			]
		]
		+ SHorizontalBox::Slot().AutoWidth().Padding(10, 0, 0, 0)[ Button(LOCTEXT("JoinGo", "Connect"), [this, GI]() { GI->JoinGame(World()); }, 160.0f) ]
	];
	B->AddSlot().AutoHeight().Padding(0, 8)[ Label(LOCTEXT("MpNote", "Direct IP needs UDP port 7777 forwarded on the host's router; the server list does not. Testing uses Steam's public test app (Spacewar), so friends see you \"playing Spacewar\"."), 12, TextFaint) ];
	return Page(LOCTEXT("MpTitle", "Multiplayer"), B);
}

void SKartMenu::Tick(const FGeometry& AllottedGeometry, const double InCurrentTime, const float InDeltaTime)
{
	SCompoundWidget::Tick(AllottedGeometry, InCurrentTime, InDeltaTime);
	if (Current != EPage::Multiplayer || !ServerList.IsValid()) { return; }
	const UKartGameInstance* GI = UKartGameInstance::Get(World());
	if (GI && GI->GetServersVersion() != SeenServersVersion) { RebuildServerList(); }
}

void SKartMenu::RebuildServerList()
{
	UKartGameInstance* GI = UKartGameInstance::Get(World());
	if (!GI || !ServerList.IsValid()) { return; }
	SeenServersVersion = GI->GetServersVersion();
	ServerList->ClearChildren();
	const TArray<FKartServerInfo>& L = GI->GetServers();
	if (L.Num() == 0)
	{
		ServerList->AddSlot().AutoHeight().Padding(0, 8)
		[
			SNew(STextBlock).Font(Font(14)).ColorAndOpacity(TextDim)
			.Text_Lambda([GI]() { return GI->IsSearching() ? LOCTEXT("Searching", "Searching for servers...") : LOCTEXT("NoServers", "No servers right now. Host one, or press Refresh."); })
		];
		return;
	}
	for (int32 I = 0; I < L.Num(); ++I)
	{
		const FKartServerInfo& S = L[I];
		const bool bFull = S.MaxPlayers > 0 && S.Players >= S.MaxPlayers;
		ServerList->AddSlot().AutoHeight().Padding(0, 3)
		[
			SNew(SBorder).BorderImage(PillBrush()).Padding(FMargin(12.0f, 4.0f))
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().FillWidth(0.34f).VAlign(VAlign_Center)
				[
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight()[ SNew(STextBlock).Text(FText::FromString(S.Name.IsEmpty() ? TEXT("KRPKNKOFF server") : S.Name)).Font(Font(15, true)).ColorAndOpacity(TextMain) ]
					+ SVerticalBox::Slot().AutoHeight()[ SNew(STextBlock).Text(FText::FromString(S.Host)).Font(LightFont(11)).ColorAndOpacity(TextFaint) ]
				]
				+ SHorizontalBox::Slot().FillWidth(0.2f).VAlign(VAlign_Center)[ SNew(STextBlock).Text(FText::FromString(S.Track)).Font(Font(14)).ColorAndOpacity(TextDim) ]
				+ SHorizontalBox::Slot().FillWidth(0.14f).VAlign(VAlign_Center)[ SNew(STextBlock).Text(FText::FromString(S.KartClass)).Font(Font(14)).ColorAndOpacity(TextDim) ]
				+ SHorizontalBox::Slot().FillWidth(0.12f).VAlign(VAlign_Center)
				[
					SNew(STextBlock).Text(FText::FromString(FString::Printf(TEXT("%d / %d"), S.Players, S.MaxPlayers))).Font(Font(14, true)).ColorAndOpacity(bFull ? TextFaint : Good)
				]
				+ SHorizontalBox::Slot().FillWidth(0.2f).HAlign(HAlign_Right).VAlign(VAlign_Center)
				[
					bFull ? StaticCastSharedRef<SWidget>(SNew(STextBlock).Text(LOCTEXT("Full", "FULL")).Font(Font(13, true)).ColorAndOpacity(TextFaint))
					      : PrimaryButton(LOCTEXT("JoinRow", "JOIN"), [GI, I]() { GI->JoinServer(I); }, 130.0f)
				]
			]
		];
	}
}

TSharedRef<SWidget> SKartMenu::BuildGarage()
{
	TSharedRef<SVerticalBox> B = SNew(SVerticalBox);
	B->AddSlot().AutoHeight().Padding(0, 0, 0, 8)[ Caption(LOCTEXT("ClassCap", "KART CLASS")) ];
	TSharedRef<SHorizontalBox> Classes = SNew(SHorizontalBox);
	for (int32 I = 0; I < UE_ARRAY_COUNT(GClassFiles); ++I)
	{
		Classes->AddSlot().AutoWidth().Padding(0, 0, 8, 0)
		[
			SNew(SBox).WidthOverride(250.0f).HeightOverride(48.0f).Padding(FMargin(0, 3))
			[
				SNew(SButton).ButtonStyle(SecondaryStyle()).HAlign(HAlign_Center).VAlign(VAlign_Center)
				.OnClicked_Lambda([this, I]() { SelectClass(I); return FReply::Handled(); })
				[
					// The kart on track: accent text.
					SNew(STextBlock).Text(FText::FromString(GClassShort[I])).Font(Font(15, true, 60))
					.ColorAndOpacity_Lambda([this, I]() { const AKartVehiclePawn* K = Kart(); return FSlateColor(K && ClassIndex(K->KartConfigFile) == I ? Accent : TextMain); })
				]
			]
		];
	}
	B->AddSlot().AutoHeight()[ Classes ];
	B->AddSlot().AutoHeight().Padding(0, 10, 0, 18)[ Label(TAttribute<FText>::CreateLambda([this]()
	{
		const AKartVehiclePawn* K = Kart();
		if (!K) { return FText::FromString(TEXT("No kart")); }
		const kartsim::KartConfig& C = K->GetEditConfig();
		return FText::FromString(FString::Printf(TEXT("%s   ·   %.1f kg with driver   ·   front %.1f %%   ·   final drive %d:%d (%.2f)   ·   CoG %.0f mm"),
			UTF8_TO_TCHAR(C.Name.c_str()), C.Chassis.Mass, C.Chassis.FrontWeightFraction * 100.0, C.FinalDrive.EngineSprocket, C.FinalDrive.AxleSprocket,
			static_cast<double>(C.FinalDrive.AxleSprocket) / C.FinalDrive.EngineSprocket, C.Chassis.CogHeight * 1000.0));
	}), 13, TextDim) ];

	B->AddSlot().AutoHeight().Padding(0, 0, 0, 8)[ Caption(LOCTEXT("SetupCap", "SETUP")) ];
	int32 Count = 0;
	const kartsim::SetupParam* Params = kartsim::GetSetupParams(Count);
	for (int32 I = 0; I < Count; ++I)
	{
		const kartsim::SetupParam* P = &Params[I];
		B->AddSlot().AutoHeight().Padding(0, 4)
		[
			SNew(SHorizontalBox)
			.Visibility_Lambda([this, P]() { const AKartVehiclePawn* K = Kart(); return (K && P->IsAvailable(K->GetEditConfig())) ? EVisibility::Visible : EVisibility::Collapsed; })
			+ SHorizontalBox::Slot().FillWidth(0.34f).VAlign(VAlign_Center)
			[
				Label(FText::FromString(FString::Printf(TEXT("%s%s"), UTF8_TO_TCHAR(P->Label), P->Unit[0] ? *FString::Printf(TEXT("  (%s)"), UTF8_TO_TCHAR(P->Unit)) : TEXT(""))), 14, TextMain)
			]
			+ SHorizontalBox::Slot().FillWidth(0.2f).Padding(10, 0).VAlign(VAlign_Center)
			[
				SNew(SBox).HeightOverride(38.0f)
				[
					SNew(SSpinBox<double>).MinValue(P->Min).MaxValue(P->Max).MinSliderValue(P->Min).MaxSliderValue(P->Max).Delta(P->Step).Font(Font(14, true))
					.Value_Lambda([this, P]() { const AKartVehiclePawn* K = Kart(); return K ? P->Get(K->GetEditConfig()) : P->Min; })
					.OnValueChanged_Lambda([this, I](double V) { if (AKartVehiclePawn* K = Kart()) { K->GarageSetValue(I, V); } })
				]
			]
			+ SHorizontalBox::Slot().FillWidth(0.46f).VAlign(VAlign_Center)[ Label(FText::FromString(UTF8_TO_TCHAR(P->Help)), 12, TextFaint) ]
		];
	}
	TSharedRef<SHorizontalBox> Actions = SNew(SHorizontalBox);
	Actions->AddSlot().AutoWidth().Padding(0, 0, 10, 0)[ PrimaryButton(LOCTEXT("Apply", "APPLY + SAVE"), [this]() { if (AKartVehiclePawn* K = Kart()) { K->GarageApply(); } }, 220.0f) ];
	Actions->AddSlot().AutoWidth().Padding(0, 0, 10, 0)[ Button(LOCTEXT("Revert", "Revert"), [this]() { if (AKartVehiclePawn* K = Kart()) { K->GarageBegin(); } }, 150.0f) ];
	Actions->AddSlot().AutoWidth().Padding(0, 0, 10, 0)[ Button(LOCTEXT("Defaults", "Class defaults"), [this]() { if (AKartVehiclePawn* K = Kart()) { K->GarageDefaults(); } }, 200.0f) ];
	Actions->AddSlot().AutoWidth()[ Button(LOCTEXT("NewTyres", "Fit new tyres"), [this]() { if (AKartVehiclePawn* K = Kart()) { K->FitNewTyres(); } }, 200.0f) ];
	TSharedRef<SVerticalBox> Foot = SNew(SVerticalBox);
	Foot->AddSlot().AutoHeight()[ Actions ];
	Foot->AddSlot().AutoHeight().Padding(0, 4, 0, 0)[ Label(TAttribute<FText>::CreateLambda([this]() { const AKartVehiclePawn* K = Kart(); return FText::FromString(K ? K->GetSetupMessage() : FString()); }), 13, Good) ];

	B->AddSlot().AutoHeight().Padding(0, 18, 0, 8)[ Caption(LOCTEXT("TyreCap", "TYRE SETS")) ];
	B->AddSlot().AutoHeight()[ Label(TAttribute<FText>::CreateLambda([this]()
	{
		const AKartVehiclePawn* K = Kart();
		if (!K || !K->GetSim()) { return FText::GetEmpty(); }
		const kartsim::KartConfig& C = K->GetSim()->GetConfig();
		FString S;
		for (int32 Set = 0; Set < C.NumCompounds; ++Set)
		{
			S += FString::Printf(TEXT("%s%s%s:   tread  FL %.1f   FR %.1f   RL %.1f   RR %.1f mm   (new %.1f)"), Set ? TEXT("\n") : TEXT(""),
				UTF8_TO_TCHAR(C.CompoundName[Set].c_str()), C.Compound == Set ? TEXT("  [fitted]") : TEXT(""),
				C.CompoundFront[Set].TreadDepth * (1.0 - K->GetSetTyreWear(Set, 0)), C.CompoundFront[Set].TreadDepth * (1.0 - K->GetSetTyreWear(Set, 1)),
				C.CompoundRear[Set].TreadDepth * (1.0 - K->GetSetTyreWear(Set, 2)), C.CompoundRear[Set].TreadDepth * (1.0 - K->GetSetTyreWear(Set, 3)),
				C.CompoundRear[Set].TreadDepth);
		}
		return FText::FromString(S);
	}), 13, TextMain) ];
	// Jetting chart for today's air (the Rotax jetting app / IAME table equivalent, see EngineTune.h).
	B->AddSlot().AutoHeight().Padding(0, 12, 0, 0)[ SNew(STextBlock).Font(Font(13)).ColorAndOpacity(TextMain).Text_Lambda([this]()
	{
		const AKartVehiclePawn* K = Kart();
		if (!K || !K->GetSim()) { return FText::GetEmpty(); }
		kartsim::AirState A;
		if (const AKartConditionsActor* CA = K->GetConditions())
		{
			const kartsim::WeatherState& W = CA->Get().Weather;
			A.TempC = W.AirTemp; A.PressureKPa = W.PressureKPa; A.Humidity = W.Humidity;
		}
		const kartsim::EngineTuneParams& T = K->GetEditConfig().Tuning;
		const bool bJet = T.Adjust == kartsim::ECarbAdjust::MainJet;
		return FText::FromString(FString::Printf(TEXT("Air %.0f C  %.0f hPa  %.0f %% RH  ->  density %.3f kg/m3,  power %+.1f %% vs rated.   Jetting chart: %s %s  (fitted %s)%s"),
			A.TempC, A.PressureKPa * 10.0, A.Humidity * 100.0, kartsim::MoistAirDensity(A), (kartsim::IsoPowerCorrection(A) - 1.0) * 100.0,
			bJet ? TEXT("main jet") : TEXT("needle"),
			*(bJet ? FString::Printf(TEXT("%.0f"), kartsim::BestJet(T, A)) : FString::Printf(TEXT("%.2f turns"), kartsim::BestJet(T, A))),
			*(bJet ? FString::Printf(TEXT("%.0f"), T.Jet) : FString::Printf(TEXT("%.2f"), T.Jet)),
			T.bWaterCooled ? *FString::Printf(TEXT(".   Water window %.0f-%.0f C: tape the radiator on cold days."), T.OptimumLow, T.OptimumHigh) : TEXT(".")));
	}) ];
	B->AddSlot().AutoHeight().Padding(0, 14, 0, 0)[ Label(LOCTEXT("GarageNote", "Changes take effect on Apply (the kart is reset where it stands). Setups are saved per class and loaded automatically. Ranges: CIK limits where they exist, otherwise typical adjustment spans."), 12, TextFaint) ];
	return Page(LOCTEXT("GarageTitle", "Garage"), B, Foot);
}

TSharedRef<SWidget> SKartMenu::BuildSettings()
{
	TSharedRef<SVerticalBox> B = SNew(SVerticalBox);
	B->AddSlot().AutoHeight().Padding(0, 0, 0, 6)[ Caption(LOCTEXT("GfxCap", "GRAPHICS")) ];
	auto GfxRow = [&B](const FText& Name, TFunction<FString()> Value, TFunction<void(int32)> Step)
	{
		B->AddSlot().AutoHeight().Padding(0, 5)[ Cycler(Name, TAttribute<FText>::CreateLambda([Value]() { return FText::FromString(Value()); }), Step) ];
	};
	GfxRow(LOCTEXT("Quality", "Quality"), []() { return FString(FKartGraphicsSettings::PresetName(FKartGraphicsSettings::Get().Preset)); },
	       [](int32 D) { FKartGraphicsSettings& G = FKartGraphicsSettings::Get(); G.SetPreset(FMath::Clamp(G.Preset + D, 0, 3)); });
	GfxRow(LOCTEXT("RScale", "Render scale"), []() { return FString::Printf(TEXT("%d %%"), FKartGraphicsSettings::Get().RenderScale); },
	       [](int32 D) { FKartGraphicsSettings& G = FKartGraphicsSettings::Get(); G.RenderScale = FMath::Clamp(G.RenderScale + 10 * D, 40, 100); G.Apply(); G.Save(); });
	GfxRow(LOCTEXT("FpsCap", "Frame limit"), []() { const int32 F = FKartGraphicsSettings::Get().FrameLimit; return F == 0 ? FString(TEXT("Unlimited")) : FString::Printf(TEXT("%d fps"), F); },
	       [](int32 D)
	       {
		       static const int32 Steps[] = {30, 60, 90, 120, 144, 0};
		       FKartGraphicsSettings& G = FKartGraphicsSettings::Get();
		       int32 I = 0;
		       while (I < 5 && Steps[I] != G.FrameLimit) { ++I; }
		       G.FrameLimit = Steps[FMath::Clamp(I + D, 0, 5)];
		       G.Apply();
		       G.Save();
	       });
	GfxRow(LOCTEXT("VSync", "VSync"), []() { return FString(FKartGraphicsSettings::Get().bVSync ? TEXT("On") : TEXT("Off")); },
	       [](int32) { FKartGraphicsSettings& G = FKartGraphicsSettings::Get(); G.bVSync = !G.bVSync; G.Apply(); G.Save(); });
	GfxRow(LOCTEXT("WinMode", "Display"), []() { return FKartGraphicsSettings::WindowModeName(); }, [](int32 D) { FKartGraphicsSettings::CycleWindowMode(D); });
	GfxRow(LOCTEXT("ShowFps", "Show FPS"), []() { return FString(FKartGraphicsSettings::Get().bShowFps ? TEXT("On") : TEXT("Off")); },
	       [](int32) { FKartGraphicsSettings& G = FKartGraphicsSettings::Get(); G.bShowFps = !G.bShowFps; G.Save(); });
	B->AddSlot().AutoHeight().Padding(0, 4, 0, 0)[ Label(LOCTEXT("GfxNote",
		"Low / Medium: no Lumen global illumination or ray tracing (much faster, flatter light). High / Epic need a GPU with "
		"ray tracing. Laptops: make sure the game runs on the dedicated GPU (Windows Settings > System > Display > Graphics)."), 12, TextFaint) ];
	B->AddSlot().AutoHeight().Padding(0, 14, 0, 6)[ Caption(LOCTEXT("VolCap", "VOLUMES")) ];
	auto Vol = [&B](const FText& Name, float FKartSoundSettings::* Field)
	{
		B->AddSlot().AutoHeight().Padding(0, 5)
		[
			Cycler(Name, TAttribute<FText>::CreateLambda([Field]() { return FText::FromString(FString::Printf(TEXT("%.0f %%"), FKartSoundSettings::Get().*Field * 100.0f)); }),
			       [Field](int32 D) { FKartSoundSettings& S = FKartSoundSettings::Get(); S.*Field = FMath::Clamp(FMath::RoundToFloat((S.*Field + 0.1f * D) * 10.0f) / 10.0f, 0.0f, 1.0f); S.Save(); })
		];
	};
	Vol(LOCTEXT("Master", "Master"), &FKartSoundSettings::Master);
	Vol(LOCTEXT("EngineVol", "Your engine"), &FKartSoundSettings::Engine);
	Vol(LOCTEXT("OthersVol", "Other karts"), &FKartSoundSettings::OtherKarts);
	Vol(LOCTEXT("FxVol", "Tyres and contacts"), &FKartSoundSettings::Effects);
	Vol(LOCTEXT("WindVol", "Wind"), &FKartSoundSettings::Wind);
	B->AddSlot().AutoHeight().Padding(0, 5)
	[
		Cycler(LOCTEXT("EngTone", "Engine tone"), TAttribute<FText>::CreateLambda([]()
		{
			const float T = FKartSoundSettings::Get().EngineTone;
			return FText::FromString(FMath::IsNearlyEqual(T, 1.0f, 0.01f) ? FString(TEXT("Default")) : FString::Printf(TEXT("%s %.0f %%"), T < 1.0f ? TEXT("Darker") : TEXT("Brighter"), FMath::Abs(T - 1.0f) * 100.0f));
		}),
		[](int32 D) { FKartSoundSettings& S = FKartSoundSettings::Get(); S.EngineTone = FMath::Clamp(FMath::RoundToFloat((S.EngineTone + 0.1f * D) * 10.0f) / 10.0f, 0.5f, 1.6f); S.Save(); })
	];
	B->AddSlot().AutoHeight().Padding(0, 5)
	[
		Cycler(LOCTEXT("Mute", "Mute all"), TAttribute<FText>::CreateLambda([]() { return FText::FromString(FKartSoundSettings::Get().bMute ? TEXT("On") : TEXT("Off")); }),
		       [](int32) { FKartSoundSettings& S = FKartSoundSettings::Get(); S.bMute = !S.bMute; S.Save(); })
	];
	B->AddSlot().AutoHeight().Padding(0, 14, 0, 6)[ Caption(LOCTEXT("RealCap", "REALISM")) ];
	auto Toggle = [&B](const FText& Name, bool FKartRealismSettings::* Field)
	{
		B->AddSlot().AutoHeight().Padding(0, 5)
		[
			Cycler(Name, TAttribute<FText>::CreateLambda([Field]() { return FText::FromString(FKartRealismSettings::Get().*Field ? TEXT("On") : TEXT("Off")); }),
			       [Field](int32) { FKartRealismSettings& S = FKartRealismSettings::Get(); S.*Field = !(S.*Field); S.Save(); })
		];
	};
	Toggle(LOCTEXT("Dmg", "Mechanical damage"), &FKartRealismSettings::bDamage);
	Toggle(LOCTEXT("Fail", "Engine seizure (lean / overheating)"), &FKartRealismSettings::bEngineFailures);
	Toggle(LOCTEXT("Dash", "Mychron on the HUD (chase camera)"), &FKartRealismSettings::bDashOverlay);
	B->AddSlot().AutoHeight().Padding(0, 18, 0, 0)[ Label(LOCTEXT("SoundNote",
		"Karts are silent while a menu is open. Engine sound is synthesised until real recordings are added: 16-bit WAV loops named "
		"on_<rpm>.wav / off_<rpm>.wav in Content/KartData/Sounds/<class id>/ are used automatically. Settings are saved immediately."), 12, TextFaint) ];
	return Page(LOCTEXT("SettingsTitle", "Settings"), B);
}

TSharedRef<SWidget> SKartMenu::BuildWheel()
{
	FKartWheelInput* W = &FKartWheelInput::Get();
	TSharedRef<SVerticalBox> B = SNew(SVerticalBox);
	FString Devs;
	for (int32 I = 0; I < W->NumDevices(); ++I) { Devs += FString::Printf(TEXT("%s%s"), I ? TEXT(",  ") : TEXT(""), *W->DeviceName(I)); }
	B->AddSlot().AutoHeight().Padding(0, 0, 0, 6)[ Caption(LOCTEXT("DevCap", "DEVICES")) ];
	B->AddSlot().AutoHeight()[ Label(FText::FromString(Devs.IsEmpty() ? TEXT("None found (connect the wheel before starting the game)") : Devs), 14, TextMain) ];
	B->AddSlot().AutoHeight()[ Label(FText::FromString(FString::Printf(TEXT("Force feedback: %s"), W->HasForceFeedback() ? *W->FfbDeviceName() : TEXT("not available"))), 13, TextDim) ];
	B->AddSlot().AutoHeight().Padding(0, 12, 0, 4)
	[
		SNew(SBorder).BorderImage(PillBrush()).Padding(FMargin(16.0f, 10.0f))
		[
			SNew(STextBlock).Font(Font(15, true)).ColorAndOpacity(Accent).Text_Lambda([W]()
			{
				return FText::FromString(FString::Printf(TEXT("LIVE   wheel %+6.1f deg    throttle %3.0f %%    brake %3.0f %%    clutch %3.0f %%"), W->SteeringDeg(),
					W->Pedal(FKartWheelInput::Throttle) * 100.0f, W->Pedal(FKartWheelInput::Brake) * 100.0f, W->Pedal(FKartWheelInput::Clutch) * 100.0f));
			})
		]
	];
	B->AddSlot().AutoHeight().Padding(0, 4)[ Label(TAttribute<FText>::CreateLambda([W]() { return FText::FromString(W->IsDetecting() ? W->DetectionPrompt() : W->StatusLine()); }), 14, Good) ];

	B->AddSlot().AutoHeight().Padding(0, 14, 0, 6)[ Caption(LOCTEXT("DetCap", "DETECT CONTROLS")) ];
	B->AddSlot().AutoHeight().Padding(0, 0, 0, 6)[ Label(LOCTEXT("DetectHelp", "Click a control, then move it within 4 s. Steering: turn RIGHT about a quarter turn from centre."), 12, TextFaint) ];
	TSharedRef<SHorizontalBox> Det = SNew(SHorizontalBox);
	auto DetectAxis = [W, &Det](const TCHAR* Name, FKartWheelInput::EControl C)
	{
		Det->AddSlot().AutoWidth().Padding(0, 0, 6, 0)[ Button(FText::FromString(Name), [W, C]() { W->BeginDetectAxis(C); }, 128.0f) ];
	};
	DetectAxis(TEXT("Steering"), FKartWheelInput::Steering);
	DetectAxis(TEXT("Throttle"), FKartWheelInput::Throttle);
	DetectAxis(TEXT("Brake"), FKartWheelInput::Brake);
	DetectAxis(TEXT("Clutch"), FKartWheelInput::Clutch);
	Det->AddSlot().AutoWidth().Padding(0, 0, 6, 0)[ Button(LOCTEXT("ShUp", "Shift up"), [W]() { W->BeginDetectButton(FKartWheelInput::ShiftUp); }, 128.0f) ];
	Det->AddSlot().AutoWidth()[ Button(LOCTEXT("ShDn", "Shift down"), [W]() { W->BeginDetectButton(FKartWheelInput::ShiftDown); }, 128.0f) ];
	B->AddSlot().AutoHeight()[ Det ];
	B->AddSlot().AutoHeight().Padding(0, 6, 0, 0)[ Button(LOCTEXT("Rescan", "Rescan devices"), [this, W]() { W->Rescan(); Show(EPage::Wheel); }, 220.0f) ];

	B->AddSlot().AutoHeight().Padding(0, 18, 0, 6)[ Caption(LOCTEXT("FfbCap", "STEERING AND FORCE FEEDBACK")) ];
	auto Row = [&B](TSharedRef<SWidget> X) { B->AddSlot().AutoHeight().Padding(0, 5)[ X ]; };
	Row(Cycler(LOCTEXT("Rot", "Wheel rotation (match your wheel software)"), TAttribute<FText>::CreateLambda([W]() { return FText::FromString(FString::Printf(TEXT("%.0f deg"), W->RotationDeg)); }),
	           [W](int32 D) { W->RotationDeg = FMath::Clamp(W->RotationDeg + 90.0f * D, 180.0f, 2520.0f); W->Save(); }));
	Row(Cycler(LOCTEXT("Gain", "Force feedback strength"), TAttribute<FText>::CreateLambda([W]() { return FText::FromString(FString::Printf(TEXT("%.0f %%"), W->FfbGain * 100.0f)); }),
	           [W](int32 D) { W->FfbGain = FMath::Clamp(FMath::RoundToFloat((W->FfbGain + 0.1f * D) * 10.0f) / 10.0f, 0.0f, 1.5f); W->Save(); }));
	Row(Cycler(LOCTEXT("Inv", "Invert force feedback"), TAttribute<FText>::CreateLambda([W]() { return FText::FromString(W->bInvertFfb ? TEXT("On") : TEXT("Off")); }),
	           [W](int32) { W->bInvertFfb = !W->bInvertFfb; W->Save(); }));
	Row(Cycler(LOCTEXT("Soft", "Soft lock strength"), TAttribute<FText>::CreateLambda([W]() { return FText::FromString(FString::Printf(TEXT("%.0f %%"), W->SoftLockStrength * 100.0f)); }),
	           [W](int32 D) { W->SoftLockStrength = FMath::Clamp(FMath::RoundToFloat((W->SoftLockStrength + 0.1f * D) * 10.0f) / 10.0f, 0.0f, 1.0f); W->Save(); }));
	B->AddSlot().AutoHeight().Padding(0, 14, 0, 0)[ Label(LOCTEXT("WheelNote",
		"Steering is 1:1: the wheel angle is the kart's steering-column angle. Karts have a short lock (about 100 deg each way), so the "
		"soft lock stops you there. If the force pulls the wrong way (the wheel runs away from centre), turn on Invert. "
		"Simagic: SimPro Manager game mode / DirectInput, and close other apps that use force feedback."), 12, TextFaint) ];
	return Page(LOCTEXT("WheelTitle", "Wheel & pedals"), B);
}

TSharedRef<SWidget> SKartMenu::BuildControls()
{
	struct FKeyRow { const TCHAR* Action; const TCHAR* Keys; };
	static const FKeyRow Driving[] = {
		{TEXT("Throttle / brake"), TEXT("W / S     pad RT / LT")},
		{TEXT("Steer"), TEXT("A / D     pad left stick")},
		{TEXT("Clutch"), TEXT("Left Shift (manual-clutch karts)")},
		{TEXT("Shift up / down"), TEXT("E / Q (KZ gearbox)")},
		{TEXT("Starter / push start"), TEXT("B / hold P")},
		{TEXT("Carburettor leaner / richer"), TEXT("[ / ]  (one jet size or 1/16 needle turn; watch LAMBDA)")},
		{TEXT("Reset kart (back to the pits: repairs)"), TEXT("R")},
		{TEXT("Camera"), TEXT("C chase / cockpit;  cockpit PgUp/PgDn tilt, Home/End FOV")},
	};
	static const FKeyRow Tools[] = {
		{TEXT("Menu / quick setup"), TEXT("Esc / Tab")},
		{TEXT("Replay / save replay"), TEXT("F5 / F7")},
		{TEXT("Ghost lap"), TEXT("F6")},
		{TEXT("Lap analysis"), TEXT("L")},
		{TEXT("Telemetry / debug / record CSV"), TEXT("T / G / K")},
		{TEXT("Sessions (host)"), TEXT("Y next session,  U AI strength,  N weather,  M rubber")},
		{TEXT("Kart class"), TEXT("1 / 2 / 3")},
		{TEXT("Aids"), TEXT("H auto-shift,  J auto-clutch,  O autopilot")},
	};
	TSharedRef<SVerticalBox> B = SNew(SVerticalBox);
	auto Group = [&B](const FText& Title, const FKeyRow* Rows, int32 N)
	{
		B->AddSlot().AutoHeight().Padding(0, 10, 0, 6)[ Caption(Title) ];
		for (int32 I = 0; I < N; ++I)
		{
			B->AddSlot().AutoHeight().Padding(0, 3)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().FillWidth(0.38f)[ SNew(STextBlock).Text(FText::FromString(Rows[I].Action)).Font(Font(14)).ColorAndOpacity(TextDim) ]
				+ SHorizontalBox::Slot().FillWidth(0.62f)[ SNew(STextBlock).Text(FText::FromString(Rows[I].Keys)).Font(Font(14, true)).ColorAndOpacity(TextMain) ]
			];
		}
	};
	Group(LOCTEXT("DriveCap", "DRIVING"), Driving, UE_ARRAY_COUNT(Driving));
	Group(LOCTEXT("ToolsCap", "TOOLS"), Tools, UE_ARRAY_COUNT(Tools));
	B->AddSlot().AutoHeight().Padding(0, 16, 0, 0)[ Label(LOCTEXT("CtlNote", "Steering wheels, pedals and paddles: Wheel & pedals (DirectInput, with force feedback)."), 12, TextFaint) ];
	return Page(LOCTEXT("CtlTitle", "Controls"), B);
}

#undef LOCTEXT_NAMESPACE
