#include "KartHUD.h"

#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/Font.h"
#include "KartConditionsActor.h"
#include "KartDash.h"
#include "KartRealismSettings.h"
#include "KartGraphicsSettings.h"
#include "KartLicense.h"
#include "Misc/App.h"
#include "KartGameInstance.h"
#include "KartMenu.h"
#include "KartWheelInput.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Widgets/SWindow.h"
#include "Engine/GameViewportClient.h"
#include "Widgets/SWeakWidget.h"
#include "Kismet/GameplayStatics.h"
#include "KartRaceWorld.h"
#include "KartSim/Setup.h"
#include "KartVehiclePawn.h"
#include "Misc/Paths.h"

namespace
{
	const FLinearColor PanelBg(0.0f, 0.0f, 0.0f, 0.55f);
	const FLinearColor Dim(0.65f, 0.65f, 0.65f);
	const FLinearColor Warn(1.0f, 0.75f, 0.1f);
	const FLinearColor Bad(1.0f, 0.25f, 0.2f);
	const FLinearColor Good(0.3f, 1.0f, 0.4f);
}

void AKartHUD::Text(const FString& S, float X, float Y, const FLinearColor& Color, float Scale)
{
	DrawText(S, Color, X, Y, GEngine->GetSmallFont(), Scale);
}

void AKartHUD::DrawBar(float X, float Y, float W, float H, float Fraction, const FLinearColor& Color, const FString& Label)
{
	DrawRect(FLinearColor(0.2f, 0.2f, 0.2f, 0.8f), X, Y, W, H);
	DrawRect(Color, X, Y, W * FMath::Clamp(Fraction, 0.0f, 1.0f), H);
	Text(Label, X + 4.0f, Y + 1.0f);
}

void AKartHUD::BeginPlay()
{
	Super::BeginPlay();
	if (FKartLicense::Get().Blocked()) { ShowLicenseScreen(); return; }
	// Main menu at startup, unless a test/script launched the game with kart flags, the level was
	// started from the menu, or this is a multiplayer client.
	const UKartGameInstance* GI = UKartGameInstance::Get(this);
	if (GI && !GI->bLaunchedFromMenu && !UKartGameInstance::HasScriptedLaunch() && GetNetMode() == NM_Standalone)
	{
		ShowMenu(false);
	}
}

void AKartHUD::ShowLicenseScreen()
{
	if (LicenseHost.IsValid() || !GEngine || !GEngine->GameViewport) { return; }
	TWeakObjectPtr<AKartHUD> Self(this);
	TSharedRef<SWidget> Screen = FKartLicense::MakeActivationScreen([Self]()
	{
		if (!Self.IsValid()) { return; }
		if (GEngine && GEngine->GameViewport && Self->LicenseHost.IsValid()) { GEngine->GameViewport->RemoveViewportWidgetContent(Self->LicenseHost.ToSharedRef()); }
		Self->LicenseHost.Reset();
		Self->LicenseScreen.Reset();
		if (APlayerController* PC = Self->GetOwningPlayerController()) { PC->SetInputMode(FInputModeGameOnly()); PC->bShowMouseCursor = false; }
		if (Self->GetNetMode() == NM_Standalone) { Self->ShowMenu(false); }
	});
	LicenseScreen = Screen;
	LicenseHost = SNew(SWeakWidget).PossiblyNullContent(Screen);
	GEngine->GameViewport->AddViewportWidgetContent(LicenseHost.ToSharedRef(), 200);
	if (APlayerController* PC = GetOwningPlayerController())
	{
		PC->SetInputMode(FInputModeUIOnly()); // the screen itself takes no keyboard focus; clicking the key box does
		PC->bShowMouseCursor = true;
	}
}

void AKartHUD::EndPlay(const EEndPlayReason::Type Reason)
{
	if (LicenseHost.IsValid() && GEngine && GEngine->GameViewport) { GEngine->GameViewport->RemoveViewportWidgetContent(LicenseHost.ToSharedRef()); }
	LicenseHost.Reset();
	LicenseScreen.Reset();
	HideMenu();
	Super::EndPlay(Reason);
}

void AKartHUD::ShowMenu(bool bInGame)
{
	if (Menu.IsValid() || !GEngine || !GEngine->GameViewport) { return; }
	SAssignNew(Menu, SKartMenu).Owner(this).bInGame(bInGame);
	MenuHost = SNew(SWeakWidget).PossiblyNullContent(Menu.ToSharedRef());
	GEngine->GameViewport->AddViewportWidgetContent(MenuHost.ToSharedRef(), 100);
	if (APlayerController* PC = GetOwningPlayerController())
	{
		FInputModeUIOnly Mode;
		Mode.SetWidgetToFocus(Menu);
		PC->SetInputMode(Mode);
		PC->bShowMouseCursor = true;
	}
	if (bInGame && GetNetMode() == NM_Standalone) { UGameplayStatics::SetGamePaused(this, true); }
}

void AKartHUD::HideMenu()
{
	if (!Menu.IsValid()) { return; }
	if (GEngine && GEngine->GameViewport && MenuHost.IsValid()) { GEngine->GameViewport->RemoveViewportWidgetContent(MenuHost.ToSharedRef()); }
	Menu.Reset();
	MenuHost.Reset();
	if (APlayerController* PC = GetOwningPlayerController())
	{
		PC->SetInputMode(FInputModeGameOnly());
		PC->bShowMouseCursor = false;
	}
	if (GetWorld() && UGameplayStatics::IsGamePaused(this)) { UGameplayStatics::SetGamePaused(this, false); }
}

void AKartHUD::DrawHUD()
{
	Super::DrawHUD();
	// Wheel / pedals: polled here because the HUD draws every frame, also while paused (menu detection).
	FKartWheelInput& Wheel = FKartWheelInput::Get();
	// -KartNoWheel: never open the wheel (automated runs must not drive a direct-drive base unattended).
	if (!Wheel.IsInitialized() && !FParse::Param(FCommandLine::Get(), TEXT("KartNoWheel")) && GEngine && GEngine->GameViewport && GEngine->GameViewport->GetWindow().IsValid() &&
	    GEngine->GameViewport->GetWindow()->GetNativeWindow().IsValid())
	{
		Wheel.Initialize(GEngine->GameViewport->GetWindow()->GetNativeWindow()->GetOSWindowHandle());
	}
	Wheel.Poll();
	if (Wheel.IsDetecting()) { Wheel.UpdateDetection(); }
	if (FKartGraphicsSettings::Get().bShowFps && Canvas)
	{
		static double Avg = 1.0 / 60.0;
		const double Dt = FApp::GetDeltaTime();
		if (Dt > 0.0) { Avg += (Dt - Avg) * 0.05; }
		Text(FString::Printf(TEXT("%.0f fps  %.1f ms  [%s]"), 1.0 / FMath::Max(Avg, 1e-4), Avg * 1000.0, FKartGraphicsSettings::PresetName(FKartGraphicsSettings::Get().Preset)),
		     Canvas->ClipX - 300.0f, 8.0f, FLinearColor(0.85f, 0.95f, 0.85f), 1.0f);
	}
	// A license revoked (or past the offline grace) during the session: back to the activation screen.
	if (!LicenseHost.IsValid() && FKartLicense::Get().Blocked()) { HideMenu(); ShowLicenseScreen(); return; }
	if (LicenseHost.IsValid()) { return; }
	if (Canvas)
	{
		// Tester watermark: name + machine on every frame (screenshots / streams of a leaked build name the leaker).
		const FString Wm = FKartLicense::Get().Watermark();
		if (!Wm.IsEmpty())
		{
			float Tw = 0.0f, Th = 0.0f;
			Canvas->StrLen(GEngine->GetSmallFont(), Wm, Tw, Th);
			Text(Wm, 0.5f * (Canvas->ClipX - Tw * 0.85f), Canvas->ClipY - 22.0f, FLinearColor(1.0f, 1.0f, 1.0f, 0.45f), 0.85f); // bottom centre
		}
	}
	if (Menu.IsValid()) { return; }
	if (APlayerController* PC = GetOwningPlayerController(); PC && PC->WasInputKeyJustPressed(EKeys::Escape)) { ShowMenu(true); return; }
	const AKartVehiclePawn* Kart = Cast<AKartVehiclePawn>(GetOwningPawn());
	if (!Kart || !Canvas)
	{
		return;
	}
	if (!Kart->GetSim())
	{
		DrawRect(PanelBg, 20, 20, 900, 120);
		Text(Kart->GetStatusMessage().IsEmpty() ? FString(TEXT("Kart simulation not initialised")) : Kart->GetStatusMessage(), 30, 30, Bad, 1.2f);
		return;
	}

	if (const AKartRaceWorld* RW = Kart->GetRaceWorld())
	{
		if (!RW->GetReplayMessage().IsEmpty())
		{
			DrawRect(PanelBg, 20.0f, Canvas->ClipY * 0.5f - 20.0f, Canvas->ClipX - 40.0f, 34.0f);
			Text(RW->GetReplayMessage(), 32.0f, Canvas->ClipY * 0.5f - 14.0f, Warn, 1.1f);
		}
		if (RW->IsReplaying()) { DrawReplay(*RW); return; }
	}
	if (Kart->IsAnalysisOpen()) { DrawAnalysis(*Kart); return; }

	const kartsim::TelemetryFrame& F = Kart->GetSim()->GetTelemetry();
	// Always-on minimal readout.
	const float CX = Canvas->ClipX;
	// Mychron overlay (chase camera; the cockpit view shows the screen on the steering wheel). Settings page toggle.
	if (!Kart->IsCockpitCamera() && FKartRealismSettings::Get().bDashOverlay && !Kart->IsSetupOpen())
	{
		const float DH = FMath::Clamp(Canvas->ClipY * 0.24f, 150.0f, 300.0f);
		UKartDashComponent::DrawMychron(Canvas, Canvas->ClipX - 2.0f * DH - 24.0f, Canvas->ClipY - DH - 24.0f, 2.0f * DH, DH, *Kart);
	}
	// Damage / failure banner (4 s after each new event) and what to do about a dead kart.
	{
		const kartsim::KartSimulation& S = *Kart->GetSim();
		const kartsim::KartDamageState& D = S.GetDamage();
		FString Msg;
		if (S.GetEngineCondition().IsSeized()) { Msg = TEXT("ENGINE SEIZED  -  R: back to the pits (repairs the kart)"); }
		else if (D.bChainOff) { Msg = TEXT("CHAIN OFF  -  R: back to the pits (repairs the kart)"); }
		else if (D.LastEventTime >= 0.0 && S.GetTime() - D.LastEventTime < 4.0) { Msg = FString(UTF8_TO_TCHAR(D.LastEvent.c_str())).ToUpper(); }
		if (!Msg.IsEmpty())
		{
			DrawRect(FLinearColor(0.35f, 0.02f, 0.02f, 0.8f), CX * 0.5f - 300.0f, 96.0f, 600.0f, 36.0f);
			Text(Msg, CX * 0.5f - 288.0f, 102.0f, FLinearColor::White, 1.3f);
		}
	}
	DrawRect(PanelBg, CX * 0.5f - 130.0f, Canvas->ClipY - 70.0f, 260.0f, 56.0f);
	Text(FString::Printf(TEXT("%5.1f km/h"), F.SpeedKmh), CX * 0.5f - 120.0f, Canvas->ClipY - 66.0f, FLinearColor::White, 1.8f);
	const int32 GearNum = static_cast<int32>(F.Gear);
	const FString GearStr = GearNum == 0 ? FString(TEXT("N")) : FString::FromInt(GearNum);
	Text(FString::Printf(TEXT("%5.0f rpm   %s"), F.EngineRpm, *GearStr), CX * 0.5f - 120.0f, Canvas->ClipY - 36.0f,
	     F.RevLimiter > 0.5 ? Bad : FLinearColor::White, 1.3f);
	if (F.EngineRunning < 0.5 && !Kart->GetSim()->GetEngineCondition().IsSeized())
	{
		const bool bStarter = Kart->GetSim()->GetEngine().HasStarter();
		const FString Hint = bStarter ? FString(TEXT("ENGINE STOPPED - hold B (starter)"))
		                              : FString(TEXT("ENGINE STOPPED - hold P (push start), or R (reset)"));
		DrawRect(PanelBg, CX * 0.5f - 230.0f, Canvas->ClipY - 112.0f, 460.0f, 34.0f);
		Text(Kart->IsPushing() ? FString(TEXT("PUSHING...")) : Hint, CX * 0.5f - 220.0f, Canvas->ClipY - 106.0f, Warn, 1.4f);
	}

	// --- Lap timing (always on) ---
	if (Kart->HasTimingLines())
	{
		const kartsim::LapTimer& LT = Kart->GetLapTimer();
		auto Fmt = [](double T) { const int32 Min = static_cast<int32>(T / 60.0); return FString::Printf(TEXT("%d:%06.3f"), Min, T - 60.0 * Min); };
		const float TX = CX * 0.5f - 210.0f, TY = 12.0f;
		DrawRect(PanelBg, TX, TY, 420.0f, 74.0f);
		if (Kart->IsAutopilot()) { Text(TEXT("AUTOPILOT"), TX + 330.0f, TY + 52.0f, Warn); }
		const double Now = Kart->GetClock();
		if (!LT.IsTiming())
		{
			Text(TEXT("Cross the start/finish line to start timing"), TX + 12.0f, TY + 8.0f, Dim, 1.1f);
		}
		else
		{
			const bool bValid = LT.IsCurrentLapValid();
			Text(FString::Printf(TEXT("LAP %d   %s"), static_cast<int32>(LT.Laps().size()) + 1, *Fmt(LT.CurrentLapTime(Now))), TX + 12.0f, TY + 6.0f,
			     bValid ? FLinearColor::White : Bad, 1.4f);
			if (!bValid) { Text(TEXT("INVALID (4 wheels off)"), TX + 250.0f, TY + 10.0f, Bad, 1.0f); }
			// Completed sectors of the current lap: green = session best, yellow = slower.
			const kartsim::LapRecord& Cur = LT.CurrentSplits();
			for (int32 S = 0; S < LT.NumSectors(); ++S)
			{
				const bool bDone = S < LT.CurrentSector();
				const double Best = LT.BestSector(S);
				const FLinearColor Col = !bDone ? Dim : ((Best == 0.0 || Cur.Sectors[S] <= Best) ? Good : Warn);
				Text(bDone ? FString::Printf(TEXT("S%d %.3f"), S + 1, Cur.Sectors[S]) : FString::Printf(TEXT("S%d  --"), S + 1), TX + 12.0f + S * 100.0f, TY + 32.0f, Col);
			}
		}
		const auto& Laps = LT.Laps();
		const FString Last = Laps.empty() ? FString(TEXT("--")) : (Fmt(Laps.back().LapTime) + (Laps.back().bValid ? TEXT("") : TEXT(" (inv)")));
		const FString BestStr = LT.BestLapIndex() >= 0 ? Fmt(Laps[LT.BestLapIndex()].LapTime) : FString(TEXT("--"));
		Text(FString::Printf(TEXT("LAST %s    BEST %s"), *Last, *BestStr), TX + 12.0f, TY + 52.0f, FLinearColor::White);
		const FString GhostStr = Kart->HasGhost() ? FString::Printf(TEXT("ghost %s %s"), *Fmt(Kart->GetGhostTime()), Kart->IsGhostEnabled() ? TEXT("on") : TEXT("off"))
		                                          : FString(TEXT("ghost: set a valid lap"));
		Text(FString::Printf(TEXT("%s (F6)    F5 replay   F7 save replay   L lap analysis"), *GhostStr), TX + 4.0f, TY + 78.0f, Dim, 0.9f);
	}

	DrawSession(*Kart, CX * 0.5f + 220.0f, 12.0f);
	DrawConditions(*Kart, 20.0f, Canvas->ClipY - 150.0f);

	if (Kart->IsSetupOpen())
	{
		int32 Count = 0;
		const kartsim::SetupParam* Params = kartsim::GetSetupParams(Count);
		const float PX = CX * 0.5f - 330.0f, PY = 90.0f, RowH = 26.0f;
		DrawRect(FLinearColor(0.0f, 0.0f, 0.0f, 0.82f), PX, PY, 660.0f, 110.0f + Count * RowH);
		Text(FString::Printf(TEXT("KART SETUP - %s"), UTF8_TO_TCHAR(Kart->GetEditConfig().Name.c_str())), PX + 16.0f, PY + 12.0f, Warn, 1.4f);
		Text(TEXT("parameter"), PX + 16.0f, PY + 44.0f, Dim);
		Text(TEXT("value"), PX + 300.0f, PY + 44.0f, Dim);
		Text(TEXT("legal / typical range"), PX + 470.0f, PY + 44.0f, Dim);
		for (int32 I = 0; I < Count; ++I)
		{
			const kartsim::SetupParam& P = Params[I];
			const float Y = PY + 66.0f + I * RowH;
			const bool bSel = I == Kart->GetSetupCursor();
			if (bSel) { DrawRect(FLinearColor(0.2f, 0.4f, 0.9f, 0.45f), PX + 8.0f, Y - 2.0f, 644.0f, RowH - 2.0f); }
			if (!P.IsAvailable(Kart->GetEditConfig()))
			{
				Text(FString::Printf(TEXT("%s   n/a for this class"), UTF8_TO_TCHAR(P.Label)), PX + 16.0f, Y, Dim, 1.0f);
				continue;
			}
			const double V = P.Get(Kart->GetEditConfig());
			const double Live = P.Get(Kart->GetSim()->GetConfig());
			const FString ValStr = P.Step >= 1.0 ? FString::Printf(TEXT("%.0f %s"), V, UTF8_TO_TCHAR(P.Unit)) : FString::Printf(TEXT("%.2f %s"), V, UTF8_TO_TCHAR(P.Unit));
			Text(FString::Printf(TEXT("%s"), UTF8_TO_TCHAR(P.Label)), PX + 16.0f, Y, bSel ? FLinearColor::White : Dim, 1.15f);
			Text(ValStr + (FMath::Abs(V - Live) > 1e-9 ? TEXT("  *") : TEXT("")), PX + 300.0f, Y, bSel ? Good : FLinearColor::White, 1.15f);
			Text(FString::Printf(TEXT("%g - %g"), P.Min, P.Max), PX + 470.0f, Y, Dim, 1.0f);
		}
		Text(Kart->GetSetupMessage(), PX + 16.0f, PY + 76.0f + Count * RowH, Dim);
		Text(TEXT("* = changed, not yet applied"), PX + 16.0f, PY + 92.0f + Count * RowH, Dim);
		return;
	}
	if (!Kart->bShowTelemetry)
	{
		Text(TEXT("T: telemetry"), 20, 20, Dim);
		return;
	}
	DrawPanel(*Kart, 20.0f, 20.0f);
	DrawWheelGrid(*Kart, 20.0f, 380.0f);
	DrawGG(*Kart, CX - 130.0f, 140.0f, 100.0f);
	DrawSpeedTrace(*Kart, CX - 520.0f, Canvas->ClipY - 250.0f, 500.0f, 160.0f);
}

void AKartHUD::DrawSession(const AKartVehiclePawn& Kart, float X, float Y)
{
	const kartsim::LapDelta& D = Kart.GetLapAnalysis().GetDelta();
	const double Now = Kart.GetClock();
	auto Fmt = [](double T) { const int32 Min = static_cast<int32>(T / 60.0); return FString::Printf(TEXT("%d:%06.3f"), Min, T - 60.0 * Min); };
	DrawRect(PanelBg, X, Y, 300.0f, 74.0f);
	// Live delta to the best lap (green = gaining).
	if (D.HasDelta())
	{
		const double Dt = D.Delta();
		Text(FString::Printf(TEXT("%+.2f"), Dt), X + 10.0f, Y + 4.0f, FMath::Abs(Dt) < 0.005 ? FLinearColor::White : (Dt < 0.0 ? Good : Bad), 1.6f);
		Text(FString::Printf(TEXT("pred %s"), *Fmt(D.Predicted())), X + 110.0f, Y + 10.0f, Dim);
	}
	else
	{
		Text(D.HasReference() ? TEXT("delta: next lap") : TEXT("delta: set a clean lap"), X + 10.0f, Y + 8.0f, Dim);
	}
	const AKartRaceWorld* RW = AKartRaceWorld::Find(GetWorld());
	if (!RW || !RW->GetSession().bActive)
	{
		Text(GetNetMode() == NM_Client ? TEXT("Waiting for the host's session") : TEXT("Y: start session (practice / qualifying / race)"), X + 10.0f, Y + 50.0f, Dim);
		return;
	}
	const FKartNetSession& Ses = RW->GetSession();
	const TArray<FKartNetStanding>& Rows = RW->GetStandings();
	const kartsim::ESessionType Type = static_cast<kartsim::ESessionType>(Ses.Type);
	const bool bRace = Type == kartsim::ESessionType::Race;
	int32 MyPos = 0;
	const FKartNetStanding* Me = nullptr;
	for (int32 I = 0; I < Rows.Num(); ++I) { if (Rows[I].Kart == &Kart) { MyPos = I + 1; Me = &Rows[I]; } }
	FString Line;
	FLinearColor Col = FLinearColor::White;
	switch (static_cast<kartsim::ESessionPhase>(Ses.Phase))
	{
	case kartsim::ESessionPhase::Grid:
		Line = Ses.bRolling ? FString(TEXT("FORMATION LAP - green at the line")) : FString::Printf(TEXT("RACE START  lights %.1f s"), Ses.Lights);
		Col = Ses.bRolling ? Warn : Bad;
		break;
	case kartsim::ESessionPhase::Running:
		if (bRace) { Line = FString::Printf(TEXT("RACE  P%d  lap %d / %d   %s"), MyPos, FMath::Min((Me ? Me->Laps : 0) + 1, Ses.RaceLaps), Ses.RaceLaps, *Fmt(Ses.Elapsed)); }
		else if (Ses.Remaining > 0.0f) { Line = FString::Printf(TEXT("%s  P%d  %s left"), UTF8_TO_TCHAR(kartsim::RaceDirector::TypeName(Type)), MyPos, *Fmt(Ses.Remaining)); }
		else { Line = FString::Printf(TEXT("PRACTICE  P%d  %s"), MyPos, *Fmt(Ses.Elapsed)); }
		break;
	case kartsim::ESessionPhase::Chequered:
		Line = (Me && Me->bFinished) ? FString::Printf(TEXT("FINISHED P%d - others finishing"), MyPos) : FString(TEXT("CHEQUERED FLAG - finish this lap"));
		Col = Warn;
		break;
	case kartsim::ESessionPhase::Finished:
		Line = FString::Printf(TEXT("FINISHED P%d   (host: Y next session)"), MyPos);
		Col = Good;
		break;
	}
	Text(Line, X + 10.0f, Y + 32.0f, Col, 1.1f);
	Text(FString::Printf(TEXT("laps %d  best %s  inv %d   AI %.0f%% (U)"), Me ? Me->Laps : 0, (Me && Me->BestLap > 0.0f) ? *Fmt(Me->BestLap) : TEXT("--"),
	                     Me ? Me->InvalidLaps : 0, Ses.AIStrength * 100.0f), X + 10.0f, Y + 54.0f, Dim);

	// Timing tower (right edge, below the g-g plot when telemetry is shown).
	const float TX = Canvas->ClipX - 300.0f, TY = Kart.bShowTelemetry ? 330.0f : 100.0f, Row = 20.0f;
	DrawRect(PanelBg, TX, TY, 290.0f, 26.0f + Row * Rows.Num());
	Text(FString::Printf(TEXT("%s   contacts %d/s%s"), UTF8_TO_TCHAR(kartsim::RaceDirector::TypeName(Type)), Ses.ContactsPerSecond,
	                     GetNetMode() == NM_Client ? TEXT("   [online]") : (GetNetMode() == NM_ListenServer ? TEXT("   [host]") : TEXT(""))), TX + 8.0f, TY + 4.0f, Warn);
	for (int32 P = 0; P < Rows.Num(); ++P)
	{
		const FKartNetStanding& R = Rows[P];
		const AKartVehiclePawn* K = R.Kart.Get();
		if (!K) { continue; }
		FString Info;
		if (bRace)
		{
			if (P == 0) { Info = R.bFinished ? Fmt(R.FinishTime) : FString::Printf(TEXT("lap %d"), FMath::Min(R.Laps + 1, Ses.RaceLaps)); }
			else if (R.LapsDown > 0) { Info = FString::Printf(TEXT("+%d lap%s"), R.LapsDown, R.LapsDown > 1 ? TEXT("s") : TEXT("")); }
			else { Info = FString::Printf(TEXT("+%.1f"), R.Gap); }
		}
		else
		{
			Info = R.BestLap > 0.0f ? (P == 0 ? Fmt(R.BestLap) : FString::Printf(TEXT("+%.3f"), R.Gap)) : FString(TEXT("--"));
		}
		const bool bMe = K == &Kart;
		const float RY = TY + 24.0f + Row * P;
		if (bMe) { DrawRect(FLinearColor(0.2f, 0.4f, 0.9f, 0.45f), TX + 2.0f, RY - 1.0f, 286.0f, Row - 1.0f); }
		DrawRect(K->BodyColor, TX + 6.0f, RY + 3.0f, 6.0f, 12.0f);
		Text(FString::Printf(TEXT("%2d  %s"), P + 1, bMe ? TEXT("You") : *K->DriverName), TX + 18.0f, RY, bMe ? FLinearColor::White : FLinearColor(0.85f, 0.85f, 0.85f));
		Text(Info, TX + 150.0f, RY, R.bFinished ? Good : FLinearColor::White);
		Text(AKartRaceWorld::AiStateName(R.AiState), TX + 225.0f, RY, Dim, 0.8f);
	}
}

void AKartHUD::DrawConditions(const AKartVehiclePawn& Kart, float X, float Y)
{
	const AKartConditionsActor* CA = Kart.GetConditions();
	DrawRect(PanelBg, X, Y, 400.0f, 130.0f);
	{
		// Tyres: fitted set, then per wheel temperature (colour vs the compound's window) and tread left.
		const kartsim::KartSimulation& S = *Kart.GetSim();
		const kartsim::KartConfig& Cfg = S.GetConfig();
		Text(FString::Printf(TEXT("Tyres: %s"), UTF8_TO_TCHAR(Cfg.CompoundName[Cfg.Compound].c_str())), X + 8.0f, Y + 90.0f, Warn);
		static const TCHAR* Names[] = {TEXT("FL"), TEXT("FR"), TEXT("RL"), TEXT("RR")};
		for (int32 I = 0; I < kartsim::NumWheels; ++I)
		{
			const kartsim::TyreModel& T = S.GetTyreModel(I);
			const kartsim::TyreParams& P = I < 2 ? Cfg.FrontTyre : Cfg.RearTyre;
			const double Dt = T.GetTemperature() - P.OptimumTemp;
			const FLinearColor C = Dt < -0.6 * P.TempWindow ? FLinearColor(0.4f, 0.7f, 1.0f) : (Dt > 0.6 * P.TempWindow ? Bad : Good);
			Text(FString::Printf(TEXT("%s %3.0fC %3.1fmm"), Names[I], T.GetTemperature(), T.GetTreadRemaining()), X + 8.0f + I * 96.0f, Y + 108.0f,
			     T.GetWear() > 0.8 ? Bad : C);
		}
	}
	if (!CA || !CA->IsActive())
	{
		Text(TEXT("Track conditions off"), X + 8.0f, Y + 6.0f, Dim);
		return;
	}
	const kartsim::TrackConditions& C = CA->Get();
	const kartsim::WeatherState& W = C.Weather;
	Text(FString::Printf(TEXT("%s, %s track   (N weather, M rubber)"), *CA->GetWeatherName(), *CA->GetTrackName()), X + 8.0f, Y + 6.0f, Warn);
	Text(FString::Printf(TEXT("air %.0f C  %.0f hPa  %.0f%% RH  track %.1f C  cloud %.0f%%  rain %.1f mm/h"), W.AirTemp, W.PressureKPa * 10.0, W.Humidity * 100.0,
	                     W.TrackTemp, W.CloudCover * 100.0, W.RainRate), X + 8.0f, Y + 26.0f);
	// Under the kart: mean over the wheels in contact.
	const kartsim::KartSimulation& Sim = *Kart.GetSim();
	double Rubber = 0.0, Water = 0.0, Grip = 0.0;
	const double Nw = static_cast<double>(kartsim::NumWheels);
	for (int32 I = 0; I < kartsim::NumWheels; ++I)
	{
		double R = 0.0, Wt = 0.0;
		// Road factor x the fitted compound's wet loss = what the tyre actually gets.
		const kartsim::TyreModel& Tm = Sim.GetTyreModel(I);
		double Wn = 0.0;
		const double Road = C.GripAt(Sim.GetWheel(I).Contact.ContactPoint, &R, &Wt, &Wn); // sets Wn: must run first
		Grip += Road * Tm.WetGripFactor(Wn) / Nw;
		Rubber += R / Nw;
		Water += Wt / Nw;
	}
	Text(FString::Printf(TEXT("under kart: rubber %.0f%%  water %.2f mm  grip x%.3f"), Rubber * 100.0, Water, Grip), X + 8.0f, Y + 46.0f,
	     Grip < 0.9 ? Bad : (Grip > 1.01 ? Good : FLinearColor::White));
	Text(FString::Printf(TEXT("track mean water %.2f mm, line rubber %.0f%%"), C.MeanWater(), C.MeanRubberOnLine() * 100.0), X + 8.0f, Y + 66.0f, Dim);
}

void AKartHUD::DrawSpeedTrace(const AKartVehiclePawn& Kart, float X, float Y, float W, float H)
{
	// Speed vs distance: reference (best) lap grey, current lap yellow.
	const kartsim::LapDelta& D = Kart.GetLapAnalysis().GetDelta();
	const size_t N = D.SamplesPerLap();
	DrawRect(PanelBg, X - 6.0f, Y - 22.0f, W + 12.0f, H + 30.0f);
	Text(TEXT("speed vs distance   grey: best lap   yellow: this lap"), X, Y - 18.0f, Dim);
	if (N < 2) { return; }
	const float VMax = 140.0f;
	auto Plot = [&](const std::vector<double>& V, const FLinearColor& C)
	{
		for (size_t I = 1; I < V.size() && I < N; ++I)
		{
			const float X0 = X + W * (I - 1) / (N - 1), X1 = X + W * I / (N - 1);
			DrawLine(X0, Y + H - H * FMath::Min(static_cast<float>(V[I - 1]) / VMax, 1.0f), X1, Y + H - H * FMath::Min(static_cast<float>(V[I]) / VMax, 1.0f), C, 1.5f);
		}
	};
	for (float Kmh = 20.0f; Kmh < VMax; Kmh += 20.0f)
	{
		DrawLine(X, Y + H - H * Kmh / VMax, X + W, Y + H - H * Kmh / VMax, FLinearColor(0.25f, 0.25f, 0.25f));
	}
	Plot(D.ReferenceLap().Speed, FLinearColor(0.7f, 0.7f, 0.7f));
	Plot(D.CurrentLap().Speed, FLinearColor::Yellow);
	Text(TEXT("0"), X - 4.0f, Y + H - 12.0f, Dim);
	Text(FString::Printf(TEXT("%.0f km/h"), VMax), X, Y - 4.0f, Dim);
}

void AKartHUD::DrawPanel(const AKartVehiclePawn& Kart, float X, float Y)
{
	const kartsim::KartSimulation& Sim = *Kart.GetSim();
	const kartsim::TelemetryFrame& F = Sim.GetTelemetry();
	DrawRect(PanelBg, X, Y, 560.0f, 350.0f);
	float L = Y + 6.0f;
	const float Dy = 16.0f;
	auto Line = [&](const FString& S, const FLinearColor& C = FLinearColor::White) { Text(S, X + 8.0f, L, C); L += Dy; };

	Line(FString::Printf(TEXT("%s"), UTF8_TO_TCHAR(Sim.GetConfig().Name.c_str())), Warn);
	Line(FString::Printf(TEXT("Speed %6.1f km/h   long %6.2f  lat %6.2f m/s   sideslip %5.1f deg"), F.SpeedKmh, F.VelLong, F.VelLat, F.SideslipDeg));
	Line(FString::Printf(TEXT("Engine %6.0f rpm  %5.1f Nm  thr eff %3.0f%%  %s"), F.EngineRpm, F.EngineTorque, F.ThrottleEff * 100.0,
	                     F.RevLimiter > 0.5 ? TEXT("LIMITER") : TEXT("")));
	const bool bDirect = Sim.GetConfig().Clutch.Type == kartsim::EClutchType::DirectDrive;
	const TCHAR* ClutchState = bDirect ? TEXT("DIRECT") : (F.Gear < 0.5 ? TEXT("NEUTRAL") : (F.ClutchLocked > 0.5 ? TEXT("LOCKED") : TEXT("SLIP  ")));
	Line(FString::Printf(TEXT("Clutch %s  slip %6.0f rpm  torque %5.1f Nm   gear %d   axle %5.0f rpm"),
	                     ClutchState, F.ClutchSlipRpm, F.ClutchTorque,
	                     static_cast<int32>(F.Gear), F.RearAxleRpm),
	     F.ClutchLocked > 0.5 ? FLinearColor::White : Warn);
	Line(FString::Printf(TEXT("Accel  long %5.2f g  lat %5.2f g  vert %5.2f g"), F.AccLongG, F.AccLatG, F.AccVertG));
	Line(FString::Printf(TEXT("Chassis roll %5.2f  pitch %5.2f  twist %5.3f deg  yaw rate %6.1f deg/s"),
	                     F.RollDeg, F.PitchDeg, F.ChassisTwistDeg, F.YawRateDeg));
	Line(FString::Printf(TEXT("Steer column %6.1f deg  wheels L %5.1f R %5.1f deg"), F.SteerColumnDeg, F.SteerLeftDeg, F.SteerRightDeg));
	Line(FString::Printf(TEXT("FFB column %6.2f Nm  out %5.2f  clip %3.0f%%  dev: %s"), F.ColumnTorque, F.FfbOutput, F.FfbClipping * 100.0,
	                     Kart.GetFFBDevice() ? *Kart.GetFFBDevice()->GetName() : TEXT("-")));
	Line(FString::Printf(TEXT("Brake disc %4.0f C  torque %5.0f Nm"), F.BrakeDiscTemp, F.BrakeTorque));
	const AKartVehiclePawn::FInputDiagnostics& In = Kart.GetInputDiagnostics();
	Line(FString::Printf(TEXT("Input key thr %.2f brk %.2f str %+.2f | pad thr %.2f%s brk %.2f%s str %+.2f%s"),
	                     In.KeyThrottle, In.KeyBrake, In.KeySteer,
	                     In.AxisThrottle, In.bThrottleArmed ? TEXT("") : TEXT("(ignored)"),
	                     In.AxisBrake, In.bBrakeArmed ? TEXT("") : TEXT("(ignored)"),
	                     In.AxisSteer, In.bSteerArmed ? TEXT("") : TEXT("(ignored)")), Dim);
	Line(FString::Printf(TEXT("Key/pad steering lock available: %3.0f%%  (wheels are never limited)"), In.SteerLockFraction * 100.0f), Dim);
	Line(FString::Printf(TEXT("Physics %4.0f Hz (x%d sub)  %d steps/frame  %.3f ms/frame"), Kart.GetMeasuredPhysicsHz(),
	                     Kart.Substeps, Kart.GetStepsLastFrame(), Kart.GetPhysicsMsPerFrame()), Dim);
	if (Kart.IsRecording())
	{
		Line(FString::Printf(TEXT("REC %s"), *FPaths::GetCleanFilename(Kart.GetRecordingPath())), Bad);
	}
	else
	{
		Line(TEXT("K: record CSV   G: debug draw   C: camera   R: reset   T: hide"), Dim);
		Line(FString::Printf(TEXT("O autopilot   Tab setup   1/2/3: Rotax / KZ2 / KT100   E/Q shift   Shift clutch   H auto-shift [%s]   J auto-clutch [%s]"),
		                     Kart.bAutoShift ? TEXT("ON") : TEXT("off"), Kart.bAutoClutch ? TEXT("ON") : TEXT("off")), Dim);
	}

	const float BY = L + 6.0f;
	DrawBar(X + 8.0f, BY, 130.0f, 16.0f, static_cast<float>(F.Throttle), FLinearColor(0.1f, 0.7f, 0.2f), TEXT("Throttle"));
	DrawBar(X + 146.0f, BY, 130.0f, 16.0f, static_cast<float>(F.Brake), FLinearColor(0.8f, 0.1f, 0.1f), TEXT("Brake"));
	// Steering: centred bar.
	const float SX = X + 284.0f, SW = 136.0f;
	DrawRect(FLinearColor(0.2f, 0.2f, 0.2f, 0.8f), SX, BY, SW, 16.0f);
	const double MaxCol = Sim.GetConfig().Steering.MaxColumnAngle * kartsim::kRadToDeg;
	const float S = static_cast<float>(FMath::Clamp(-F.SteerColumnDeg / MaxCol, -1.0, 1.0)); // right on screen = steering right
	DrawRect(FLinearColor(0.2f, 0.5f, 1.0f), SX + SW * 0.5f + FMath::Min(0.0f, S * SW * 0.5f), BY, FMath::Abs(S) * SW * 0.5f, 16.0f);
	Text(TEXT("Steer"), SX + 4.0f, BY + 1.0f);
}

void AKartHUD::DrawWheelGrid(const AKartVehiclePawn& Kart, float X, float Y)
{
	const kartsim::KartSimulation& Sim = *Kart.GetSim();
	static const TCHAR* Names[] = {TEXT("FL"), TEXT("FR"), TEXT("RL"), TEXT("RR")};
	const float BoxW = 210.0f, BoxH = 120.0f;
	for (int32 I = 0; I < kartsim::NumWheels; ++I)
	{
		const kartsim::WheelState& W = Sim.GetWheel(I);
		const int32 Col = (I % 2 == 0) ? 0 : 1; // left wheels on the left
		const int32 Row = I / 2;
		const float BX = X + Col * (BoxW + 10.0f);
		const float BY = Y + Row * (BoxH + 10.0f);
		const float Lam = static_cast<float>(FMath::Clamp(W.Tyre.CombinedSlip, 0.0, 1.2));
		const FLinearColor Edge = W.bLifted ? FLinearColor(0.8f, 0.2f, 0.9f) : FLinearColor::LerpUsingHSV(Good, Bad, FMath::Min(Lam, 1.0f));
		DrawRect(PanelBg, BX, BY, BoxW, BoxH);
		DrawRect(Edge, BX, BY, 4.0f, BoxH);
		float L = BY + 4.0f;
		auto Line = [&](const FString& S, const FLinearColor& C = FLinearColor::White) { Text(S, BX + 10.0f, L, C); L += 15.0f; };
		Line(FString::Printf(TEXT("%s  %s  [%s]"), Names[I], W.bLifted ? TEXT("LIFTED") : TEXT(""),
		                     UTF8_TO_TCHAR(kartsim::SurfaceName(W.Contact.Surface))), W.bLifted ? FLinearColor(0.9f, 0.4f, 1.0f) : Warn);
		Line(FString::Printf(TEXT("Fz %6.0f N"), W.Tyre.Fz));
		Line(FString::Printf(TEXT("Fx %6.0f  Fy %6.0f N"), W.Tyre.Fx, W.Tyre.Fy));
		Line(FString::Printf(TEXT("slip %6.3f  angle %5.1f deg"), W.Tyre.SlipRatio, W.Tyre.SlipAngle * kartsim::kRadToDeg));
		Line(FString::Printf(TEXT("mu %4.2f  lambda %4.2f"), W.Tyre.Mu, W.Tyre.CombinedSlip), Lam >= 1.0f ? Bad : FLinearColor::White);
		const kartsim::TelemetryFrame& F = Sim.GetTelemetry();
		const double Temps[4] = {F.TempFL, F.TempFR, F.TempRL, F.TempRR};
		const double Wear[4] = {F.WearFL, F.WearFR, F.WearRL, F.WearRR};
		Line(FString::Printf(TEXT("T %5.1f C  wear %5.2f%%  %4.2f mm"), Temps[I], Wear[I] * 100.0, Sim.GetTyreModel(I).GetTreadRemaining()));
		Line(FString::Printf(TEXT("omega %6.1f rad/s"), W.Omega), Dim);
	}
}

void AKartHUD::DrawGG(const AKartVehiclePawn& Kart, float CX, float CY, float Radius)
{
	const kartsim::TelemetryFrame& F = Kart.GetSim()->GetTelemetry();
	const float Scale = Radius / 2.5f; // 2.5 g at the rim
	DrawRect(PanelBg, CX - Radius - 10.0f, CY - Radius - 10.0f, 2.0f * Radius + 20.0f, 2.0f * Radius + 40.0f);
	for (float G = 1.0f; G <= 2.5f; G += 1.0f)
	{
		const int32 N = 48;
		for (int32 I = 0; I < N; ++I)
		{
			const float A0 = 2.0f * PI * I / N, A1 = 2.0f * PI * (I + 1) / N;
			DrawLine(CX + FMath::Cos(A0) * G * Scale, CY + FMath::Sin(A0) * G * Scale,
			         CX + FMath::Cos(A1) * G * Scale, CY + FMath::Sin(A1) * G * Scale, FLinearColor(0.4f, 0.4f, 0.4f));
		}
	}
	DrawLine(CX - Radius, CY, CX + Radius, CY, FLinearColor(0.3f, 0.3f, 0.3f));
	DrawLine(CX, CY - Radius, CX, CY + Radius, FLinearColor(0.3f, 0.3f, 0.3f));

	// Screen: right = force to the right (sim lateral is +left), up = acceleration.
	const FVector2D P(CX - static_cast<float>(F.AccLatG) * Scale, CY - static_cast<float>(F.AccLongG) * Scale);
	const int32 HistoryLen = 240;
	if (GGHistory.Num() < HistoryLen) { GGHistory.Add(P); }
	else { GGHistory[GGHead] = P; GGHead = (GGHead + 1) % HistoryLen; }
	for (const FVector2D& H : GGHistory)
	{
		DrawRect(FLinearColor(0.2f, 0.6f, 1.0f, 0.5f), H.X - 1.0f, H.Y - 1.0f, 2.0f, 2.0f);
	}
	DrawRect(FLinearColor::Yellow, P.X - 3.0f, P.Y - 3.0f, 6.0f, 6.0f);
	Text(FString::Printf(TEXT("g-g  lat %4.2f  long %4.2f"), F.AccLatG, F.AccLongG), CX - Radius, CY + Radius + 12.0f);
}


// ------------------------------------------------------------------------------- replay overlay

void AKartHUD::DrawReplay(const AKartRaceWorld& World)
{
	const FKartReplay& R = World.GetReplay();
	const float W = FMath::Min(Canvas->ClipX - 40.0f, 900.0f);
	const float X = (Canvas->ClipX - W) * 0.5f, Y = Canvas->ClipY - 118.0f;
	DrawRect(FLinearColor(0.0f, 0.0f, 0.0f, 0.7f), X, Y, W, 104.0f);
	auto Fmt = [](double T) { const int32 Min = static_cast<int32>(T / 60.0); return FString::Printf(TEXT("%d:%04.1f"), Min, T - 60.0 * Min); };
	const double T = R.GetTime() - R.GetStart(), Len = FMath::Max(R.GetEnd() - R.GetStart(), 1e-3);
	Text(FString::Printf(TEXT("REPLAY   %s   x%g   %s / %s"), R.IsPaused() ? TEXT("PAUSED") : TEXT("PLAY"), R.GetSpeed(), *Fmt(T), *Fmt(Len)), X + 14.0f, Y + 8.0f, Warn, 1.4f);
	Text(FString::Printf(TEXT("%s   camera: %s   (%d karts)"), *R.GetFocusName(), R.GetCameraName(), R.NumKarts()), X + W - 360.0f, Y + 12.0f, FLinearColor::White, 1.1f);
	// Timeline.
	const float BX = X + 14.0f, BY = Y + 44.0f, BW = W - 28.0f;
	DrawRect(FLinearColor(0.25f, 0.25f, 0.25f, 1.0f), BX, BY, BW, 8.0f);
	DrawRect(Warn, BX, BY, BW * static_cast<float>(T / Len), 8.0f);
	Text(TEXT("Space pause   Left/Right -/+5 s (Shift 30 s)   Up/Down speed   V next kart   X camera   F7 save   F5 exit"), X + 14.0f, Y + 64.0f, Dim, 1.0f);
	Text(FString::Printf(TEXT("track: %s"), *FPaths::GetBaseFilename(R.GetCircuit())), X + 14.0f, Y + 82.0f, Dim, 0.9f);
}

// ------------------------------------------------------------------------------- lap analysis

void AKartHUD::DrawAnalysis(const AKartVehiclePawn& Kart)
{
	const float CW = Canvas->ClipX, CH = Canvas->ClipY;
	DrawRect(FLinearColor(0.02f, 0.02f, 0.03f, 0.94f), 0.0f, 0.0f, CW, CH);
	const kartsim::LapDelta& D = Kart.GetLapAnalysis().GetDelta();
	const std::vector<kartsim::LapTrace>& H = D.History();
	Text(TEXT("LAP ANALYSIS"), 24.0f, 14.0f, Warn, 1.5f);
	Text(TEXT("Up/Down lap   PgUp/PgDn reference lap   Left/Right cursor (Shift fast)   Home start   L close"), 260.0f, 20.0f, Dim, 1.0f);
	if (H.empty() || Kart.GetAnalysisLap() < 0)
	{
		Text(TEXT("No complete lap yet: drive a full timed lap (start/finish to start/finish)."), 24.0f, 70.0f, FLinearColor::White, 1.2f);
		return;
	}
	auto Fmt = [](double T) { const int32 Min = static_cast<int32>(T / 60.0); return FString::Printf(TEXT("%d:%06.3f"), Min, T - 60.0 * Min); };
	const int32 SelIdx = FMath::Clamp(Kart.GetAnalysisLap(), 0, static_cast<int32>(H.size()) - 1);
	const kartsim::LapTrace& Sel = H[SelIdx];
	const bool bRefBest = Kart.GetAnalysisRefLap() < 0 || Kart.GetAnalysisRefLap() >= static_cast<int32>(H.size());
	const kartsim::LapTrace& Ref = bRefBest ? D.ReferenceLap() : H[Kart.GetAnalysisRefLap()];
	const bool bHasRef = !Ref.Time.empty();
	const size_t N = Sel.Time.size();
	if (N < 2) { return; }
	const size_t Cur = FMath::Clamp(static_cast<size_t>(Kart.GetAnalysisCursor()), static_cast<size_t>(0), N - 1);
	const std::vector<double> Delta = bHasRef ? kartsim::DeltaTrace(Sel, Ref) : std::vector<double>();

	const FLinearColor SelCol(1.0f, 0.85f, 0.2f), RefCol(0.55f, 0.55f, 0.6f), ThrCol(0.2f, 0.9f, 0.3f), BrkCol(0.95f, 0.25f, 0.2f);
	Text(FString::Printf(TEXT("Lap %d  %s%s"), Sel.LapNumber, *Fmt(Sel.LapTime), Sel.bValid ? TEXT("") : TEXT("  INVALID")), 24.0f, 46.0f, SelCol, 1.2f);
	if (bHasRef)
	{
		Text(FString::Printf(TEXT("vs %s  %s   (%+.3f s)"), bRefBest ? TEXT("best lap") : *FString::Printf(TEXT("lap %d"), Ref.LapNumber), *Fmt(Ref.LapTime), Sel.LapTime - Ref.LapTime),
		     300.0f, 46.0f, RefCol, 1.2f);
	}
	Text(FString::Printf(TEXT("cursor %.0f m"), Cur * kartsim::LapTrace::Spacing), CW * 0.62f - 120.0f, 46.0f, FLinearColor::White, 1.1f);

	// --- Stacked graphs vs distance ---
	struct FRow { const TCHAR* Name; const std::vector<double>* S; const std::vector<double>* R; double Lo, Hi; int32 Dec; FLinearColor C; const std::vector<double>* S2; const std::vector<double>* R2; FLinearColor C2; };
	double SteerMax = 30.0, LatMax = 1.0, RpmMax = 1000.0, SpdMax = 50.0, DMax = 0.1;
	for (size_t I = 0; I < N; ++I)
	{
		SteerMax = FMath::Max(SteerMax, FMath::Abs(Sel.Steer[I]));
		LatMax = FMath::Max(LatMax, FMath::Abs(Sel.LatG[I]));
		RpmMax = FMath::Max(RpmMax, Sel.Rpm[I]);
		SpdMax = FMath::Max(SpdMax, Sel.Speed[I]);
		if (I < Delta.size()) { DMax = FMath::Max(DMax, FMath::Abs(Delta[I])); }
	}
	if (bHasRef)
	{
		for (size_t I = 0; I < Ref.Time.size(); ++I)
		{
			SteerMax = FMath::Max(SteerMax, FMath::Abs(Ref.Steer[I])); RpmMax = FMath::Max(RpmMax, Ref.Rpm[I]); SpdMax = FMath::Max(SpdMax, Ref.Speed[I]);
			LatMax = FMath::Max(LatMax, FMath::Abs(Ref.LatG[I]));
		}
	}
	const FRow Rows[] = {
		{TEXT("Speed km/h"), &Sel.Speed, bHasRef ? &Ref.Speed : nullptr, 0.0, SpdMax * 1.05, 1, SelCol, nullptr, nullptr, SelCol},
		{TEXT("Throttle / brake"), &Sel.Throttle, bHasRef ? &Ref.Throttle : nullptr, 0.0, 1.05, 2, ThrCol, &Sel.Brake, bHasRef ? &Ref.Brake : nullptr, BrkCol},
		{TEXT("Steering deg (+ left)"), &Sel.Steer, bHasRef ? &Ref.Steer : nullptr, -SteerMax * 1.05, SteerMax * 1.05, 1, SelCol, nullptr, nullptr, SelCol},
		{TEXT("Lateral g"), &Sel.LatG, bHasRef ? &Ref.LatG : nullptr, -LatMax * 1.05, LatMax * 1.05, 2, SelCol, nullptr, nullptr, SelCol},
		{TEXT("Engine rpm"), &Sel.Rpm, bHasRef ? &Ref.Rpm : nullptr, 0.0, RpmMax * 1.05, 0, SelCol, nullptr, nullptr, SelCol},
		{TEXT("Delta s (+ = slower)"), bHasRef ? &Delta : nullptr, nullptr, -DMax * 1.1, DMax * 1.1, -3, FLinearColor(0.4f, 0.8f, 1.0f), nullptr, nullptr, SelCol},
	};
	const int32 NumRows = UE_ARRAY_COUNT(Rows);
	const float GX = 24.0f, GW = CW * 0.62f - 40.0f, GTop = 76.0f;
	const float RowH = (CH - GTop - 20.0f) / NumRows;
	const size_t Step = FMath::Max<size_t>(1, N / static_cast<size_t>(FMath::Max(GW / 2.0f, 50.0f)));
	auto XOf = [&](size_t I) { return GX + GW * static_cast<float>(I) / static_cast<float>(N - 1); };
	for (int32 RI = 0; RI < NumRows; ++RI)
	{
		const FRow& Row = Rows[RI];
		const float Y0 = GTop + RI * RowH, Hh = RowH - 18.0f;
		DrawRect(FLinearColor(0.08f, 0.08f, 0.1f, 1.0f), GX, Y0 + 14.0f, GW, Hh);
		Text(Row.Name, GX + 4.0f, Y0, Dim, 0.9f);
		if (!Row.S || Row.S->empty()) { Text(TEXT("(no reference lap)"), GX + 200.0f, Y0, Dim, 0.9f); continue; }
		auto YOf = [&](double V) { return Y0 + 14.0f + Hh * (1.0f - static_cast<float>(FMath::Clamp((V - Row.Lo) / FMath::Max(Row.Hi - Row.Lo, 1e-9), 0.0, 1.0))); };
		if (Row.Lo < 0.0 && Row.Hi > 0.0) { DrawLine(GX, YOf(0.0), GX + GW, YOf(0.0), FLinearColor(0.25f, 0.25f, 0.3f), 1.0f); }
		auto Plot = [&](const std::vector<double>* V, const FLinearColor& C, float Th)
		{
			if (!V || V->size() < 2) { return; }
			const size_t M = FMath::Min(V->size(), N);
			for (size_t I = Step; I < M; I += Step) { DrawLine(XOf(I - Step), YOf((*V)[I - Step]), XOf(I), YOf((*V)[I]), C, Th); }
		};
		Plot(Row.R, RefCol, 1.0f);
		Plot(Row.R2, RefCol * 0.8f, 1.0f);
		Plot(Row.S, Row.C, 1.5f);
		Plot(Row.S2, Row.C2, 1.5f);
		// Values at the cursor.
		auto Num = [&Row](double X)
		{
			switch (Row.Dec)
			{
			case 0: return FString::Printf(TEXT("%.0f"), X);
			case 1: return FString::Printf(TEXT("%.1f"), X);
			case 2: return FString::Printf(TEXT("%.2f"), X);
			default: return FString::Printf(TEXT("%+.3f"), X); // -3: signed seconds
			}
		};
		FString V = Num((*Row.S)[FMath::Min(Cur, Row.S->size() - 1)]);
		if (Row.R && Cur < Row.R->size()) { V += TEXT("  /  ") + Num((*Row.R)[Cur]); }
		if (Row.S2) { V += FString::Printf(TEXT("   brake %.2f"), (*Row.S2)[FMath::Min(Cur, Row.S2->size() - 1)]); }
		if (RI == 4) { V += FString::Printf(TEXT("   gear %.0f"), Sel.Gear[Cur]); }
		Text(V, GX + GW - 300.0f, Y0, FLinearColor::White, 0.9f);
	}
	DrawLine(XOf(Cur), GTop + 14.0f, XOf(Cur), CH - 24.0f, FLinearColor(1.0f, 1.0f, 1.0f, 0.6f), 1.0f);

	// --- Track map: colour = where this lap gained (green) or lost (red) time against the reference ---
	const float MX = CW * 0.62f + 10.0f, MW = CW - MX - 24.0f, MTop = 76.0f, MH = FMath::Min(MW, CH * 0.5f);
	DrawRect(FLinearColor(0.08f, 0.08f, 0.1f, 1.0f), MX, MTop, MW, MH);
	double X0 = 1e30, X1 = -1e30, Y0m = 1e30, Y1m = -1e30;
	for (size_t I = 0; I < N; ++I) { X0 = FMath::Min(X0, Sel.X[I]); X1 = FMath::Max(X1, Sel.X[I]); Y0m = FMath::Min(Y0m, Sel.Y[I]); Y1m = FMath::Max(Y1m, Sel.Y[I]); }
	const double Sc = FMath::Min((MW - 30.0) / FMath::Max(X1 - X0, 1.0), (MH - 30.0) / FMath::Max(Y1m - Y0m, 1.0));
	auto MapPt = [&](size_t I) { return FVector2D(MX + 15.0 + (Sel.X[I] - X0) * Sc, MTop + 15.0 + (Y1m - Sel.Y[I]) * Sc); }; // sim +Y (left) = up
	const size_t MS = FMath::Max<size_t>(1, Step / 2);
	for (size_t I = MS; I < N; I += MS)
	{
		FLinearColor C = SelCol;
		if (Delta.size() > I)
		{
			const double Rate = (Delta[I] - Delta[I - MS]) / (MS * kartsim::LapTrace::Spacing) * 100.0; // s per 100 m
			const float A = FMath::Clamp(static_cast<float>(FMath::Abs(Rate) / 0.05), 0.15f, 1.0f);
			C = Rate < 0.0 ? FLinearColor::LerpUsingHSV(FLinearColor(0.5f, 0.5f, 0.5f), ThrCol, A) : FLinearColor::LerpUsingHSV(FLinearColor(0.5f, 0.5f, 0.5f), BrkCol, A);
		}
		const FVector2D A = MapPt(I - MS), B = MapPt(I);
		DrawLine(A.X, A.Y, B.X, B.Y, C, 3.0f);
	}
	const FVector2D P0 = MapPt(0), PC = MapPt(Cur);
	DrawRect(FLinearColor::White, P0.X - 2.0f, P0.Y - 6.0f, 4.0f, 12.0f);
	DrawRect(SelCol, PC.X - 5.0f, PC.Y - 5.0f, 10.0f, 10.0f);
	Text(bHasRef ? TEXT("green: gaining   red: losing   (vs reference)") : TEXT("no reference lap yet"), MX + 8.0f, MTop + MH + 4.0f, Dim, 0.9f);

	// --- Lap list ---
	float LY = MTop + MH + 28.0f;
	Text(TEXT("lap      time       to best"), MX + 8.0f, LY, Dim, 1.0f);
	LY += 20.0f;
	const double BestT = D.HasReference() ? D.ReferenceLap().LapTime : 0.0;
	const int32 MaxRows = FMath::Max(1, static_cast<int32>((CH - LY - 20.0f) / 18.0f));
	const int32 First = FMath::Clamp(SelIdx - MaxRows / 2, 0, FMath::Max(0, static_cast<int32>(H.size()) - MaxRows));
	for (int32 I = First; I < static_cast<int32>(H.size()) && I < First + MaxRows; ++I)
	{
		const kartsim::LapTrace& L = H[I];
		const bool bIsRef = !bRefBest && I == Kart.GetAnalysisRefLap();
		const FLinearColor C = I == SelIdx ? SelCol : (bIsRef ? RefCol : (L.bValid ? FLinearColor::White : Bad));
		Text(FString::Printf(TEXT("%3d   %s   %s%s%s"), L.LapNumber, *Fmt(L.LapTime), BestT > 0.0 ? *FString::Printf(TEXT("%+.3f"), L.LapTime - BestT) : TEXT("--"),
		                     L.bValid ? TEXT("") : TEXT("  inv"), bIsRef ? TEXT("  [ref]") : TEXT("")), MX + 8.0f, LY, C, 1.0f);
		LY += 18.0f;
	}
}
