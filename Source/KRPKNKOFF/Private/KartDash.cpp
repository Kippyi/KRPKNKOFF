#include "KartDash.h"

#include "CanvasItem.h"
#include "Engine/Canvas.h"
#include "Engine/CanvasRenderTarget2D.h"
#include "Engine/Engine.h"
#include "Engine/Font.h"
#include "KartConditionsActor.h"
#include "KartVehiclePawn.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

namespace
{
	const FLinearColor DashBg(0.006f, 0.007f, 0.009f, 1.0f);
	const FLinearColor DashPanel(0.04f, 0.045f, 0.05f, 1.0f);
	const FLinearColor DashText(0.92f, 0.93f, 0.95f, 1.0f);
	const FLinearColor DashDim(0.45f, 0.48f, 0.52f, 1.0f);
	const FLinearColor DashGreen(0.10f, 0.85f, 0.25f, 1.0f);
	const FLinearColor DashYellow(1.0f, 0.78f, 0.05f, 1.0f);
	const FLinearColor DashRed(1.0f, 0.12f, 0.08f, 1.0f);
	const FLinearColor DashBlue(0.15f, 0.45f, 1.0f, 1.0f);

	void Box(UCanvas* C, float X, float Y, float W, float H, const FLinearColor& Col)
	{
		FCanvasTileItem T(FVector2D(X, Y), FVector2D(W, H), Col);
		T.BlendMode = SE_BLEND_Opaque;
		C->DrawItem(T);
	}

	// Align: 0 left, 1 centre, 2 right.
	void Txt(UCanvas* C, const FString& S, float X, float Y, float Scale, const FLinearColor& Col, int32 Align = 0)
	{
		UFont* F = GEngine ? GEngine->GetLargeFont() : nullptr;
		if (!F) { return; }
		Scale *= 2.2f; // the engine's large font is ~12 px: layout scales are in "dash units"
		float W = 0.0f, H = 0.0f;
		C->TextSize(F, S, W, H, Scale, Scale);
		if (Align == 1) { X -= 0.5f * W; } else if (Align == 2) { X -= W; }
		FCanvasTextItem T(FVector2D(X, Y), FText::FromString(S), F, Col);
		T.Scale = FVector2D(Scale, Scale);
		T.bOutlined = false;
		C->DrawItem(T);
	}

	FString LapStr(double T)
	{
		if (T <= 0.0) { return TEXT("-:--.---"); }
		const int32 Min = static_cast<int32>(T / 60.0);
		return FString::Printf(TEXT("%d:%06.3f"), Min, T - 60.0 * Min);
	}
}

void UKartDashComponent::Init(AKartVehiclePawn* InKart, const FVector& Centre, const FVector& Up, const FVector& Right, float WidthCm, float HeightCm)
{
	Kart = InKart;
	const FVector Hw = Right * (0.5f * WidthCm), Hh = Up * (0.5f * HeightCm);
	// Top-left, top-right, bottom-right, bottom-left (as the driver sees it); both windings (never culled).
	const TArray<FVector> V = {Centre - Hw + Hh, Centre + Hw + Hh, Centre + Hw - Hh, Centre - Hw - Hh};
	const TArray<FVector2D> UV = {FVector2D(0, 0), FVector2D(1, 0), FVector2D(1, 1), FVector2D(0, 1)};
	const FVector N = FVector::CrossProduct(Right, Up).GetSafeNormal();
	const TArray<FVector> Nm = {N, N, N, N};
	const TArray<int32> Tri = {0, 1, 2, 0, 2, 3, 0, 2, 1, 0, 3, 2};
	CreateMeshSection_LinearColor(0, V, Tri, Nm, UV, TArray<FLinearColor>(), TArray<FProcMeshTangent>(), false);
	SetCastShadow(false);
	SetCollisionEnabled(ECollisionEnabled::NoCollision);

	Target = UCanvasRenderTarget2D::CreateCanvasRenderTarget2D(this, UCanvasRenderTarget2D::StaticClass(), 512, 256);
	if (Target)
	{
		Target->ClearColor = DashBg;
		Target->OnCanvasRenderTargetUpdate.AddDynamic(this, &UKartDashComponent::OnDraw);
	}
	if (UMaterialInterface* M = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/KartVisuals/M_KartDash.M_KartDash")))
	{
		ScreenMat = UMaterialInstanceDynamic::Create(M, this);
		ScreenMat->SetTextureParameterValue(TEXT("Screen"), Target);
		// Physical units (the scene's exposure is EV ~12-14): -KartDashNits= tunes it (ESTIMATE: a sunlight-readable TFT).
		float Nits = 5000.0f;
		FParse::Value(FCommandLine::Get(), TEXT("KartDashNits="), Nits);
		ScreenMat->SetScalarParameterValue(TEXT("Brightness"), Nits);
		SetMaterial(0, ScreenMat);
	}
	Refresh();
}

void UKartDashComponent::Refresh()
{
	if (Target) { Target->UpdateResource(); }
}

void UKartDashComponent::OnDraw(UCanvas* Canvas, int32 Width, int32 Height)
{
	if (const AKartVehiclePawn* K = Kart.Get()) { DrawMychron(Canvas, 0.0f, 0.0f, static_cast<float>(Width), static_cast<float>(Height), *K); }
}

void UKartDashComponent::DrawMychron(UCanvas* C, float X, float Y, float W, float H, const AKartVehiclePawn& Kart)
{
	const kartsim::KartSimulation* Sim = Kart.GetSim();
	if (!C || !Sim) { return; }
	const kartsim::TelemetryFrame& F = Sim->GetTelemetry();
	const kartsim::KartConfig& Cfg = Sim->GetConfig();
	const kartsim::EngineTuneParams& Tu = Cfg.Tuning;
	const float U = H / 256.0f;
	const double Now = Sim->GetTime();
	const bool bBlink = FMath::Fmod(Now, 0.5) < 0.25;
	Box(C, X, Y, W, H, DashBg);

	// --- Shift LEDs (10, from 70 % of the rev limit), blue while the limiter cuts ---
	const double Frac = F.EngineRpm / FMath::Max(Cfg.Engine.RevLimitRpm, 1.0);
	const float LedW = (W - 22.0f * U) / 10.0f;
	for (int32 I = 0; I < 10; ++I)
	{
		const bool bOn = Frac > 0.70 + 0.028 * I;
		FLinearColor Col = I < 4 ? DashGreen : (I < 7 ? DashYellow : DashRed);
		if (F.RevLimiter > 0.5) { Col = bBlink ? DashBlue : DashBg; }
		Box(C, X + 6.0f * U + I * (LedW + 1.0f * U), Y + 5.0f * U, LedW - 3.0f * U, 12.0f * U, bOn || F.RevLimiter > 0.5 ? Col : DashPanel);
	}
	// --- rpm bar graph ---
	const float BarX = X + 8.0f * U, BarY = Y + 24.0f * U, BarW = W - 16.0f * U, BarH = 16.0f * U;
	Box(C, BarX, BarY, BarW, BarH, DashPanel);
	Box(C, BarX, BarY, BarW * static_cast<float>(FMath::Clamp(Frac, 0.0, 1.0)), BarH, Frac > 0.93 ? DashRed : DashText);
	for (int32 K = 1; K < 15; ++K)
	{
		const double Rpm = K * 1000.0;
		if (Rpm >= Cfg.Engine.RevLimitRpm) { break; }
		Box(C, BarX + BarW * static_cast<float>(Rpm / Cfg.Engine.RevLimitRpm), BarY + BarH - 5.0f * U, 1.5f * U, 5.0f * U, DashBg);
	}

	// --- left: rpm, speed (gear for gearbox karts) ---
	Txt(C, TEXT("RPM"), X + 10.0f * U, Y + 46.0f * U, 0.75f * U, DashDim);
	Txt(C, FString::Printf(TEXT("%.0f"), F.EngineRpm), X + 10.0f * U, Y + 56.0f * U, 1.9f * U, F.RevLimiter > 0.5 ? DashBlue : DashText);
	Txt(C, FString::Printf(TEXT("%.0f km/h"), F.SpeedKmh), X + 10.0f * U, Y + 110.0f * U, 1.2f * U, DashText);
	if (Cfg.Gearbox.NumGears > 1)
	{
		const int32 G = static_cast<int32>(F.Gear);
		Txt(C, G == 0 ? FString(TEXT("N")) : FString::Printf(TEXT("G%d"), G), X + 160.0f * U, Y + 110.0f * U, 1.2f * U, DashYellow);
	}

	// --- right: lap time (held for 5 s after the line, like the logger), lap number, best, live delta ---
	const float RX = X + W - 10.0f * U;
	if (Kart.HasTimingLines())
	{
		const kartsim::LapTimer& LT = Kart.GetLapTimer();
		const auto& Laps = LT.Laps();
		const double Clock = Kart.GetClock();
		const bool bHold = !Laps.empty() && Clock - LT.LastStartFinishTime() < 5.0;
		const bool bBest = bHold && LT.BestLapIndex() == static_cast<int32>(Laps.size()) - 1;
		const double Shown = bHold ? Laps.back().LapTime : LT.CurrentLapTime(Clock);
		Txt(C, FString::Printf(TEXT("LAP %d"), static_cast<int32>(Laps.size()) + (bHold ? 0 : 1)), RX, Y + 46.0f * U, 0.75f * U, DashDim, 2);
		const FLinearColor LapCol = bHold ? (bBest ? DashGreen : (Laps.back().bValid ? DashYellow : DashRed)) : (LT.IsTiming() ? DashText : DashDim);
		Txt(C, LT.IsTiming() || bHold ? LapStr(Shown) : FString(TEXT("-:--.---")), RX, Y + 56.0f * U, 1.9f * U, LapCol, 2);
		const FString Best = LT.BestLapIndex() >= 0 ? LapStr(Laps[LT.BestLapIndex()].LapTime) : FString(TEXT("-:--.---"));
		Txt(C, TEXT("BEST ") + Best, RX, Y + 116.0f * U, 0.85f * U, DashText, 2);
		const kartsim::LapDelta& D = Kart.GetLapAnalysis().GetDelta();
		if (D.HasDelta())
		{
			Txt(C, FString::Printf(TEXT("%+.2f"), D.Delta()), RX - 175.0f * U, Y + 112.0f * U, 1.1f * U, D.Delta() <= 0.0 ? DashGreen : DashRed, 2);
		}
	}
	else
	{
		Txt(C, TEXT("NO BEACON"), RX, Y + 64.0f * U, 1.0f * U, DashDim, 2);
	}

	// --- bottom: water / CHT and EGT tiles ---
	const float TileY = Y + 146.0f * U, TileH = 60.0f * U, TileW = (W - 32.0f * U) / 3.0f;
	auto Tile = [&](float TX, const TCHAR* Label, const FString& Value, const FLinearColor& Col)
	{
		Box(C, TX, TileY, TileW, TileH, DashPanel);
		Txt(C, Label, TX + 6.0f * U, TileY + 4.0f * U, 0.7f * U, DashDim);
		Txt(C, Value, TX + TileW - 6.0f * U, TileY + 13.0f * U, 1.4f * U, Col, 2);
	};
	const double Wt = F.WaterTemp;
	const FLinearColor WCol = Wt > Tu.OptimumHigh + 8.0 ? DashRed : (Wt > Tu.OptimumHigh ? DashYellow : (Wt < Tu.OptimumLow - 5.0 ? DashBlue : DashGreen));
	Tile(X + 8.0f * U, Tu.bWaterCooled ? TEXT("WATER") : TEXT("CHT"), FString::Printf(TEXT("%.0f°"), Wt), WCol);
	const FLinearColor ECol = F.Egt > Tu.EgtFull + 60.0 ? DashRed : (F.Egt > Tu.EgtFull + 25.0 ? DashYellow : DashText);
	Tile(X + 16.0f * U + TileW, TEXT("EGT"), FString::Printf(TEXT("%.0f°"), F.Egt), ECol);

	// Lambda: green around best power (0.84-0.89), yellow / red lean, blue rich; off throttle it reads lean (white).
	const double Lam = F.Lambda;
	const FLinearColor LCol = F.ThrottleEff < 0.5 ? DashText : (Lam > 0.92 ? DashRed : (Lam > 0.89 ? DashYellow : (Lam < 0.80 ? DashBlue : DashGreen)));
	Tile(X + 24.0f * U + 2.0f * TileW, TEXT("LAMBDA"), FString::Printf(TEXT("%.2f"), Lam), LCol);

	// --- alarm line: only what a real logger's sensors know (temperatures), plus the jet just changed ([ / ]) ---
	FString Alarm;
	FLinearColor ACol = DashRed;
	if (Now - Kart.GetJetChangeTime() < 2.5)
	{
		Alarm = Tu.Adjust == kartsim::ECarbAdjust::MainJet ? FString::Printf(TEXT("MAIN JET %.0f"), Tu.Jet) : FString::Printf(TEXT("NEEDLE %.2f TURNS"), Tu.Jet);
		ACol = DashYellow;
	}
	else if (Wt > Tu.OptimumHigh + 8.0) { Alarm = Tu.bWaterCooled ? TEXT("WATER HIGH") : TEXT("CHT HIGH"); }
	else if (F.Egt > Tu.EgtFull + 60.0) { Alarm = TEXT("EGT HIGH"); }
	else if (Wt < Tu.OptimumLow - 8.0 && F.SpeedKmh > 20.0) { Alarm = Tu.bWaterCooled ? TEXT("WATER LOW") : TEXT("CHT LOW"); ACol = DashBlue; }
	if (!Alarm.IsEmpty())
	{
		const bool bShow = ACol == DashRed ? bBlink : true;
		Box(C, X + 8.0f * U, Y + 212.0f * U, W - 16.0f * U, 38.0f * U, bShow ? ACol * 0.85f : DashPanel);
		Txt(C, Alarm, X + 0.5f * W, Y + 216.0f * U, 0.95f * U, bShow ? FLinearColor::White : ACol, 1);
	}
	else
	{
		const kartsim::LapTimer& LT = Kart.GetLapTimer();
		Txt(C, FString::Printf(TEXT("%s   LAPS %d"), UTF8_TO_TCHAR(Cfg.Name.c_str()), static_cast<int32>(LT.Laps().size())), X + 0.5f * W, Y + 220.0f * U, 0.75f * U, DashDim, 1);
	}
}
