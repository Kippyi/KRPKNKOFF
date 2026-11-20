#include "KartGraphicsSettings.h"

#include "Engine/Engine.h"
#include "Engine/PostProcessVolume.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/GameUserSettings.h"
#include "HAL/IConsoleManager.h"
#include "Misc/CommandLine.h"
#include "Misc/ConfigCacheIni.h"
#include "Misc/Parse.h"
#include "RHIGlobals.h"
#include "Scalability.h"

DEFINE_LOG_CATEGORY_STATIC(LogKartGfx, Log, All);

namespace
{
	const TCHAR* GSection = TEXT("KartGraphics");

	void SetCVar(const TCHAR* Name, int32 Value)
	{
		if (IConsoleVariable* V = IConsoleManager::Get().FindConsoleVariable(Name))
		{
			V->Set(Value, ECVF_SetByCode); // above the project-setting priority of DefaultEngine.ini (console / -ExecCmds still win)
		}
	}

	int32 PresetFromName(const FString& S)
	{
		if (S.Equals(TEXT("Low"), ESearchCase::IgnoreCase)) { return 0; }
		if (S.Equals(TEXT("Medium"), ESearchCase::IgnoreCase)) { return 1; }
		if (S.Equals(TEXT("High"), ESearchCase::IgnoreCase)) { return 2; }
		if (S.Equals(TEXT("Epic"), ESearchCase::IgnoreCase)) { return 3; }
		return FCString::Atoi(*S);
	}
}

const TCHAR* FKartGraphicsSettings::PresetName(int32 P)
{
	switch (P)
	{
	case 0: return TEXT("Low");
	case 1: return TEXT("Medium");
	case 2: return TEXT("High");
	case 3: return TEXT("Epic");
	default: return TEXT("Auto");
	}
}

FKartGraphicsSettings& FKartGraphicsSettings::Get()
{
	static FKartGraphicsSettings S;
	static bool bLoaded = false;
	if (!bLoaded && GConfig)
	{
		bLoaded = true;
		GConfig->GetInt(GSection, TEXT("Preset"), S.Preset, GGameUserSettingsIni);
		GConfig->GetInt(GSection, TEXT("RenderScale"), S.RenderScale, GGameUserSettingsIni);
		GConfig->GetInt(GSection, TEXT("FrameLimit"), S.FrameLimit, GGameUserSettingsIni);
		GConfig->GetBool(GSection, TEXT("VSync"), S.bVSync, GGameUserSettingsIni);
		GConfig->GetBool(GSection, TEXT("ShowFps"), S.bShowFps, GGameUserSettingsIni);
		S.Preset = FMath::Clamp(S.Preset, -1, 3);
		S.RenderScale = FMath::Clamp(S.RenderScale, 40, 100);
	}
	return S;
}

void FKartGraphicsSettings::Save() const
{
	if (!GConfig) { return; }
	GConfig->SetInt(GSection, TEXT("Preset"), Preset, GGameUserSettingsIni);
	GConfig->SetInt(GSection, TEXT("RenderScale"), RenderScale, GGameUserSettingsIni);
	GConfig->SetInt(GSection, TEXT("FrameLimit"), FrameLimit, GGameUserSettingsIni);
	GConfig->SetBool(GSection, TEXT("VSync"), bVSync, GGameUserSettingsIni);
	GConfig->SetBool(GSection, TEXT("ShowFps"), bShowFps, GGameUserSettingsIni);
	GConfig->Flush(false, GGameUserSettingsIni);
}

int32 FKartGraphicsSettings::DetectPreset()
{
	UGameUserSettings* G = GEngine ? GEngine->GetGameUserSettings() : nullptr;
	if (!G) { return 1; }
	G->RunHardwareBenchmark();
	G->ApplyHardwareBenchmarkResults();
	// The benchmark sets each scalability group; take the lower of the GPU-bound groups (shadows, GI, post) as the preset.
	int32 P = FMath::Min3(G->GetShadowQuality(), G->GetGlobalIlluminationQuality(), G->GetPostProcessingQuality());
	P = FMath::Clamp(P, 0, 3);
	const bool bHwRt = GRHISupportsRayTracing && GRHISupportsRayTracingShaders;
	if (!bHwRt) { P = FMath::Min(P, 1); }
	UE_LOG(LogKartGfx, Log, TEXT("Graphics auto-detect: CPU %.0f GPU %.0f, hardware ray tracing %d -> %s"), G->GetLastCPUBenchmarkResult(),
	       G->GetLastGPUBenchmarkResult(), bHwRt ? 1 : 0, PresetName(P));
	return P;
}

void FKartGraphicsSettings::SetPreset(int32 P)
{
	Preset = FMath::Clamp(P, 0, 3);
	RenderScale = DefaultRenderScale(Preset);
	Apply();
	Save();
}

void FKartGraphicsSettings::Apply()
{
	// -KartGraphics=Low|Medium|High|Epic (testing) overrides the saved preset for this run.
	FString Cmd;
	if (FParse::Value(FCommandLine::Get(), TEXT("KartGraphics="), Cmd))
	{
		const int32 P = FMath::Clamp(PresetFromName(Cmd), 0, 3);
		if (P != Preset) { Preset = P; RenderScale = DefaultRenderScale(P); }
	}
	if (Preset < 0)
	{
		Preset = DetectPreset();
		RenderScale = DefaultRenderScale(Preset);
		Save();
	}
	UGameUserSettings* G = GEngine ? GEngine->GetGameUserSettings() : nullptr;
	if (G)
	{
		G->SetOverallScalabilityLevel(Preset);
		G->SetResolutionScaleValueEx(static_cast<float>(RenderScale));
		G->SetFrameRateLimit(static_cast<float>(FrameLimit));
		G->SetVSyncEnabled(bVSync);
		G->ApplySettings(true); // window mode / resolution: UE's own saved values (menu: CycleWindowMode)
		G->SaveSettings();
	}
	// Engine scalability groups and the render scale directly (UGameUserSettings alone was overridden at startup).
	Scalability::FQualityLevels Q;
	Q.SetFromSingleQualityLevel(Preset);
	Q.AntiAliasingQuality = FMath::Max(Preset, 2); // TAA quality 0 has no history filter: grainy, dithered karts (cheap to keep)
	Q.ResolutionQuality = static_cast<float>(RenderScale);
	Scalability::SetQualityLevels(Q, true);
	if (IConsoleVariable* Sp = IConsoleManager::Get().FindConsoleVariable(TEXT("r.ScreenPercentage"))) { Sp->Set(static_cast<float>(RenderScale), ECVF_SetByCode); }
	// The big costs on top of engine scalability (all switchable at runtime).
	const bool bLumen = Preset >= 2;
	SetCVar(TEXT("r.DynamicGlobalIlluminationMethod"), bLumen ? 1 : 0); // 0: sky light + SSAO only
	SetCVar(TEXT("r.ReflectionMethod"), bLumen ? 1 : 2);                // 2: screen-space reflections
	SetCVar(TEXT("r.Shadow.Virtual.Enable"), bLumen ? 1 : 0);           // shadow maps on Low / Medium
	SetCVar(TEXT("r.AntiAliasingMethod"), bLumen ? 4 : 2);              // TSR / TAA
	SetCVar(TEXT("r.ContactShadows"), Preset >= 1 ? 1 : 0);
	SetCVar(TEXT("r.RayTracing.Enable"), bLumen ? 1 : 0);               // needs r.RayTracing.EnableOnDemand (DefaultEngine.ini)
	// Scene post-process volumes (KartGameMode) force Lumen quality: lower it below Epic.
	if (GEngine)
	{
		for (const FWorldContext& Ctx : GEngine->GetWorldContexts())
		{
			UWorld* W = Ctx.World();
			if (!W) { continue; }
			for (TActorIterator<APostProcessVolume> It(W); It; ++It)
			{
				FPostProcessSettings& S = It->Settings;
				if (!S.bOverride_LumenSceneDetail) { continue; }
				S.LumenFinalGatherQuality = S.LumenReflectionQuality = S.LumenSceneDetail = LumenPostQuality(Preset);
			}
		}
	}
	UE_LOG(LogKartGfx, Log, TEXT("Graphics: %s, render scale %d %%, frame limit %d, vsync %d"), PresetName(Preset), RenderScale, FrameLimit, bVSync ? 1 : 0);
}

void FKartGraphicsSettings::CycleWindowMode(int32 Dir)
{
	UGameUserSettings* G = GEngine ? GEngine->GetGameUserSettings() : nullptr;
	if (!G) { return; }
	const int32 M = (static_cast<int32>(G->GetFullscreenMode()) + (Dir < 0 ? 2 : 1)) % 3; // Fullscreen, WindowedFullscreen, Windowed
	G->SetFullscreenMode(static_cast<EWindowMode::Type>(M));
	if (M != EWindowMode::Windowed) { G->SetScreenResolution(G->GetDesktopResolution()); }
	G->ApplySettings(false);
	G->SaveSettings();
}

FString FKartGraphicsSettings::WindowModeName()
{
	UGameUserSettings* G = GEngine ? GEngine->GetGameUserSettings() : nullptr;
	if (!G) { return TEXT("-"); }
	switch (G->GetFullscreenMode())
	{
	case EWindowMode::Fullscreen: return TEXT("Fullscreen");
	case EWindowMode::WindowedFullscreen: return TEXT("Borderless window");
	default: return FString::Printf(TEXT("Window %dx%d"), G->GetScreenResolution().X, G->GetScreenResolution().Y);
	}
}
