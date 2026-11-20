// Graphics quality (user: a friend's laptop ran at 3 fps: the game always rendered Epic + Lumen hardware ray tracing +
// virtual shadow maps at native resolution). Saved in GameUserSettings.ini section [KartGraphics], applied at start
// (UKartGameInstance::Init) and whenever the menu's Settings page changes it.
//   Low     engine scalability 0, no Lumen (sky light + SSAO), screen-space reflections, shadow maps, TAA, 67 % render scale,
//           ray tracing off
//   Medium  scalability 1, no Lumen, shadow maps, TAA, 80 %
//   High    scalability 2, Lumen (hardware RT), virtual shadow maps, TSR, 100 %, Lumen post quality 1
//   Epic    scalability 3, everything (the look of the screenshots so far), Lumen post quality 2
// Preset -1 = Auto: the first launch runs UE's hardware benchmark and stores the result (capped at Medium on GPUs
// without ray-tracing hardware: the High / Epic lighting relies on it, procedural meshes have no distance fields).
#pragma once

#include "CoreMinimal.h"

struct KRPKNKOFF_API FKartGraphicsSettings
{
	int32 Preset = -1;          // -1 auto (not chosen yet), 0 Low .. 3 Epic
	int32 RenderScale = 100;    // % of the window resolution that is rendered (upscaled by TAA / TSR)
	int32 FrameLimit = 0;       // fps, 0 = unlimited
	bool bVSync = false;
	bool bShowFps = false;      // frame rate / frame time in the HUD corner

	static FKartGraphicsSettings& Get();
	void Save() const;

	// Applies everything (engine scalability, the extra cvars, render scale, frame limit, vsync).
	void Apply();
	// Window mode lives in UE's GameUserSettings (Fullscreen / borderless / windowed).
	static void CycleWindowMode(int32 Dir);
	static FString WindowModeName();
	// Sets a preset and its default render scale, applies and saves.
	void SetPreset(int32 P);
	// Runs the hardware benchmark and returns the suggested preset (0..3).
	static int32 DetectPreset();
	static int32 DefaultRenderScale(int32 P) { return P <= 0 ? 67 : (P == 1 ? 80 : 100); }
	static const TCHAR* PresetName(int32 P);
	// Lumen post-process quality for the scene's post-process volume (KartGameMode).
	static float LumenPostQuality(int32 P) { return P >= 3 ? 2.0f : 1.0f; }
};
