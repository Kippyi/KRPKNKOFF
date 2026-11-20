# Graphics settings (performance)

User report (2026-11-19): a friend's laptop ran the game at 3 fps. Cause: the game always rendered Epic engine
scalability + Lumen GI/reflections with hardware ray tracing + virtual shadow maps + TSR at native resolution, with no
way to lower anything.

`FKartGraphicsSettings` (KartGraphicsSettings.h/.cpp), saved in GameUserSettings.ini `[KartGraphics]`, menu: Esc >
Settings > GRAPHICS (Quality, Render scale, Frame limit, VSync, Display, Show FPS).

| Preset | Engine sg | GI / reflections | Shadows | AA | Render scale | Ray tracing | Lumen PP quality |
|---|---|---|---|---|---|---|---|
| Low | 0 (AA 2) | none (sky light + SSAO) / SSR | shadow maps, no contact shadows | TAA | 67 % | off | - |
| Medium | 1 (AA 2) | none / SSR | shadow maps | TAA | 80 % | off | - |
| High | 2 | Lumen (HW RT) | virtual SM | TSR | 100 % | on | 1 |
| Epic | 3 | Lumen (HW RT) | virtual SM | TSR | 100 % | on | 2 |

* Auto (first launch, Preset = -1): UE hardware benchmark; preset = min of the shadow / GI / post groups it picks; capped at
  Medium without ray-tracing hardware (High/Epic lighting relies on HW RT; procedural meshes have no distance fields).
* Applied in UKartGameInstance::Init AND in AKartGameMode::InitGame: UE's startup re-applied the saved scalability after
  the GameInstance (verified: sg.* stayed 3). Scalability is set via Scalability::SetQualityLevels and the extra cvars with
  ECVF_SetByCode (DefaultEngine.ini sets them at project priority, above SetByGameSetting).
* `r.RayTracing.EnableOnDemand=1` (DefaultEngine.ini) so `r.RayTracing.Enable` can switch ray tracing off at runtime.
* TAA quality 0 (sg.AntiAliasingQuality 0) has no history filter: grainy, dithered karts. Kept at 2 on all presets.
* `-KartGraphics=Low|Medium|High|Epic` overrides for one run (testing).

Measured on the dev PC (VERIFIED, 1920x1080, 6 karts, csvprofile, median after warm-up):

| Preset | Frame | GPU passes |
|---|---|---|
| Low | 3.7 ms | 2.3 ms |
| Medium | 5.0 ms | 3.5 ms |
| High | 7.3 ms | 6.3 ms |
| Epic | 11.1 ms | 10.6 ms |

Low is ~4.6x lighter on the GPU than the old forced Epic. Not measured on a laptop: whether the friend's machine reaches a
playable rate is unknown until they try. Hybrid-GPU laptops running on the integrated GPU are a common cause; the README
explains the Windows per-app GPU setting.
