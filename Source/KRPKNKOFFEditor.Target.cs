using UnrealBuildTool;

public class KRPKNKOFFEditorTarget : TargetRules
{
	public KRPKNKOFFEditorTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;
		DefaultBuildSettings = BuildSettingsVersion.Latest;
		IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
		ExtraModuleNames.AddRange(new string[] { "KartSimCore", "KRPKNKOFF" });
	}
}
