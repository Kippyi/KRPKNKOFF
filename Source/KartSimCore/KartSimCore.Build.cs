using UnrealBuildTool;

// Engine-independent kart physics. Only the module boilerplate touches Unreal; everything under
// Public/KartSim and Private/KartSim is plain C++ and also builds standalone (Tests/build_tests.bat).
public class KartSimCore : ModuleRules
{
	public KartSimCore(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		PublicDependencyModuleNames.AddRange(new string[] { "Core" });
		bEnableExceptions = false;
	}
}
