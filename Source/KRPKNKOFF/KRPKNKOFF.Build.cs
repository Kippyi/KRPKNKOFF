using UnrealBuildTool;

public class KRPKNKOFF : ModuleRules
{
	public KRPKNKOFF(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		if (Target.Platform == UnrealTargetPlatform.Win64)
		{
			PublicSystemLibraries.AddRange(new string[] { "dinput8.lib", "dxguid.lib" }); // wheels / FFB (KartWheelInput)
		}
		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"Slate",
			"SlateCore",
			"AudioExtensions",
			"AudioMixer",
			"InputCore",
			"EnhancedInput",
			"ProceduralMeshComponent",
			"PhysicsCore",
			"KartSimCore",
			"OnlineSubsystem",      // Steam lobbies: host / server list / join
			"OnlineSubsystemUtils",
			"ApplicationCore",      // clipboard (license activation screen)
			"HTTP"                  // license revocation list download
		});
	}
}
