// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class PluginTest : ModuleRules
{
	public PluginTest(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[] {
			"Core",
			"CoreUObject",
			"Engine",
			"InputCore",
			"EnhancedInput",
			"AIModule",
			"StateTreeModule",
			"GameplayStateTreeModule",
			"UMG",
			"Slate",
			"SimpleObjectPool"

		});

		PrivateDependencyModuleNames.AddRange(new string[] { });

		PublicIncludePaths.AddRange(new string[] {
			"PluginTest",
			"PluginTest/Variant_Platforming",
			"PluginTest/Variant_Platforming/Animation",
			"PluginTest/Variant_Combat",
			"PluginTest/Variant_Combat/AI",
			"PluginTest/Variant_Combat/Animation",
			"PluginTest/Variant_Combat/Gameplay",
			"PluginTest/Variant_Combat/Interfaces",
			"PluginTest/Variant_Combat/UI",
			"PluginTest/Variant_SideScrolling",
			"PluginTest/Variant_SideScrolling/AI",
			"PluginTest/Variant_SideScrolling/Gameplay",
			"PluginTest/Variant_SideScrolling/Interfaces",
			"PluginTest/Variant_SideScrolling/UI"
		});

		// Uncomment if you are using Slate UI
		// PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore" });

		// Uncomment if you are using online features
		// PrivateDependencyModuleNames.Add("OnlineSubsystem");

		// To include OnlineSubsystemSteam, add it to the plugins section in your uproject file with the Enabled attribute set to true
	}
}
