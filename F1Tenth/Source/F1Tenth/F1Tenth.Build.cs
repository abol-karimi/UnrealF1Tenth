// Copyright 1998-2018 Epic Games, Inc. All Rights Reserved.

using System;
using System.IO;
using UnrealBuildTool;

public class F1Tenth : ModuleRules
{
	public F1Tenth(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "PhysXVehicles", "HeadMountedDisplay" });

		PublicDefinitions.Add("HMD_MODULE_INCLUDED=1");

		string BoostIncludePath = "/home/ak/Downloads/boost_1_72_0";
		PublicIncludePaths.Add(BoostIncludePath);
		PrivateIncludePaths.Add(BoostIncludePath);
		PublicDefinitions.Add("BOOST_NO_EXCEPTIONS");
        PublicDefinitions.Add("BOOST_DISABLE_ABI_HEADERS");
	}
}
