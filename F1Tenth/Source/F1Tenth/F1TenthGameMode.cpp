// Copyright 1998-2018 Epic Games, Inc. All Rights Reserved.

#include "F1TenthGameMode.h"
#include "F1TenthPawn.h"
#include "F1TenthHud.h"

AF1TenthGameMode::AF1TenthGameMode()
{
	DefaultPawnClass = AF1TenthPawn::StaticClass();
	HUDClass = AF1TenthHud::StaticClass();
}
