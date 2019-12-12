// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "F1TenthPawn.h"

#include "CoreMinimal.h"
#include "AIController.h"
#include "PurepursuitAIController.generated.h"

/**
 * 
 */
UCLASS()
class F1TENTH_API APurepursuitAIController : public AAIController
{
	GENERATED_BODY()
	
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;

private:
	float wheelbase = 0.3; // Distance (in meters) of rear axle to front axel
	float lookahead = 1.0; // Distance (in meters) between the rear axel and the goal point
	AF1TenthPawn* ControlledVehicle = nullptr;
};
