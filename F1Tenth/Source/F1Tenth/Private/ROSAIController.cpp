// Fill out your copyright notice in the Description page of Project Settings.


#include "ROSAIController.h"

void AROSAIController::BeginPlay()
{
	Super::BeginPlay();
	
	ControlledVehicle = Cast<AF1TenthPawn>(GetPawn());
	if (!ControlledVehicle)
	{
		UE_LOG(LogTemp, Warning, TEXT("VoronoiAIController not possesing a vehicle!"));
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("VoronoiAIController possesing: %s"), *(ControlledVehicle->GetName()));
	}

    rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());

    MotorDutyCycle = NewObject<UTopic>(UTopic::StaticClass());
    MotorDutyCycle->Init(rosinst->ROSIntegrationCore, TEXT("/commands/motor/duty_cycle"), TEXT("std_msgs/Float32"));
    MotorDutyCycle->Subscribe(DutyCycleCallback);

    ServoPosition = NewObject<UTopic>(UTopic::StaticClass());
    ServoPosition->Init(rosinst->ROSIntegrationCore, TEXT("/commands/servo/position"), TEXT("std_msgs/Float32"));
    ServoPosition->Subscribe(ServoPositionCallback);
}


