// Fill out your copyright notice in the Description page of Project Settings.


#include "PurepursuitAIController.h"

void APurepursuitAIController::BeginPlay()
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
}


void APurepursuitAIController::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	FVector rearAxelLocation = ControlledVehicle->GetActorLocation() - 0.5*wheelbase*ControlledVehicle->GetActorForwardVector();
	double x = rearAxelLocation.X / 100.f;
	double y = rearAxelLocation.Y / 100.f;
	double theta = ControlledVehicle->GetActorRotation().Yaw / 180.f * PI;
	double l2 = lookahead * lookahead;
	double y2 = y * y;
	double L = wheelbase;
	double steering_angle_rad = atan(2 * L * (-sqrt(l2-y2)*sin(theta)-y*cos(theta))/l2);
	double steering_ratio = 2*steering_angle_rad / PI;

	ControlledVehicle->MoveRight(steering_ratio);
	ControlledVehicle->MoveForward(0.3);
}
