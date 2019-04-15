// Fill out your copyright notice in the Description page of Project Settings.

#include "VoronoiAIController.h"

#include "Runtime/Engine/Public/DrawDebugHelpers.h"
#include "Runtime/Engine/Classes/Engine/World.h"

void AVoronoiAIController::BeginPlay()
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


void AVoronoiAIController::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	Scan();

}


void AVoronoiAIController::Scan()
{
	if (!ControlledVehicle)
	{
		return;
	}
	FHitResult HitResult;
	FVector HitLocation;
	auto StartLocation = ControlledVehicle->GetActorLocation();
	for (int i = 0; i < 1081; i++)
	{
		float MeasuringAngle = -135 + i * AngularResolution;
		const FRotator Rot(0, -MeasuringAngle, 0);
		FVector MeasuringDirection = Rot.RotateVector(ControlledVehicle->GetActorForwardVector());
		auto EndLocation = StartLocation + MeasuringDirection * Range * 100; // *100 to convert to cm

		if (GetWorld()->LineTraceSingleByChannel(
			HitResult,
			StartLocation,
			EndLocation,
			ECollisionChannel::ECC_Visibility)
			)
		{
			HitLocation = HitResult.Location;
			if (MeasuringAngle >= -135 && MeasuringAngle <= 135) { // To control visualization range
				DrawDebugLine(GetWorld(), StartLocation, HitLocation, FColor(255, 0, 0), false, 0.f, 0.f, 0.f);
			}
			Distances[i] = (HitLocation - StartLocation).Size() / 100; //divide by 100 to convert cm to meters
		}
		else
		{
			Distances[i] = OutOfRange;
		}
	}

}