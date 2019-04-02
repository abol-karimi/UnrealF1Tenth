// Fill out your copyright notice in the Description page of Project Settings.

#include "Scanner.h"
#include "Runtime/Engine/Classes/GameFramework/Actor.h"
#include "Runtime/Engine/Public/DrawDebugHelpers.h"
#include "Runtime/Engine/Classes/Engine/World.h"

#include "Runtime/Core/Public/Misc/Paths.h"
#include <fstream>
#include <iostream>


// Sets default values for this component's properties
UScanner::UScanner()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// ...
}


// Called when the game starts
void UScanner::BeginPlay()
{
	Super::BeginPlay();

	FHitResult HitResult;
	FVector HitLocation;
	auto StartLocation = GetOwner()->GetActorLocation();
	float ranges[1081];
	float AngularResolution = 0.25; // 4 measurements per angle
	float LidarRange = 10; // range in meters

	for (int i = 0; i < 1081; i++)
	{
		const FRotator Rot(0, 135-i*AngularResolution, 0);
		FVector MeasuringDirection = Rot.RotateVector(GetOwner()->GetActorForwardVector());
		auto EndLocation = StartLocation + MeasuringDirection * LidarRange * 100; // *100 to convert to cm
		if (GetWorld()->LineTraceSingleByChannel(
			HitResult,
			StartLocation,
			EndLocation,
			ECollisionChannel::ECC_Visibility)
			)
		{
			HitLocation = HitResult.Location;
			//DrawDebugLine(GetWorld(), StartLocation, HitLocation, FColor(255, 0, 0), true, 1000.f, 0.f, 1.f);
			ranges[i] = (HitLocation - StartLocation).Size()/100; //divide by 100 to convert cm to meters
		}
		else
		{
			ranges[i] = 31;
		}
	}
	
	FString FileName = "ranges.floats";
	FString AbsoluteFilePath = FPaths::ProjectSavedDir() + FileName;

	//Creates an instance of ofstream, and creates a new file with address AbsoluteFilePath
	std::ofstream rangesfile(std::string(TCHAR_TO_UTF8(*AbsoluteFilePath)), std::ios::trunc);

	for (int i=0; i<1081; i++)
	{
		rangesfile << ranges[i] << std::endl;
	}

	// Close the file stream explicitly
	rangesfile.close();

	return;
}


// Called every frame
void UScanner::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	FHitResult HitResult;
	FVector HitLocation;
	auto StartLocation = GetOwner()->GetActorLocation();
	float AngularResolution = 0.25; // 4 measurements per angle
	float LidarRange = 10; // range in meters
	for (int i = 0; i < 1081; i++)
	{
		const FRotator Rot(0, 135 - i * AngularResolution, 0);
		FVector MeasuringDirection = Rot.RotateVector(GetOwner()->GetActorForwardVector());
		auto EndLocation = StartLocation + MeasuringDirection * LidarRange * 100; // *100 to convert to cm

		if (GetWorld()->LineTraceSingleByChannel(
			HitResult,
			StartLocation,
			EndLocation,
			ECollisionChannel::ECC_Visibility)
			)
		{
			HitLocation = HitResult.Location;
			DrawDebugLine(GetWorld(), StartLocation, HitLocation, FColor(255, 0, 0), false, 0.f, 0.f, 1.f);
		}
	}

}

