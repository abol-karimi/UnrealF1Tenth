// Fill out your copyright notice in the Description page of Project Settings.


#include "LidarComponent.h"
#include "UObject/ConstructorHelpers.h"

#include <vector>

#include "Kismet/GameplayStatics.h" 


ULidarComponent::ULidarComponent()
{
	static ConstructorHelpers::FObjectFinder<UStaticMesh> LidarMesh(TEXT("StaticMesh'/Game/StarterContent/Shapes/Lidar.Lidar'"));
	SetStaticMesh(LidarMesh.Object);

	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
}

void ULidarComponent::BeginPlay()
{
	Super::BeginPlay();

	ScanTopic = NewObject<UTopic>(UTopic::StaticClass()); 
	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(UGameplayStatics::GetGameInstance(this));
	ScanTopic->Init(rosinst->ROSIntegrationCore, TEXT("/scan"), TEXT("sensor_msgs/LaserScan"));

	// (Optional) Advertise the topic
	ScanTopic->Advertise();
}

void ULidarComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	// TODO: fix the frequency of lidar to 40Hz.

	Scan();
	Publish();
}

void ULidarComponent::Scan()
{
	FHitResult HitResult;
	FVector LidarLocation = GetComponentLocation();
	FVector LidarXAxis = GetForwardVector();
	FVector LidarYAxis = -GetRightVector();
	FVector LidarZAxis = GetUpVector();

	for (int i = 0; i < 1081; i++)
	{
		float MeasuringAngle = -135 + i * AngularResolution;
		const FRotator Rot(0, -MeasuringAngle, 0);
		FVector MeasuringDirection = Rot.RotateVector(LidarXAxis);
		FVector StartLocation = LidarLocation + MeasuringDirection * 10; // 10cm away from the center of lidar
		FVector EndLocation = LidarLocation + MeasuringDirection * Range * 100; // *100 to convert to cm

		if (GetWorld()->LineTraceSingleByChannel(
			HitResult,
			StartLocation, 
			EndLocation, ECollisionChannel::ECC_Visibility)
			)
			Distances[i] = (HitResult.Location - LidarLocation).Size() / 100; //divide by 100 to convert cm to meters
		else
			Distances[i] = OutOfRange;
	}
}

void ULidarComponent::Publish()
{
	// Publish the data to the scan topic
	LaserData = new ROSMessages::sensor_msgs::LaserScan();
	LaserData->angle_min = LidarMinDegree*PI/180.f;
	LaserData->angle_max = LidarMaxDegree*PI/180.f;
	LaserData->angle_increment = AngularResolution*PI/180.f;
	LaserData->time_increment = 0.f;
	LaserData->scan_time = 0.f;		// time between scans[seconds]
	LaserData->range_min = 0.021;		// minimum range value[m]
	LaserData->range_max = Range;		// maximum range value[m]
	LaserData->ranges = TArray<float>(Distances, 1081);
	LaserData->intensities = TArray<float>(Distances, 1081);

	TSharedPtr<ROSMessages::sensor_msgs::LaserScan> LaserMessage(LaserData);
	LaserMessage->header.time = FROSTime::Now();
	LaserMessage->header.frame_id = "laser";
	ScanTopic->Publish(LaserMessage);
}
