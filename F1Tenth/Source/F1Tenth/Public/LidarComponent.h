// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

THIRD_PARTY_INCLUDES_START
#include <boost/polygon/segment_data.hpp>
#include <boost/polygon/point_data.hpp>
THIRD_PARTY_INCLUDES_END

#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "ROSIntegration/Public/sensor_msgs/LaserScan.h"

#include "CoreMinimal.h"
#include "Components/StaticMeshComponent.h"
#include "LidarComponent.generated.h"

typedef double coordinate_type;
typedef boost::polygon::point_data<coordinate_type> point_type;
typedef boost::polygon::segment_data<coordinate_type> segment_type;

// TODO: replace with boost types (geometry, polygon, or uBLAS::vector)
struct PointFloat {
	float x;
	float y;
	PointFloat(float x0, float y0) : x(x0), y(y0) {}
};

struct SegmentFloat {
	PointFloat p0;
	PointFloat p1;
	SegmentFloat(float x1, float y1, float x2, float y2) : p0(x1, y1), p1(x2, y2) {}
};

/**
 * 
 */
UCLASS()
class F1TENTH_API ULidarComponent : public UStaticMeshComponent
{
	GENERATED_BODY()

	virtual void BeginPlay() override;
	void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction) override;
	
public:
	ULidarComponent();
	void Scan(); // Linetrace to measure distances
	void Publish(); // Publish the scan to ROS

public:
	float Range = 17; // Maximum detectable distance in meters
	float OutOfRange = 65.533; // Value to return if distance > LidarRange
	float AngularResolution = 0.25; // 4 measurements per degree
	float MinDegree = -135;
	float LidarMaxDegree = 135;

private:
// Private properties
	int Steps = floor((LidarMaxDegree-MinDegree)/AngularResolution) + 1; // e.g. [(135-(-135))/0.25] + 1 = 1081
	TArray<float> Distances;

	UPROPERTY()
	UTopic *ScanTopic;

	//UPROPERTY()
	ROSMessages::sensor_msgs::LaserScan* LaserData;
};

