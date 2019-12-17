// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "VoronoiDefinitions.h"

#include "CoreMinimal.h"
#include "Components/StaticMeshComponent.h"
#include "LidarComponent.generated.h"

/**
 * 
 */
UCLASS()
class F1TENTH_API ULidarComponent : public UStaticMeshComponent
{
	GENERATED_BODY()

public:
	ULidarComponent();
	void Scan(); // Linetrace to sense distances
	void Polylinize(std::vector<segment_type>& OutLineSegments); // Convert raw distances to line segments
	bool GetSegment(SegmentFloat& OutSegment, float& OutStartAngle, float StepAngle, float DiscontinuityThreshold);
	bool GetPointAtAngle(PointFloat& OutPoint, float angle_deg); // Calculates the lidar point at angle_deg in Distances[1081] 
	bool GetDistanceAtAngle(float& OutDistance, float angle_deg); // Returns the corresponding distance in Distances[1081] 
	float Distance(PointFloat p0, PointFloat p1);
	float DistanceToLine(PointFloat point, PointFloat p0, PointFloat p1);
	FVector LidarToWorldLocation(point_type point); // Convert a point in Lidar's xy-coordinates to world's coordinate (for visualization)

private:
	float DiscontinuityThreshold = 0.7; // cm
	float AngularResolution = 0.25; // 4 measurements per angle
	float Range = 17; // Maximum detectable distance in meters
	float OutOfRange = 65.533; // Value to return if distance > LidarRange
	float LidarMinDegree = -135;
	float LidarMaxDegree = 135; // (LidarMaxDegree - LidarMinDegree)*AngularResolution + 1 = 1081
	float Distances[1081]; // Array of distances
};
