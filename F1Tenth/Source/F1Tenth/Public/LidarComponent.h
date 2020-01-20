// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "VoronoiDefinitions.h"

#include "CoreMinimal.h"
#include "Components/StaticMeshComponent.h"
#include "LidarComponent.generated.h"

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

public:
	ULidarComponent();
	void Scan(); // Linetrace to sense distances
	void Polylinize(std::vector<segment_type>& OutLineSegments); // Convert raw distances to line segments
	FVector LidarToWorldLocation(point_type point); // Convert a point in Lidar's xy-coordinates to world's coordinate (for visualization)

private:
// Private methods
	bool GetSegment(SegmentFloat& OutSegment, float& OutStartAngle, float StepAngle, float DiscontinuityThreshold);
	bool GetPointAtAngle(PointFloat& OutPoint, float angle_deg); // Calculates the lidar point at angle_deg in Distances[1081] 
	bool GetDistanceAtAngle(float& OutDistance, float angle_deg); // Returns the corresponding distance in Distances[1081] 
	float Distance(PointFloat p0, PointFloat p1);
	float DistanceToLine(PointFloat point, PointFloat p0, PointFloat p1);

private:
// Private properties
	float DiscontinuityThreshold = 0.3; // in meters
	float AngularResolution = 0.25; // 4 measurements per angle
	float Range = 17; // Maximum detectable distance in meters
	float OutOfRange = 65.533; // Value to return if distance > LidarRange
	float LidarMinDegree = -135;
	float LidarMaxDegree = 135; // (LidarMaxDegree - LidarMinDegree)*AngularResolution + 1 = 1081
	float Distances[1081]; // Array of distances
};

