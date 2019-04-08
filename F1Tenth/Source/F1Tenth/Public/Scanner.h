// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Scanner.generated.h"

struct Point {
	float x;
	float y;
	Point(float x0, float y0) : x(x0), y(y0) {}
};

struct Segment {
	Point p0;
	Point p1;
	Segment(float x1, float y1, float x2, float y2) : p0(x1, y1), p1(x2, y2) {}
};

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class F1TENTH_API UScanner : public UActorComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UScanner();

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
	void Scan(); // Linetrace to sense distances
	void Polylinize(); // Converty raw distances to line segments
	bool GetDistanceAtAngle(float& OutDistance, float angle_deg); // Returns the corresponding distance in Distances[1081] 
	bool GetPointAtAngle(Point& OutPoint, float angle_deg); // Calculates the lidar point at angle_deg in Distances[1081] 
	bool GetSegment(Segment& OutSegment, float& OutStartAngle, float StepAngle, float DiscontinuityThreshold);
	FVector LidarToWorldLocation(Point point); // Convert a point in Lidar's xy-coordinates to world's coordinate (for visualization)
	float Distance(Point p0, Point p1);
	float DistanceToLine(Point point, Point p0, Point p1);

	float Distances[1081]; // Array of distances
	float AngularResolution = 0.25; // 4 measurements per angle
	float Range = 17; // Maximum detectable distance in meters
	float OutOfRange = 65.533; // Value to return if distance > LidarRange
};

