// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "VoronoiDefinitions.h"
#include <set>
#include "VoronoiGraph.h"

#include "F1TenthPawn.h"
#include "CoreMinimal.h"
#include "AIController.h"
#include "VoronoiAIController.generated.h"


/**
 * 
 */
UCLASS()
class F1TENTH_API AVoronoiAIController : public AAIController
{
	GENERATED_BODY()

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;

private:
	FVector LidarToWorldLocation(const point_type& point); // Convert a point in Lidar's xy-coordinates to world's coordinate (for visualization)
	point_type LidarToRearAxle(const point_type& point);
	point_type RearAxleToLidar(const point_type& point);
	float pure_pursuit(point_type goal_point);
	void DrawRoadmap(); // TODO Pass pose of Lidar
	void DrawPlan(std::vector<point_type>& Plan);
	point_type get_plan_at_lookahead(const std::vector<point_type>& Plan);

private:
	// Controller properties
	AF1TenthPawn* ControlledVehicle = nullptr;
	ULidarComponent* Lidar = nullptr;
	float MinTrackWidth = 1.5; // in meters
	float PurepursuitLookahead = 1.4; // Distance (in meters) between the rear axel and the goal point

	// Vehicle properties
	float wheelbase = 0.3; // Distance (in meters) of rear axle to front axel
	float max_turn_degrees = 34;

	// VoronoiGraph properties
	VoronoiGraph Planner;
	std::vector<segment_type> Walls;
};