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
	FVector LidarToWorldLocation(point_type point); // Convert a point in Lidar's xy-coordinates to world's coordinate (for visualization)
	float pure_pursuit(point_type goal_point);

	// VoronoiGraph Methods
	void sample_curved_edge(const edge_type& edge, std::vector<point_type>* sampled_edge);
	point_type retrieve_point(const cell_type& cell);
	segment_type retrieve_segment(const cell_type& cell);
	void DrawVD(); // Pass pose of Lidar
	bool get_trackopening(point_type& OutTrackOpening, double min_gap);
	bool get_closest_vertex(std::size_t& OutIndex, point_type point);
	bool get_purepursuit_goal(point_type& OutGoalPoint, point_type track_opening);
	bool isObstacle(point_type point);
	bool get_closest_front_vertex(std::size_t& OutIndex, point_type point);


private:
	// Controller properties
	AF1TenthPawn* ControlledVehicle = nullptr;
	ULidarComponent* Lidar = nullptr;
	float MinTrackWidth = 0.7; // cm
	float PurepursuitLookahead = 2.0; // Distance (in meters) between the rear axel and the goal point

	// Vehicle properties
	float wheelbase = 0.3; // Distance (in meters) of rear axle to front axel
	float max_turn_degrees = 34;

	// Lidar properties
	FVector LidarLocation;
	FVector LidarXAxis, LidarYAxis, LidarZAxis;

	// VoronoiGraph properties
	VoronoiGraph Planner;

	VD VDiagram;
	std::vector<point_type> VDPoints;
	std::vector<segment_type> Walls;

	
};