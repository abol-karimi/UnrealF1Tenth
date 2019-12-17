// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "VoronoiDefinitions.h"
#include <set>

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

	// Lidar Methods
	//void Scan(); // Linetrace to sense distances
	//void Polylinize(); // Converty raw distances to line segments
	//bool GetDistanceAtAngle(float& OutDistance, float angle_deg); // Returns the corresponding distance in Distances[1081] 
	//bool GetPointAtAngle(PointFloat& OutPoint, float angle_deg); // Calculates the lidar point at angle_deg in Distances[1081] 
	//bool GetSegment(SegmentFloat& OutSegment, float& OutStartAngle, float StepAngle, float DiscontinuityThreshold);
	//float Distance(PointFloat p0, PointFloat p1);
	//float DistanceToLine(PointFloat point, PointFloat p0, PointFloat p1);


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
	float distance_to_purepursuit_goal = 2.0; // Distance (in meters) between the rear axel and the goal point

	// Vehicle properties
	float wheelbase = 0.3; // Distance (in meters) of rear axle to front axel
	float max_turn_degrees = 34;

	// Lidar properties
	float DiscontinuityThreshold = 0.7; // cm
	FVector LidarLocation;
	FVector LidarXAxis, LidarYAxis, LidarZAxis;
	float AngularResolution = 0.25; // 4 measurements per angle
	float Range = 17; // Maximum detectable distance in meters
	float OutOfRange = 65.533; // Value to return if distance > LidarRange
	float LidarMinDegree = -135;
	float LidarMaxDegree = 135; // (LidarMaxDegree - LidarMinDegree)*AngularResolution + 1 = 1081
	float Distances[1081]; // Array of distances

	// VoronoiGraph properties
	VD VDiagram;
	std::vector<point_type> VDPoints;
	std::vector<segment_type> VDInputLineSegments;
};