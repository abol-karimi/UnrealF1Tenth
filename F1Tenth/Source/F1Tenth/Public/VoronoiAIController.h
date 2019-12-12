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

	void Scan(); // Linetrace to sense distances
	void Polylinize(); // Converty raw distances to line segments
	bool GetDistanceAtAngle(float& OutDistance, float angle_deg); // Returns the corresponding distance in Distances[1081] 
	bool GetPointAtAngle(PointFloat& OutPoint, float angle_deg); // Calculates the lidar point at angle_deg in Distances[1081] 
	bool GetSegment(SegmentFloat& OutSegment, float& OutStartAngle, float StepAngle, float DiscontinuityThreshold);
	FVector LidarToWorldLocation(point_type point); // Convert a point in Lidar's xy-coordinates to world's coordinate (for visualization)
	float Distance(PointFloat p0, PointFloat p1);
	float DistanceToLine(PointFloat point, PointFloat p0, PointFloat p1);
	void sample_curved_edge(const edge_type& edge, std::vector<point_type>* sampled_edge);
	point_type retrieve_point(const cell_type& cell);
	segment_type retrieve_segment(const cell_type& cell);
	void DrawVD();
	bool get_trackopening(point_type& OutTrackOpening, double min_gap);
	bool get_closest_vertex(std::size_t& OutIndex, point_type point);
	bool get_purepursuit_goal(point_type& OutGoalPoint, point_type track_opening);
	bool isObstacle(point_type point);
	bool get_closest_front_vertex(std::size_t& OutIndex, point_type point);
	float pure_pursuit(point_type goal_point);
	// Nathan's code:
	float duty_cycle_from_distance(float distance);
	float scale_speed_linearly(float speed_low, float speed_high, float distance, float distance_low, float distance_high);
	
private:
	float DiscontinuityThreshold = 0.7; // cm
	AF1TenthPawn* ControlledVehicle = nullptr;


	FVector LidarLocation;
	FVector LidarXAxis, LidarYAxis, LidarZAxis;

	float Distances[1081]; // Array of distances
	float AngularResolution = 0.25; // 4 measurements per angle
	float Range = 17; // Maximum detectable distance in meters
	float OutOfRange = 65.533; // Value to return if distance > LidarRange
	VD vd_;
	std::vector<point_type> point_data_;
	std::vector<segment_type> segment_data_;
	std::vector<point_type> segment_vertices;
	float wheelbase = 0.3; // Distance (in meters) of rear axle to front axel
	float max_turn_degrees = 34;
	float distance_to_purepursuit_goal = 2.0; // Distance (in meters) between the rear axel and the goal point
	float LidarMinDegree = -135;
	float LidarMaxDegree = 135;
	float prev_steering_ratio = 0;

	// Nathan's variables
	float min_speed = 0.1;
	// The maximum speed the car will go(the absolute max for the motor is
	// 0.5, which is *very* fast). 0.15 is a good max for slow testing.
	float max_speed = 0.5; //.60
	float absolute_max_speed = 0.6; // 0.65
	// The forward distance at which the car will go its minimum speed.
	// If there's not enough clearance in front of the car it will stop.
	float min_distance = 0.35;
	// The forward distance over which the car will go its maximum speed.
	// Any distance between this and the minimum scales the speed linearly.
	float max_distance = 3.0;
	// The forward distance over which the car will go its *absolute
	// maximum* speed.This distance indicates there are no obstacles in
	// the near path of the car.Distance between this and the max_distance
	// scales the speed linearly.
	float no_obstacles_distance = 6.0;
};