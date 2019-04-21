// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <vector>
#include <array>
#include "F1TenthPawn.h"

#include "CoreMinimal.h"
#include "AIController.h"
#include "DisparityAIController.generated.h"


/**
 * 
 */
UCLASS()
class F1TENTH_API ADisparityAIController : public AAIController
{
	GENERATED_BODY()

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;
	void Scan(); // Linetrace to sense distances
	FVector LidarToWorldLocation(float x, float y);
	
	/* Holds configuration options and some state for controlling the car
	using the simplified obstacle - bloating algorithm I'm calling "disparity
	extending" for now, since I don't know if there's an already established
	term for it. */

private:
	AF1TenthPawn* ControlledVehicle = nullptr;
	FVector LidarLocation;
	FVector LidarXAxis, LidarYAxis, LidarZAxis;
	float AngularResolution = 0.25; // 4 measurements per angle
	float Range = 17; // Maximum detectable distance in meters
	float OutOfRange = 65.533; // Value to return if distance > LidarRange


	/* Initializes the class with default values, for our 1/10 scale
	traxxas car controlled using a FOCBox. */
	// This is actually "half" of the car width, plus some tolerance.
	// Controls the amount disparities are extended by.
	float car_width = 0.5;
	// This is the difference between two successive LIDAR scan points that
	// can be considered a "disparity". (As a note, at 7m there should be
	// ~0.04m between scan points.)
	float disparity_threshold = 0.2;
	// This is the arc width of the full LIDAR scan data, in degrees
	float scan_width = 270.0;
	// This is the radius to the left or right of the car that must be clear
	// when the car is attempting to turn left or right.
	float turn_clearance = 0.3;
	// This is the maximum steering angle of the car, in degrees.
	float max_turn_angle = 34.0;
	// The slowest speed the car will go
	// Good value here is 0.1
	float min_speed = 0.1;
	// The maximum speed the car will go(the absolute max for the motor is
	// 0.5, which is *very* fast). 0.15 is a good max for slow testing.
	float max_speed = 0.6; // 0.35;
	float absolute_max_speed = 0.68; // 0.499;
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
	// If forward distance is lower than this, use the alternative method
	// for choosing an angle(where the angle favors a disparity rather than
	// simple distance)
	float no_u_distance = 4.0;
	// These allow us to adjust the angle of points to consider based on how
	// the wheels are oriented.
	float min_considered_angle = -89.0;
	float max_considered_angle = 89.0;
	// We'll use this lock to essentially drop LIDAR packets if we're still
	// processing an older one.
	//float lock = threading.Lock()
	//float should_stop = False
	//float total_packets = 0
	//float dropped_packets = 0
		
	// This is just a field that will hold the LIDAR distances.
	std::array<float, 1081> lidar_distances;
	// This contains the LIDAR distances, but only "safe" reachable
	// distances, generated by extending disparities.
	std::array<float, 1081> masked_disparities;
	// If we're constraining turns to be in the direction of a disparity,
	// then use this array to find indices of possible directions to move.
	std::vector<size_t> possible_disparity_indices;
	// This field will hold the number of LIDAR samples per degree.
	// Initialized when we first get some LIDAR scan data.
	float samples_per_degree = 0;



public:
	std::vector<size_t> find_disparities();
	size_t half_car_samples_at_distance(float distance);
	void extend_disparities();
	float angle_from_index(size_t i);
	size_t index_from_angle(float theta);
	bool find_widest_disparity_index(size_t& Out_max_disparity_index);
	void find_new_angle(float& OutDistance, float& OutAngle);
	float scale_speed_linearly(float speed_low, float speed_high, float distance,
		float distance_low, float distance_high);
	float duty_cycle_from_distance(float distance);
	float degrees_to_steering_percentage(float degrees);
	float adjust_angle_for_car_side(float target_angle);
	float adjust_angle_to_avoid_uturn(float target_angle, float forward_distance);
	void update_considered_angle(float steering_angle);
	void lidar_callback();
};

