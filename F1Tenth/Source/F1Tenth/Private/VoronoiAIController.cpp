// Fill out your copyright notice in the Description page of Project Settings.

#include "../Public/VoronoiAIController.h"
#include "LidarComponent.h"
#include "voronoi_visual_utils.hpp"

#include "Runtime/Engine/Public/DrawDebugHelpers.h"
#include "Runtime/Engine/Classes/Engine/World.h"

void AVoronoiAIController::BeginPlay()
{
	Super::BeginPlay();
	
	ControlledVehicle = Cast<AF1TenthPawn>(GetPawn());
	if (!ControlledVehicle)
	{
		UE_LOG(LogTemp, Warning, TEXT("VoronoiAIController not possesing a vehicle!"));
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("VoronoiAIController possesing: %s"), *(ControlledVehicle->GetName()));

		// Connect to vehicle's lidar sensor
		TArray<ULidarComponent*> Lidars;
		ControlledVehicle->GetComponents<ULidarComponent>(Lidars);
		if (Lidars.Num() == 1)
		{
			Lidar = Lidars[0];
			UE_LOG(LogTemp, Warning, TEXT("VoronoiAIController connected to lidar sensor."));
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("VoronoiAIController did not find lidar sensor!"));
		}
	}
}


void AVoronoiAIController::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// Make a set of polylines out of lidar 2D point cloud.
	Lidar->Polylinize(Walls, discontinuity_threshold);

	// Get the plan as list of line segments and draw it
	Planner.MakeRoadmap(Walls, allowed_obs_dist);

	std::vector<point_type> Plan;
	Planner.GetPlan(Plan, Walls);

	point_type rear_axle = RearAxleToLidar(point_type(0.f, 0.f)); // Coordinates of rear axle in lidar's frame
	point_type PurePursuitGoal; // Coordinates of the goal point in rear axle's frame
	if (Plan.size() == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("No plan found!"));
	}
	else if (euclidean_distance(Plan[0], rear_axle) >= PurepursuitLookahead)
	{
		UE_LOG(LogTemp, Warning, TEXT("Plan does not start inside of lookahead circle!"));
		PurePursuitGoal = LidarToRearAxle(Plan[0]);
	}
	else if (euclidean_distance(Plan[Plan.size() - 1], rear_axle) <= PurepursuitLookahead)
	{
		UE_LOG(LogTemp, Warning, TEXT("Plan does not end outside of lookahead circle!"));
		PurePursuitGoal = LidarToRearAxle(Plan[Plan.size() - 1]);
	}
	else
	{
		PurePursuitGoal = get_plan_at_lookahead(Plan);
	}
	float steering_ratio = -pure_pursuit(PurePursuitGoal);
	ControlledVehicle->MoveRight(steering_ratio);
	ControlledVehicle->MoveForward(0.45);

	// Visualizations
	DrawLaser();
	DrawWalls();
	DrawRoadmap();
	DrawPlan(Plan);
	// visualize the purepusuit goalpoint
	DrawDebugSphere(GetWorld(), LidarToWorldLocation(RearAxleToLidar(PurePursuitGoal)),
		9.f, 5.f, FColor(100, 10, 10), false, 0.f, 30.f, 2.2f);
	// Draw circle corresponding to pure_pursuit lookahead distance (to rear axle)
	DrawDebugCircle(GetWorld(),
		LidarToWorldLocation(RearAxleToLidar(point_type(0.f, 0.f))),
		PurepursuitLookahead*100.f, 72, FColor(0, 0, 0), false, 0.f, 30, 2.f, FVector(0, 1, 0), FVector(1, 0, 0));
}

FVector AVoronoiAIController::LidarToWorldLocation(const point_type& point)
{
	FVector LidarLocation = Lidar->GetComponentLocation();
	FVector LidarXAxis = Lidar->GetForwardVector();
	FVector LidarYAxis = -Lidar->GetRightVector();
	return LidarLocation + LidarXAxis * point.x() * 100 + LidarYAxis * point.y() * 100;
}

void AVoronoiAIController::DrawLaser()
{
	std::vector<point_type> points;
	Lidar->GetLidarData(points);
	for (const auto& point : points)
	{
		double distance = euclidean_distance(point, point_type(0.f, 0.f))*5.f;
		point_type End = point;
		point_type Start = point_type(point.x()/distance, point.y()/distance);
		DrawDebugLine(GetWorld(), 
			LidarToWorldLocation(Start), 
			LidarToWorldLocation(End), 
			FColor(255, 0, 0), false, 0.f, 0.f, 0.f);
	}	
}

void AVoronoiAIController::DrawWalls()
{
	for (auto& wall : Walls)
	{
		DrawDebugLine(
			GetWorld(),
			LidarToWorldLocation(wall.low()),
			LidarToWorldLocation(wall.high()), FColor(0, 255, 0), false, 0.f, 1.f, 10.f);
	}
}

void AVoronoiAIController::DrawRoadmap()
{
	std::list<point_type> points;
	Planner.GetRoadmapPoints(points);
	for (const point_type& point : points)
	{
		DrawDebugSphere(GetWorld(), LidarToWorldLocation(point),
			15.f, 5.f, FColor(0, 0, 0), false, 0.f, 10.f, 1.f);
	}
	std::vector<segment_type> segments;
	Planner.GetRoadmapSegments(segments);
	for (const segment_type& segment : segments)
	{
		DrawDebugLine(GetWorld(), LidarToWorldLocation(segment.low()), LidarToWorldLocation(segment.high()),
			FColor(64, 64, 255), false, 0.f, 5.f, 10.f);
	}
}

void AVoronoiAIController::DrawPlan(std::vector<point_type>& Plan)
{
	if (Plan.size() > 1)
	{
		for (auto si = Plan.begin(); si != Plan.end() - 1; ++si)
		{
			DrawDebugLine(GetWorld(),
				LidarToWorldLocation(*si), LidarToWorldLocation(*(si + 1)), 
				FColor(255, 255, 255), false, 0.f, 20.f, 7.f);
		}
	}
}

/// Assumes that Plan starts inside the lookahead circle and ends outside.
/// The returned value is in rear_axle's coordinates.
point_type AVoronoiAIController::get_plan_at_lookahead(const std::vector<point_type>& Plan)
{
	point_type rear_axle = RearAxleToLidar(point_type(0.f, 0.f)); // Plan is in lidar's coordinates
	size_t end_index = 1;
	while (euclidean_distance(Plan[end_index], rear_axle) <= PurepursuitLookahead)
		++end_index;
	point_type p_in = LidarToRearAxle(Plan[end_index - 1]);
	point_type p_out = LidarToRearAxle(Plan[end_index]);
	double x1 = p_in.x(); 
	double y1 = p_in.y();
	double x2 = p_out.x();
	double y2 = p_out.y();
	double dx = x2 - x1;
	double dy = y2 - y1;
	double A = dx * dx + dy * dy;
	double B = x1 * dx + y1 * dy;
	double C = x1 * x1 + y1 * y1 - PurepursuitLookahead * PurepursuitLookahead;
	double t = (-B + sqrt(B*B - A*C)) / A;
	return point_type(x1 + t * dx, y1 + t * dy);
}

float AVoronoiAIController::pure_pursuit(point_type goal_point)
{
	// goal_point is in rear axle's coordinates.
	// goal_point must have a positive x coordinate.

	double d = euclidean_distance(goal_point, point_type(0.f, 0.f));
	double d2 = d*d;
	double steering_angle_rad = atan(2 * wheelbase * goal_point.y() / d2);
	double steering_angle_deg = steering_angle_rad * 180.f / PI;

		// The wheels cannot physically turn more than 34 degrees
	if (steering_angle_deg < -max_turn_degrees)
	{
		steering_angle_deg = -max_turn_degrees;
	}
	else if (steering_angle_deg > max_turn_degrees)
	{
		steering_angle_deg = max_turn_degrees;
	}
	// The steering command is in percentages
	return steering_angle_deg/max_turn_degrees;
}

/// Assumes that orientation of lidar's frame and the rear axle's frame are the same
point_type AVoronoiAIController::LidarToRearAxle(const point_type& point)
{
	FVector Scale = ControlledVehicle->GetActorScale3D();
	FVector LidarInVehicle = Scale * (Lidar->GetRelativeTransform().GetLocation()) / 100.f;
	FVector VehicleInRearAxle = 0.5f*wheelbase*FVector(1.f, 0.0, 0.0);
	FVector LidarInRearAxle = LidarInVehicle + VehicleInRearAxle;
	return point_type(point.x() + LidarInRearAxle.X, point.y() + LidarInRearAxle.Y);
}

/// Assumes that orientation of lidar's frame and the rear axle's frame are the same
point_type AVoronoiAIController::RearAxleToLidar(const point_type& point)
{
	FVector Scale = ControlledVehicle->GetActorScale3D();
	FVector LidarInVehicle = Scale*(Lidar->GetRelativeTransform().GetLocation()) / 100.f;
	FVector VehicleInRearAxle = 0.5f*wheelbase*FVector(1.f, 0.0, 0.0);
	FVector LidarInRearAxle = LidarInVehicle + VehicleInRearAxle;
	return point_type(point.x() - LidarInRearAxle.X, point.y() - LidarInRearAxle.Y);
}
