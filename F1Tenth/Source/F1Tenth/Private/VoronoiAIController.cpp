// Fill out your copyright notice in the Description page of Project Settings.

#include "../Public/VoronoiAIController.h"
#include "PathMaker.h"
#include "LidarComponent.h"

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

	// TODO: Skip frame if Deltatime < 25ms (frequency of lidar is 40Hz)

	LidarLocation = Lidar->GetComponentLocation();
	LidarXAxis = Lidar->GetForwardVector();
	LidarYAxis = -Lidar->GetRightVector();
	LidarZAxis = Lidar->GetUpVector();
	// Linetrace to gather lidar measurements
	Lidar->Scan();

	// Make a set of polylines out of lidar 2D point cloud
	Lidar->Polylinize(VDInputLineSegments);

	// Make a voronoi diagram
	VDiagram.clear();
	construct_voronoi(VDInputLineSegments.begin(), VDInputLineSegments.end(), &VDiagram);

	// Visualize the voronoi diagram (stored in vd_)
	DrawVD();

	// Get the discontinuity midpoint with the least angle from the x-axis (vehicle forward direction)
	point_type track_opening;
	point_type PurePursuitGoal;
	float steering_ratio = 0.f;
	if (!get_trackopening(track_opening, DiscontinuityThreshold*1000)) // convert to mm
	{
		UE_LOG(LogTemp, Warning, TEXT("No discontinuity found!"));
	}
	else if (!get_purepursuit_goal(PurePursuitGoal, track_opening))
	{
		UE_LOG(LogTemp, Warning, TEXT("Discontinuity found, but no pure_pursuit goal!"));
	}
	else
	{
		// visualize the pure_pusuit goalpoint
		DrawDebugSphere(GetWorld(), LidarToWorldLocation(PurePursuitGoal),
			9.f, 5.f, FColor(100, 10, 10), false, 0.f, 30.f, 2.2f);
		steering_ratio = -pure_pursuit(PurePursuitGoal);
		ControlledVehicle->MoveRight(steering_ratio);
	}
	//prev_steering_ratio
	// Draw circle corresponding to pure_pursuit lookahead distance (to rear axle)
	DrawDebugCircle(GetWorld(),
		ControlledVehicle->GetActorLocation() - 0.5f*wheelbase*100.f*ControlledVehicle->GetActorForwardVector(),
		distance_to_purepursuit_goal*100.f, 36, FColor(0, 0, 0), false, 0.f, 30, 2.f, FVector(0, 1, 0), FVector(1, 0, 0));

	//UE_LOG(LogTemp, Warning, TEXT("Steering ratio: %f"), steering_ratio);
	ControlledVehicle->MoveForward(0.45);
	
}

FVector AVoronoiAIController::LidarToWorldLocation(point_type point)
{
	FVector LocationInLidar = FVector(point.x() * 100, point.y() * 100, 0); // *100 to convert to cm
	return LidarLocation + LidarXAxis * point.x() * 100 + LidarYAxis * point.y() * 100;
}


void AVoronoiAIController::sample_curved_edge(const edge_type& edge, std::vector<point_type>* sampled_edge)
{
	coordinate_type max_dist = 300; // in milimeters
	point_type point = edge.cell()->contains_point() ? retrieve_point(*edge.cell()) : retrieve_point(*edge.twin()->cell());
	segment_type segment = edge.cell()->contains_point() ? retrieve_segment(*edge.twin()->cell()) : retrieve_segment(*edge.cell());
	voronoi_visual_utils<coordinate_type>::discretize(point, segment, max_dist, sampled_edge);
}

point_type AVoronoiAIController::retrieve_point(const cell_type& cell)
{
	source_index_type index = cell.source_index();
	source_category_type category = cell.source_category();
	if (category == SOURCE_CATEGORY_SINGLE_POINT) {
		return VDPoints[index];
	}
	index -= VDPoints.size();
	if (category == SOURCE_CATEGORY_SEGMENT_START_POINT) {
		return low(VDInputLineSegments[index]);
	}
	else {
		return high(VDInputLineSegments[index]);
	}
}

segment_type AVoronoiAIController::retrieve_segment(const cell_type& cell) {
	source_index_type index = cell.source_index() - VDPoints.size();
	return VDInputLineSegments[index];
}

void AVoronoiAIController::DrawVD()
{
	for (const_edge_iterator it = VDiagram.edges().begin(); it != VDiagram.edges().end(); ++it)
	{
		if (it->is_finite() && it->is_primary())
		{
			if (isObstacle(point_type(it->vertex0()->x(), it->vertex0()->y())))
				continue;
			else if (isObstacle(point_type(it->vertex1()->x(), it->vertex1()->y())))
				continue;
			if (it->is_linear())
			{
				point_type vertex0(it->vertex0()->x() / 1000.f, it->vertex0()->y() / 1000.f);
				point_type vertex1(it->vertex1()->x() / 1000.f, it->vertex1()->y() / 1000.f);
				DrawDebugLine(GetWorld(), LidarToWorldLocation(vertex0), LidarToWorldLocation(vertex1), FColor(64, 64, 255), false, 0.f, 5.f, 10.f);
			}
			else if (it->is_curved())
			{
				point_type vertex0(it->vertex0()->x(), it->vertex0()->y());
				point_type vertex1(it->vertex1()->x(), it->vertex1()->y());
				std::vector<point_type> samples;
				samples.push_back(vertex0);
				samples.push_back(vertex1);
				sample_curved_edge(*it, &samples);
				for (std::size_t i = 0; i + 1 < samples.size(); ++i)
				{
					point_type sample_i(samples[i].x() / 1000.f, samples[i].y() / 1000.f);
					point_type sample_ii(samples[i + 1].x() / 1000.f, samples[i + 1].y() / 1000.f);
					DrawDebugLine(GetWorld(), LidarToWorldLocation(sample_i), LidarToWorldLocation(sample_ii), FColor(0, 0, 64), false, 0.f, 5.f, 10.f);
				}
			}
		}
	}
	for (const_vertex_iterator it = VDiagram.vertices().begin(); it != VDiagram.vertices().end(); ++it)
	{
		if (true/*!it->is_degenerate()*/)
		{
			point_type vertex(it->x() / 1000.f, it->y() / 1000.f);
			DrawDebugSphere(GetWorld(), LidarToWorldLocation(vertex),
				15.f, 5.f, FColor(0, 0, 0), false, 0.f, 10.f, 1.f);
		}
	}

}


bool AVoronoiAIController::get_trackopening(point_type& OutTrackOpening, double min_gap) // min_gap is in millimeters
{
	std::vector<point_type> discontinuities;
	double max_cos = -1; // The angle behind the car has cos=-1
	int max_cos_index;
	for (std::size_t i = 0; i + 1 < VDInputLineSegments.size(); ++i)
	{
		if (euclidean_distance(VDInputLineSegments[i].high(), VDInputLineSegments[i + 1].low()) > min_gap)
		{
			point_type endpoint = VDInputLineSegments[i].high();
			convolve(endpoint, VDInputLineSegments[i + 1].low()); // add the seond point to the first
			point_type midpoint = scale_down(endpoint, 2);
			discontinuities.push_back(midpoint);
			DrawDebugSphere(GetWorld(),
				LidarToWorldLocation(point_type(midpoint.x() / 1000.f, midpoint.y() / 1000.f)),
				10.f, 10.f, FColor(255, 255, 255), false, 0.f, 0.f, 1.f);
			double cos = midpoint.x() / euclidean_distance(midpoint, point_type(0, 0));
			//UE_LOG(LogTemp, Warning, TEXT("i: %d, x: %f, y:%f"), i, midpoint.x(), midpoint.y());
			if (cos > max_cos)
			{
				max_cos_index = discontinuities.size() - 1; // current discontinuity is closest to front of the car
				max_cos = cos;
			}
		}
	}
	if (discontinuities.size() > 0)
	{
		OutTrackOpening = discontinuities[max_cos_index];
		DrawDebugSphere(GetWorld(),
			LidarToWorldLocation(point_type(OutTrackOpening.x() / 1000.f, OutTrackOpening.y() / 1000.f)),
			15.f, 10.f, FColor(255, 255, 0), false, 0.f, 15.f, 1.f);
		return true;
	}
	else { return false; }
}

bool AVoronoiAIController::get_closest_vertex(std::size_t& OutIndex, point_type point)
{
	if (VDiagram.vertices().size() == 0)
	{
		return false;
	}
	// If vd_vertics() is nonempty, then there must be a closest nonobstacle vertex
	double closest_distance = 1e10; // infinity
	std::size_t current_index = 0;
	for (const_vertex_iterator it = VDiagram.vertices().begin(); it != VDiagram.vertices().end(); ++it)
	{
		if (isObstacle(point_type(it->x(), it->y()))) // candid_point is an endpoint of an input segment
		{
			current_index++;
			continue;
		}

		double candid_distance = euclidean_distance(point_type(it->x(), it->y()), point);
		if (candid_distance < closest_distance)
		{
			closest_distance = candid_distance;
			OutIndex = current_index;
		}
		current_index++;
	}
	return true;
}


bool AVoronoiAIController::get_closest_front_vertex(std::size_t& OutIndex, point_type point)
{
	if (VDiagram.vertices().size() == 0)
	{
		return false;
	}
	// If vd_vertics() is nonempty, then there must be a closest nonobstacle vertex
	double closest_distance = 1e10; // infinity
	std::size_t current_index = 0;
	for (const_vertex_iterator it = VDiagram.vertices().begin(); it != VDiagram.vertices().end(); ++it)
	{
		if (it->x() + wheelbase <= 0) // Ignore points that are behind the rear axle
		{
			current_index++;
			continue;
		}
		if (isObstacle(point_type(it->x(), it->y()))) // candid_point is an endpoint of an input segment
		{
			current_index++;
			continue;
		}

		double candid_distance = euclidean_distance(point_type(it->x(), it->y()), point);
		if (candid_distance < closest_distance)
		{
			closest_distance = candid_distance;
			OutIndex = current_index;
		}
		current_index++;
	}
	return true;
}


bool AVoronoiAIController::get_purepursuit_goal(point_type& OutGoalPoint, point_type track_opening)
{
	// OutGoalPoint is in lidar coordinates

	std::size_t goal_index;
	std::size_t source_index;
	if (get_closest_vertex(goal_index, track_opening) && get_closest_front_vertex(source_index, point_type(0, 0)))
	{
		point_type source_point = point_type(VDiagram.vertices()[source_index].x() / 1000.f, VDiagram.vertices()[source_index].y() / 1000.f);
		point_type goal_point = point_type(VDiagram.vertices()[goal_index].x() / 1000.f, VDiagram.vertices()[goal_index].y() / 1000.f);
		point_type rear_axle(-wheelbase, 0);
		if (euclidean_distance(goal_point, rear_axle) < distance_to_purepursuit_goal)
		{
			OutGoalPoint = goal_point;
			return true;
		}

		DrawDebugSphere(GetWorld(), LidarToWorldLocation(source_point),
			8.f, 5.f, FColor(0, 255, 0), false, 0.f, 15.f, 2.1f);

		DrawDebugSphere(GetWorld(), LidarToWorldLocation(goal_point),
			8.f, 5.f, FColor(0, 255, 0), false, 0.f, 15.f, 2.1f);

		// If reached here, goalpoint from real axel is further than distance_to_purepursuit_goal meters
		PathMaker pmaker(DiscontinuityThreshold);
		pmaker.set_segments(VDInputLineSegments);
		pmaker.set_points(VDPoints);
		std::vector<point_type> path;
		bool found_path = pmaker.get_path(path, VDiagram, source_point, goal_point);
		if (found_path)
		{
			for (std::vector<point_type>::iterator it = path.end() - 1; it != path.begin(); --it)
			{
				// PathMaker gives points in meters
				// UE_LOG(LogTemp, Warning, TEXT("*it: x: %f, y: %f"), (*it).x(), (*it).y());
				DrawDebugLine(GetWorld(), LidarToWorldLocation(*it), LidarToWorldLocation(*(it - 1)), FColor(255, 255, 255), false, 0.f, 20.f, 3.f);
				if ((*it).x() > 0 // point in front of the car
					&& euclidean_distance(*(it - 1), rear_axle) < distance_to_purepursuit_goal) // next point too close.
					// TODO interpolate based on distance instead of giving an endpoint.
				{
					double x1 = (it - 1)->x() + wheelbase; // Convert lidar coordinates to rear axle coordinates (assuming lidar at front axle)
					double y1 = (it - 1)->y();
					double x2 = it->x() + wheelbase;
					double y2 = it->y();
					double dx = x2 - x1;
					double dy = y2 - y1;
					double A = dx * dx + dy * dy;
					double B = x1 * dx + y1 * dy;
					double C = x1 * x1 + y1 * y1 - distance_to_purepursuit_goal * distance_to_purepursuit_goal;
					double t = (-B + sqrt(B*B - A * C)) / A;
					OutGoalPoint = point_type(x1 + t * dx - wheelbase, y1 + t * dy); // Convert back to lidar coordinates
					return true;
				}
			}
			// If reached here, the source_point is not within distance_to_purepursuit_goal meters of rear axle
			double x1 = source_point.x() + wheelbase;
			double y1 = source_point.y();
			double d = sqrt(x1*x1 + y1 * y1);
			OutGoalPoint = point_type(x1 / d * distance_to_purepursuit_goal - wheelbase, y1 / d * distance_to_purepursuit_goal);
			return true; // TODO pushback rear axel to path to avoid this extra case
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}

}


bool AVoronoiAIController::isObstacle(point_type point) // input point in millimeters
{
	for (auto itr = VDInputLineSegments.begin(); itr != VDInputLineSegments.end(); ++itr)
	{
		if (euclidean_distance(point, itr->high()) <= 5 || euclidean_distance(point, itr->low()) <= 5) // 5mm 
		{
			return true;
		}
	}

	return false;
}


float AVoronoiAIController::pure_pursuit(point_type goal_point)
{
	// goalpoint is in lidar's coordinates
	// goalpoint_angle_rad is alpha in CMU's formulation
	// goalpoint_angle_rad must be in(-pi / 2, pi / 2)
	// wheelbase is the distance(in meters) between the rear and front axels
	// distance_to_goalpoint is the distance(in meters) between the rear axel and the goal point
	// distance_to_goalpoint must be positive

	//double goalpoint_angle_rad = atan(goal_point.y() / (goal_point.x()+wheelbase));
	double d2 = distance_to_purepursuit_goal * distance_to_purepursuit_goal;
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
