// Fill out your copyright notice in the Description page of Project Settings.

#include "VoronoiAIController.h"

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
	}
}


void AVoronoiAIController::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// TODO: Skip frame if Deltatime < 25ms (frequency of lidar is 40Hz)

	LidarXAxis = ControlledVehicle->GetActorForwardVector();
	LidarYAxis = -ControlledVehicle->GetActorRightVector();
	LidarZAxis = ControlledVehicle->GetActorUpVector();
	LidarLocation = ControlledVehicle->GetActorLocation() + LidarXAxis * 15 + LidarZAxis * 16;
	
	// Linetrace to gather lidar measurements
	Scan();

	// Make a set of polylines out of lidar 2D point cloud
	Polylinize();

	// Make a voronoi diagram
	vd_.clear();
	construct_voronoi(segment_data_.begin(), segment_data_.end(), &vd_);

	// Visualize the voronoi diagram (stored in vd_)
	DrawVD();

	// Get the discontinuity midpoint with the least angle from the x-axis (vehicle forward direction)
	point_type track_opening;
	point_type PurePursuitGoal;
	float steering_ratio = 0.f;
	if (!get_trackopening(track_opening, 1500.f)) // minimum 1500mm gap
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
			9.f, 5.f, FColor(100, 10, 10), false, 0.f, 0.f, 1.f);
		steering_ratio = -pure_pursuit(PurePursuitGoal);
		ControlledVehicle->MoveRight(steering_ratio);
	}
	//prev_steering_ratio
	// Draw circle corresponding to pure_pursuit lookahead distance (to rear axle)
	DrawDebugCircle(GetWorld(), LidarToWorldLocation(point_type(-wheelbase, 0)),
		distance_to_purepursuit_goal*100.f, 36, FColor(0, 0, 0), false, 0.f, 0, 2.f, FVector(0, 1, 0), FVector(1, 0, 0));

	//UE_LOG(LogTemp, Warning, TEXT("Steering ratio: %f"), steering_ratio);
	float forward_distance = 0;
	GetDistanceAtAngle(forward_distance, 0);
	float throttle = duty_cycle_from_distance(forward_distance);
	ControlledVehicle->MoveForward(throttle);
	
}


void AVoronoiAIController::Scan()
{
	if (!ControlledVehicle)
	{
		return;
	}
	FHitResult HitResult;
	FVector HitLocation;
	for (int i = 0; i < 1081; i++)
	{
		float MeasuringAngle = -135 + i * AngularResolution;
		const FRotator Rot(0, -MeasuringAngle, 0);
		FVector MeasuringDirection = Rot.RotateVector(LidarXAxis);
		auto EndLocation = LidarLocation + MeasuringDirection * Range * 100; // *100 to convert to cm

		if (GetWorld()->LineTraceSingleByChannel(
			HitResult,
			LidarLocation,
			EndLocation,
			ECollisionChannel::ECC_Visibility)
			)
		{
			HitLocation = HitResult.Location;
			if (MeasuringAngle >= -135 && MeasuringAngle <= 135) { // To control visualization range
				DrawDebugLine(GetWorld(), LidarLocation, HitLocation, FColor(255, 0, 0), false, 0.f, 0.f, 0.f);
			}
			Distances[i] = (HitLocation - LidarLocation).Size() / 100; //divide by 100 to convert cm to meters
		}
		else
		{
			Distances[i] = OutOfRange;
		}
	}

}


void AVoronoiAIController::Polylinize()
{
	segment_data_.clear();
	segment_vertices.clear();

	SegmentFloat NewSegment(0, 0, 10, 10);
	float NewStartAngle = -135;
	float DiscontinuityThreshold = 1; // Unit is meters.
	float StepAngle = 2; // Unit is degrees.
	while (NewStartAngle < 135)
	{
		// Search for segments counterclockwise starting from NewStartAngle.
		// GetSegment() updates NewStartAngle for the next search.
		bool FoundNewSegment = GetSegment(NewSegment, NewStartAngle, StepAngle, DiscontinuityThreshold);
		if (FoundNewSegment)
		{
			// Convert SegmentFloat to segment_type, and meters to millimeters
			double x1, y1, x2, y2;
			x1 = NewSegment.p0.x;
			y1 = NewSegment.p0.y;
			x2 = NewSegment.p1.x;
			y2 = NewSegment.p1.y;
			DrawDebugLine(
				GetWorld(),
				LidarToWorldLocation(point_type(x1, y1)),
				LidarToWorldLocation(point_type(x2, y2)), FColor(0, 255, 0), false, 0.f, 0.f, 5.f);
			x1 *= 1000.f;
			y1 *= 1000.f;
			x2 *= 1000.f;
			y2 *= 1000.f;
			segment_data_.push_back(segment_type(point_type(x1, y1), point_type(x2, y2))); // TODO what lp and hp? Any requiremtns on the order of points?
			segment_vertices.push_back(point_type(x1, y1));
			segment_vertices.push_back(point_type(x2, y2));
		}
	}
}


bool AVoronoiAIController::GetDistanceAtAngle(float& OutDistance, float angle_deg)
{
	// Linearly interpolate between (-135, 0) and (135, 1080) on the angle-index coordinates
	// Slope is (1080-0)/(135-(-135)) = 1080/270
	// Index-intercept is 1080/2 = 540
	// Interpolation function is slope*angle_deg + intercept
	if (angle_deg < LidarMinDegree || angle_deg > LidarMaxDegree)
	{
		return false;
	}
	int index = static_cast<int>(1080 * angle_deg / 270 + 540);
	float Distance = Distances[index];
	if (Distance < OutOfRange)
	{
		OutDistance = Distance;
		return true;
	}
	else { return false; }
}


bool AVoronoiAIController::GetPointAtAngle(PointFloat& OutPoint, float angle_deg)
{
	float angle_rad = angle_deg * PI / 180.0;
	float Distance;
	if (GetDistanceAtAngle(Distance, angle_deg))
	{
		OutPoint.x = Distance * cos(angle_rad);
		OutPoint.y = Distance * sin(angle_rad);
		return true;
	}
	else { return false; } // Out of range distance
}


bool AVoronoiAIController::GetSegment(SegmentFloat& OutSegment, float& OutStartAngle, float StepAngle, float DiscontinuityThreshold)
{
	PointFloat StartPoint(0, 0);
	float StartAngle = OutStartAngle;
	//UE_LOG(LogTemp, Warning, TEXT("Starting at angle: %f"), StartAngle);
	while (!GetPointAtAngle(StartPoint, StartAngle))
	{
		StartAngle += StepAngle;
		if (StartAngle > 135)
		{
			// OutStartAngle can be used for the next call to GetSegment. Here it is already out of bounds.
			OutStartAngle = StartAngle;
			return false; // No segment found
		}
	}
	PointFloat InterPoint(0, 0);
	float InterAngle = StartAngle + StepAngle; // Angle of the intermediate point of the segment (if any).
	if (!GetPointAtAngle(InterPoint, InterAngle)) // TODO: Report discontinuity
	{
		// No point at InterAngle, so skip it for the next call to GetSegment().
		OutStartAngle = InterAngle + StepAngle;
		//UE_LOG(LogTemp, Warning, TEXT("!GetPointAtAngle(InterPoint, InterAngle)"));
		return false; // No segment found
	}
	if (Distance(StartPoint, InterPoint) > DiscontinuityThreshold) // TODO Report discontinuity
	{
		// Setup the next search from the beginning of the discontinuity.
		OutStartAngle = InterAngle;
		//UE_LOG(LogTemp, Warning, TEXT("Distance(StartPoint, InterPoint) > DiscontinuityThreshold"));
		return false; // No segment found
	}

	// If reached here, a segment exists starting at StartPoint and passing through SegmentInterPoint.
	// Now search how much the track extends along this segment.
	float EndAngle = InterAngle;
	PointFloat EndPoint = InterPoint;
	float CandidEndAngle = EndAngle;
	PointFloat CandidEndPoint = EndPoint;
	// While-loop invariant:
	//  If EndAngle and EndPoint are valid before the loop,
	//  then they are valid after the loop.
	while (CandidEndAngle + StepAngle <= 135)
	{
		CandidEndAngle += StepAngle;
		bool FoundNewPoint = GetPointAtAngle(CandidEndPoint, CandidEndAngle);
		if (!FoundNewPoint) // Point at CandidEndAngle is OutOfRange TODO: Report discontinuity.
		{
			// Return with the current EndPoint and EndAngle
			OutSegment.p0 = StartPoint;
			OutSegment.p1 = EndPoint;
			OutStartAngle = CandidEndAngle + StepAngle; // Skip the OutOfRange angle for next call to GetSegment()
			//UE_LOG(LogTemp, Warning, TEXT("!FoundNewPoint"));
			return true; // Found a segment
		}
		else if (Distance(EndPoint, CandidEndPoint) > DiscontinuityThreshold) // TODO Report discontinuity
		{
			// Finalize the segment with the current EndAngle and EndPoint
			OutSegment.p0 = StartPoint;
			OutSegment.p1 = EndPoint;
			OutStartAngle = CandidEndAngle; // Skip the discontinuity
			//UE_LOG(LogTemp, Warning, TEXT("Distance(EndPoint, CandidEndPoint) > DiscontinuityThreshold"));
			return true; // Found a segment
		}
		else if (DistanceToLine(CandidEndPoint, StartPoint, InterPoint) > 0.4)
		{
			// Finalize the segment with the current EndAngle and EndPoint
			OutSegment.p0 = StartPoint;
			OutSegment.p1 = EndPoint;
			OutStartAngle = EndAngle;
			//UE_LOG(LogTemp, Warning, TEXT("DistanceToLine(CandidEndPoint, StartPoint, InterPoint) > 0.4"));
			return true; // Found a segment
		}
		else // CandidEndPoint is close enough to the interpolated line (imposing curvature threshold)
		{
			// Extend the segment to CandidEndPoint
			EndAngle = CandidEndAngle;
			EndPoint = CandidEndPoint;
		}
	}
	OutSegment.p0 = StartPoint;
	OutSegment.p1 = EndPoint;
	OutStartAngle = CandidEndAngle + StepAngle;
	return true;
}


FVector AVoronoiAIController::LidarToWorldLocation(point_type point)
{
	FVector LocationInLidar = FVector(point.x() * 100, point.y() * 100, 0); // *100 to convert to cm
	return LidarLocation + LidarXAxis * point.x() * 100 + LidarYAxis * point.y() * 100;
}

float AVoronoiAIController::Distance(PointFloat p0, PointFloat p1)
{
	float DeltaX = p1.x - p0.x;
	float DeltaY = p1.y - p0.y;
	return sqrt(DeltaX * DeltaX + DeltaY * DeltaY);
}

// https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points
// Calculate distance of p0 to the line passing through p0 and p1
float AVoronoiAIController::DistanceToLine(PointFloat point, PointFloat p0, PointFloat p1)
{
	float delta_y = p1.y - p0.y;
	float delta_x = p1.x - p0.x;
	float denominator = Distance(p0, p1);
	float numerator_const_term = p1.x * p0.y - p1.y * p0.x;
	float numerator = abs(delta_y * point.x - delta_x * point.y + numerator_const_term);
	return (numerator / denominator);
}

void AVoronoiAIController::sample_curved_edge(const edge_type& edge, std::vector<point_type>* sampled_edge)
{
	coordinate_type max_dist = 10;
	point_type point = edge.cell()->contains_point() ? retrieve_point(*edge.cell()) : retrieve_point(*edge.twin()->cell());
	segment_type segment = edge.cell()->contains_point() ? retrieve_segment(*edge.twin()->cell()) : retrieve_segment(*edge.cell());
	voronoi_visual_utils<coordinate_type>::discretize(point, segment, max_dist, sampled_edge);
}

point_type AVoronoiAIController::retrieve_point(const cell_type& cell)
{
	source_index_type index = cell.source_index();
	source_category_type category = cell.source_category();
	if (category == SOURCE_CATEGORY_SINGLE_POINT) {
		return point_data_[index];
	}
	index -= point_data_.size();
	if (category == SOURCE_CATEGORY_SEGMENT_START_POINT) {
		return low(segment_data_[index]);
	}
	else {
		return high(segment_data_[index]);
	}
}

segment_type AVoronoiAIController::retrieve_segment(const cell_type& cell) {
	source_index_type index = cell.source_index() - point_data_.size();
	return segment_data_[index];
}

void AVoronoiAIController::DrawVD()
{
	for (const_edge_iterator it = vd_.edges().begin(); it != vd_.edges().end(); ++it)
	{
		if (it->is_finite() && it->is_primary())
		{
			if (it->is_linear())
			{
				point_type vertex0(it->vertex0()->x() / 1000.f, it->vertex0()->y() / 1000.f);
				point_type vertex1(it->vertex1()->x() / 1000.f, it->vertex1()->y() / 1000.f);
				DrawDebugLine(GetWorld(), LidarToWorldLocation(vertex0), LidarToWorldLocation(vertex1), FColor(0, 0, 255), false, 0.f, 0.f, 5.f);
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
					DrawDebugLine(GetWorld(), LidarToWorldLocation(sample_i), LidarToWorldLocation(sample_ii), FColor(0, 0, 255), false, 0.f, 0.f, 5.f);
				}
			}
		}
	}
	for (const_vertex_iterator it = vd_.vertices().begin(); it != vd_.vertices().end(); ++it)
	{
		if (true/*!it->is_degenerate()*/)
		{
			point_type vertex(it->x() / 1000.f, it->y() / 1000.f);
			DrawDebugSphere(GetWorld(), LidarToWorldLocation(vertex),
				5.f, 5.f, FColor(0, 0, 0), false, 0.f, 0.f, 1.f);
		}
	}

}


bool AVoronoiAIController::get_trackopening(point_type& OutTrackOpening, double min_gap) // min_gap is in millimeters
{
	std::vector<point_type> discontinuities;
	double max_cos = -1; // The angle behind the car has cos=-1
	int max_cos_index;
	for (std::size_t i = 0; i + 1 < segment_data_.size(); ++i)
	{
		if (euclidean_distance(segment_data_[i].high(), segment_data_[i + 1].low()) > min_gap)
		{
			point_type endpoint = segment_data_[i].high();
			convolve(endpoint, segment_data_[i + 1].low()); // add the seond point to the first
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
			15.f, 10.f, FColor(255, 255, 0), false, 0.f, 0.f, 1.f);
		return true;
	}
	else { return false; }
}

bool AVoronoiAIController::get_closest_vertex(std::size_t& OutIndex, point_type point)
{
	if (vd_.vertices().size() == 0)
	{
		return false;
	}
	// If vd_vertics() is nonempty, then there must be a closest nonobstacle vertex
	double closest_distance = 1e10; // infinity
	std::size_t current_index = 0;
	for (const_vertex_iterator it = vd_.vertices().begin(); it != vd_.vertices().end(); ++it)
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
	if (vd_.vertices().size() == 0)
	{
		return false;
	}
	// If vd_vertics() is nonempty, then there must be a closest nonobstacle vertex
	double closest_distance = 1e10; // infinity
	std::size_t current_index = 0;
	for (const_vertex_iterator it = vd_.vertices().begin(); it != vd_.vertices().end(); ++it)
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
	std::size_t goal_index;
	std::size_t source_index;
	if (get_closest_vertex(goal_index, track_opening) && get_closest_front_vertex(source_index, point_type(0, 0)))
	{
		point_type source_point = point_type(vd_.vertices()[source_index].x() / 1000.f, vd_.vertices()[source_index].y() / 1000.f);
		point_type goal_point = point_type(vd_.vertices()[goal_index].x() / 1000.f, vd_.vertices()[goal_index].y() / 1000.f);
		point_type rear_axle(-wheelbase, 0);
		if (euclidean_distance(goal_point, rear_axle) < distance_to_purepursuit_goal)
		{
			OutGoalPoint = goal_point;
			return true;
		}

		DrawDebugSphere(GetWorld(), LidarToWorldLocation(source_point),
			8.f, 5.f, FColor(100, 100, 100), false, 0.f, 0.f, 1.f);

		DrawDebugSphere(GetWorld(), LidarToWorldLocation(goal_point),
			8.f, 5.f, FColor(100, 100, 100), false, 0.f, 0.f, 1.f);

		// If reached here, goalpoint is further than distance_to_purepursuit_goal
		PathMaker pmaker;
		pmaker.set_segments(segment_data_);
		std::vector<point_type> path;
		bool found_path = pmaker.get_path(path, vd_, source_point, goal_point);
		if (found_path)
		{
			for (std::vector<point_type>::iterator it = path.end() - 1; it != path.begin(); --it)
			{
				// PathMaker gives points in meters
				// UE_LOG(LogTemp, Warning, TEXT("*it: x: %f, y: %f"), (*it).x(), (*it).y());
				DrawDebugLine(GetWorld(), LidarToWorldLocation(*it), LidarToWorldLocation(*(it - 1)), FColor(255, 255, 255), false, 0.f, 0.f, 1.f);
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
					OutGoalPoint = point_type(x1 + t * dx - wheelbase, y1 + t * dy);
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
	for (auto itr = segment_vertices.begin(); itr != segment_vertices.end(); ++itr)
	{
		if (euclidean_distance(point, point_type(itr->x(), itr->y())) <= 5)
		{
			return true;
		}
	}
	return false;
}


float AVoronoiAIController::pure_pursuit(point_type goal_point)
{
	// goalpoint_angle_rad is alpha in CMU's formulation
	// goalpoint_angle_rad must be in(-pi / 2, pi / 2)
	// wheelbase is the distance(in meters) between the rear and front axels
	// distance_to_goalpoint is the distance(in meters) between the rear axel and the goal point
	// distance_to_goalpoint must be positive

	double goalpoint_angle_rad = atan(goal_point.y() / (goal_point.x()+wheelbase));
	double steering_angle_rad = atan(2 * wheelbase * sin(goalpoint_angle_rad) / distance_to_purepursuit_goal);
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


float AVoronoiAIController::duty_cycle_from_distance(float distance)
{
	/* Takes a forward distance and returns a duty cycle value to set the
		car's velocity. Fairly unprincipled, basically just scales the speed
		directly based on distance, and stops if the car is blocked. */
	if (distance <= min_distance)
	{
		return 0.0;
	}
	if (distance >= no_obstacles_distance)
	{
		return absolute_max_speed;
	}
	if (distance >= max_distance)
	{
		return scale_speed_linearly(max_speed, absolute_max_speed, distance, max_distance, no_obstacles_distance);
	}
	return scale_speed_linearly(min_speed, max_speed, distance, min_distance, max_distance);
}

float AVoronoiAIController::scale_speed_linearly(float speed_low, float speed_high, float distance, float distance_low, float distance_high)
{
	/* Scales the speed linearly in [speed_low, speed_high] based on the
		distance value, relative to the range[distance_low, distance_high]. */
	float distance_range = distance_high - distance_low;
	float ratio = (distance - distance_low) / distance_range;
	float speed_range = speed_high - speed_low;
	return speed_low + (speed_range * ratio);
}
