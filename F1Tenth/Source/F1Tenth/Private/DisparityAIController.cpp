// Fill out your copyright notice in the Description page of Project Settings.

#include "DisparityAIController.h"

#include "Runtime/Engine/Public/DrawDebugHelpers.h"
#include "Runtime/Engine/Classes/Engine/World.h"

void ADisparityAIController::BeginPlay()
{
	Super::BeginPlay();

	ControlledVehicle = Cast<AF1TenthPawn>(GetPawn());
	if (!ControlledVehicle)
	{
		UE_LOG(LogTemp, Warning, TEXT("DisparityAIController not possesing a vehicle!"));
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("DisparityAIController possesing: %s"), *(ControlledVehicle->GetName()));
	}
}

void ADisparityAIController::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// TODO: Skip frame if Deltatime < 25ms (frequency of lidar is 40Hz)

	LidarXAxis = ControlledVehicle->GetActorForwardVector();
	LidarYAxis = -ControlledVehicle->GetActorRightVector();
	LidarZAxis = ControlledVehicle->GetActorUpVector();
	LidarLocation = ControlledVehicle->GetActorLocation() + LidarXAxis * 15 + LidarZAxis * 16;

	// Linetrace to gather lidar measurements
	Scan();

	lidar_callback();

}

void ADisparityAIController::Scan()
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
				DrawDebugLine(GetWorld(), LidarLocation, HitLocation, FColor(255, 128, 192), false, 0.f, 0.f, 0.f);
			}
			lidar_distances[i] = (HitLocation - LidarLocation).Size() / 100; //divide by 100 to convert cm to meters
		}
		else
		{
			lidar_distances[i] = OutOfRange;
		}
	}

}


std::vector<size_t> ADisparityAIController::find_disparities()
{
	/* Scans each pair of subsequent values, and returns an array of indices
	where the difference between the two values is larger than the given
	threshold.The returned array contains only the index of the first value
	in pairs beyond the threshold. */
	std::vector<size_t> to_return;
	for (size_t i = 0; i < 1081; i++)
	{
		if (abs(lidar_distances[i] - lidar_distances[i + 1]) >= disparity_threshold)
		{
			to_return.push_back(i);
		}
	}
	return to_return;
}


size_t ADisparityAIController::half_car_samples_at_distance(float distance)
{
	/* Returns the number of points in the LIDAR scan that will cover half of
	the width of the car along an arc at the given distance. */
	// This isn't exact, because it's really calculated based on the arc length
	// when it should be calculated based on the straight - line distance.
	// However, for simplicty we can just compensate for it by inflating the
	// "car width" slightly.
	float distance_between_samples = PI * distance / (180.0 * samples_per_degree);
	return int(ceil(car_width / distance_between_samples));
}

void ADisparityAIController::extend_disparities()
{
	/* For each disparity in the list of distances, extends the nearest
	value by the car width in whichever direction covers up the more -
	distant points.Puts the resulting values in self.masked_disparities.
	*/
	std::array<float, 1081> masked_disparities_tmp = lidar_distances;
	std::vector<size_t> disparities = find_disparities();
	// Keep a list of disparity end points corresponding to safe driving
	// angles directly past a disparity.We will find the longest of these
	// constrained distances in situations where we need to turn towards a
	// disparity.
	possible_disparity_indices.clear();
	//print "Got %d disparities." % (len(disparities), );
	for (std::vector<size_t>::iterator d = disparities.begin(); d != disparities.end(); ++d)
	{
		float a = lidar_distances[*d];
		float b = lidar_distances[*d + 1];
		// If extend_positive is true, then extend the nearer value to
		// higher indices, otherwise extend it to lower indices.
		float nearer_value = a;
		std::size_t nearer_index = *d;
		bool extend_positive = true;
		if (b < a)
		{
			extend_positive = false;
			nearer_value = b;
			nearer_index = *d + 1;
		}
		size_t samples_to_extend = half_car_samples_at_distance(nearer_value);
		size_t current_index = nearer_index;
		for (size_t i = 0; i < samples_to_extend; i++)
		{
			// Stop trying to "extend" the disparity point if we reach the
			// end of the array.
			if (current_index < 0)
			{
				current_index = 0;
				break;
			}
			if (current_index >= masked_disparities_tmp.size())
			{
				current_index = masked_disparities_tmp.size() - 1;
				break;
			}
			// Don't overwrite lidar_distances if we've already found a nearer point
			if (masked_disparities_tmp[current_index] > nearer_value)
			{
				masked_disparities_tmp[current_index] = nearer_value;
			}
			// Finally, move left or right depending on the direction of the
			// disparity.
			if (extend_positive)
			{
				current_index += 1;
			}
			else
			{
				current_index -= 1;
			}
		}
		possible_disparity_indices.push_back(current_index);
	}
	masked_disparities = masked_disparities_tmp;
}


float ADisparityAIController::angle_from_index(size_t i)
{
	/* Returns the angle, in degrees, corresponding to index i in the
	LIDAR samples. */
	float min_angle = -(scan_width / 2.0);
	return min_angle + (float(i) / samples_per_degree);
}

size_t ADisparityAIController::index_from_angle(float theta)
{
	size_t center_index = scan_width * (samples_per_degree / 2);
	return center_index + size_t(theta * float(samples_per_degree));
}
	
bool ADisparityAIController::find_widest_disparity_index(size_t& Out_max_disparity_index)
{
	/* Returns the index of the distance corresponding to the "widest"
	disparity that we can safely target. */
		// Keep this at 0.1 so that we won't identify noise as a disparity
	float max_disparity = 0.1;
	bool foundIndex = false;
	for (std::vector<size_t>::iterator d = possible_disparity_indices.begin(); d != possible_disparity_indices.end(); d++)
	{
		// Ignore disparities that are behind the car.
		float angle = angle_from_index(*d);
		if (angle < min_considered_angle || angle > max_considered_angle)
		{
			continue;
		}
		angle = *d * samples_per_degree;
		float distance = masked_disparities[*d];
		float prev = distance;
		float after = distance;
		// The disparity must have been extended from one of the two
		// directions, so we can calculate the distance of the disparity by
		// checking the distance between the points on either side of the
		// index(note that something on the endpoint won't matter here
		// either.The inequalities are just for bounds checking, if either
		// one is outside the array, then we already know the disparity was
		// extended from a different direction.
		if (*d - 1 > 0)
		{
			prev = masked_disparities[*d - 1];
		}
		if (*d + 1 < masked_disparities.size())
		{
			after = masked_disparities[*d + 1];
		}
		float difference = abs(prev - after);
		if (difference > max_disparity)
		{
			max_disparity = difference;
			Out_max_disparity_index = *d;
			foundIndex = true;
		}
	}
	return foundIndex;
}

void ADisparityAIController::find_new_angle(float& OutDistance, float& OutAngle)
{
	/* Returns the angle of the farthest possible distance that can be reached
	in a direct line without bumping into edges.Returns the distance in meters
	and the angle in degrees. */
	extend_disparities();
	float current_max_distance = -1.0e10;
	OutAngle = 0.0;
	// Constrain the arc of possible angles we consider.
	size_t min_sample_index = index_from_angle(min_considered_angle);
	size_t max_sample_index = index_from_angle(max_considered_angle);

	// visualize
	for (size_t i = min_sample_index; i <= max_sample_index; i+=5)
	{
		float theta_deg = angle_from_index(i);
		float theta_rad = PI * theta_deg / 180.f;
		float r = masked_disparities[i];
		float x = r * cos(theta_rad);
		float y = r * sin(theta_rad);
		DrawDebugSphere(GetWorld(), LidarToWorldLocation(x, y), 10.f, 10.f, FColor(255, 0, 0), false, 0.f, 0.f, 0.f);
	}

	std::vector<float> limited_values(max_sample_index-min_sample_index);
	std::copy(masked_disparities.begin()+min_sample_index, masked_disparities.begin()+max_sample_index, limited_values.begin());
	for (size_t i = 0; i < limited_values.size(); i++)
	{
		OutDistance = limited_values[i];
		if (OutDistance > current_max_distance)
		{
			OutAngle = min_considered_angle + float(i) / samples_per_degree;
			current_max_distance = OutDistance;
		}
	}
}

float ADisparityAIController::scale_speed_linearly(float speed_low, float speed_high, float distance,
	float distance_low, float distance_high)
{
	/* Scales the speed linearly in [speed_low, speed_high] based on the
	distance value, relative to the range[distance_low, distance_high]. */
	float distance_range = distance_high - distance_low;
	float ratio = (distance - distance_low) / distance_range;
	float speed_range = speed_high - speed_low;
	return speed_low + (speed_range * ratio);
}

float ADisparityAIController::duty_cycle_from_distance(float distance)
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
		return scale_speed_linearly(max_speed, absolute_max_speed,
distance, max_distance, no_obstacles_distance);
	}
	return scale_speed_linearly(min_speed, max_speed, distance,
		min_distance, max_distance);
}

float ADisparityAIController::degrees_to_steering_percentage(float degrees)
{
	/* Returns a steering "percentage" value between 0.0 (left) and 1.0
	(right)that is as close as possible to the requested degrees.The car's
	wheels can't turn more than max_angle in either direction. */
	float max_angle = max_turn_angle;
	if (degrees > max_angle)
	{
		return 0.0;
	}
	if (degrees < -max_angle)
	{
		return 1.0;
	}
	// This maps degrees from - max_angle to + max_angle to values from 0 to 1.
	//   (degrees - min_angle) / (max_angle - min_angle)
	// = (degrees - (-max_angle)) / (max_angle - (-max_angle))
	// = (degrees + max_angle) / (max_angle * 2)
	return 1.0 - ((degrees + max_angle) / (2 * max_angle));
}

float ADisparityAIController::adjust_angle_for_car_side(float target_angle)
{
	/* Takes the target steering angle, the distances from the LIDAR, and the
	angle covered by the LIDAR distances.Basically, this function attempts to
	keep the car from cutting corners too close to the wall.In short, it will
	make the car go straight if it's currently turning right and about to hit
	the right side of the car, or turning left or about to hit the left side
	f the car. */
	float car_tolerance = turn_clearance;
	bool turning_left = target_angle > 0.0;
	// Get the portion of the LIDAR samples facing sideways and backwards on
	// the side of the car in the direction of the turn.
	//float samples_per_degree = float(lidar_distances.size()) / scan_width;
	float number_of_back_degrees = (scan_width / 2.0) - 90.0;
	size_t needed_sample_count = size_t(number_of_back_degrees * samples_per_degree);
	std::vector<float> side_samples(needed_sample_count);
	if (turning_left) // target_angle > 0.0
	{
		std::copy(lidar_distances.end() - needed_sample_count, lidar_distances.end(), side_samples.begin());
	}
	else // target_angle <= 0.0 (Does it work for 0.0?)
	{
		std::copy(lidar_distances.begin(), lidar_distances.begin() + needed_sample_count, side_samples.begin());
	}
	// Finally, just make sure no point in the backwards scan is too close.
	// This could definitely be more exact with some proper math.
	for (std::vector<float>::iterator v = side_samples.begin(); v != side_samples.end(); v++)
	{
		if (*v <= car_tolerance)
		{
			return 0.0;
		}
	}
	return target_angle;
}


float ADisparityAIController::adjust_angle_to_avoid_uturn(float target_angle, float forward_distance)
{
	/* When the car's forward distance is small, it can favor turning to
	the side of a wide track.This function attempts to detect when such a
	case may occur and force the steering angle to follow a disparity
	instead. */
	if (forward_distance > no_u_distance)
	{
		return target_angle;
	}
	size_t target_index;
	if (!find_widest_disparity_index(target_index))
	{
		return target_angle;
	}
	return angle_from_index(target_index);
}

void ADisparityAIController::update_considered_angle(float steering_angle)
{
	float actual_angle = steering_angle;
	if (actual_angle < -max_turn_angle)
	{
		actual_angle = -max_turn_angle;
	}
	if (actual_angle > max_turn_angle)
	{
		actual_angle = max_turn_angle;
	}
	min_considered_angle = -89.0;
	max_considered_angle = 89.0;
	if (actual_angle > 0)
	{
		min_considered_angle -= actual_angle;
	}
	if (actual_angle < 0)
	{
		max_considered_angle += actual_angle;
	}			
}

void ADisparityAIController::lidar_callback()
{
	samples_per_degree = float(lidar_distances.size()) / scan_width;
	float target_distance;
	float target_angle;
	find_new_angle(target_distance, target_angle);
	float forward_distance = lidar_distances[lidar_distances.size() / 2];
	// target_angle = self.adjust_angle_to_avoid_uturn(target_angle,
	//   forward_distance)
	target_angle = adjust_angle_for_car_side(target_angle);
	float desired_speed = duty_cycle_from_distance(forward_distance);
	update_considered_angle(target_angle);
	float steering_percentage = degrees_to_steering_percentage(target_angle);

	ControlledVehicle->MoveRight(2.0f*steering_percentage-1);
	ControlledVehicle->MoveForward(desired_speed);
}


FVector ADisparityAIController::LidarToWorldLocation(float x, float y)
{
	FVector LocationInLidar = FVector(x * 100, y * 100, 0); // *100 to convert to cm
	return LidarLocation + LidarXAxis * x * 100 + LidarYAxis * y * 100;
}