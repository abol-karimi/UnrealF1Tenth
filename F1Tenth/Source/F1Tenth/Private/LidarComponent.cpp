// Fill out your copyright notice in the Description page of Project Settings.


#include "LidarComponent.h"
#include "UObject/ConstructorHelpers.h"
#include "Runtime/Engine/Public/DrawDebugHelpers.h"

#include <vector>

ULidarComponent::ULidarComponent()
{
	static ConstructorHelpers::FObjectFinder<UStaticMesh> LidarMesh(TEXT("StaticMesh'/Game/StarterContent/Shapes/Lidar.Lidar'"));
	SetStaticMesh(LidarMesh.Object);
}

void ULidarComponent::Scan()
{
	FHitResult HitResult;
	FVector HitLocation;
	FVector LidarLocation = GetComponentLocation();
	FVector LidarXAxis = GetForwardVector();
	FVector LidarYAxis = -GetRightVector();
	FVector LidarZAxis = GetUpVector();

	for (int i = 0; i < 1081; i++)
	{
		float MeasuringAngle = -135 + i * AngularResolution;
		const FRotator Rot(0, -MeasuringAngle, 0);
		FVector MeasuringDirection = Rot.RotateVector(LidarXAxis);
		FVector StartLocation = LidarLocation + MeasuringDirection * 10; // 10cm away from the center of lidar
		FVector EndLocation = LidarLocation + MeasuringDirection * Range * 100; // *100 to convert to cm

		if (GetWorld()->LineTraceSingleByChannel(
			HitResult,
			StartLocation, 
			EndLocation,
			ECollisionChannel::ECC_Visibility)
			)
		{
			HitLocation = HitResult.Location;
			DrawDebugLine(GetWorld(), StartLocation, HitLocation, FColor(255, 0, 0), false, 0.f, 0.f, 0.f);
			Distances[i] = (HitLocation - LidarLocation).Size() / 100; //divide by 100 to convert cm to meters
		}
		else
		{
			Distances[i] = OutOfRange;
		}
	}

}

void ULidarComponent::Polylinize(std::vector<segment_type>& OutLineSegments)
{
	OutLineSegments.clear();

	SegmentFloat NewSegment(0, 0, 10, 10);
	float NewStartAngle = -135;
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
				LidarToWorldLocation(point_type(x2, y2)), FColor(0, 255, 0), false, 0.f, 1.f, 10.f);
			x1 *= 1000.f;
			y1 *= 1000.f;
			x2 *= 1000.f;
			y2 *= 1000.f;
			OutLineSegments.push_back(segment_type(point_type(x1, y1), point_type(x2, y2))); // TODO what lp and hp? Any requiremtns on the order of points?
		}
	}
}

bool ULidarComponent::GetSegment(SegmentFloat& OutSegment, float& OutStartAngle, float StepAngle, float DiscontThreshold)
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
	if (Distance(StartPoint, InterPoint) > DiscontThreshold) // TODO Report discontinuity
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
		else if (Distance(EndPoint, CandidEndPoint) > DiscontThreshold) // TODO Report discontinuity
		{
			// Finalize the segment with the current EndAngle and EndPoint
			OutSegment.p0 = StartPoint;
			OutSegment.p1 = EndPoint;
			OutStartAngle = CandidEndAngle; // Skip the discontinuity
			//UE_LOG(LogTemp, Warning, TEXT("Distance(EndPoint, CandidEndPoint) > DiscontinuityThreshold"));
			return true; // Found a segment
		}
		else if (DistanceToLine(CandidEndPoint, StartPoint, InterPoint) > 0.1) // TODO: Expose 0.1 as a class property
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

bool ULidarComponent::GetPointAtAngle(PointFloat& OutPoint, float angle_deg)
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

bool ULidarComponent::GetDistanceAtAngle(float& OutDistance, float angle_deg)
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

float ULidarComponent::Distance(PointFloat p0, PointFloat p1)
{
	float DeltaX = p1.x - p0.x;
	float DeltaY = p1.y - p0.y;
	return sqrt(DeltaX * DeltaX + DeltaY * DeltaY);
}

// https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points
// Calculate distance of p0 to the line passing through p0 and p1
float ULidarComponent::DistanceToLine(PointFloat point, PointFloat p0, PointFloat p1)
{
	float delta_y = p1.y - p0.y;
	float delta_x = p1.x - p0.x;
	float denominator = Distance(p0, p1);
	float numerator_const_term = p1.x * p0.y - p1.y * p0.x;
	float numerator = abs(delta_y * point.x - delta_x * point.y + numerator_const_term);
	return (numerator / denominator);
}

FVector ULidarComponent::LidarToWorldLocation(point_type point)
{
	FVector LocationInLidar = FVector(point.x() * 100, point.y() * 100, 0); // *100 to convert to cm
	FVector LidarLocation = GetComponentLocation();
	FVector LidarXAxis = GetForwardVector();
	FVector LidarYAxis = -GetRightVector();
	return LidarLocation + LidarXAxis * point.x() * 100 + LidarYAxis * point.y() * 100;
}