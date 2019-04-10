// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <boost/polygon/polygon.hpp>
#include <boost/polygon/voronoi.hpp>
#include "voronoi_visual_utils.hpp"

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Scanner.generated.h"


//typedef long long int integral_coordinate_type;
//
//struct Point {
//	integral_coordinate_type x;
//	integral_coordinate_type y;
//	Point(integral_coordinate_type x0, integral_coordinate_type y0) : x(x0), y(y0) {}
//};
//
//struct Segment {
//	Point p0;
//	Point p1;
//	Segment(integral_coordinate_type x1, integral_coordinate_type y1, integral_coordinate_type x2, integral_coordinate_type y2) : p0(x1, y1), p1(x2, y2) {}
//};

struct PointFloat {
	float x;
	float y;
	PointFloat(float x0, float y0) : x(x0), y(y0) {}
};

struct SegmentFloat {
	PointFloat p0;
	PointFloat p1;
	SegmentFloat(float x1, float y1, float x2, float y2) : p0(x1, y1), p1(x2, y2) {}
};


using namespace boost::polygon;
typedef double coordinate_type;
typedef boost::polygon::point_data<coordinate_type> point_type;
typedef segment_data<coordinate_type> segment_type;
typedef rectangle_data<coordinate_type> rect_type;
typedef voronoi_builder<int> VB;
typedef voronoi_diagram<coordinate_type> VD;
typedef VD::cell_type cell_type;
typedef VD::cell_type::source_index_type source_index_type;
typedef VD::cell_type::source_category_type source_category_type;
typedef VD::edge_type edge_type;
typedef VD::cell_container_type cell_container_type;
typedef VD::cell_container_type vertex_container_type;
typedef VD::edge_container_type edge_container_type;
typedef VD::const_cell_iterator const_cell_iterator;
typedef VD::const_vertex_iterator const_vertex_iterator;
typedef VD::const_edge_iterator const_edge_iterator;


UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class F1TENTH_API UScanner : public UActorComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UScanner();

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

public:
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
	std::vector<point_type> get_discontinuity_midpoints();

	float Distances[1081]; // Array of distances
	float AngularResolution = 0.25; // 4 measurements per angle
	float Range = 17; // Maximum detectable distance in meters
	float OutOfRange = 65.533; // Value to return if distance > LidarRange
	VD vd_;
	std::vector<point_type> point_data_;
	std::vector<segment_type> segment_data_;
};

