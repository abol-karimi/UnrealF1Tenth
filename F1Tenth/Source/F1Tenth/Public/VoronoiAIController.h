// Fill out your copyright notice in the Description page of Project Settings.

#pragma once


#include <boost/polygon/polygon.hpp>
#include <boost/polygon/voronoi.hpp>
#include "voronoi_visual_utils.hpp"
#include <set>

#include "F1TenthPawn.h"
#include "CoreMinimal.h"
#include "AIController.h"
#include "VoronoiAIController.generated.h"



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

typedef VD::vertex_type vertex_type;



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
	float wheelbase = 0.33; // Distance (in meters) of rear axle to front axel
	float max_turn_degrees = 34;
	float distance_to_purepursuit_goal = 1.1; // Distance (in meters) between the rear axel and the goal point
	float LidarMinDegree = -135;
	float LidarMaxDegree = 135;
	float prev_steering_ratio = 0;

	// Nathan's variables
	float min_speed = 0.1;
	// The maximum speed the car will go(the absolute max for the motor is
	// 0.5, which is *very* fast). 0.15 is a good max for slow testing.
	float max_speed = 0.6; //.20
	float absolute_max_speed = 0.6; 
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



struct GNode {
	point_type _vertex;
	std::vector<GNode*> _neighbors;
	std::vector<float> _edge_weights;
	bool _flag;
	float _dist;
	GNode* _parent;
	GNode(point_type& v) {
		_vertex = v;
		_flag = false;
		_parent = NULL;
		_dist = 0.f;
	}
	bool operator==(GNode* node) {
		if (this->_vertex == node->_vertex)
			return true;
		else
			return false;
	}
};

class Graph {
private:
	std::vector<GNode*> _nodes;
public:

	Graph() {
		_nodes.clear();
	};

	~Graph() {
		std::vector<GNode*>::iterator n_it = _nodes.begin();
		for (; n_it != _nodes.end(); ++n_it)
			delete *n_it;
	};

	GNode* get_node(const point_type& v);
	void add_node(GNode* node) {
		_nodes.push_back(node);
	};

	void add_edge(GNode*, GNode*);
	bool compute_shortest_path(std::vector<point_type>&, GNode*, GNode*);
	void print();
};


class PathMaker
{

public:
	PathMaker(float DiscontinuityThreshold)
	{
		_points.clear();
		_segments.clear();
		_discontinuityThreshold = DiscontinuityThreshold;
	};

private:
	VD _vd;
	std::vector<point_type> _points;
	std::vector<segment_type> _segments;
	float _discontinuityThreshold;
protected:
	point_type cast_to_point_type(const PointFloat& pf) {
		int x1, y1;
		x1 = pf.x*1000.f;
		y1 = pf.y*1000.f;
		return point_type(x1, y1);
	}

	segment_type cast_to_segment_type(const SegmentFloat& sf) {
		point_type lp = cast_to_point_type(sf.p0);
		point_type hp = cast_to_point_type(sf.p1);
		return segment_type(lp, hp);
	}
	void construct_graph_from_vd(const VD&, Graph&);
	void color_close_vertices(const VD&, const double);
	void print_point_type(const point_type&);
	void print_vertex_type(const vertex_type&);
	bool sample_close_to_obstacle(const point_type&, const edge_type&, const float&);
public:
	int print_primary_edges();
	void set_segments(std::vector<segment_type>& segments) {
		_segments.clear();
		_segments = segments;
	}

	void set_points(std::vector<point_type>& points) {
		_points.clear();
		_points = points;
	}

	void construct_vd(const std::vector<SegmentFloat>&);
	void construct_vd(const std::vector<PointFloat>&);
	void construct_vd(const std::vector<SegmentFloat>&, const std::vector<PointFloat>&);
	void count_and_color_cells();
	float distance_to_line(const point_type&, const point_type&, const point_type&);
	float distance_between_points(const point_type&, const point_type&);
	bool get_path(std::vector<point_type>&, const point_type&, const point_type&);
	bool get_path(std::vector<point_type>&, const VD&, const point_type&, const point_type&);
	void sample_curved_edge(const edge_type& edge, std::vector<point_type>* sampled_edge);
	point_type retrieve_point(const cell_type& cell);
	segment_type retrieve_segment(const cell_type& cell);
};