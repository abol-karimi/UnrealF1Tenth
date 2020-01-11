// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include <boost/polygon/polygon.hpp>
#include <vector>

typedef double coordinate_type;
typedef boost::polygon::point_data<coordinate_type> point_type;
typedef boost::polygon::segment_data<coordinate_type> segment_type;

THIRD_PARTY_INCLUDES_START
#include <boost/graph/adjacency_list.hpp>
THIRD_PARTY_INCLUDES_END

namespace boost {
	enum vertex_coordinates_t { vertex_coordinates = 111 }; // a unique id for the type tag
	BOOST_INSTALL_PROPERTY(vertex, coordinates);
}
typedef boost::adjacency_list < boost::vecS, boost::vecS, boost::undirectedS,
	boost::property<boost::vertex_coordinates_t, point_type>,
	boost::property<boost::edge_weight_t, float> > Roadmap_t;

/**
 * 
 */
class F1TENTH_API VoronoiGraph
{
public:
	VoronoiGraph();
	~VoronoiGraph();
	void MakeRoadmap(const std::vector<segment_type>& Walls);
	void GetPlan(std::vector<segment_type>& OutPlan);

private:
	bool get_trackopening(point_type& OutTrackOpening, const std::vector<segment_type>& Walls, double min_gap);
	Roadmap_t Roadmap;
};

