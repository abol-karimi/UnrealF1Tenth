// Fill out your copyright notice in the Description page of Project Settings.


#include "VoronoiGraph.h"


#include <boost/polygon/voronoi.hpp>
typedef boost::polygon::voronoi_diagram<coordinate_type> VD;
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

VoronoiGraph::VoronoiGraph()
{
}

VoronoiGraph::~VoronoiGraph()
{
}

void VoronoiGraph::MakeRoadmap(const std::vector<segment_type>& Walls)
{
	VD VDiagram;
	VDiagram.clear();
	construct_voronoi(Walls.begin(), Walls.end(), &VDiagram);

	// add all the edges to the graph


}

void VoronoiGraph::GetPlan(std::vector<segment_type>& OutPlan)
{
	point_type TrackOpening;
	//if (!get_trackopening(TrackOpening, Walls, 300)) // 300 milimeters = 30 centimeters
	//{
	//	// print "no track opening found"
	//	return;
	//}
}

bool VoronoiGraph::get_trackopening(point_type& OutTrackOpening,
	const std::vector<segment_type>& Walls,
	double min_gap) // min_gap is in millimeters
{
	std::vector<point_type> discontinuities;
	double max_cos = -1; // The angle behind the car has cos=-1
	int max_cos_index;
	for (std::size_t i = 0; i + 1 < Walls.size(); ++i)
	{
		if (euclidean_distance(Walls[i].high(), Walls[i + 1].low()) > min_gap)
		{
			point_type endpoint = Walls[i].high();
			convolve(endpoint, Walls[i + 1].low()); // add the seond point to the first
			point_type midpoint = scale_down(endpoint, 2);
			discontinuities.push_back(midpoint);
			double cos = midpoint.x() / euclidean_distance(midpoint, point_type(0, 0));
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
		return true;
	}
	else { return false; }
}
