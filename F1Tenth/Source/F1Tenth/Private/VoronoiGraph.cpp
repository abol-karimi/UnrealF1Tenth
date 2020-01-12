// Fill out your copyright notice in the Description page of Project Settings.


#include "VoronoiGraph.h"

THIRD_PARTY_INCLUDES_START
#include <boost/polygon/voronoi.hpp>
THIRD_PARTY_INCLUDES_END

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

THIRD_PARTY_INCLUDES_START
#include <boost/property_map/property_map.hpp>
#include <boost/tuple/tuple.hpp>
THIRD_PARTY_INCLUDES_END

typedef boost::property_map<Roadmap_t, boost::vertex_coordinates_t>::type coordinates_map_t; // The type of the mapping from a vertex descriptor to its coordiantes property
typedef boost::property_map<Roadmap_t, boost::edge_weight_t>::type weight_map_t;

VoronoiGraph::VoronoiGraph()
{
}

VoronoiGraph::~VoronoiGraph()
{
}

void VoronoiGraph::MakeRoadmap(const std::vector<segment_type>& Walls)
{
	// Clear the previous roadmap
	Roadmap.clear();
	
	// Compute the Voronoid diagram for the walls
	VD VDiagram;
	construct_voronoi(Walls.begin(), Walls.end(), &VDiagram);

	// Add the Voronoi vertices to the roadmap
	for (const_vertex_iterator it_v = VDiagram.vertices().begin(); it_v != VDiagram.vertices().end(); ++it_v) {
		point_type coords(it_v->x() / 1000.f, it_v->y() / 1000.f); // The coordinates are in millimeters in VDiagram, but in meters in the roadmap.
		// if the vertex is too close to the walls
			// skip
		// Add the vertex to the graph
		typename boost::graph_traits<Roadmap_t>::vertex_descriptor v;
		v = add_vertex(Roadmap);
		// Assign the coordinates property of the new vertex
		coordinates_map_t coordinates_map = get(boost::vertex_coordinates, Roadmap);
		coordinates_map[v] = coords;
	}

	// add all the edges to the graph


}

void VoronoiGraph::GetRoadmapPoints(std::list<point_type>& points)
{
	using namespace boost;

	coordinates_map_t coordinates_map = get(vertex_coordinates, Roadmap);
	graph_traits<Roadmap_t>::vertex_iterator vi, vi_end;
	point_type coords;
	for (tie(vi, vi_end) = vertices(Roadmap); vi != vi_end; ++vi)
	{
		coords = get(coordinates_map, *vi);
		points.push_back(coords);
	}
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
	int max_cos_index = -1; // If there is any gaps, the index will be updated to nonnegative.
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

