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

THIRD_PARTY_INCLUDES_START
#include <boost/functional/hash.hpp>
THIRD_PARTY_INCLUDES_END
#include <unordered_map>





VoronoiGraph::VoronoiGraph()
{
}

VoronoiGraph::~VoronoiGraph()
{
}

// Assumes that the Voronoi diagram has only input segments, i.e. no input points.
point_type retrieve_endpoint(const cell_type& cell, const std::vector<segment_type>& Walls)
{
	source_index_type index = cell.source_index();
	source_category_type category = cell.source_category();
	if (category == boost::polygon::SOURCE_CATEGORY_SEGMENT_START_POINT) {
		return low(Walls[index]);
	}
	else {
		return high(Walls[index]);
	}
}

// Assumes that the Voronoi diagram has only input segments, i.e. no input points.
segment_type retrieve_segment(const cell_type& cell, const std::vector<segment_type>& Walls) {
	source_index_type index = cell.source_index();
	return Walls[index];
}

void color_close_vertices(const VD& vd, const std::vector<segment_type>& Walls)
{
	const double allowed_obs_dist = 150.f; // 150 milimeters == 15 centimeters
	for (const auto& vertex : vd.vertices())
		vertex.color(0);
	for (const auto& vertex : vd.vertices())
	{
		point_type voronoi_point(vertex.x(), vertex.y());
		const cell_type* cell = vertex.incident_edge()->cell();
		if (cell->contains_point())
		{
			point_type endpoint = retrieve_endpoint(*cell, Walls);
			if (euclidean_distance(voronoi_point, endpoint) < allowed_obs_dist)
				vertex.color(1);
		}
		else { // i.e. cell contains a segment
			segment_type segment = retrieve_segment(*cell, Walls);
			if (euclidean_distance(segment, voronoi_point) < allowed_obs_dist)
				vertex.color(1);
		}
	}
}

void color_close_edges(const VD& vd, const std::vector<segment_type>& Walls)
{

}

void VoronoiGraph::MakeRoadmap(const std::vector<segment_type>& Walls)
{
	// Clear the previous roadmap
	Roadmap.clear();
	
	// Compute the Voronoid diagram for the walls
	VD VDiagram;
	construct_voronoi(Walls.begin(), Walls.end(), &VDiagram);

	// Color too-close (to the walls) Voronoi vertices,
	// then add the colorless to the roadmap.
	color_close_vertices(VDiagram, Walls);
	typedef typename boost::graph_traits<Roadmap_t>::vertex_descriptor vertex_descriptor; 	
	std::unordered_map<const vertex_type*, vertex_descriptor> voronoi_to_roadmap; // Maintain a mapping from Voronoi vertices to roadmap vertices

	for (const_vertex_iterator it = VDiagram.vertices().begin(); it != VDiagram.vertices().end(); ++it) {
		// If vertex is too close to the walls, skip.
		if (it->color() == 1)
			continue;

		// Add vertex to the graph and set its coordinates property
		point_type newvertex_coordinates(it->x() / 1000.f, it->y() / 1000.f); // The coordinates are in millimeters in VDiagram, but in meters in the roadmap.
		vertex_descriptor newvertex = add_vertex(Roadmap);
		voronoi_to_roadmap.insert({ &(*it), newvertex });
		coordinates_map_t coordinates_map = get(boost::vertex_coordinates, Roadmap);
		coordinates_map[newvertex] = newvertex_coordinates;
	}
	// Color too-close (to the walls) Voronoi edges
	// Add far-enough Voronoi edges to the roadmap
	//roadmap_vertex = voronoi_to_roadmap.find(vertex_ptr)
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

