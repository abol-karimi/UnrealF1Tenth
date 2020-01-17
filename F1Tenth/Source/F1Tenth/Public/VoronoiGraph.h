// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include <vector>

THIRD_PARTY_INCLUDES_START
#include <boost/polygon/polygon.hpp>
#include <boost/polygon/voronoi.hpp>
THIRD_PARTY_INCLUDES_END
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


/**
 * 
 */
class F1TENTH_API VoronoiGraph
{
public:
	VoronoiGraph();
	~VoronoiGraph();
	void MakeRoadmap(const std::vector<segment_type>& Walls);
	void GetPlan(std::vector<point_type>& OutPlan, const std::vector<segment_type>& Walls);
	void GetRoadmapPoints(std::list<point_type>& points);
	void GetRoadmapSegments(std::list<segment_type>& segments);

	const double allowed_obs_dist = 0.2; // 0.2 m = 20 cm 
	const double max_discretization_error = 0.3;
	const double min_track_width = 1.5f;

private:
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

	typedef boost::adjacency_list < boost::vecS, boost::vecS, boost::undirectedS,
		boost::property<boost::vertex_coordinates_t, point_type>,
		boost::property<boost::edge_weight_t, float> > Roadmap_t;
	typedef typename boost::graph_traits<VoronoiGraph::Roadmap_t>::vertex_descriptor vertex_descriptor;
	typedef typename boost::graph_traits<VoronoiGraph::Roadmap_t>::edge_descriptor edge_descriptor;
	typedef boost::property_map<Roadmap_t, boost::vertex_coordinates_t>::type coordinates_map_t; // The type of the mapping from a vertex descriptor to its coordiantes property
	typedef boost::property_map<Roadmap_t, boost::edge_weight_t>::type weight_map_t;

	Roadmap_t Roadmap;
	bool get_trackopening(point_type& OutTrackOpening, const std::vector<segment_type>& Walls, double min_gap);

	// Voronoi diagram processing
	point_type retrieve_endpoint(const cell_type& cell, const std::vector<segment_type>& Walls);
	segment_type retrieve_segment(const cell_type& cell, const std::vector<segment_type>& Walls);
	void sample_curved_edge(const edge_type& edge, std::vector<point_type>* sampled_edge, const std::vector<segment_type>& Walls);
	void color_close_vertices(const VD& vd, const std::vector<segment_type>& Walls);
	void add_linear_edge(const edge_type& edge, std::unordered_map<const vertex_type*, vertex_descriptor>& voronoi_to_roadmap, const std::vector<segment_type>& Walls);
	void add_curved_edge(const edge_type& edge, std::unordered_map<const vertex_type*, vertex_descriptor>& voronoi_to_roadmap, const std::vector<segment_type>& Walls);
	vertex_descriptor add_roadmap_vertex(point_type point);
	edge_descriptor add_roadmap_edge(vertex_descriptor vertex0, vertex_descriptor vertex1, double weight);
	vertex_descriptor get_closest_vertex(point_type point);
};

