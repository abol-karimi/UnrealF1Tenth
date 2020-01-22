// Fill out your copyright notice in the Description page of Project Settings.


#include "VoronoiGraph.h"

THIRD_PARTY_INCLUDES_START
#include "voronoi_visual_utils.hpp"
THIRD_PARTY_INCLUDES_END

THIRD_PARTY_INCLUDES_START
#include <boost/graph/graph_traits.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/numeric/ublas/vector.hpp>
THIRD_PARTY_INCLUDES_END

THIRD_PARTY_INCLUDES_START
#pragma push_macro("check")
#undef check
#include <boost/graph/dijkstra_shortest_paths_no_color_map.hpp>
#pragma pop_macro("check")
THIRD_PARTY_INCLUDES_END
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Winvalid-noreturn"
namespace boost
{
#ifdef BOOST_NO_EXCEPTIONS
	void throw_exception(std::exception const & e)
	{} // user defined
#endif
}
#pragma clang diagnostic pop

#include <unordered_map>

VoronoiGraph::VoronoiGraph()
{
}

VoronoiGraph::~VoronoiGraph()
{
}

// Assumes that the Voronoi diagram has only input segments, i.e. no input points.
point_type VoronoiGraph::retrieve_endpoint(const cell_type& cell, const std::vector<segment_type>& Walls)
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
segment_type VoronoiGraph::retrieve_segment(const cell_type& cell, const std::vector<segment_type>& Walls) {
	source_index_type index = cell.source_index();
	return Walls[index];
}

void VoronoiGraph::sample_curved_edge(const edge_type& edge, std::vector<point_type>* sampled_edge, const std::vector<segment_type>& Walls) {
	point_type point = edge.cell()->contains_point() ? retrieve_endpoint(*edge.cell(), Walls) : retrieve_endpoint(*edge.twin()->cell(), Walls);
	segment_type segment = edge.cell()->contains_point() ? retrieve_segment(*edge.twin()->cell(), Walls) : retrieve_segment(*edge.cell(), Walls);
	boost::polygon::voronoi_visual_utils<coordinate_type>::discretize(point, segment, max_discretization_error*1000.f, sampled_edge);
}

void VoronoiGraph::color_close_vertices(const VD& vd, const std::vector<segment_type>& Walls)
{
	for (const auto& vertex : vd.vertices())
		vertex.color(0);
	for (const auto& vertex : vd.vertices())
	{
		point_type voronoi_point(vertex.x(), vertex.y());
		const cell_type* cell = vertex.incident_edge()->cell();
		if (cell->contains_point())
		{
			point_type endpoint = retrieve_endpoint(*cell, Walls);
			if (euclidean_distance(endpoint, voronoi_point) < allowed_obs_dist * 1000.f) // *1000.f to convert to milimeters
				vertex.color(1);
		}
		else { // i.e. cell contains a segment
			segment_type segment = retrieve_segment(*cell, Walls);
			if (euclidean_distance(segment, voronoi_point) < allowed_obs_dist * 1000.f)
				vertex.color(1);
		}
	}
}

void VoronoiGraph::MakeRoadmap(const std::vector<segment_type>& Walls, float allowed_obs_dist)
{
	this->allowed_obs_dist = allowed_obs_dist;
	// Clear the previous roadmap
	Roadmap.clear();

	// Compute the Voronoid diagram for the walls
	std::vector<segment_type> Walls_mm(Walls.size());
	for (size_t i = 0; i < Walls.size(); ++i)
	{
		segment_type seg = Walls[i];
		Walls_mm[i] = boost::polygon::scale_up(seg, 1000);
	}
	VD VDiagram;
	construct_voronoi(Walls_mm.begin(), Walls_mm.end(), &VDiagram);

	// Maintain a mapping from Voronoi vertices to roadmap vertices
	std::unordered_map<const vertex_type*, vertex_descriptor> voronoi_to_roadmap; 

	// Color too-close (to the walls) Voronoi vertices,
	// then add the colorless to the roadmap.
	color_close_vertices(VDiagram, Walls_mm);
	for (const_vertex_iterator it = VDiagram.vertices().begin(); it != VDiagram.vertices().end(); ++it) {
		// If vertex is too close to the walls, skip.
		if (it->color() == 1)
			continue;
		vertex_descriptor newvertex = add_roadmap_vertex(point_type(it->x() / 1000.f, it->y() / 1000.f)); // The coordinates are in millimeters in VDiagram, but in meters in the roadmap.
		voronoi_to_roadmap.insert({ &(*it), newvertex });
	}

	for (const auto& edge : VDiagram.edges())
	{
		if (!edge.is_primary())
			continue;
		if (edge.is_finite() && edge.vertex0()->color() == 0 && edge.vertex1()->color() == 0)
		{
			if (edge.is_linear()) 
				add_linear_edge(edge, voronoi_to_roadmap);
			else
				add_curved_edge(edge, voronoi_to_roadmap, Walls_mm);
		}
	}
	add_start_vertex();
	add_finish_vertex();
}

VoronoiGraph::vertex_descriptor VoronoiGraph::add_roadmap_vertex(point_type point)
{
	vertex_descriptor newvertex = add_vertex(Roadmap);
	coordinates_map_t coordinates_map = get(boost::vertex_coordinates, Roadmap);
	coordinates_map[newvertex] = point;
	return newvertex;
}

VoronoiGraph::edge_descriptor VoronoiGraph::add_roadmap_edge(vertex_descriptor vertex0, vertex_descriptor vertex1)
{
	coordinates_map_t coordinates_map = get(boost::vertex_coordinates, Roadmap);
	double weight = euclidean_distance(coordinates_map[vertex0], coordinates_map[vertex1]);
	return add_roadmap_edge(vertex0, vertex1, weight);
}

VoronoiGraph::edge_descriptor VoronoiGraph::add_roadmap_edge(vertex_descriptor vertex0, vertex_descriptor vertex1, double weight)
{
	edge_descriptor roadmap_edge;
	bool inserted;
	tie(roadmap_edge, inserted) = add_edge(vertex0, vertex1, Roadmap);
	weight_map_t weight_map = get(boost::edge_weight, Roadmap);
	weight_map[roadmap_edge] = weight;
	return roadmap_edge;
}

/// Assumes the roadmap vertices have been added before.
void VoronoiGraph::add_linear_edge(const edge_type& edge, std::unordered_map<const vertex_type*, vertex_descriptor>& voronoi_to_roadmap)
{
	const vertex_descriptor roadmap_vertex0 = voronoi_to_roadmap.at(edge.vertex0());
	const vertex_descriptor roadmap_vertex1 = voronoi_to_roadmap.at(edge.vertex1());
	point_type p0(edge.vertex0()->x(), edge.vertex0()->y());
	point_type p1(edge.vertex1()->x(), edge.vertex1()->y());
	add_roadmap_edge(roadmap_vertex0, roadmap_vertex1, euclidean_distance(p0, p1) / 1000.f);
}

void VoronoiGraph::add_curved_edge(const edge_type& edge, std::unordered_map<const vertex_type*, vertex_descriptor>& voronoi_to_roadmap, const std::vector<segment_type>& Walls_mm)
{
	point_type vertex0(edge.vertex0()->x(), edge.vertex0()->y());
	point_type vertex1(edge.vertex1()->x(), edge.vertex1()->y());
	std::vector<point_type> samples;
	samples.push_back(vertex0);
	samples.push_back(vertex1);
	sample_curved_edge(edge, &samples, Walls_mm);

	// Check that all segments of the discretization are far enough from the focus of the parabola
	point_type focus = edge.cell()->contains_point() ? retrieve_endpoint(*edge.cell(), Walls_mm) : retrieve_endpoint(*edge.twin()->cell(), Walls_mm);
	for (std::size_t i = 0; i < samples.size() - 1; ++i)
	{
		segment_type segment(samples[i], samples[i + 1]);
		if(euclidean_distance(segment, focus) < allowed_obs_dist * 1000.f)
			return;
	}
	std::vector<vertex_descriptor> vertices(samples.size());
	vertices[0] = voronoi_to_roadmap.at(edge.vertex0());
	vertices[samples.size()-1] = voronoi_to_roadmap.at(edge.vertex1());
	// Add all the new points from discretization to the roadmap
	for (std::size_t i = 1; i < samples.size() - 1; ++i)
		vertices[i] = add_roadmap_vertex(boost::polygon::scale_down(samples[i], 1000));
	// Add all the edges of the discretization
	for (std::size_t i = 0; i < samples.size() - 1; ++i)
		add_roadmap_edge(vertices[i], vertices[i + 1], boost::polygon::euclidean_distance(samples[i], samples[i + 1]) / 1000.f);
}

void VoronoiGraph::add_start_vertex()
{
	using namespace boost;
	coordinates_map_t coordinates_map = get(vertex_coordinates, Roadmap);
	edge_descriptor e_closest = get_closest_edge(point_type(0.f, 0.f));
	vertex_descriptor vertex0 = source(e_closest, Roadmap);
	vertex_descriptor vertex1 = target(e_closest, Roadmap);
	point_type point0 = get(coordinates_map, vertex0);
	point_type point1 = get(coordinates_map, vertex1);
	using namespace boost::numeric;
	ublas::vector<double> v0(2), v1(2), v_closest(2);
	v0[0] = point0.x(); v0[1] = point0.y();
	v1[0] = point1.x(); v1[1] = point1.y();
	v_closest = v0 - (ublas::inner_prod(v0, v1 - v0) / ublas::norm_2_square(v1 - v0))*(v1 - v0);
	point_type p_closest(v_closest[0], v_closest[1]);
	start_vertex = add_roadmap_vertex(p_closest);
	add_roadmap_edge(start_vertex, vertex0);
	add_roadmap_edge(start_vertex, vertex1);
}

void VoronoiGraph::add_finish_vertex()
{

}

void VoronoiGraph::GetRoadmapPoints(std::list<point_type>& points) // TODO: Change to vector and reserve space
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

void VoronoiGraph::GetRoadmapSegments(std::vector<segment_type>& segments) // TODO: Change to vector and reserve space
{
	using namespace boost;
	coordinates_map_t coordinates_map = get(vertex_coordinates, Roadmap);
	graph_traits<Roadmap_t>::edge_iterator ei, ei_end;
	for (tie(ei, ei_end) = edges(Roadmap); ei != ei_end; ++ei)
	{
		vertex_descriptor vertex0 = source(*ei, Roadmap);
		vertex_descriptor vertex1 = target(*ei, Roadmap);
		point_type point0 = get(coordinates_map, vertex0);
		point_type point1 = get(coordinates_map, vertex1);
		segments.push_back(segment_type(point0, point1));
	}
}

VoronoiGraph::vertex_descriptor VoronoiGraph::get_closest_vertex(point_type point)
{
	using namespace boost;
	coordinates_map_t coordinates_map = get(vertex_coordinates, Roadmap);
	graph_traits<Roadmap_t>::vertex_iterator vi, vi_end;
	double closest_distance = std::numeric_limits<double>::max(); // infinity
	vertex_descriptor current_vertex;
	for (tie(vi, vi_end) = vertices(Roadmap); vi != vi_end; ++vi)
	{
		point_type current_point = get(coordinates_map, *vi);
		double current_distance = polygon::euclidean_distance(point, current_point);
		if (current_distance < closest_distance)
		{
			current_vertex = *vi;
			closest_distance = current_distance;
		}
	}
	return current_vertex;
}

VoronoiGraph::edge_descriptor VoronoiGraph::get_closest_edge(point_type point)
{
	using namespace boost;
	coordinates_map_t coordinates_map = get(vertex_coordinates, Roadmap);
	graph_traits<Roadmap_t>::edge_iterator ei, ei_end;
	double min_distance = std::numeric_limits<double>::max(); // infinity
	edge_descriptor e_closest;
	for (tie(ei, ei_end) = edges(Roadmap); ei != ei_end; ++ei)
	{
		vertex_descriptor vertex0 = source(*ei, Roadmap);
		vertex_descriptor vertex1 = target(*ei, Roadmap);
		point_type point0 = get(coordinates_map, vertex0);
		point_type point1 = get(coordinates_map, vertex1);
		double current_distance = euclidean_distance(segment_type(point0, point1), point);
		if (current_distance < min_distance)
		{
			e_closest = *ei;
			min_distance = current_distance;
		}
	}
	return e_closest;
}

void VoronoiGraph::GetPlan(std::vector<point_type>& OutPlan, const std::vector<segment_type>& Walls)
{
	point_type track_opening;
	float steering_ratio = 0.f;
	if (!get_trackopening(track_opening, Walls, min_track_width))
		return;
	// shortest paths from source
	std::vector<vertex_descriptor> pred(num_vertices(Roadmap));
	std::vector<double> distances(num_vertices(Roadmap));
	dijkstra_shortest_paths_no_color_map(Roadmap, start_vertex,
		predecessor_map(boost::make_iterator_property_map(pred.begin(), get(boost::vertex_index, Roadmap))).
		distance_map(boost::make_iterator_property_map(distances.begin(), get(boost::vertex_index, Roadmap))));

	vertex_descriptor vertex = get_closest_vertex(track_opening), child;
	coordinates_map_t coordinates_map = get(boost::vertex_coordinates, Roadmap);
	do {
		point_type point = get(coordinates_map, vertex);
		OutPlan.push_back(point);
		child = vertex;
		vertex = pred[child];
	} while (vertex != child);
	std::reverse(std::begin(OutPlan), std::end(OutPlan));
}

bool VoronoiGraph::get_trackopening(point_type& OutTrackOpening, const std::vector<segment_type>& Walls, double min_gap)
{
	std::vector<point_type> discontinuities;
	double max_cos = -1; // The angle behind the car has cos=-1
	int max_cos_index = -1; // If there is any gaps, the index will be updated to nonnegative.
	for (std::size_t i = 0; i + 1 < Walls.size(); ++i)
	{
		if (boost::polygon::euclidean_distance(Walls[i].high(), Walls[i + 1].low()) >= min_gap)
		{
			point_type endpoint = Walls[i].high(); // TODO: use boost::geometry::centroid
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

