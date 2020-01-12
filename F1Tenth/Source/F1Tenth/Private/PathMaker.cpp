#include "PathMaker.h"
#include "../Public/VoronoiAIController.h"

THIRD_PARTY_INCLUDES_START
#include "voronoi_visual_utils.hpp"
THIRD_PARTY_INCLUDES_END

#include <cstdio>
#include <vector>
#include <fstream>
#include <iostream>
#include <vector>
#include <math.h>

using namespace std;


GNode* Graph::get_node(const point_type& v) {
	bool found = false;
	vector<GNode*>::iterator node_it = _nodes.begin();
	for(; node_it != _nodes.end(); ++node_it) {
		if ((*node_it)->_vertex == v) {
			found = true;
			break;
		}
	}
	if (found == true)
		return *node_it;
	else
		return NULL;

}

void Graph::add_edge(GNode* node1, GNode* node2) {
	node1->_neighbors.push_back(node2);
	node2->_neighbors.push_back(node1);
};

float PathMaker::distance_between_points(const point_type& p1, const point_type& p2) {
	float x_val = p1.x() - p2.x();
	float y_val = p1.y() - p2.y();
	float weight = sqrt(pow(x_val, 2) + pow(y_val, 2));
	return weight;
}

// https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points
// Calculate distance of p0 to the line passing through p0 and p1
float PathMaker::distance_to_line(const point_type& point, const point_type& p0, const point_type& p1)
{
	float delta_y = p1.y() - p0.y();
	float delta_x = p1.x() - p0.x();
	float denominator = distance_between_points(p0, p1);
	float numerator_const_term = p1.x() * p0.y() - p1.y() * p0.x();
	float numerator = abs(delta_y * point.x() - delta_x * point.y() + numerator_const_term);
	return (numerator / denominator);
}

void PathMaker::count_and_color_cells() {
  for(const_cell_iterator it = _vd.cells().begin(); it != _vd.cells().end(); ++it) {
	  std::size_t cnt = 0;
	  it->color(cnt);
  }
  int open_cells = 0;
  for (const_edge_iterator it = _vd.edges().begin(); it != _vd.edges().end(); ++it) {
	  std::size_t cnt = it->cell()->color();
	  if (it->is_infinite() && cnt == 0) {
		  it->cell()->color(1);
		  open_cells++;
	  }
    }
  std::cout << "Number of open cells " <<  open_cells << std::endl;
  std::cout << "Number of closed cells " << _vd.cells().size() - open_cells << std::endl;
};


void PathMaker::color_close_vertices(const VD& vd, const double allowed_obs_dist) {

	//double allowed_obs_dist = 0.3;
	for(const_vertex_iterator it_v = vd.vertices().begin(); it_v != vd.vertices().end(); ++it_v) 
		it_v->color(0);

	for(const_edge_iterator it_e = vd.edges().begin(); it_e != vd.edges().end(); ++it_e) {
		const edge_type edge = *it_e;
		if (edge.is_finite() && edge.is_primary()) {
			const vertex_type* e_vertex0 = edge.vertex0();
			const vertex_type* e_vertex1 = edge.vertex1();
			if ((e_vertex0->color() == 1) || (e_vertex1->color() == 1))
				continue;
			point_type v0(e_vertex0->x()/1000.f, e_vertex0->y()/1000.f);
			point_type v1(e_vertex1->x()/1000.f, e_vertex1->y()/1000.f);
			const cell_type* edge_cell = edge.cell();
			//point_type point = edge_cell->contains_point() ? retrieve_point(*edge_cell) : retrieve_point(*edge.twin()->cell());
			//segment_type segment = edge_cell->contains_point() ? retrieve_segment(*edge.twin()->cell()) : retrieve_segment(*edge_cell);
			
			std::size_t index = edge_cell->source_index();
			if (edge_cell->contains_segment()) {
				segment_type segment = _segments[index];
				point_type p0 = low(segment);
				p0 = point_type(p0.x() / 1000.f, p0.y() / 1000.f);
				point_type p1 = high(segment);
				p1 = point_type(p1.x() / 1000.f, p1.y() / 1000.f);

				if (distance_to_line(v0, p0, p1) < allowed_obs_dist)
					e_vertex0->color(1);
				if (distance_to_line(v1, p0, p1) < allowed_obs_dist)
					e_vertex1->color(1);
			}
		}
	}
}

void PathMaker::sample_curved_edge(const edge_type& edge, std::vector<point_type>* sampled_edge)
{
	coordinate_type max_dist = 300;
	point_type point = edge.cell()->contains_point() ? retrieve_point(*edge.cell()) : retrieve_point(*edge.twin()->cell());
	segment_type segment = edge.cell()->contains_point() ? retrieve_segment(*edge.twin()->cell()) : retrieve_segment(*edge.cell());
	boost::polygon::voronoi_visual_utils<coordinate_type>::discretize(point, segment, max_dist, sampled_edge);
}

point_type PathMaker::retrieve_point(const cell_type& cell)
{
	source_index_type index = cell.source_index();
	source_category_type category = cell.source_category();
	if (category == boost::polygon::SOURCE_CATEGORY_SINGLE_POINT) {
		return _points[index];
	}
	index -= _points.size();
	if (category == boost::polygon::SOURCE_CATEGORY_SEGMENT_START_POINT) {
		return low(_segments[index]);
	}
	else {
		return high(_segments[index]);
	}
}

segment_type PathMaker::retrieve_segment(const cell_type& cell) {
	source_index_type index = cell.source_index() - _points.size();
	return _segments[index];
}

bool PathMaker::sample_close_to_obstacle(const point_type& sample, const edge_type& edge, const float& allowed_obs_dist) {
	if (edge.cell()->contains_point() == false) {
		segment_type segment = retrieve_segment(*edge.cell());
		point_type p0 = low(segment);
		p0 = point_type(p0.x() / 1000.f, p0.y() / 1000.f);
		point_type p1 = high(segment);
		p1 = point_type(p1.x() / 1000.f, p1.y() / 1000.f);

		if (distance_to_line(sample, p0, p1) < allowed_obs_dist)
			return true;
		else
			return false;
	}
	else
		return true;
}

void PathMaker::construct_graph_from_vd(const VD& vd, Graph& g) {
	float allowed_obs_dist = _discontinuityThreshold/2;
	color_close_vertices(vd, allowed_obs_dist);
	for(const_vertex_iterator it_v = vd.vertices().begin(); it_v != vd.vertices().end(); ++it_v) {
		if (it_v->color() == 1)
			continue;
		point_type vertex(it_v->x()/1000.f, it_v->y()/1000.f);
		GNode* gNode = new GNode(vertex); // TODO delete GNode after the current frame
		g.add_node(gNode);
	}

	for(const_edge_iterator it_e = vd.edges().begin(); it_e != vd.edges().end(); ++it_e)
	{
		if (it_e->is_finite() && it_e->is_primary())
		{
			if (it_e->vertex0()->color() == 1 || it_e->vertex1()->color() == 1)
				continue;
			if (it_e->is_curved())
			{
				point_type point0(it_e->vertex0()->x(), it_e->vertex0()->y());
				point_type point1(it_e->vertex1()->x(), it_e->vertex1()->y());

				std::vector<point_type> samples;
				samples.push_back(point0);
				samples.push_back(point1);
				sample_curved_edge(*it_e, &samples); // TODO add this function to PathMaker

				// Create graph nodes for each sample point except the end points
				// since they have already been added in the graph before.
				std::vector<point_type>::iterator it = samples.begin() + 1;
				for (; it != samples.end() - 1; ++it) {
					point_type sample_pt(it->x() / 1000.f, it->y() / 1000.f);
					if (sample_close_to_obstacle(sample_pt, *it_e, allowed_obs_dist) == false) {
						GNode* gNode = new GNode(sample_pt);
						g.add_node(gNode);
					}
				}

				// Iterate over all the sampled nodes for this edge and add
				// their neighbors in the graph.
				it = samples.begin();
				for (; it + 1 != samples.end(); ++it) {
					point_type sampled_pt0(it->x() / 1000.f, it->y() / 1000.f);
					point_type sampled_pt1((it + 1)->x() / 1000.f, (it + 1)->y() / 1000.f);
					GNode* node0 = g.get_node(sampled_pt0);
					GNode* node1 = g.get_node(sampled_pt1);
					if (!node0 || !node1) continue;
					node0->_neighbors.push_back(node1);
					node1->_neighbors.push_back(node0);
					float weight = distance_between_points(sampled_pt0, sampled_pt1);
					node0->_edge_weights.push_back(weight);
					node1->_edge_weights.push_back(weight);
				}
			}
			else
			{
				point_type point0(it_e->vertex0()->x() / 1000.f, it_e->vertex0()->y() / 1000.f);
				point_type point1(it_e->vertex1()->x() / 1000.f, it_e->vertex1()->y() / 1000.f);
				GNode* node0 = g.get_node(point0);
				GNode* node1 = g.get_node(point1);
				node0->_neighbors.push_back(node1);
				node1->_neighbors.push_back(node0);
				float weight = distance_between_points(point0, point1);
				node0->_edge_weights.push_back(weight);
				node1->_edge_weights.push_back(weight);

			}
		}

	}	
};

GNode* minDistance(std::list<GNode*>& temp_nodes) {

	std::list<GNode*>::iterator n_it = temp_nodes.begin();
	float min_dist = (*n_it)->_dist;
	GNode* min_node = *n_it;
	++n_it;
	for(; n_it != temp_nodes.end(); ++n_it) {
		float temp_dist = (*n_it)->_dist;
		if (temp_dist < min_dist) {
			min_dist = temp_dist;
			min_node = *n_it;
		}
	}
	return min_node;
}


bool Graph::compute_shortest_path(std::vector<point_type>& out_path, GNode* start_node, GNode* end_node) {

	bool flag = false;	
	std::list<point_type> s_path;
	std::list<GNode*> temp_nodes;
	std::vector<GNode*>::iterator node_it = this->_nodes.begin();
	
	if (!start_node || !end_node) {
		std::cout << "Please specify both start and end nodes" << std::endl;
		return flag;
	}
	
	temp_nodes.push_back(start_node);
	for(; node_it != this->_nodes.end(); ++node_it) {
		(*node_it)->_dist = 1000000.f;
		temp_nodes.push_back(*node_it);
	}
	start_node->_dist = 0.f;

	for(size_t count = 0; count < this->_nodes.size()-1; ++count) {

		GNode* minNode = minDistance(temp_nodes);
		temp_nodes.remove(minNode);
		minNode->_flag = true;
		for(size_t count2 = 0; count2 < minNode->_neighbors.size(); ++count2) {
			GNode* neighbor = minNode->_neighbors[count2];
			float temp_dist = minNode->_dist + minNode->_edge_weights[count2];
			if(neighbor->_flag == false && temp_dist < neighbor->_dist) {
				neighbor->_dist = temp_dist;
				neighbor->_parent = minNode;
			}
		}
	}

	if (end_node->_flag == false) {
		std::cout << "There is no path from the start node to the end node." << std::endl;
		return flag;
	}

	GNode* node = end_node;
	std::string dist_string = "";
	dist_string += std::to_string(node->_dist);
	s_path.push_back(node->_vertex);
	while(node->_parent) {
		node = node->_parent;
		dist_string += "--> ";
		dist_string += std::to_string(node->_dist);
		s_path.push_back(node->_vertex);
	}
	if (node == start_node) {
		std::cout << "The distances (in reverse order) along the path are: " << dist_string << std::endl;
		s_path.reverse();
		for (std::list<point_type>::iterator it = s_path.begin(); it != s_path.end(); ++it)
			out_path.push_back(*it);
		flag = true;
	}
	else {
		std::cout << "There is no path from the start node to the end node." << std::endl;
		s_path.clear();
	}
	return flag;
}

bool PathMaker::get_path(std::vector<point_type>& out_path, const VD& vd, const point_type& sVertex, const point_type& gVertex) {

	Graph graph;
	construct_graph_from_vd(vd, graph);
	GNode* start_node = graph.get_node(sVertex);
	GNode* goal_node = graph.get_node(gVertex);
	bool result = graph.compute_shortest_path(out_path, start_node, goal_node);
	return result;
}
