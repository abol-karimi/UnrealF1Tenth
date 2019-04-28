#include <cstdio>
#include <vector>
#include <fstream>
#include <iostream>
#include <vector>
#include "../Public/VoronoiAIController.h"
#include <math.h>
#include "voronoi_visual_utils.hpp"
using namespace std;
class Graph;

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

double PathMaker::distance_between_points(const point_type& p1, const point_type& p2) {
	double x_val = p1.x() - p2.x();
	double y_val = p1.y() - p2.y();
	double weight = sqrt(pow(x_val, 2) + pow(y_val, 2));
	return weight;
}

// https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points
// Calculate distance of p0 to the line passing through p0 and p1
double PathMaker::distance_to_line(const point_type& point, const point_type& p0, const point_type& p1)
{
	double delta_y = p1.y() - p0.y();
	double delta_x = p1.x() - p0.x();
	float denominator = distance_between_points(p0, p1);
	float numerator_const_term = p1.x() * p0.y() - p1.y() * p0.x();
	float numerator = abs(delta_y * point.x() - delta_x * point.y() + numerator_const_term);
	return (numerator / denominator);
}

void PathMaker::construct_vd(const std::vector<SegmentFloat>& floatsegments, const std::vector<PointFloat>& floatpoints) {
	std::vector<PointFloat>::const_iterator pf_it = floatpoints.begin();
	for(; pf_it != floatpoints.end(); ++pf_it)
		_points.push_back(cast_to_point_type(*pf_it));

	std::vector<SegmentFloat>::const_iterator sf_it = floatsegments.begin();
	for( ; sf_it != floatsegments.end(); ++sf_it)
		_segments.push_back(cast_to_segment_type(*sf_it));
	
	construct_voronoi(_points.begin(), _points.end(), _segments.begin(), _segments.end(), &_vd);
}

void PathMaker::construct_vd(const std::vector<SegmentFloat>& floatsegments) {
	std::vector<SegmentFloat>::const_iterator sf_it = floatsegments.begin();
	for( ; sf_it != floatsegments.end(); ++sf_it)
		_segments.push_back(cast_to_segment_type(*sf_it));
	
	construct_voronoi(_segments.begin(), _segments.end(), &_vd);
}

void PathMaker::construct_vd(const std::vector<PointFloat>& floatpoints) {
	std::vector<PointFloat>::const_iterator pf_it = floatpoints.begin();
	for(; pf_it != floatpoints.end(); ++pf_it)
		_points.push_back(cast_to_point_type(*pf_it));

	construct_voronoi(_points.begin(), _points.end(), &_vd);
}

int PathMaker::print_primary_edges() {
  int result = 0;
  for (const_edge_iterator it = _vd.edges().begin(); it != _vd.edges().end(); ++it) {
	  if (it->is_finite() && it->is_primary()) {
		  point_type v0(it->vertex0()->x()/1000.f, it->vertex0()->y()/1000.f);
		  point_type v1(it->vertex1()->x()/1000.f, it->vertex1()->y()/1000.f);
		  printf("\nedge: vertex0=(%f, %f), vertex1=(%f, %f)", v0.x(), v0.y(), v1.x(), v1.y());
		  ++result;
    }
  }
  return result;
};

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

void PathMaker::print_point_type(const point_type& point) {
	std::cout << "Point: (" << point.x() << ", " << point.y() << ")" << std::endl;
}

void PathMaker::print_vertex_type(const vertex_type& vertex) {
	point_type v_point(vertex.x()/1000.f, vertex.y()/1000.f);
	std::cout << "Vertex: (" << v_point.x() << ", " << v_point.y() << ")" << std::endl;
}

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
			point_type point = edge_cell->contains_point() ? retrieve_point(*edge_cell) : retrieve_point(*edge.twin()->cell());
			segment_type segment = edge_cell->contains_point() ? retrieve_segment(*edge.twin()->cell()) : retrieve_segment(*edge_cell);
			point_type p0 = low(segment);
			p0 = point_type(p0.x() / 1000.f, p0.y() / 1000.f);
			point_type p1 = high(segment);
			p1 = point_type(p1.x() / 1000.f, p1.y() / 1000.f);

			if (distance_to_line(v0, p0, p1) < allowed_obs_dist || distance_between_points(v0, point) < allowed_obs_dist)
				e_vertex0->color(1);
			if (distance_to_line(v1, p0, p1) < allowed_obs_dist || distance_between_points(v1, point) < allowed_obs_dist)
				e_vertex1->color(1);
		}
	}
}

void PathMaker::sample_curved_edge(const edge_type& edge, std::vector<point_type>* sampled_edge)
{
	coordinate_type max_dist = 10;
	point_type point = edge.cell()->contains_point() ? retrieve_point(*edge.cell()) : retrieve_point(*edge.twin()->cell());
	segment_type segment = edge.cell()->contains_point() ? retrieve_segment(*edge.twin()->cell()) : retrieve_segment(*edge.cell());
	voronoi_visual_utils<coordinate_type>::discretize(point, segment, max_dist, sampled_edge);
}

point_type PathMaker::retrieve_point(const cell_type& cell)
{
	source_index_type index = cell.source_index();
	source_category_type category = cell.source_category();
	if (category == SOURCE_CATEGORY_SINGLE_POINT) {
		return _points[index];
	}
	index -= _points.size();
	if (category == SOURCE_CATEGORY_SEGMENT_START_POINT) {
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

bool PathMaker::sample_close_to_obstacle(const point_type& sample, const edge_type& edge, const double& allowed_obs_dist) {
	point_type point = edge.cell()->contains_point() ? retrieve_point(*edge.cell()) : retrieve_point(*edge.twin()->cell());
	segment_type segment = edge.cell()->contains_point() ? retrieve_segment(*edge.twin()->cell()) : retrieve_segment(*edge.cell());
	point_type p0 = low(segment);
	p0 = point_type(p0.x() / 1000.f, p0.y() / 1000.f);
	point_type p1 = high(segment);
	p1 = point_type(p1.x() / 1000.f, p1.y() / 1000.f);

	if (distance_to_line(sample, p0, p1) < allowed_obs_dist || distance_between_points(sample, point) < allowed_obs_dist)
		return true;
	else
		return false;
}

void PathMaker::construct_graph_from_vd(const VD& vd, Graph& g) {
	double allowed_obs_dist = _discontinuityThreshold/2;
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
				std::size_t idx_s = 1;
				//std::vector<point_type>::iterator it_s = samples.begin()+1;
				for ( ; idx_s < samples.size(); ++idx_s) {
					point_type sample_pt(samples[idx_s].x() / 1000.f, samples[idx_s].y() / 1000.f);
					if (sample_close_to_obstacle(sample_pt, *it_e, allowed_obs_dist) == false) {
							GNode* gNode = new GNode(sample_pt);
							g.add_node(gNode);
					}
					else {
						++idx_s;
						break;
					}
				}

				std::size_t closest_sample_idx = idx_s;
				// Iterate over all the sampled nodes for this edge and add
				// their neighbors in the graph.
				
				idx_s = 0;
				for (; idx_s < closest_sample_idx; ++idx_s) {
					point_type sample_pt0(samples[idx_s].x() / 1000.f, samples[idx_s].y() / 1000.f);
					point_type sample_pt1(samples[idx_s+1].x() / 1000.f, samples[idx_s+1].y() / 1000.f);
					GNode* node0 = g.get_node(sample_pt0);
					GNode* node1 = g.get_node(sample_pt1);
					if (node1 == NULL) break;
					node0->_neighbors.push_back(node1);
					node1->_neighbors.push_back(node0);
					double weight = distance_between_points(sample_pt0, sample_pt1);
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
				double weight = distance_between_points(point0, point1);
				node0->_edge_weights.push_back(weight);
				node1->_edge_weights.push_back(weight);

			}
		}

	}	
};

GNode* minDistance(std::list<GNode*>& temp_nodes) {

	std::list<GNode*>::iterator n_it = temp_nodes.begin();
	double min_dist = (*n_it)->_dist;
	GNode* min_node = *n_it;
	++n_it;
	for(; n_it != temp_nodes.end(); ++n_it) {
		double temp_dist = (*n_it)->_dist;
		if (temp_dist < min_dist) {
			min_dist = temp_dist;
			min_node = *n_it;
		}
	}
	return min_node;
}

void Graph::print() {

	for(std::vector<GNode*>::iterator node_it = _nodes.begin();
			node_it != _nodes.end(); ++node_it)
		cout << (*node_it)->_dist << std::endl;
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
			double temp_dist = minNode->_dist + minNode->_edge_weights[count2];
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

bool PathMaker::get_path(std::vector<point_type>& out_path, const point_type& sVertex, const point_type& gVertex) {

	Graph graph;	
	construct_graph_from_vd(_vd, graph);
	//point_type sVertex(0/1000.f, 0/1000.f);
	GNode* start_node = graph.get_node(sVertex);
	//point_type eVertex(1500/1000.f, 2500/1000.f);
	GNode* goal_node = graph.get_node(gVertex);
	bool result = graph.compute_shortest_path(out_path, start_node, goal_node);
	return result;
};

bool PathMaker::get_path(std::vector<point_type>& out_path, const VD& vd, const point_type& sVertex, const point_type& gVertex) {

	Graph graph;
	construct_graph_from_vd(vd, graph);
	GNode* start_node = graph.get_node(sVertex);
	GNode* goal_node = graph.get_node(gVertex);
	bool result = graph.compute_shortest_path(out_path, start_node, goal_node);
	return result;
}

//int main() {
//
//	std::vector<SegmentFloat> sf;
//	sf.push_back(SegmentFloat(1, 0, 1, 1));
//	sf.push_back(SegmentFloat(1, 1, 2, 2));
//	sf.push_back(SegmentFloat(-1, 0, -1, 1));
//	sf.push_back(SegmentFloat(-1, 1, -2, 2));
//	sf.push_back(SegmentFloat(0, 2, 1, 3));
//	sf.push_back(SegmentFloat(0, 2, -1, 3));
//	PathMaker vs;
//	vs.construct_vd(sf);
//	std::vector<point_type> path;
//	point_type sVertex(0/1000.f, 0/1000.f);
//	point_type eVertex(1500/1000.f, 2500/1000.f);
//	bool result = vs.get_path(path, sVertex, eVertex); 
//	if (result) {
//		for (std::vector<point_type>::iterator it = path.begin(); it != path.end(); ++it) {
//			printf("\nvertex=(%f, %f)", (*it).x(), (*it).y());
//		}
//	}
//	return 0;
//}
