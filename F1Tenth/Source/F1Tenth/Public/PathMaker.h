#include "VoronoiDefinitions.h"

#include <vector>

/*bool operator==(const point_type& p1, const point_type& p2) {
	if (p1.x() == p2.x() && p1.y() == p2.y())
		return true;
	else
		return false;
}*/


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