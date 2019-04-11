#include <boost/polygon/polygon.hpp>
#include <boost/polygon/voronoi.hpp>
#include "voronoi_visual_utils.hpp"
#include <vector>
using namespace boost::polygon;
using namespace std;

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

/*bool operator==(const point_type& p1, const point_type& p2) {
	if (p1.x() == p2.x() && p1.y() == p2.y())
		return true;
	else
		return false;
}*/

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

