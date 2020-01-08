#pragma once

#include <boost/polygon/polygon.hpp>
#include <boost/polygon/voronoi.hpp>

typedef double coordinate_type;
typedef boost::polygon::point_data<coordinate_type> point_type;
typedef boost::polygon::segment_data<coordinate_type> segment_type;
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

