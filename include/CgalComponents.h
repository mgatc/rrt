#ifndef CGALCOMPONENTS_H
#define CGALCOMPONENTS_H

#include <vector>
#include <list>
#include <algorithm>
#include <ctime>
#include <unordered_set>
#include <fstream>
#include <iterator>
#include <string>
#include <random>
#include <chrono>



#include <boost/graph/adjacency_list.hpp>

//#include <boost/thread/thread.hpp>
/**********************CGAL headers files**************************/
#include <CGAL/point_generators_2.h>
#include <CGAL/random_convex_set_2.h>
#include <CGAL/algorithm.h>
#include <CGAL/random_selection.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Point_set_2.h>
#include <CGAL/squared_distance_2.h>
#include <CGAL/Circular_kernel_intersections.h>
#include <CGAL/Exact_circular_kernel_2.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Search_traits_2.h>

#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
#include <CGAL/Algebraic_kernel_for_circles_2_2.h>
#include <CGAL/Circular_kernel_2.h>

#include <CGAL/Iso_rectangle_2.h>
//
//#include <CGAL/Simple_cartesian.h>
//typedef CGAL::Simple_cartesian<double> K;

//#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
//typedef CGAL::Exact_predicates_exact_constructions_kernel K;

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;


/******************************************************************/
using namespace CGAL;
using namespace std;



typedef K::Point_2                       Point;
typedef K::Segment_2                     Segment;
typedef Creator_uniform_2<double,Point>  Creator;
typedef Circle_2<K> Circle;
typedef Iso_rectangle_2<K> Rectangle;

namespace MAG {
    struct vertex_info {
        Point p;
    };
    struct edge_info {

    };
}

typedef boost::adjacency_list<boost::listS, boost::listS, boost::undirectedS, MAG::vertex_info, MAG::edge_info> Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
typedef CGAL::Triangulation_vertex_base_with_info_2<vertex_descriptor, K> Vb;
typedef CGAL::Triangulation_data_structure_2<Vb> Tds;
typedef CGAL::Delaunay_triangulation_2<K, Tds> DelaunayTriangulation;
typedef DelaunayTriangulation::Vertex_handle Vertex_handle;
typedef CGAL::Point_set_2<K, Tds> PS;



#endif
