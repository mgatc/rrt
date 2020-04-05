#ifndef CGALCOMPONENTS_H
#define CGALCOMPONENTS_H

/********************** STL header files **************************/


#include <list>                             // path output
#include <fstream>                          // read from file
#include <string>                           // string editing for output
#include <vector>                           // A*
#include <iostream>                         // A*
#include <math.h>                           // for sqrt, A*
//#include <algorithm>
//#include <iterator>
//#include <random>
//#include <ctime>
//#include <unordered_set>
//#include <chrono>



/********************** BOOST header files **************************/

#include <boost/graph/adjacency_list.hpp>               // graph
#include <boost/graph/astar_search.hpp>                 // A*
#include <boost/graph/random.hpp>                       // A*
#include <boost/random.hpp>                             // A*
#include <boost/property_map/property_map.hpp>          // A*
#include <boost/graph/graphviz.hpp>                     // A*
//#include <boost/thread/thread.hpp>

// these were added to troubleshoot BOOST_INSTALL_PROPERTY not working
//#include <boost/config.hpp>
//#include <boost/version.hpp>
//#include <boost/graph/graph_utility.hpp>
//#include <boost/static_assert.hpp>




/********************** CGAL headers files **************************/

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h> // all CGAL components
#include <CGAL/Delaunay_triangulation_2.h>                      // nearest neighbors
#include <CGAL/nearest_neighbor_delaunay_2.h>                   // nearest neighbors
#include <CGAL/Triangulation_vertex_base_with_info_2.h>         // nearest neighbors
#include <CGAL/point_generators_2.h>                            // random point generation
#include <CGAL/squared_distance_2.h>                            // goal test
//#include <CGAL/algorithm.h>
//#include <CGAL/Algebraic_kernel_for_circles_2_2.h>
//#include <CGAL/Circular_kernel_2.h>
//#include <CGAL/Iso_rectangle_2.h>
//#include <CGAL/random_convex_set_2.h>
//#include <CGAL/random_selection.h>
//#include <CGAL/Point_set_2.h>
//#include <CGAL/Circular_kernel_intersections.h>
//#include <CGAL/Exact_circular_kernel_2.h>
//#include <CGAL/Orthogonal_k_neighbor_search.h>
//#include <CGAL/Search_traits_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>



/********************** TYPEDEFs **************************/

namespace boost {
    enum vertex_location_t { vertex_location };
    BOOST_INSTALL_PROPERTY( vertex, location );
}

namespace MAG {

//    typedef CGAL::Exact_predicates_exact_constructions_kernel
//        K;
    typedef CGAL::Exact_predicates_inexact_constructions_kernel
        K;

    typedef K::Point_2
        Point;

    typedef CGAL::Creator_uniform_2<double,Point>
        Creator;

    typedef double
        Cost;

    struct Vertex;

    typedef boost::adjacency_list<
            boost::listS,
            boost::vecS,
            boost::undirectedS,
            boost::property< boost::vertex_location_t, Vertex >,
            boost::property< boost::edge_weight_t, Cost >
        >
        Graph;

    typedef boost::graph_traits<Graph>::vertex_descriptor
        GraphVertex;

    typedef boost::graph_traits<Graph>::edge_descriptor
        GraphEdge;

    typedef CGAL::Triangulation_vertex_base_with_info_2<GraphVertex, K>
        Vb;

    typedef CGAL::Triangulation_data_structure_2<Vb>
        Tds;

    typedef CGAL::Delaunay_triangulation_2<K, Tds>
        DelaunayTriangulation;

    typedef DelaunayTriangulation::Vertex_handle
        DtVertex;

    typedef boost::property_map< Graph, boost::edge_weight_t >::type
        WeightMap;

    typedef boost::property_map< Graph, boost::vertex_location_t >::type
        LocationMap;
//
//    struct vertex_info {
//        Point *point;
//        //add getPoint function
////        Point point() {
////            DtVertex v = handle;
////            return v->point();
////        }
//    };

    struct Vertex {
        Point point;
        GraphVertex graph;
        DtVertex handle;
    };
    const double GOAL_EPSILON = 2;

//    typedef K::Segment_2
//        Segment;
//
//    typedef CGAL::Circle_2<K>
//        Circle;
//
//    typedef CGAL::Iso_rectangle_2<K>
//        Rectangle;
//
//    typedef CGAL::Point_set_2<K, Tds>
//        PS;

}

#endif
