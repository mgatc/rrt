#ifndef CGALCOMPONENTS_H
#define CGALCOMPONENTS_H

/********************** STL header files **************************/

#include <list>                             // path output
#include <fstream>                          // read from file
#include <string>                           // string editing for output
#include <vector>                           // A*
#include <iostream>                         // A*
#include <math.h>                           // for sqrt, A*



/********************** BOOST header files **************************/

#include <boost/graph/adjacency_list.hpp>               // graph
#include <boost/graph/astar_search.hpp>                 // A*
#include <boost/graph/random.hpp>                       // A*
#include <boost/random.hpp>                             // A*
#include <boost/property_map/property_map.hpp>          // A*
#include <boost/graph/graphviz.hpp>                     // A*



/********************** CGAL headers files **************************/

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h> // all CGAL components
#include <CGAL/Delaunay_triangulation_2.h>                      // nearest neighbors
#include <CGAL/nearest_neighbor_delaunay_2.h>                   // nearest neighbors
#include <CGAL/Triangulation_vertex_base_with_info_2.h>         // nearest neighbors
#include <CGAL/point_generators_2.h>                            // random point generation
#include <CGAL/squared_distance_2.h>                            // A*
#include <CGAL/number_utils.h>                                  // A*



/********************** TYPEDEFs **************************/

namespace boost {
    enum vertex_location_t { vertex_location };
    BOOST_INSTALL_PROPERTY( vertex, location );
}

namespace MAG {

    typedef CGAL::Exact_predicates_inexact_constructions_kernel
        K;

    typedef K::Point_2
        Point;

    typedef double
        NumberType;

    typedef CGAL::Creator_uniform_2< NumberType, Point >
        Creator;

    typedef NumberType
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

    struct Vertex {
        Point point;
        GraphVertex graph;
        DtVertex handle;

        Vertex operator=( Vertex rhs ) {
            point = rhs.point;
            graph = rhs.graph;
            handle = rhs.handle;
        }
    };
    const double GOAL_EPSILON = 2;

}

#endif
