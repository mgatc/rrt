#ifndef POINT2_H
#define POINT2_H

#include <cmath>                           // for sqrt, A*
#include <fstream>                          // read from file
#include <iostream>                             //
#include <limits>
#include <list>                                 // path output
#include <string>                               // string editing for output and files
#include <vector>                               // property maps

#include <boost/graph/adjacency_list.hpp>               // graph
#include <boost/graph/astar_search.hpp>         // A*
#include <boost/graph/graphviz.hpp>                     // A*
#include <boost/graph/properties.hpp>           // graph
#include <boost/graph/random.hpp>                       // A*
#include <boost/property_map/property_map.hpp>          // A*
#include <boost/random.hpp>                             // A*

#include <CGAL/Delaunay_triangulation_2.h>                      // nearest neighbors
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h> // all CGAL components
#include <CGAL/nearest_neighbor_delaunay_2.h>                   // nearest neighbors
#include <CGAL/number_utils.h>                                  // A*
#include <CGAL/point_generators_2.h>                            // random point generation
#include <CGAL/Random.h>                                        // seeding random generator
#include <CGAL/squared_distance_2.h>                            // A*
//#include <CGAL/Simple_cartesian.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>         // nearest neighbors

namespace boost {
    enum vertex_location_t { vertex_location };
    BOOST_INSTALL_PROPERTY( vertex, location );
}

namespace MAG{

    const size_t SIZE_T_MAX = std::numeric_limits<size_t>::max();

    const double GOAL_EPSILON = 1;

    typedef CGAL::Exact_predicates_inexact_constructions_kernel
        K;

    typedef K::Point_2
        Point;

    typedef double
        Cost;

    typedef CGAL::Creator_uniform_2< Cost, Point >
        Creator;

    struct Vertex;

    typedef boost::adjacency_list<
            boost::listS,
            boost::vecS,
            boost::bidirectionalS,
            boost::property< boost::vertex_location_t, Vertex >,
            boost::property< boost::edge_weight_t, Cost >
        >
        Graph;

    typedef boost::graph_traits<Graph>::vertex_descriptor
        GraphVertex;

    typedef boost::graph_traits<Graph>::edge_descriptor
        GraphEdge;

    typedef boost::property_map< Graph, boost::vertex_index_t >::type
        VertexPropertyMapType;

    typedef CGAL::Triangulation_vertex_base_with_info_2<GraphVertex, K>
        Vb;

    typedef CGAL::Triangulation_data_structure_2<Vb>
        Tds;

    typedef CGAL::Delaunay_triangulation_2<K, Tds>
        DelaunayTriangulation;

    typedef DelaunayTriangulation::Vertex_handle
        DtVertex;

    typedef boost::property_map< Graph, boost::edge_weight_t >::type
        EdgeMap;

    typedef boost::property_map< Graph, boost::vertex_location_t >::type
        VertexMap;



    struct Point2 {
        double x;
        double y;

        friend bool operator<( const Point2& l, const Point2& r ) {
            return l.x<r.x || (l.x==r.x&&l.y<r.y);
        }
    };

    struct Vertex {
        Point point;
        GraphVertex graph = SIZE_T_MAX;
        DtVertex handle = nullptr;
        Cost cost = 0;

        Vertex operator=( Vertex rhs ) {
            point = rhs.point;
            graph = rhs.graph;
            handle = rhs.handle;
            cost = rhs.cost;

            return rhs;
        }
    };

}

#endif
