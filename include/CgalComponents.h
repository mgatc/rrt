#ifndef CGALCOMPONENTS_H
#define CGALCOMPONENTS_H

/********************** STL header files **************************/





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



namespace MAG {



}

#endif
