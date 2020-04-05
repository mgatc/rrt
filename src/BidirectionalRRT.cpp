#include "../include/BidirectionalRRT.h"
#include "../include/CgalComponents.h"
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/nearest_neighbor_delaunay_2.h>



using namespace MAG;



/* Constructor(s) */

BidirectionalRRT::BidirectionalRRT( Point startPoint, Point goalPoint, double step, int maxNodes ) :
                                        RRT( startPoint, goalPoint, step, maxNodes, 0 )
{

}



std::list<Vertex> BidirectionalRRT::buildRRT() {
    Point rand;

//    // Initialize T with start location
//    insertIntoTree( nearestNeighborTree, startv.point, std::nullopt );
//    insertIntoTree( nearestNeighborTreeB, goalv.point, std::nullopt );

    for( unsigned k=0; k<K; k++ ) {
        rand = randomState();    // generate random point
        if( extend( nearestNeighborTree, rand ) != Trapped  ) {
            if( last && ( extend( nearestNeighborTreeB, *last ) == Reached ) ) {
                // Trees do not connect properly here
                return path();
            }
        }
        std::swap( nearestNeighborTree, nearestNeighborTreeB );
    }
    return std::list<Vertex>();
}

void BidirectionalRRT::init( Point start, Point goal ) {

    startv = insertVertex( nearestNeighborTree, start );
    goalv = insertVertex( nearestNeighborTreeB, goal );

}

