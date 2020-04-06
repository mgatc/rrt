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
    Vertex rand;

    for( unsigned k=0; k<K; k++ ) {
        rand = randomState();    // generate random point
        if( extend( nearestNeighborTree, rand ) != Trapped  ) {
            if( last && ( extend( nearestNeighborTreeB, *last ) == Reached ) ) {
                return path();
            }
        }
        std::swap( nearestNeighborTree, nearestNeighborTreeB );
    }

    return std::list<Vertex>();
}

RRT::Result BidirectionalRRT::extend( DelaunayTriangulation &Dt, Vertex x ) {
    GraphVertex near = nearestNeighbor( Dt, x.point );

    std::optional<Vertex> xNew = newState( x, locationMap[near], false );

    if( xNew ) {
        last = { insertIntoTree( Dt, *xNew, near ) };
        return ( xNew->point == x.point ) ? Reached : Advanced;
    }
    last = std::nullopt; // if no xNew, set last to null
    return Trapped;
}

void BidirectionalRRT::init( Point start, Point goal ) {

    startv = insertVertex( nearestNeighborTree, start );
    goalv = insertVertex( nearestNeighborTreeB, goal );

}


