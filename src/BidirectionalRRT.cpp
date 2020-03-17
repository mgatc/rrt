#include "BidirectionalRRT.h"
#include "CgalComponents.h"
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/nearest_neighbor_delaunay_2.h>



using namespace MAG;



/* Constructor(s) */

MAG::BidirectionalRRT::BidirectionalRRT( Point startPoint, Point goalPoint, double step, int maxNodes ) :
                                        RRT( startPoint, goalPoint, step, maxNodes, 0 )
{

}



/* Public member functions */

//bool MAG::BidirectionalRRT::go() {
//    return buildRRT();
//}

/* Private RRT specified functions */

bool MAG::BidirectionalRRT::buildRRT() {
    Point rand;

    // Initialize T with start location
    insertIntoTree( nearestNeighborTree, start, std::nullopt );
    insertIntoTree( nearestNeighborTreeB, goal, std::nullopt );

    for( unsigned k=0; k<K; k++ ) {
        rand = randomState();    // generate random point
        if( ! extend( nearestNeighborTree, rand ) == Trapped ) {
            if( last && extend( nearestNeighborTreeB, *last ) == Reached ) {
                return true;
            }
        }
        std::swap( nearestNeighborTree, nearestNeighborTreeB );
    }
    return false;
}



/* Private member functions */



