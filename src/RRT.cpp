#include "RRT.h"
#include "CgalComponents.h"

RRT::RRT( Point start, Point goal ) :
    start( start ), goal( goal ) {
    Random_points_in_square_2<Point,Creator> g( size ); // generate random points bounded by size
    this->g1 = &g;

    cout << "start: " << this->start.x() << "," << this->start.y() << endl;
    cout <<  "goal: " << this->goal.x() << "," << this->goal.y() << endl;

}

bool RRT::buildRRT( Point xInit ) {
    // Initialize T with xInit
    // for k=1 to K
        // xRand = this->randomState();
        // this->extend( xRand );
    // return T
    return false;
}
bool RRT::extend( Point x ) {
    // xNear = this->nearestNeighbor( x );
    // xNew = this->newState( x, xNear, uNew );
    // if xNew
        // tree.addVertex( xNew );
        // tree.addEdge( xNear, xNew, uNew );
        // if xNew == x
            //return reached
        // else
            // return advanced
    // return trapped
    return false;
}

Point RRT::randomState() {
    advance( *&(this->g1) );
    return this->g1;
}
Point RRT::nearestNeighbor( Point x ) {

}
Point RRT::newState( Point x, Point xNear, Point uNew ) {
    // determines if x is reachable from xNear under global constraints
    // returns the new state or failure notice
}
