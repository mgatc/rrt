//#include "RRT.h"
//#include "CgalComponents.h"
//#include <CGAL/Delaunay_triangulation_2.h>
//#include <CGAL/nearest_neighbor_delaunay_2.h>
//
//
//RRT::RRT( Point start, Point goal ) :
//    start( start ), goal( goal ), g1( this->size ) {
//
//
//    cout << "start: " << this->start.x() << "," << this->start.y() << endl;
//    cout <<  "goal: " << this->goal.x()  << "," << this->goal.y()  << endl;
//
//    this->buildRRT();
//
//    for( unsigned i=0; i<this->T.E.size(); i++ ) {
//        cout << this->T.E[i].p.x() << "," << this->T.E[i].p.y() << " " << this->T.E[i].q.x() << "," << this->T.E[i].q.y()  << endl;
//    }
//
//}
//
//bool RRT::buildRRT() {
//    Point rand;
//    // Initialize T with start location
//    this->T.V.insert( start );
//    for( unsigned k=0; k<(this->K); k++ ) {
//        rand = this->randomState();    // generate random point
//        this->extend( rand );
//    }
//    // return T
//    return false;
//}
//bool RRT::extend( Point x ) {
//    Point near = this->nearestNeighbor( x );
//    Point xNew = this->newState( x, near, false );
//    if( xNew != NULL ) {
//        // tree.addVertex( xNew );
//        this->T.V.insert( xNew );
//        this->T.E.push_back( { near, xNew, false } );
//        // tree.addEdge( xNear, xNew, uNew );
//        // if xNew == x
//            //return reached
//        // else
//            // return advanced
//    }
//    // return trapped
//    return false;
//}
//
//Point RRT::randomState() {
//    return *( this->g1++ );
//}
//Point RRT::nearestNeighbor( Point &x ) {
//    Vertex_handle handleToTheNearestPoint = this->T.V.nearest_vertex( *x );
//    return handleToTheNearestPoint->point();
//}
//Point RRT::newState( Point x, Point xNear, bool uNew ) {
//    // determines if x is reachable from xNear under global constraints
//    // returns the new state or failure notice
//    return x;
//}
