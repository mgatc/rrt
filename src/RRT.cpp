#include "RRT.h"
#include "CgalComponents.h"
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/nearest_neighbor_delaunay_2.h>


RRT::RRT( RRT_Tree &tree, Point start, Point goal ) :
    T( &tree ), start( start ), goal( goal ), K( 2000 ), size( 50 ), epsilon( 2 ), g1( this->size ) {

    cout << "start: " << this->start.x() << "," << this->start.y() << endl;
    cout <<  "goal: " << this->goal.x()  << "," << this->goal.y()  << endl;

    this->buildRRT();
}

bool RRT::buildRRT() {
    Point rand;
    // Initialize T with start location
    this->T->V.insert( start );
    for( unsigned k=0; k<(this->K); k++ ) {
        rand = this->randomState();    // generate random point
        this->extend( rand );
    }
    // return T
    return false;
}
int RRT::extend( Point x ) {
    Point near = this->nearestNeighbor( x );
    optional<Point> xNew = this->newState( x, near, false );
    if( xNew ) {
        this->T->V.insert( *xNew );
        this->T->E.push_back( { near, *xNew, false } );
        if( *xNew == x ) {
            return REACHED;
        } else {
            return ADVANCED;
        }
    }
    return TRAPPED;
}

Point RRT::randomState() {
    return *( this->g1++ );
}
Point RRT::nearestNeighbor( Point x ) {
    Vertex_handle handleToTheNearestPoint = this->T->V.nearest_vertex( x );
    return handleToTheNearestPoint->point();
}
optional<Point> RRT::newState( Point x, Point xNear, bool uNew ) {
    //Point xNew = x;
    // if x not reachable from xNear under global constraints
        // return {};
    // make a move from xNear towards x under discrete time law
    double factor = this->epsilon / sqrt( pow(x.x()-xNear.x(),2) + pow(x.y()-xNear.y(),2) );

    Point p( xNear.x()+factor*(x.x()-xNear.x()), xNear.y()+factor*(x.y()-xNear.y()) );

    // returns the new state or failure notice
    return { p };
}
