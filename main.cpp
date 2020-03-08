#include "CgalComponents.h"
#include "reactphysics3d-0.7.1/src/reactphysics3d.h"
#include "RRT.h"
//#include <chrono>

using namespace MAG;

int main() {
    //RRT_Tree T;
    // Read input from file for map, start, goal, and parameters separately, as these inputs would likely originate from differing origins
    int SIZE = 25;
    Random_points_in_square_2<Point,Creator> g1(SIZE); // random point iterator
    Point start( -20, -20 );
    Point goal( 20, 20 );
//    Point start( *( g1++ ) );
//    Point goal( *( g1++ ) ); // may need to be more generally typed to support goal areas too
    // environment
    // parameters like steering system, constraints, etc

    RRT rrt( start, goal );
    rrt.displayPDF( "tempOut" );

    return EXIT_SUCCESS;
}
