#include "CgalComponents.h"
#include "BidirectionalRRT.h"
//#include "reactphysics3d-0.7.1/src/reactphysics3d.h"

using namespace MAG;

int main() {
    //RRT_Tree T;
    // Read input from file for map, start, goal, and parameters separately, as these inputs would likely originate from differing origins
    int SIZE = 25;
    CGAL::Random_points_in_square_2<Point,Creator> g1( SIZE ); // random point iterator
    Point start( -20, -20 );
    Point goal( 20, 20 );
//    Point start( *( g1++ ) );
//    Point goal( *( g1++ ) ); // may need to be more generally typed to support goal areas too
    // environment
    // parameters like steering system, constraints, etc

    //RRT rrt( start, goal, 2, 2000 );
    BidirectionalRRT rrt( start, goal, 2, 2000 );
    //rrt.setGoalSkewProbability(50);

    rrt.go();
    rrt.displayPDF( "tempOut" );

    return EXIT_SUCCESS;
}
