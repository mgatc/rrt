#include "RRTPrinter.h"
#include "UnitDiskCoverCenters.h"
#include "UnitDiskCoverUnitGrid.h"
#include "RRT.h"
#include "RRT_Tree.h"
#include "CgalComponents.h"
#include <chrono>

#define PI 3.14159265

int main() {
    RRT_Tree T;
    // Read input from file for map, start, goal, and parameters separately, as these inputs would likely originate from differing origins
    Point start(-25,-40);
    Point goal(35,20); // may need to be more generally typed to support goal areas too
    // environment
    // parameters like steering system, constraints, etc

    RRT rrt( T, start, goal );



// How to print something
    RRTPrinter q( T,start,goal,"someOutput" );
    q.displayPDF();

    return EXIT_SUCCESS;
}
