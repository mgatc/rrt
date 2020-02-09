#include "UnitDiskCoverPrinter.h"
#include "UnitDiskCoverCenters.h"
#include "UnitDiskCoverUnitGrid.h"
#include "RRT.h"
#include "CgalComponents.h"
#include <chrono>

#define PI 3.14159265

int main() {
    // Read input from file for map, start, goal, and parameters separately, as these inputs would likely originate from differing origins
    Point start(0,0);
    Point goal(5,4); // may need to be more generally typed to support goal areas too
    // environment
    // parameters like steering system, constraints, etc

    RRT rrt( start, goal );



// How to print something
//    UnitDiskCoverPrinter q(P,C1,1,"someOutput2");
//    q.displayPDF();

    return EXIT_SUCCESS;
}
