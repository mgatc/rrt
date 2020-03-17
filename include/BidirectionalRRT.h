#ifndef BIDIRECTIONALRRT_H
#define BIDIRECTIONALRRT_H

#include "CgalComponents.h"
#include "RRT.h"

namespace MAG {

class BidirectionalRRT : public RRT {

public:
    /* Public data members */

    /* Constructor(s) */
        BidirectionalRRT( Point startPoint, Point goalPoint, double step, int maxNodes );

    /* Public member functions */
//        bool go();

private:
    /* Private data members */
        DelaunayTriangulation nearestNeighborTreeB;

    /* Private RRT specified functions */
        bool buildRRT();

    /* Private member functions */

};

}

#endif
