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

protected:
    /* Protected data members */
        DelaunayTriangulation nearestNeighborTreeB;

    /* Protected RRT specified functions */
        std::list<Vertex> buildRRT();

    /* Protected member functions */
        void init( Point start, Point goal );

private:

};

}

#endif
