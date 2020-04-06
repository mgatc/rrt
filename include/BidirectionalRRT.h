#ifndef BIDIRECTIONALRRT_H
#define BIDIRECTIONALRRT_H

#include "../include/CgalComponents.h"
#include "../include/RRT.h"

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
        std::optional<Vertex> last;  // the latest point to be added to the tree

    /* Protected RRT specified functions */
        std::list<Vertex> buildRRT();
        Result extend( DelaunayTriangulation &Dt, Vertex x );

    /* Protected member functions */
        void init( Point start, Point goal );
        //Vertex insertIntoTree( DelaunayTriangulation &Dt, Vertex v, std::optional<GraphVertex> parent );


private:

};

}

#endif
