#ifndef RRT_H
#define RRT_H

#include "CgalComponents.h"

class RRT {

    unsigned size = 10;
    Random_points_in_square_2<Point,Creator> *g1; // random point iterator, bounded by size
    // T the tree
    // And the parameters
    // and the map

    public:
        Point start;
        Point goal;

        RRT( Point start, Point goal );
        bool buildRRT( Point xInit );
        bool extend( Point x );

    private:
        Point randomState();
        Point nearestNeighbor( Point x );
        Point newState( Point x, Point xNear, Point uNew );
};

#endif
