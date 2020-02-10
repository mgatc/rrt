#ifndef RRT_H
#define RRT_H

#include "CgalComponents.h"
#include "RRT_Tree.h"

#define REACHED 0
#define ADVANCED 1
#define TRAPPED 2

class RRT {

    unsigned size = 50;   // size of random number bounds (from origin)
    unsigned K = 2000;    // number of samples to take
    double epsilon = 2.0; // distance can travel in one discrete time increment
    Random_points_in_square_2<Point,Creator> g1; // random point iterator
    RRT_Tree *T;
    Point start;
    Point goal;
    // the parameters
    // the map

    public:
        RRT( RRT_Tree &tree, Point start, Point goal );
        bool buildRRT();
        int extend( Point x );

    private:
        Point randomState();
        Point nearestNeighbor( Point x );
        optional<Point> newState( Point x, Point xNear, bool uNew );
};

#endif
