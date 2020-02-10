#ifndef RRT_TREE_H
#define RRT_TREE_H

#include "CgalComponents.h"

struct Edge {
    Point p, q;
    bool u; // placeholder for u

    Edge( Point p, Point q, bool u ) : p(p), q(q), u( false ) {}
};

struct RRT_Tree {
    DelaunayTriangulation V;
    vector<Edge> E;
};

#endif
