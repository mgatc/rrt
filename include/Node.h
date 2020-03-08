#ifndef NODE_H
#define NODE_H

#include "CgalComponents.h"

namespace MAG {

    class Node {
        public:
            Point p;
            optional<unsigned> parent;
            bool u;

            Node();
            Node( Point p, optional<unsigned> parent = {} );
            bool setPoint( Point p );
            bool setParent( Node &parent );
            Node next();
            MAG::Node &operator=( const Node &n );
    };
}


#endif
