#include "Node.h"

MAG::Node::Node() {}

MAG::Node::Node( Point p, optional<unsigned> parent )
        : p(p), parent(parent) {

}
// Copier
MAG::Node & MAG::Node::operator=( const Node &n ) {
    if( this != &n ) {
        p = n.p;
        parent = n.parent;
        u = n.u;
    }
    return *this;
}
