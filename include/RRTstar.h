#ifndef RRTSTAR_H
#define RRTSTAR_H

#include "RRT.h"
#include "Astar.h"

#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/nearest_neighbor_delaunay_2.h>

namespace MAG {

template< class W >
class RRTstar : public RRT<W> {
    using RRT<W>::newState;
    using RRT<W>::randomState;
    using RRT<W>::insertIntoTree;
    using RRT<W>::getVertex;
    using RRT<W>::obstacleFree;
    using RRT<W>::cost;
    using RRT<W>::insertEdge;

public:
    /* Constructor(s) */
        RRTstar( W& world, double start_x, double start_y, double goal_x, double goal_y,
                 double step, int maxNodes, unsigned seed );

protected:
    /* Protected RRT specified functions */
        void buildRRT();
        Vertex chooseParent( std::list<GraphVertex>& Znear, Vertex nearest, Vertex xNew );
        void nearestNeighbors( std::list<GraphVertex>& Znear, Vertex z, int n );
        void rewire( std::list<GraphVertex> Znear, Vertex zMin, Vertex xNew  );
        void reconnect( Vertex xNew, Vertex zNear );
};

/* Constructor(s) */

template< class W >
RRTstar<W>::RRTstar( W& world, double start_x, double start_y, double goal_x, double goal_y,
                     double step, int maxNodes, unsigned seed )
            : RRT<W>( world, start_x, start_y, goal_x, goal_y, step, maxNodes, seed )
{

}

template< class W >
void RRTstar<W>::buildRRT() {
    int n;
    Vertex rand;
    Vertex nearest;
    Vertex zMin;
    std::optional<Vertex> xNew;
    //std::cout<<"RRTstar buildRRT called"<<std::endl;

    for( unsigned k=0; k<this->K; k++ ) {
        rand = randomState();    // generate random point
        nearest = this->nearestNeighbor( this->nearestNeighborTree, rand );
        xNew = this->newState( rand, nearest ); // put call to obstacleFree in newState

        if( xNew ) {
            std::list<GraphVertex> Znear;
            n = boost::num_vertices( this->T );
            nearestNeighbors( Znear, *xNew, log(n)/log(2) );
            zMin = chooseParent( Znear, nearest, *xNew );
            Vertex v = this->insertIntoTree( this->nearestNeighborTree, *xNew, zMin );
            rewire( Znear, zMin, v );
        }
    }

    return;
}

template< class W >
void RRTstar<W>::rewire( std::list<GraphVertex> Znear, Vertex zMin, Vertex xNew ) {
    Vertex zNear;
    for( auto near : Znear ) {
        if( near != zMin.graph ) { // not zMin itself
            zNear = getVertex( near );
            //std::cout<< "rewire: " << xNew.point << " "<< zNear.point << std::endl;
            if( distanceBetween<Cost>( xNew.point, zNear.point ) < this->epsilon   // within moving distance
                && obstacleFree( xNew, zNear )                                  // path is obstacle free
                && cost( xNew, zNear) < zNear.cost                            // cost is cheaper
            ) {
                std::cout << "reconnect " << zNear.graph << " to " << xNew.graph << std::endl;
                reconnect( xNew, zNear );
            }
        }
    }
}

template< class W >
void RRTstar<W>::reconnect( Vertex xNew, Vertex zNear ) {
    //std::cout<< zNear.point << " degree: " << boost::in_degree(zNear.graph, T) << std::endl;
    // remove edge between zNear and its parent

    boost::clear_in_edges( zNear.graph, this->T );
    // insert edge from xNew to zNear
    //std::cout<< xNew.graph<<std::endl;
    GraphEdge e = insertEdge( zNear, xNew );
    //std::cout << "insertEdge returned edge( " << getVertex( boost::target( e, this->T ) ).graph
    //        << "," << getVertex( boost::source( e, this->T ) ).graph << " )" << std::endl;
}

template< class W >
Vertex RRTstar<W>::chooseParent( std::list<GraphVertex>& Znear, Vertex nearest, Vertex xNew ){
    Cost c, cMin;
    Vertex zMin = nearest;
    Vertex zNear;

    cMin = cost( nearest, xNew );

    for( auto near : Znear ) {
        zNear = getVertex( near );
        if( distanceBetween<Cost>( zNear.point, xNew.point ) < this->epsilon && obstacleFree( zNear, xNew ) ) {
            c = cost( zNear, xNew );
            if( c < cMin ) {
                zMin = zNear;
                cMin = c;
            }
        }
    }

    return zMin;
}

template< class W >
void RRTstar<W>::nearestNeighbors( std::list<GraphVertex>& Znear, Vertex z, int n ) {
    // nearest_neighbors will return dt handles, so make a temp list
    std::vector<DtVertex> handles;
    CGAL::nearest_neighbors( this->nearestNeighborTree, z.point, n, std::back_inserter(handles));
    Znear.clear();

    for( DtVertex handle : handles ) {
        Znear.push_back( handle->info() );
    }

}





}

#endif
