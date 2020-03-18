#ifndef RRT_H
#define RRT_H


#include "CgalComponents.h"
//#include "Node.h"


namespace MAG {

class RRT {

public:
    /* Types */
        enum Result { Advanced, Trapped, Reached };

    /* Public data members */
        Point start;
        Point goal;
        unsigned size;          // size of random number bounds (from origin)
        unsigned K;             // number of samples to take
        double epsilon;         // distance can travel in one discrete time increment
        DelaunayTriangulation nearestNeighborTree;
        Graph T;
        CGAL::Random_points_in_square_2<Point,Creator> g1; // random point iterator
        std::optional<Point> last;  // the latest point to be added to the tree

    /* Constructor(s) */
        RRT( Point startPoint, Point goalPoint, double step=2, int maxNodes=2000, double goalSkewProbability=2 );

    /* Public member functions */
        virtual bool go();
        bool pathExists();
        void setGoalSkewProbability( double p );
        void displayPDF( std::string outputFileName );
        void insertIntoTree( DelaunayTriangulation &Dt, Point p, std::optional<vertex_descriptor> parent = std::nullopt );
        bool goalTest( DelaunayTriangulation &Dt, Point target );
        Point randomPoint();
        vertex_descriptor nearestNeighbor( DelaunayTriangulation &Dt, Point x );
        std::list<Vertex_handle> nearestNeighbors( DelaunayTriangulation &Dt, Point p, int k );
        std::optional<Point> newState( Point x, Point xNear, bool uNew );


    /* RRT specified functions */
        virtual bool buildRRT();
        std::list<Point> path();
        Result extend( DelaunayTriangulation &Dt, Point x );
        Point randomState();



private:
    /* Private data members */
        double goalSkewProbability;

    /* Private member functions */

};

}

#endif
