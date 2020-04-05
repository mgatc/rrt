#ifndef RRT_H
#define RRT_H

#include "../include/CgalComponents.h"
#include "../include/Astar.h"



namespace MAG {

class RRT {

public:
    /* Types */
        enum Result { Advanced, Trapped, Reached };

    /* Public data members */

    /* Constructor(s) */
        RRT( Point startPoint, Point goalPoint, double step=2, int maxNodes=2000, double goalSkewProbability=2 );

    /* Public member functions */
        virtual std::list<Vertex> go();
        bool pathExists();
        void setGoalSkewProbability( double p );
        void displayPDF( std::string outputFileName );



protected:
    /* Protected data members */
        Vertex startv;
        Vertex goalv;
        unsigned size;          // size of random number bounds (from origin)
        unsigned K;             // number of samples to take
        double epsilon;         // distance can travel in one discrete time increment
        DelaunayTriangulation nearestNeighborTree;
        Graph T;
        WeightMap weightMap;
        LocationMap locationMap;
        CGAL::Random_points_in_square_2<Point,Creator> g1; // random point iterator
        std::optional<Point> last;  // the latest point to be added to the tree

    /* Protected member functions */
        void insertIntoTree( DelaunayTriangulation &Dt, Point p, std::optional<GraphVertex> parent = std::nullopt );
        GraphEdge insertEdge( GraphVertex vertex, GraphVertex parent );
        Vertex insertVertex( DelaunayTriangulation &Dt, Point p );
        bool goalTest( DelaunayTriangulation &Dt, Point target );
        Point randomPoint();
        GraphVertex nearestNeighbor( DelaunayTriangulation &Dt, Point x );
        std::list<DtVertex> nearestNeighbors( DelaunayTriangulation &Dt, Point p, int k );
        std::optional<Point> newState( Point x, Point xNear, bool uNew );
        virtual void init( Point start, Point goal );

    /* RRT specified functions */
        virtual std::list<Vertex> buildRRT();
        std::list<Vertex> path();
        Result extend( DelaunayTriangulation &Dt, Point x );
        Point randomState();



private:
    /* Private data members */
        double goalSkewProbability;

    /* Private member functions */

};

}

#endif
