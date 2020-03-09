#ifndef RRT_H
#define RRT_H


#include "CgalComponents.h"
//#include "Node.h"

typedef boost::adjacency_list<boost::listS, boost::listS, boost::undirectedS, MAG::vertex_info, MAG::edge_info> Graph;

namespace MAG {

    class RRT {

        public:
            Point start;
            Point goal;
            unsigned size;   // size of random number bounds (from origin)
            unsigned K;    // number of samples to take
            double epsilon; // distance can travel in one discrete time increment
            double goalSkewProbability;

            RRT( Point start, Point goal );
            bool go();
            bool pathExists();
            list<Point> getPath();
            void displayPDF( string outputFileName );

        private:
            DelaunayTriangulation Dt;
            Graph T;

            list<vertex_descriptor> path;
            unsigned currentIndex = 0;
            Random_points_in_square_2<Point,Creator> g1; // random point iterator

            optional<vertex_descriptor> bidirectionalRRT();
            optional<vertex_descriptor> buildRRT();
            int extend( Point x );
            optional<vertex_descriptor> goalTest();

            void insertIntoTree( Point p, optional<vertex_descriptor> parent = std::nullopt );
            Point randomState();
            Point randomPoint();
            vertex_descriptor nearestNeighbor( Point x );
            list<Vertex_handle> nearestNeighbors( Point p, int k );
            optional<Point> newState( Point x, Point xNear, bool uNew );
    };
}

#endif
