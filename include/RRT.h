#ifndef RRT_H
#define RRT_H


#include "CgalComponents.h"
#include "Node.h"

typedef boost::adjacency_list<boost::listS, boost::listS, boost::undirectedS, MAG::vertex_info, MAG::edge_info> Graph;

namespace MAG {

    class RRT {

        public:
            MAG::Node *root;
            MAG::Node start;
            MAG::Node goal;
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

            vector<MAG::Node> T;
            DelaunayTriangulation Dt;

            DelaunayTriangulation Dtb;
            Graph Tb;

            list<MAG::Node> path;
            unsigned currentIndex = 0;
            Random_points_in_square_2<Point,Creator> g1; // random point iterator

            optional<MAG::Node> bidirectionalRRT();
            optional<MAG::Node> buildRRT();
            int extend( Point x );
            optional<MAG::Node> goalTest();

            Vertex_handle insertIntoTree( Point p, optional<unsigned> parentIndex = std::nullopt );
            Point randomState();
            unsigned nearestNeighbor( Point x );
            list<Vertex_handle> nearestNeighbors( Point p, int k );
            optional<Point> newState( Point x, Point xNear, bool uNew );
    };
}

#endif
