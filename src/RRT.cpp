#include "RRT.h"
#include "CgalComponents.h"
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/nearest_neighbor_delaunay_2.h>



using namespace MAG;



/* Constructor(s) */

MAG::RRT::RRT( Point startPoint, Point goalPoint, double step, int maxNodes, double goalSkewProbability ) :
                                        start( startPoint ),
                                        goal( goalPoint ),
                                        K( maxNodes ),
                                        size( 50 ),
                                        epsilon( step ),
                                        goalSkewProbability( goalSkewProbability ),
                                        g1( size ),
                                        T( K )
{

}



/* Public member functions */

bool MAG::RRT::go() {
    return buildRRT();
}

void MAG::RRT::setGoalSkewProbability( double p ) {
    if( p < 0 )      goalSkewProbability = 0;
    else if( p > 1 ) goalSkewProbability = 1;
    else             goalSkewProbability = p;
}

void MAG::RRT::displayPDF( std::string fileName ) {
    double const radiusOfPoints = 1;
    std::string fName = fileName + ".tex";
    FILE *fp = fopen(fName.c_str() ,"w");

    fprintf(fp,"\\documentclass{standalone} \n\\usepackage{tikz} \n \n\n\\begin{document}\n");
    fprintf(fp,"\n\n\n\\begin{tikzpicture}\n\n");

    typename boost::graph_traits<Graph>::edge_iterator ei, ei_end;
    for( boost::tie(ei, ei_end) = boost::edges(T); ei != ei_end; ++ei ) {
        Point p = T[source(*ei, T)].p;
        Point q = T[target(*ei, T)].p;
        fprintf(
            fp,
            "\\draw [gray,thin] (%f,%f) -- (%f,%f);",
            p.x(),
            p.y(),
            q.x(),
            q.y()
        );
        //cout<< *it->parent << endl;
    }
//    if( this->pathExists() ) {
//        //print goal path
//        for( auto it=path.begin(), next=std::next(it,1); next!=path.end(); it++, next++ ) {
//            fprintf(
//                fp,
//                "\\draw [purple,ultra thick] (%f,%f) -- (%f,%f);",
//                it->p.x(),
//                it->p.y(),
//                next->p.x(),
//                next->p.y()
////                T.at(*it->parent).p.x(),
////                T.at(*it->parent).p.y()
//            );
//            //cout<< *it->parent << endl;
//        }
//    }

    // print start and goal
    fprintf( fp,"\\draw [fill=red,stroke=red] (%f,%f) circle [radius=%f];\n",start.x(),start.y(),radiusOfPoints );
    fprintf( fp,"\\draw [fill=green,stroke=green] (%f,%f) circle [radius=%f];\n",goal.x(),goal.y(),radiusOfPoints );

    fprintf(fp,"\n\n\\end{tikzpicture}");
    fprintf(fp,"\n\n\\end{document}");
    fclose(fp);

    std::cout << "\nOutput PDF generation started...\n";
    std::string command = "pdflatex " + fName + " > /dev/null";
    system(command.c_str());
    std::cout << "PDF generation terminated...\n";

    command = "atril " + fileName + ".pdf &";

    system( command.c_str() );

}



/* Private RRT specified functions */

bool MAG::RRT::path() {

}

bool MAG::RRT::buildRRT() {
    Point rand;
    // Initialize T with start location
    insertIntoTree( nearestNeighborTree, start, std::nullopt );

    for( unsigned k=0; k<K; k++ ) {
        rand = randomState();    // generate random point
        extend( nearestNeighborTree, rand );
    }
    return goalTest( nearestNeighborTree, goal );
}

RRT::Result MAG::RRT::extend( DelaunayTriangulation &Dt, Point x ) {
    vertex_descriptor near = nearestNeighbor( Dt, x );
    std::optional<Point> xNew = newState( x, T[near].p, false );
    last = xNew;
    if( xNew ) {
        //cout<< near->info() << endl;
        insertIntoTree( Dt, *xNew, near );
        //last = { *xNew };

        if( *xNew == x ) {
            return Reached;
        } else {
            return Advanced;
        }
    } else {
        //last = std::nullopt;
    }
    return Trapped;
}

Point MAG::RRT::randomState() {
    Point p = randomPoint();  // random point, used for determining whether to goal skew or now
    double chance = abs( p.x() ) + abs( p.y() ) / ( 2*size ); //

    if( chance < goalSkewProbability ) {
//        cout << "skew to goal" << endl;
        return goal;
    } else {
//        cout << "random" << endl;
        return randomPoint();
    }
}



/* Private member functions */

void MAG::RRT::insertIntoTree( DelaunayTriangulation &Dt, Point p, std::optional<vertex_descriptor> parent ) {
    Vertex_handle dtVertex;
    vertex_descriptor treeVertex;

    treeVertex = boost::add_vertex( {p}, T ); // add vertex to T
    if( parent ) // if parentIndex is set, add an edge from parentIndex to new vertex
        boost::add_edge( treeVertex, *parent, T );
    dtVertex = Dt.insert( p ); // add point to delaunay triangulation
    dtVertex->info() = treeVertex; // store vertex descriptor in dt
}

bool MAG::RRT::goalTest( DelaunayTriangulation &Dt, Point target ) {
    // get nearest point to goal
    vertex_descriptor nearest = nearestNeighbor( Dt, target );
    // check distance between nearest and goal
    return CGAL::squared_distance( T[nearest].p, target ) < 1;
}

Point MAG::RRT::randomPoint() {
    return *( g1++ );
}

vertex_descriptor MAG::RRT::nearestNeighbor( DelaunayTriangulation &Dt, Point x ) {
    return Dt.nearest_vertex( x )->info();
}

std::list<Vertex_handle> MAG::RRT::nearestNeighbors( DelaunayTriangulation &Dt, Point p, int k ) {
    std::list<Vertex_handle> L;
    nearest_neighbors( Dt, p, k, std::back_inserter(L));
    return L;
}

std::optional<Point> MAG::RRT::newState( Point x, Point xNear, bool uNew ) {

    //DO COLLISION DETECTION HERE

    // if x not reachable from xNear under global constraints
        // return {};

    // make a move from xNear towards x under discrete time law
    double a = x.x()-xNear.x(),   // horizontal difference
           b = x.y()-xNear.y(),   // vertical difference
           c = sqrt( a*a + b*b ); // total difference

    if( c < epsilon )
        return {x};

    double factor = epsilon / c;

    Point p = Point( xNear.x()+factor*(a), xNear.y()+factor*(b) );

    // returns the new state or failure notice
    return {p};
}



