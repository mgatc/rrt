#include "RRT.h"

using namespace MAG;

int ERRORTRACKER = 0;

/* Constructor(s) */

RRT::RRT( Point startPoint, Point goalPoint, double step, int maxNodes, double goalSkewProbability )
                                      : K( maxNodes ),
                                        size( 50 ),
                                        epsilon( step ),
                                        goalSkewProbability( goalSkewProbability ),
                                        g1( size ),
                                        T( K )
{
    startv.point = startPoint;
    goalv.point = goalPoint;
    weightMap = get( boost::edge_weight, T );
    locationMap = get( boost::vertex_location, T );
}



/* Public member functions */

std::list<Vertex> RRT::go() {
    init( startv.point, goalv.point );
    return buildRRT();
}

void RRT::setGoalSkewProbability( double p ) {
    if( p < 0 )      goalSkewProbability = 0;
    else if( p > 1 ) goalSkewProbability = 1;
    else             goalSkewProbability = p;
}

void RRT::displayPDF( std::string fileName ) {
    double const radiusOfPoints = 1;
    std::string fName = fileName + ".tex";
    FILE *fp = fopen(fName.c_str() ,"w");

    fprintf(fp,"\\documentclass{standalone} \n\\usepackage{tikz} \n \n\n\\begin{document}\n");
    fprintf(fp,"\n\n\n\\begin{tikzpicture}\n\n");

    locationMap = get( boost::vertex_location, T );

    typename boost::graph_traits<Graph>::edge_iterator ei, ei_end;
    for( boost::tie(ei, ei_end) = boost::edges(T); ei != ei_end; ++ei ) {
        Point p = locationMap[source(*ei, T)].point;
        Point q = locationMap[target(*ei, T)].point;
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
    fprintf( fp,"\\draw [fill=red,stroke=red] (%f,%f) circle [radius=%f];\n",startv.point.x(),startv.point.y(),radiusOfPoints );
    fprintf( fp,"\\draw [fill=green,stroke=green] (%f,%f) circle [radius=%f];\n",goalv.point.x(),goalv.point.y(),radiusOfPoints );

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

std::list<Vertex> RRT::path() {
    std::vector<GraphVertex> p( boost::num_vertices( T ) ); // predecessor map
    std::vector<Cost> d( boost::num_vertices( T ) );        // distance map

    boost::property_map< Graph, boost::vertex_index_t >::type vertex_id = boost::get( boost::vertex_index, T );
    try {
        boost::astar_search_tree(
            T,                      // the graph
            startv.graph,            // the start vertex
            distance_heuristic< Graph, Cost, LocationMap >( locationMap, goalv.graph ),
            boost::predecessor_map(
                boost::make_iterator_property_map( p.begin(), vertex_id )
            ).distance_map(
                boost::make_iterator_property_map( d.begin(), vertex_id )
            ).visitor(
                astar_goal_visitor< GraphVertex, Cost, LocationMap >( locationMap, goalv.graph )
            )
        );
    } catch( Vertex found ) { // found a path to the goal
        // Check if vertex returned in found matches our goal exactly.

        // If not, change goal
        std::list<Vertex> shortest_path;

        std::cout << "A* found the goal!" << std::endl;

        for( GraphVertex v = found.graph;; v = p[v] ) {
            shortest_path.push_front( locationMap(v) );
            if( p[v] == v )
                break;
        }
        std::cout << "Shortest path: ";

        std::list<Vertex>::iterator spi = shortest_path.begin();

        std::cout << locationMap[startv.graph].point;

        for(++spi; spi != shortest_path.end(); ++spi)
            std::cout << " -> " << locationMap[spi->graph].point;

        std::cout << std::endl << "Total distance: " << d[goalv.graph] << std::endl;

        return shortest_path;
    }
    std::cout << "A* didn't find the goal..." << std::endl;

    return std::list<Vertex>();
}

std::list<Vertex> RRT::buildRRT() {
    Point rand;

    for( unsigned k=0; k<K; k++ ) {
        rand = randomState();    // generate random point
        extend( nearestNeighborTree, rand );
    }
    return path();
}

RRT::Result RRT::extend( DelaunayTriangulation &Dt, Point x ) {
    GraphVertex near = nearestNeighbor( Dt, x );
    //std::cout<< x << " " << locationMap[near].point << std::endl;

    std::optional<Point> xNew = newState( x, locationMap[near].point, false );
    last = xNew;
    if( xNew ) {
        insertIntoTree( Dt, *xNew, near );
        return ( *xNew == x ) ? Reached : Advanced;
    }
    return Trapped;
}

Point RRT::randomState() {
    Point p = randomPoint();  // random point, used for determining whether to goal skew or now
    double chance = CGAL::to_double( CGAL::abs( p.x() ) + CGAL::abs( p.y() ) ) / ( 2*size );

    if( chance < goalSkewProbability/100 ) {
//        cout << "skew to goal" << endl;
        return goalv.point;
    } else {
//        cout << "random" << endl;
        return randomPoint();
    }
}



/* Private member functions */

void RRT::insertIntoTree( DelaunayTriangulation &Dt, Point p, std::optional<GraphVertex> parent ) {
    Vertex vertex;

    vertex = insertVertex( Dt, p );

    if( parent ) { // if parentIndex is set, add an edge from parentIndex to new vertex
        insertEdge( vertex.graph, *parent );
    }
}

GraphEdge RRT::insertEdge( GraphVertex vertex, GraphVertex parent ) {
    GraphEdge e;
    bool inserted;
    Point first, second;

    first = locationMap[vertex].point;
    second = locationMap[parent].point;

    boost::tie( e, inserted ) = boost::add_edge( vertex, parent, T );
    weightMap[e] = distanceBetween<Cost>( first, second );

    return e;
}

Vertex RRT::insertVertex( DelaunayTriangulation &Dt, Point p ) {
    GraphVertex gv = boost::add_vertex( T );        // add vertex to T

    locationMap[gv].handle = Dt.insert( p );        // add point to Dt
    locationMap[gv].point = p; // get address of point from Dt
    locationMap[gv].graph = gv;                     // add graph vertex
    locationMap[gv].handle->info() = gv;         // store graph vertex in dt

    //std::cout << "Vertex added: " << locationMap[gv].point << ", vertex given: " << p << std::endl;

    return locationMap[gv];
}

void RRT::init( Point start, Point goal ) {

    startv = insertVertex( nearestNeighborTree, start );
    // Add goal vertex to T and locationMap, but not to nearestNeighborTree
    GraphVertex gv = boost::add_vertex( T );    // add goal vertex to T
    locationMap[gv].point = goal;               // add goal location to map
    locationMap[gv].graph = gv;                 // add graph vertex

    goalv = locationMap[gv];
}

//bool RRT::goalTest( DelaunayTriangulation &Dt, Point target ) {
//    // get nearest point to goal
//    GraphVertex nearest = nearestNeighbor( Dt, target );
//    DtVertex nearest = locationMap[nearestNeighb]
//    // check distance between nearest and goal
//    return CGAL::squared_distance( T[nearest].p, target ) < 1;
//}

Point RRT::randomPoint() {
    return *( g1++ );
}

GraphVertex RRT::nearestNeighbor( DelaunayTriangulation &Dt, Point x ) {
    return Dt.nearest_vertex( x )->info();
}

std::list<DtVertex> RRT::nearestNeighbors( DelaunayTriangulation &Dt, Point p, int k ) {
    std::list<DtVertex> L;
    nearest_neighbors( Dt, p, k, std::back_inserter(L));
    return L;
}

std::optional<Point> RRT::newState( Point x, Point xNear, bool uNew ) {

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



