#ifndef RRT_H
#define RRT_H

#include "Astar.h"
#include "Point2.h"

namespace MAG {

enum Result { Advanced, Trapped, Reached };

template< class W >
class RRT {

public:
    /* Public types */

    /* Public data members */
        static constexpr double GOAL_EPSILON = 2;

    /* Constructor(s) */
        RRT( W& world, double start_x, double start_y, double goal_x, double goal_y,
          double step, int maxNodes, unsigned seed )
                                      : m_world( world ),
                                        size( world.getSize() ),
                                        K( maxNodes ),
                                        epsilon( step ),
                                        T(K),
                                        rnd( seed ),
                                        g1( size, rnd )
        {
            startv.point = Point( start_x, start_y );
            std::cout<<startv.point;

            goalv.point = Point( goal_x, goal_y );

            edgeMap = get( boost::edge_weight, T );
            vertexMap = get( boost::vertex_location, T );
        }

    /* Public member functions */
        bool go();
        bool pathExists();
        void setGoalSkewProbability( double p );
        void displayPDF( std::string outputFileName );
        std::list<Vertex> search( const Vertex& s, const Vertex& g );
        std::list<Vertex> search( const Vertex& g );
        std::list<Vertex> search();
        std::list<Vertex> getPath();
        Cost goalCost();
        void printPath();
        double cost();
        Point2 getGoalPoint();
        Point2 getStartPoint();


        std::list<Vertex> thePath;



protected:
    /* Protected types */


    /* Protected data members */
        W& m_world;
        Vertex startv;
        Vertex goalv;
        Vertex found_goalv;
        unsigned size;          // size of random number bounds (from origin)
        unsigned K;             // number of samples to take
        unsigned k=0;             // number of samples taken
        double epsilon;         // distance can travel in one discrete time increment
        DelaunayTriangulation nearestNeighborTree;
        Graph T;
        EdgeMap edgeMap;
        VertexMap vertexMap;
        std::vector<Cost> distanceMap;
        std::vector<GraphVertex> predecessorMap;
        CGAL::Random rnd;
        CGAL::Random_points_in_square_2<Point,Creator> g1; // random point iterator
        double goalSkewProbability = 0.2;

    /* Protected member functions */
        virtual Vertex insertIntoTree( DelaunayTriangulation &Dt, Vertex v, std::optional<Vertex> parent = std::nullopt );
        GraphEdge insertEdge( Vertex vertex, Vertex parent );
        Vertex insertVertex( DelaunayTriangulation &Dt, Vertex v );
        bool goalTest( DelaunayTriangulation &Dt, Point target );
        Point randomPoint();
        Vertex nearestNeighbor( DelaunayTriangulation &Dt, Vertex x );
        std::optional<Vertex> newState( Vertex x, Vertex xNear, bool uNew = false );
        bool obstacleFree( Vertex parent, Vertex child );
        virtual void init();
        Vertex getVertex( GraphVertex gv );
        Vertex getVertex( Point p );
        Vertex getVertex( DtVertex handle );
        VertexPropertyMapType getVertexPropertyMapType();
        Cost cost( Vertex p, Vertex q );
        Cost cost( Vertex p );

    /* RRT specified functions */
        virtual void buildRRT();
        virtual Result extend( DelaunayTriangulation &Dt, Vertex x );
        Vertex randomState();



private:
    /* Private data members */

    /* Private member functions */

};


/* Constructor(s) */





/* Public member functions */

template< class W >
Point2 RRT<W>::getStartPoint() {
    return { startv.point.x(), startv.point.y() };
}

template< class W >
Point2 RRT<W>::getGoalPoint() {
    return { goalv.point.x(), goalv.point.y() };
}

template< class W >
bool RRT<W>::go() {
    init();
    buildRRT();
    thePath = search( goalv );
    return pathExists();
}

template< class W >
void RRT<W>::setGoalSkewProbability( double p ) {
    if( p < 0 )      goalSkewProbability = 0;
    else if( p > 1 ) goalSkewProbability = 1;
    else             goalSkewProbability = p;
}

template< class W >
void RRT<W>::displayPDF( std::string fileName ) {
    double const radiusOfPoints = 1;
    std::string fName = fileName + ".tex";
    FILE *fp = fopen(fName.c_str() ,"w");

    // update vertexMap, get existing best path
    vertexMap = get( boost::vertex_location, T );
    std::list<Vertex> bestPath = getPath();

    // start document
    fprintf(fp,"\\documentclass{standalone} \n\\usepackage{tikz} \n \n\n\\begin{document}\n");

    fprintf(fp,"\n\n\n\\begin{tikzpicture}\n\n");
    // draw tree
    typename boost::graph_traits<Graph>::edge_iterator ei, ei_end;
    for( boost::tie(ei, ei_end) = boost::edges(T); ei != ei_end; ++ei ) {
        Point p = getVertex( source(*ei, T) ).point;
        Point q = getVertex( target(*ei, T) ).point;
        fprintf(
            fp,
            "\\draw [gray,thin] (%f,%f) -- (%f,%f);",
            p.x(),
            p.y(),
            q.x(),
            q.y()
        );
        //std::cout << source(*ei, T) << "(" << vertexMap[source(*ei,T)].point << ") <->" << target(*ei, T) << "(" << vertexMap[target(*ei,T)].point << ")" << std::endl;
    }

    // draw shortest path
    for( auto it=bestPath.begin(), next=std::next(it,1); next!=bestPath.end(); it++, next++ ) {
        fprintf(
            fp,
            "\\draw [purple,ultra thick] (%f,%f) -- (%f,%f);",
            it->point.x(),
            it->point.y(),
            next->point.x(),
            next->point.y()
        );
    }

    // print start and goal
    fprintf( fp,"\\draw [fill=green,stroke=green] (%f,%f) circle [radius=%f];\n",startv.point.x(),startv.point.y(),radiusOfPoints );
    fprintf( fp,"\\draw [fill=red,stroke=red] (%f,%f) circle [radius=%f];\n",goalv.point.x(),goalv.point.y(),GOAL_EPSILON );

    fprintf(fp,"\n\n\\end{tikzpicture}");
    // close document
    fprintf(fp,"\n\n\\end{document}");
    fclose(fp);

    // generate pdf
    std::cout << "\nOutput PDF generation started...\n";
    std::string command = "pdflatex " + fName + " > /dev/null";
    system(command.c_str());
    std::cout << "PDF generation terminated...\n";

    // open pdf
    command = "atril " + fileName + ".pdf &";
    system( command.c_str() );

}

// Find the shortest path between s and g
template< class W >
std::list<Vertex> RRT<W>::search( const Vertex& s, const Vertex& g ) {
    std::list<Vertex> shortest_path;

    distanceMap.clear();
    distanceMap.resize( boost::num_vertices( T ) );

    predecessorMap.clear();
    predecessorMap.resize( boost::num_vertices( T ) );

    VertexPropertyMapType vertex_id = boost::get( boost::vertex_index, T );

    try {
        boost::astar_search_tree(
            T,                      // the graph
            s.graph,            // the start vertex
            distance_heuristic< Graph, Cost, VertexMap >( vertexMap, g.graph ),
            boost::predecessor_map(
                boost::make_iterator_property_map( predecessorMap.begin(), vertex_id )
            ).distance_map(
                boost::make_iterator_property_map( distanceMap.begin(), vertex_id )
            ).visitor(
                astar_goal_visitor< GraphVertex, Cost, VertexMap >( vertexMap, goalv.graph )
            )
        );
    } catch( Vertex found ) { // found a path to the goal

        std::cout << "Found the goal!" << std::endl;
        found_goalv = found;

        for( GraphVertex v = found.graph;; v = predecessorMap[v] ) {
            shortest_path.push_front( vertexMap[v] );
            if( predecessorMap[v] == v )
                break;
        }
        //std::cout << "Shortest path: ";

        //std::list<Vertex>::iterator spi = shortest_path.begin();

        //std::cout << s.point;

//        for(++spi; spi != shortest_path.end(); ++spi)
//            std::cout << " -> " << spi->point;

        std::cout << std::endl << "Total distance: " << distanceMap[found.graph] << std::endl;

        return shortest_path;
    }
    //std::cout << "Didn't find the goal..." << goalv.point << std::endl;

    return std::list<Vertex>(0);
}

// Find the shortest path between startv and g
template< class W >
std::list<Vertex> RRT<W>::search( const Vertex& g ) {
    return search( startv, g );
}

// Find the shortest path between startv and goalv
template< class W >
std::list<Vertex> RRT<W>::search() {
    return search( goalv );
}

template< class W >
std::list<Vertex> RRT<W>::getPath() {
    return thePath;
}

template< class W >
bool RRT<W>::pathExists() {
    return !thePath.empty();
}



/* Protected RRT specified functions */

template< class W >
void RRT<W>::buildRRT() {
    Vertex rand;

    for( unsigned k=0; k<K; k++ ) {
        rand = randomState();    // generate random point
        extend( nearestNeighborTree, rand );
        //std::cout<<rand.point<<std::endl;
    }
    return;
}

template< class W >
MAG::Result RRT<W>::extend( DelaunayTriangulation &Dt, Vertex x ) {
    Vertex near = nearestNeighbor( Dt, x );

    std::optional<Vertex> xNew = newState( x, near, false );

    if( xNew ) {
        insertIntoTree( Dt, *xNew, near );
        return ( xNew->point == x.point ) ? Reached : Advanced;
    }
    return Trapped;
}

template< class W >
Vertex RRT<W>::randomState() {
    Vertex v;
    Point p = randomPoint();  // random point, used for determining whether to goal skew or now
    double chance = CGAL::to_double( CGAL::abs( p.x() ) + CGAL::abs( p.y() ) ) / ( 2*size );
    //std::cout<< "chance:" << chance<< " gsp:"<<goalSkewProbability<<std::endl;
    if( chance < goalSkewProbability ) {
        //std::cout << "skew to goal" << std::endl;
        return goalv;
    } else {
        //std::cout << "random" << std::endl;
        return Vertex{ randomPoint() };
    }
}



/* Protected member functions */

template< class W >
Vertex RRT<W>::insertIntoTree( DelaunayTriangulation &Dt, Vertex v, std::optional<Vertex> parent ) {

    if( parent )
        v.cost = cost( *parent, v );
    // if vertex already has gv, don't rebuild v
    if( v.graph == SIZE_T_MAX )
        v = insertVertex( Dt, v );

    if( parent ) // if parent is set, add an edge from parentIndex to new vertex
        insertEdge( v, *parent );

    return v;
}

template< class W >
GraphEdge RRT<W>::insertEdge( Vertex vertex, Vertex parent ) {
    //std::cout<<"RRT::insertEdge: "<<parent.point.x()<<" "<<parent.point.y()<<", "<<vertex.point.x()<<" "<<vertex.point.y()<<std::endl;

    GraphEdge e;
    bool inserted;

    boost::tie( e, inserted ) = boost::add_edge( parent.graph, vertex.graph, T );
    edgeMap[e] = distanceBetween<Cost>( parent.point, vertex.point );
    //vertex.cost = cost( parent, vertex );

    //distanceMap[vertex.graph] = cost( parent ) + edgeMap[e];
    //boost::put( distanceMap, vertex.graph, parent.cost + get( edgeMap, e ) );
    m_world.addPathEdge( {vertex.point.x(), vertex.point.y()}, {parent.point.x(), parent.point.y()} );

    return e;
}

template< class W >
Vertex RRT<W>::insertVertex( DelaunayTriangulation &Dt, Vertex v ) {
    GraphVertex gv = boost::add_vertex( T );// add vertex to T

    v.handle = Dt.insert( v.point );        // add point to Dt, add handle to vertex obj
    v.graph = gv;                           // add graph index to vertex obj
    v.handle->info() = gv;                  // add graph index to Dt handle for nn queries

    vertexMap[gv] = v;                    // set the vertex in the vertexMap

    return getVertex( gv );                 // use this to ensure the output is formatted the same as getVertex
}

template< class W >
void RRT<W>::init() {

    startv = insertVertex( nearestNeighborTree, startv );

    // Add goal vertex to T and vertexMap, but not to nearestNeighborTree
    goalv.graph = boost::add_vertex( T );    // add goal vertex to T

    vertexMap[goalv.graph] = goalv;

}

template< class W >
Point RRT<W>::randomPoint() {
    return *( g1++ );
}

template< class W >
Vertex RRT<W>::nearestNeighbor( DelaunayTriangulation &Dt, Vertex x ) {
    return getVertex(
        Dt.nearest_vertex( x.point )->info()
    );
}

template< class W >
std::optional<Vertex> RRT<W>::newState( Vertex x, Vertex xNear, bool uNew ) {

    // make a move from xNear towards x under discrete time law
    double a = x.point.x()-xNear.point.x(),   // horizontal difference
           b = x.point.y()-xNear.point.y(),   // vertical difference
           c = sqrt( a*a + b*b ); // total difference

    if( c < epsilon && obstacleFree( xNear, x ) )
        return {x};

    double factor = epsilon / c;

    Vertex v;
    v.point = Point( xNear.point.x()+factor*(a), xNear.point.y()+factor*(b) );

    // returns the new state or failure notice
    if( obstacleFree( xNear, v ) )
        return {v};

    return {};
}

template< class W >
bool RRT<W>::obstacleFree( Vertex parent, Vertex child ) {
    //std::cout<<"RRT::obstacleFree: "<<parent.point.x()<<" "<<parent.point.y()<<", "<<child.point.x()<<" "<<child.point.y()<<std::endl;

    return m_world.obstacleFree(
        { parent.point.x(), parent.point.y() },
        { child.point.x(), child.point.y() }
    );
}

template< class W >
Vertex RRT<W>::getVertex( GraphVertex gv ) {
    return vertexMap[gv];
}

template< class W >
Vertex RRT<W>::getVertex( DtVertex handle ) {
    return getVertex( handle->info() );
}

template< class W >
Cost RRT<W>::goalCost() {
    return pathExists() ? thePath.back().cost : std::numeric_limits<double>::infinity();
}

// Returns the actual cost from start to parent plus the estimated cost from parent to child
template< class W >
Cost RRT<W>::cost( Vertex parent, Vertex child ) {
    Cost c = parent.cost + distanceBetween<Cost>( parent.point, child.point );
    //std::cout<< "cost(p,c):"<<c<<std::endl;
    return c;
}


}

#endif
