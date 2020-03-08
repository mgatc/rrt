#include "RRT.h"
#include "CgalComponents.h"
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/nearest_neighbor_delaunay_2.h>

#define REACHED 0
#define ADVANCED 1
#define TRAPPED 2

using namespace MAG;

MAG::RRT::RRT( Point startPoint, Point goalPoint ) :
                                        start( startPoint, std::nullopt ),
                                        goal( goalPoint, std::nullopt ),
                                        K( 2500 ),
                                        size( 25 ),
                                        epsilon( 1 ),
                                        goalSkewProbability( 0 ),
                                        g1( size ),
                                        T(K) {

    optional<MAG::Node> lastNodeInGoalPath = this->buildRRT();

    if( lastNodeInGoalPath ) {
        MAG::Node walk = *lastNodeInGoalPath;
        while( *walk.parent ) {
//            cout << *walk.parent << endl;
            path.push_front( walk );
            walk = T.at( *walk.parent );
        }
        path.push_front( start );
//        for( auto p:path ) {
//            cout << p << endl;
//        }
    }
}

optional<MAG::Node> MAG::RRT::buildRRT() {
    Point rand;
    // Initialize T with start location
    this->insertIntoTree( this->start.p, std::nullopt );

    for( unsigned k=0; k<(this->K); k++ ) {
        rand = this->randomState();    // generate random point
        this->extend( rand );
    }
    return this->goalTest();
}

optional<MAG::Node> MAG::RRT::goalTest() {
    // get nearest point to goal
    MAG::Node nearest = T.at( this->nearestNeighbor( goal.p ) );
    // check distance between nearest and goal
    if( CGAL::squared_distance( nearest.p, goal.p ) < 1 ) {
        cout << "goal found" << endl;
        return nearest;
    } else {
        cout << "no goal found" << endl;
        return std::nullopt;
    }
}

bool MAG::RRT::pathExists() {
    return this->path.size() > 0;
}

Vertex_handle MAG::RRT::insertIntoTree( Point p, optional<unsigned> parentIndex ) {
    Vertex_handle dtVertex;
    vertex_descriptor treeVertex;

    treeVertex = boost::add_vertex( {p}, Tb ); // add vertex to T
    if( parentIndex ) // if parentIndex is set, add an edge from parentIndex to new vertex
        boost::add_edge( treeVertex, parentIndex );
    dtVertex = Dtb.insert( p ); // add point to delaunay triangulation
    dtVertex->info() = treeVertex; // store vertex descriptor in dt




    // PRE-boost implementation
        //cout << node.p << " " << node.parent->p << endl;
    Vertex_handle vh;
    MAG::Node node;
    vh = this->Dt.insert( p );
    vh->info() = currentIndex;

    if( parentIndex )
        this->T.insert( T.begin()+currentIndex, MAG::Node( p, *parentIndex ) );
    else
        this->T.insert( T.begin()+currentIndex, MAG::Node( p ) );

    currentIndex++;

    return vh;
}

int MAG::RRT::extend( Point x ) {
    // TODO: determine the correct way to pass near to Node constructor (to set parent of xNew)
    unsigned near = this->nearestNeighbor( x );
    optional<Point> xNew = this->newState( x, this->T.at(near).p, false );

    if( xNew ) {
        //cout<< near->info() << endl;
        this->insertIntoTree( *xNew, near );

        if( *xNew == x ) {
            return REACHED;
        } else {
            return ADVANCED;
        }
    }
    return TRAPPED;
}

Point MAG::RRT::randomState() {
    Point p = *( this->g1++ );  // random point, used for determining whether to goal skew or now
    double chance = abs( p.x() ) + abs( p.y() ) / ( 2*this->size ) * 100; //

    if( chance < this->goalSkewProbability ) {
//        cout << "skew to goal" << endl;
        return this->goal.p;
    } else {
//        cout << "random" << endl;
        return *( this->g1++ );
    }
}

unsigned MAG::RRT::nearestNeighbor( Point x ) {
    return this->Dt.nearest_vertex( x )->info();
}

list<Vertex_handle> MAG::RRT::nearestNeighbors( Point p, int k ) {
    std::list<Vertex_handle> L;
    nearest_neighbors( this->Dt, p, k, std::back_inserter(L));
    return L;
}

optional<Point> MAG::RRT::newState( Point x, Point xNear, bool uNew ) {

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

void MAG::RRT::displayPDF( string fileName ) {
    double const radiusOfPoints = 1;
    string fName = fileName + ".tex";
    FILE *fp = fopen(fName.c_str() ,"w");

    fprintf(fp,"\\documentclass{standalone} \n\\usepackage{tikz} \n \n\n\\begin{document}\n");
    fprintf(fp,"\n\n\n\\begin{tikzpicture}\n\n");

    // print all explored paths
    for( auto it=T.begin(); it<T.end(); it++ ) {
        if( it->parent ) {
            fprintf(
                fp,
                "\\draw [gray,thin] (%f,%f) -- (%f,%f);",
                it->p.x(),
                it->p.y(),
//                it->p.x()+1,
//                it->p.y()+1
                T.at(*it->parent).p.x(),
                T.at(*it->parent).p.y()
            );
            //cout<< *it->parent << endl;
        }
    }
    if( this->pathExists() ) {
        //print goal path
        for( auto it=path.begin(), next=std::next(it,1); next!=path.end(); it++, next++ ) {
            fprintf(
                fp,
                "\\draw [purple,ultra thick] (%f,%f) -- (%f,%f);",
                it->p.x(),
                it->p.y(),
                next->p.x(),
                next->p.y()
//                T.at(*it->parent).p.x(),
//                T.at(*it->parent).p.y()
            );
            //cout<< *it->parent << endl;
        }
    }

    // print start and goal
    fprintf( fp,"\\draw [fill=red,stroke=red] (%f,%f) circle [radius=%f];\n",start.p.x(),start.p.y(),radiusOfPoints );
    fprintf( fp,"\\draw [fill=green,stroke=green] (%f,%f) circle [radius=%f];\n",goal.p.x(),goal.p.y(),radiusOfPoints );

    fprintf(fp,"\n\n\\end{tikzpicture}");
    fprintf(fp,"\n\n\\end{document}");
    fclose(fp);

    cout << "\nOutput PDF generation started...\n";
    string command = "pdflatex " + fName + " > /dev/null";
    system(command.c_str());
    cout << "PDF generation terminated...\n";

    command = "atril " + fileName + ".pdf &";

    // Comment out for testing
    system(command.c_str());
}




















//#include "RRT.h"
//#include "CgalComponents.h"
//#include <CGAL/Delaunay_triangulation_2.h>
//#include <CGAL/nearest_neighbor_delaunay_2.h>
//#include <unordered_map>
//
//
//#define REACHED 0
//#define ADVANCED 1
//#define TRAPPED 2
//
//using namespace MAG;
//
//class Node {
//    public:
//        Point p;
//        Node *parent;
//        bool u; // placeholder for u
//
//        Node() {}
//
//        Node( Point p )
//                : p(p) {
//
//        }
//
//        Node( Point p, Node *parent )
//                : p(p), parent(parent) {
//
//        }
//        // Copier
//        Node & operator=( const Node &n ) {
//            if( this != &n ) {
//                p = n.p;
//                parent = n.parent;
//                u = n.u;
//            }
//            return *this;
//        }
//
//    private:
//};
//
//
//
//
//typedef CGAL::Triangulation_vertex_base_with_info_2<MAG::Node, K> Vb;
//typedef CGAL::Triangulation_data_structure_2<Vb> Tds;
//typedef CGAL::Delaunay_triangulation_2<K, Tds> DelaunayTriangulation;
//typedef DelaunayTriangulation::Vertex_handle Vertex_handle;
//typedef CGAL::Point_set_2<K, Tds> PS;
//
//
//
//
//class RRT {
//    public:
//        MAG::Node *root;
//        Point start;
//        Point goal;
//        unsigned size = 50;   // size of random number bounds (from origin)
//        unsigned K = 2000;    // number of samples to take
//        double epsilon = 2.0; // distance can travel in one discrete time increment
//        double goalSkewProbability = 36;
//
//
//        RRT( Point start, Point goal ) :
//            start( start ), goal( goal ), K( 2000 ), size( 50 ), epsilon( 1.61 ), goalSkewProbability( 36 ), g1( this->size ) {
//        //    std::vector<std::pair<Point, Node>> points;
//        //
//        //    Point a(0,0); // Put a and b in T
//        //    Point b(1,0);
//        //    Point c(4,0); // Find the nearest neighbor to point c. (should be b) Add point c
//        //    Point d(0,1); // Find two nearest neighbors to point d. (should be a and b)
//        //
//        //    points.push_back( make_pair( a, Node() ) );
//        //    points.push_back( make_pair( b, Node(a) ) );
//        //
//        //    this->T.insert( points.begin(), points.end() );
//        //
//        //
//        //
//        //
//        //    // single nearest neighbor
//        //    Vertex_handle C = this->T.nearest_vertex( c );
//        //
//        //
//        //    // single nearest neighbor referencing, including stored pointers to other points
//        //    cout<< "c's nearest neighbor: " << C->point() << " par: " << *(C->info().parent) << endl;
//        //
//        //
//        //
//        //
//        //    points.clear();
//        //    points.push_back( make_pair( c, Node(b) ) );
//        //    this->T.insert( points.begin(), points.end() );
//        //
//        //
//        //    list<Vertex_handle> L = this->nearestNeighbors( d, 5 );
//        //
//        //    for( Vertex_handle l: L ) {
//        //        cout << (*l).point() << endl;
//        //    }
//        //
//
//
//
//            this->buildRRT();
//        }
//    private:
//        DelaunayTriangulation T;
//        Random_points_in_square_2<Point,Creator> g1; // random point iterator
//
//
//
//        bool buildRRT() {
//            Point rand;
//            Vertex_handle vh;
//            // Initialize T with start location
//            MAG::Node startNode( start, NULL );
//            this->insertIntoTree( startNode );
//
////            vector<pair<Point, MAG::Node>> points;
////            points.push_back( make_pair( start, MAG::Node() ) );
////            this->T.insert( points.begin(), points.end() );
//
//
//            // Set this->root
//
//            for( unsigned k=0; k<(this->K); k++ ) {
//                rand = this->randomState();    // generate random point
//                this->extend( rand );
//            }
//            // return T
//            return false;
//        }
//        Vertex_handle insertIntoTree( MAG::Node node ) {
//            Vertex_handle vh;
//
//            vh = this->T.insert( node.p );
//            vh->info() = node;
//
//            return vh;
//        }
//        int extend( Point x ) {
//            // TODO: determine the correct way to pass near to Node constructor (to set parent of xNew)
//            Vertex_handle near = (this->nearestNeighbor( x ));
//            optional<Point> xNew = this->newState( x, near->point(), false );
//            if( xNew ) {
//            //‘MAG::Node::Node( Point_2&, CGAL::Triangulation_vertex_base_with_info_2<MAG::Node, CGAL::Epick, CGAL::Triangulation_vertex_base_2<CGAL::Epick, CGAL::Triangulation_ds_vertex_base_2<CGAL::Triangulation_data_structure_2<CGAL::Triangulation_vertex_base_with_info_2<MAG::Node, CGAL::Epick> > > > >::Info&)’|
//                MAG::Node xNewNode( *xNew, &(near->info()) );
//                this->insertIntoTree( xNewNode );
////                vector<pair<Point, MAG::Node>> points;
////                points.push_back( make_pair( *xNew, MAG::Node( *xNew, near->info() ) ) );
////                this->T.insert( points.begin(), points.end() );
//                //this->T->E.push_back( { near, *xNew, false } );
//                if( *xNew == x ) {
//                    return REACHED;
//                } else {
//                    return ADVANCED;
//                }
//            }
//            return TRAPPED;
//        }
//
//        Point randomState() {
//            Point p = *( this->g1++ );  // random point, used for determining whether to goal skew or now
//            double chance = abs( p.x() ) + abs( p.y() ) / ( 2*this->size ) * 100; //
//
//            if( chance < this->goalSkewProbability ) {
//        //        cout << "skew to goal" << endl;
//                return this->goal;
//            } else {
//        //        cout << "random" << endl;
//                return *( this->g1++ );
//            }
//        }
//        Vertex_handle nearestNeighbor( Point x ) {
//            return this->T.nearest_vertex( x );
//        }
//        list<Vertex_handle> nearestNeighbors( Point p, int k ) {
//            std::list<Vertex_handle> L;
//            nearest_neighbors( this->T, p, k, std::back_inserter(L));
//            return L;
//        }
//        optional<Point> newState( Point x, Point xNear, bool uNew ) {
//            //Point xNew = x;
//            // if x not reachable from xNear under global constraints
//                // return {};
//            // make a move from xNear towards x under discrete time law
//            double factor = this->epsilon / sqrt( pow(x.x()-xNear.x(),2) + pow(x.y()-xNear.y(),2) );
//
//            Point p( xNear.x()+factor*(x.x()-xNear.x()), xNear.y()+factor*(x.y()-xNear.y()) );
//
//            // returns the new state or failure notice
//            return { p };
//        }
//};
