#include "../include/Map.h"

#include <iostream>
#include <fstream>                          // read from file
#include <algorithm>                        // for min, max
#include <ctime>                            // random seed
#include <random>                           // map generation
#include <string>

using namespace MAG;

/* Constructor(s) */

Map::Map()
{
    std::srand( std::time(nullptr) );
}

/* Public member functions */
void Map::createRandom( int size, double density ) {
    if( size <= 0 )
        return;

    std::srand( std::time(nullptr) );
    m_size = size;
    density = std::min( std::max( 0.0, density ), 1.0 ); // 0<=density<=1
    m_density = density;

    int coord = size/2;
    /*
     * Must account for zones which do not contain obstacles to avoid making
     * the map too thick.
     *
     * Total obstacle free zones according to the conditions below is equal to:
     *
     * 0.1*0.1*size*size*5 = 0.05*size*size
     */
    int nObj = (0.95*size*size) * density;
    double goal_zone = 0.1*size;
    double neg_goal_zone = -1.0*goal_zone;
    //std::cout<<density<<std::endl;

    Point2 p1;

    while( m_obstacles.size() < nObj ) {
        // create shape
        p1 = integerize( randomPoint2( -1*size/2, size ) );

        // ensure no collisions between existing obstacles or start/goal
        if( p1.x > goal_zone || p1.x < neg_goal_zone ||
          ( ( neg_goal_zone <= p1.x && p1.x <= goal_zone ) && (p1.y > goal_zone || p1.y < neg_goal_zone ) ) )
            if(p1.x>-1*coord+goal_zone || (p1.x <=-1*coord+goal_zone && p1.y >=-1*coord+goal_zone ))
                m_obstacles.emplace( p1, false );

    }
}

int Map::getSize() {
    return m_size;
}

double Map::getDensity() {
    return m_density;
}

void Map::displayPDF( std::string fileName ) {
    double const radiusOfPoints = 1;
    std::string fName = fileName + ".tex";
    FILE *fp = fopen(fName.c_str() ,"w");

    // start document
    fprintf(fp,"\\documentclass{standalone} \n\\usepackage{tikz} \n \n\n\\begin{document}\n");
    fprintf(fp,"\n\n\n\\begin{tikzpicture}\n\n");

//    fprintf( fp,"\\draw [fill=red,stroke=green] (%f,%f) circle [radius=%f];\n",m_start_v.x,m_start_v.y,radiusOfPoints );
//    fprintf( fp,"\\draw [fill=red,stroke=red] (%f,%f) circle [radius=%f];\n",m_goal_v.x,m_goal_v.y,radiusOfPoints );
//
//    // draw map
//    for( Vector3 v : m_obstacles ) {
//        fprintf( fp,"\\filldraw (%f,%f) rectangle (%f,%f);\n", v.x, v.y, v.x+1, v.y+1 );
//    }


    // close document
    fprintf(fp,"\n\n\\end{tikzpicture}");
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





/* Protected member functions */

//AABB Map::aabbFromVector( Vector3 v ) {
//    return AABB(
//        Vector3( v.x-0.5, v.y-0.5, v.z-0.5 ),
//        Vector3( v.x+0.5, v.y+0.5, v.z+0.5 )
//    );
//}


Point2 Map::randomPoint2( int offset, int range ) {
    int dim = 2;
    int c[dim];

    for(unsigned i=0; i<dim; i++ ) {
        c[i] = randomNum( offset, range );
        //std::cout<< c[i] <<std::endl;
    }
    return { c[0], c[1] };
}

Point2 Map::integerize( Point2 p ) {
    p.x = static_cast<int>(p.x);
    p.y = static_cast<int>(p.y);
    return p;
}

//void Map::make2D( AABB &tree ) {
//    Vector3 min = tree.getMin();
//    Vector3 max = tree.getMax();
//    m_aabb.setMin( Vector3( min.x, min.y, -1 ) );
//    m_aabb.setMax( Vector3( max.x, max.y,  1 ) );
//}

int Map::randomNum( int offset, int range ) {
    return static_cast<double>( std::rand() ) / RAND_MAX * range + offset;
}

//std::string Map::to_string( Vector3& v ) {
//    return std::to_string(v.x) + " "
//         + std::to_string(v.y) + " "
//         + std::to_string(v.z);
//}

//double Map::chance() {
//    Point p = randomPoint();  // random point, used for determining whether to use big or small shape
//    return CGAL::to_double( CGAL::abs( p.x() ) + CGAL::abs( p.y() ) ) / ( size );
//}






//
//#include "../include/Map.h"
//
//#include <CGAL/nearest_neighbor_delaunay_2.h>                   // nearest neighbors
//#include <CGAL/squared_distance_2.h>                            // A*
//#include <CGAL/number_utils.h>                                  // A*
//#include <CGAL/Random.h>                                        // seeding random generator
//
//
//#include <boost/graph/adjacency_list.hpp>               // graph
//#include <boost/graph/random.hpp>                       // A*
//#include <boost/random.hpp>                             // A*
//#include <boost/property_map/property_map.hpp>          // A*
//#include <boost/graph/graphviz.hpp>                     // A*
//
//#include <fstream>                          // read from file
//#include <math.h>                           // for sqrt, A*
//
//using namespace MAG;
//
///* Constructor(s) */
//
//Map::Map( int size, double density )
//    : size(size), density(density), g1( size/2 )
//{
//    int area = size * size;
//    double goalObstacleArea = area * density;
//    double actualObstacleArea = 0;
//    int smallShapeArea = 6;
//    int largeShapeArea = 24;
//    double bigShapeThreshold = 0.2; // the larger triangle is 5x bigger
//
//    while( actualObstacleArea < goalObstacleArea ) {
//        Point p = randomPoint();
//        Point_3 p1 = Point_3( p.x(), p.y(), -1 );
//        Point_3 p2 = Point_3( p.x(), p.y()+4, -1 );
//        Point_3 p3 = Point_3( p.x()+3, p.y(), -1 );
//        Point_3 p4 = Point_3( p.x()+1, p.y()+2, 10 );
//        double area = 6;
//        if( chance() < bigShapeThreshold ) {
//            p2 = Point_3( p1.x(), p1.y()+8, -1 );
//            p3 = Point_3( p1.x()+6, p1.y(), -1 );
//            area = 24;
//        }
//        Polyhedron poly;
//        poly.make_tetrahedron( p1, p2, p3, p4 );
//        shapes.push_back( poly );
//        actualObstacleArea += area;
//        tree.insert( faces(poly).first, faces(poly).second, poly );
//    }
//    std::cout<<tree.size()<<std::endl;
//}
//
//
//
///* Public member functions */
//bool Map::doesCollide( Point_3 p, Point_3 q ) {
//    Segment seg_query = Segment( p, q );
//    return true;//tree.do_intersect( seg_query );
//}
//
//bool Map::doesCollide( Point p, Point q ) {
//    return doesCollide(
//        Point_3( p.x(), p.y(), 0 ),
//        Point_3( q.x(), q.y(), 0 )
//    );
//}
//
//void Map::displayPDF( std::string fileName ) {
//    //double const radiusOfPoints = 1;
//    std::string fName = fileName + ".tex";
//    FILE *fp = fopen(fName.c_str() ,"w");
//
//    // start document
//    fprintf(fp,"\\documentclass{standalone} \n\\usepackage{tikz} \n \n\n\\begin{document}\n");
//    fprintf(fp,"\n\n\n\\begin{tikzpicture}\n\n");
//
//    // draw map
//    for( Polyhedron p : shapes ) {
//        std::vector<Point_3> points;
//        for( auto it = p.points_begin(); it != p.points_end(); it++ ) {
//            if( it->z() < 1 )
//                points.push_back(*it);
//        }
//        Triangle t;
//        fprintf(
//            fp,
//            "\\filldraw (%f,%f) -- (%f,%f) -- (%f,%f) -- (%f,%f);\n",
//            points[0].x(),
//            points[0].y(),
//            points[1].x(),
//            points[1].y(),
//            points[2].x(),
//            points[2].y(),
//            points[0].x(),
//            points[0].y()
//        );
//        //std::cout << source(*ei, T) << "(" << locationMap[source(*ei,T)].point << ") <->" << target(*ei, T) << "(" << locationMap[target(*ei,T)].point << ")" << std::endl;
//    }
//
//
//    // close document
//    fprintf(fp,"\n\n\\end{tikzpicture}");
//    fprintf(fp,"\n\n\\end{document}");
//    fclose(fp);
//
//    // generate pdf
//    std::cout << "\nOutput PDF generation started...\n";
//    std::string command = "pdflatex " + fName + " > /dev/null";
//    system(command.c_str());
//    std::cout << "PDF generation terminated...\n";
//
//    // open pdf
//    command = "atril " + fileName + ".pdf &";
//    system( command.c_str() );
//}
//
//
//
//
//
///* Protected member functions */
//
//Point Map::randomPoint() {
//    return *( g1++ );
//}
//
//double Map::chance() {
//    Point p = randomPoint();  // random point, used for determining whether to use big or small shape
//    return CGAL::to_double( CGAL::abs( p.x() ) + CGAL::abs( p.y() ) ) / ( size );
//}
//
//
