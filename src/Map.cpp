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
void Map::createRandom( size_t size, double density ) {

    std::srand( std::time(nullptr) );
    m_size = size;
    density = std::min( std::max( 0.0, density ), 1.0 ); // 0<=density<=1
    m_density = density;

    auto coord = static_cast<int>(size/2);
    /*
     * Must account for zones which do not contain obstacles to avoid making
     * the map too thick.
     *
     * Total obstacle free zones according to the conditions below is equal to:
     *
     * 0.1*0.1*size*size*5 = 0.05*size*size
     */
    size_t nObj = (0.95*size*size) * density;
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

size_t Map::getSize() {
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

Point2 Map::randomPoint2( int offset, int range ) {
    size_t dim = 2;
    int c[dim];

    for(unsigned i=0; i<dim; i++ ) {
        c[i] = randomNum( offset, range );
        //std::cout<< c[i] <<std::endl;
    }
    return { static_cast<double>(c[0]), static_cast<double>(c[1]) };
}

Point2 Map::integerize( Point2 p ) {
    p.x = static_cast<int>(p.x);
    p.y = static_cast<int>(p.y);
    return p;
}

int Map::randomNum( int offset, int range ) {
    return static_cast<double>( std::rand() ) / RAND_MAX * range + offset;
}
