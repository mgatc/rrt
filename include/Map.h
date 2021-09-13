#ifndef MAP_H
#define MAP_H

#include <map>                              // storing obstacles
#include <string>
#include "Point2.h"



namespace MAG {

class Map {

public:

    /* Public data members */
        std::map< Point2,bool > m_obstacles;

    /* Constructor(s) */
        Map();

    /* Public member functions */
        void createRandom( int size, double density );
        void displayPDF( std::string outputFileName );
        int getSize();
        double getDensity();

protected:
    /* Protected types */

    /* Protected data members */
        int m_size;
        double m_density;

    /* Protected member functions */
        Point2 integerize( Point2 p );

private:
    /* Private data members */

    /* Private member functions */
        Point2 randomPoint2( int offset, int range );
        int randomNum( int offset, int range );
};

}

#endif
