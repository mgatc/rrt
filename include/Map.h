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
        void createRandom( size_t size, double density );
        void displayPDF( std::string outputFileName );
        size_t getSize();
        double getDensity();

protected:
    /* Protected types */

    /* Protected data members */
        size_t m_size;
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
