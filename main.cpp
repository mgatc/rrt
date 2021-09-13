#include "World.h"

using namespace MAG;
using namespace std;

int main( int argc, char *argv[] ) {

    int grid_size = 100;
    double density = 0.1;
    int window_size = 1750;

    if( argc >= 3 ){
        stringstream convert{ argv[1] };

        int grid_size_arg{};
        if( ( convert >> grid_size_arg ) ) // do the conversion
            grid_size = grid_size_arg;

        int density_arg{};
        if( ( convert >> density_arg ) ) // do the conversion
            density = density_arg;
    }


    World world;
    world.setWindowSize( window_size );
    world.createRandomMap( grid_size, density );
    world.run();

    return EXIT_SUCCESS;

}
