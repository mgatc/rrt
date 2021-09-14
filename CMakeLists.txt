cmake_minimum_required(VERSION 3.19)
project(rrt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-fopenmp -fexceptions -frounding-math -Wall -Wextra -Wno-unused")

include_directories(./include)

include_directories(/usr/include)
include_directories(../../../usr/local/lib)
#include_directories(../../../usr/lib/x86_64-linux-gnu)
include_directories(../../../usr/include/box2d/include/)
include_directories(../../../usr/include/box2d/include/box2d)
include_directories(../../../usr/local/include/SDL2)

add_executable(rrt
        include/Astar.h
        include/RRT.h
        include/RRTstar.h
        src/Map.cpp include/Map.h
        src/World.cpp include/World.h

        main.cpp )

target_link_libraries(rrt gmp)
target_link_libraries(rrt mpfr)
target_link_libraries(rrt boost_system)
target_link_libraries(rrt boost_thread)
target_link_libraries(rrt CGAL)
target_link_libraries(rrt box2d)
target_link_libraries(rrt SDL2)
target_link_libraries(rrt GL)
#target_link_libraries(rrt pthread)
#target_link_libraries(rrt dl)