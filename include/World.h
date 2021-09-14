#ifndef WORLD_H
#define WORLD_H

#include <memory>

#include <box2d.h>

#include "Map.h"
#include "Point2.h"
#include "RRTstar.h"

namespace MAG {

class World {
public:

    World();

    int run();
    void setWindowSize( int size );
    int getWindowSize();
    void setGridSize( int size );
    int getGridSize();
    void setObstacleDensity( double density );
    double getObstacleDensity();
    void createRandomMap( int size, double density );
    //void setGoal( int x, int y );
    //void setGoal( double x, double y );
    //Point2 getGoal();
    //void startAgent( double start_x, double start_y, double goal_x, double goal_y, double w, double h );
    void startAgent( double start_x, double start_y, double goal_x, double goal_y, double w );
    b2Body* addPathEdge( Point2 p, Point2 q, bool isGoalPath = false );
    int getSize();
    bool obstacleFree( Point2 p, Point2 q );


protected:

    int m_WIDTH  = 1000; // If putting in class, make these data members
    int m_HEIGHT = m_WIDTH;
    int m_GRID_SIZE = 100;
    float m_OBS_DENSITY = 0.1;
    int m_OBS_SIZE = m_WIDTH/m_GRID_SIZE;
    float m_M2P = m_OBS_SIZE;
    float m_P2M = 1/m_M2P;
    int m_WIDTH_OFFSET = -1.0*m_WIDTH/2;
    int m_HEIGHT_OFFSET = -1.0*m_HEIGHT/2;
    Map m_map;
    //std::optional<Point2> m_start = std::nullopt;
    //Point2 m_goal;
    std::unique_ptr<b2World> m_world;
    std::unique_ptr<RRTstar<World>> m_agent;

    b2Body* addAgent( double x, double y, double w, double h );
    b2Body* addRect( double x, double y, double w, double h, int angle, bool dyn=false );
    void drawAgent( b2Vec2* points, b2Vec2 center );
    void drawSquare( b2Vec2* points, b2Vec2 center, double angle, b2BodyType type );
    void drawEdge( b2Vec2 v1, b2Vec2 v2, b2Vec2 center, bool isGoalPath = false );
    void drawGoal();
    void drawStart();
    void drawGoalPath();
    void display();
    void initGL();
    void initWorld( double gx, double gy );
    void init();

};

}

#endif
