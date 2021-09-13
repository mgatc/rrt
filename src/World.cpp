

#include <chrono>
#include <cmath>
#include <cstdlib>     /* srand, rand */
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <SDL.h>
#include <SDL_opengl.h>

#include "World.h"

using namespace MAG;

void appendToFile( std::string path, std::tuple<int,double,Cost,long int> data ) {
    // Open or create file
    std::ofstream file;
    file.open( path, std::fstream::app );

    if( !file.is_open() )
        std::cout << "File error!\n";

    char DELIMITER = ',';

    file << std::get<0>(data) << DELIMITER
         << std::get<1>(data) << DELIMITER
         << std::get<2>(data) << DELIMITER
         << std::get<3>(data) << DELIMITER
         << "\n";

    file.close();
}


/* Constructor(s) */

World::World() {
    std::srand( std::time(nullptr) );
}


/* Public member functions */

int World::getSize() {
    return m_WIDTH*m_P2M/2;
}

void World::startAgent( double start_x, double start_y, double goal_x, double goal_y, double w ) {
    // create dynamic body and add to world
    //addRect( x, y, w, w, 45*180/M_PI, true );

    double step = m_OBS_SIZE * m_P2M;
    //int persistence = m_GRID_SIZE*10*pow(5,10*m_OBS_DENSITY); // old
    int persistence = 30.0 * m_OBS_DENSITY * pow( m_GRID_SIZE, 2 ); // new
    int seed = std::rand();

    // RRT
    m_agent = new RRTstar<World>( *this, start_x, start_y, goal_x, goal_y, step, persistence, seed ); // set last argument for random seed
    m_agent->setGoalSkewProbability( 0.19 );

    Cost pathCost=-1;

    auto start = std::chrono::high_resolution_clock::now();      // start time
    auto stop = std::chrono::high_resolution_clock::now();       // stop time
    auto runtime = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    start = std::chrono::high_resolution_clock::now();      // start time
    bool foundGoal = m_agent->go();
    stop = std::chrono::high_resolution_clock::now();       // stop time
    runtime = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    if( foundGoal ) {
        pathCost = m_agent->goalCost();
    }

    // write results to csv file
    // grid_size, obstacle density, path cost, runtime
    std::tuple<int,double,Cost,long int> data( m_map.getSize(), m_map.getDensity(), pathCost, runtime.count() );
    appendToFile( "./results.csv", data );
    std::cout<<"done\n";

//    // manual raycast
//    if( obstacleFree( {x, y}, {0,0} ) )
//       std::cout<<"obstacleFree: "<< x << " " << y <<std::endl;
//    else
//        std::cout<<"obstacleFreeFails: "<< x << " " << y <<std::endl;

}



b2Body* World::addPathEdge( Point2 p, Point2 q, bool isGoalPath ) {
    // bodydef
    // world->CreateBody
    // fixturedef( shape )
    // body->CreateFixture

    //std::cout<<"addpathedge"<<std::endl;
    //std::cout<<"World::addEdgePath: "<<p.x<<" "<<p.y<<", "<<q.x<<" "<<q.y<<std::endl;

    double avg_x = (p.x+q.x)/2;
    double avg_y = (p.y+q.y)/2;
    double delta_px = avg_x-p.x;
    double delta_py = avg_y-p.y;
    double delta_qx = avg_x-q.x;
    double delta_qy = avg_y-q.y;

    b2BodyDef bodydef;
    bodydef.position.Set( avg_x, avg_y );
    bodydef.type = b2_staticBody;
    b2Body* body = m_world->CreateBody( &bodydef );
    //body->SetTransform( bodydef.position, angle );

    b2EdgeShape shape;
    shape.Set( b2Vec2( delta_px, delta_py ), b2Vec2( delta_qx, delta_qy ) );

    b2FixtureDef fixturedef;
    fixturedef.shape = &shape;
    fixturedef.filter.groupIndex = -2;
    body->CreateFixture( &fixturedef );

    return body;
}

bool World::obstacleFree( Point2 p, Point2 q ) {
    //std::cout<<"World::obstacleFree: "<<p.x<<" "<<p.y<<", "<<q.x<<" "<<q.y<<std::endl;
    b2RayCastInput input;
    input.p1 = b2Vec2( p.x, p.y );
    input.p2 = b2Vec2( q.x, q.y );
    input.maxFraction = 1;

    //check every fixture of every body to find closest
    float closestFraction = 1; //start with end of line as p2
    b2Vec2 intersectionNormal(0,0);
    for (b2Body* b = m_world->GetBodyList(); b; b = b->GetNext()) {
        for (b2Fixture* f = b->GetFixtureList(); f; f = f->GetNext()) {
            b2RayCastOutput output;
            if ( f->RayCast( &output, input, 0 ) && output.fraction < 1 )
                return false;
        }
    }

    return true;
}

//void World::setGoal( int x, int y ) {
//    setGoal( m_P2M*x, m_P2M*y );
//}
//
//void World::setGoal( double x, double y ) {
//
//    m_goal = {x,y}; // creates a new Point2
//    /* Two options here,
//            1) Add goal to m_map.m_obstacles with emplace( goal, true );
//            2) Remove entry with erase(goal);
//    */
//    //m_map.m_obstacles.emplace( m_goal, true );
//    m_map.m_obstacles.erase(m_goal);
//}
//
//Point2 World::getGoal() {
//    return m_goal;
//}

void World::createRandomMap( int size, double density ) {
    setGridSize(size);
    setObstacleDensity( density );
    m_map.createRandom( size, density );
}

void World::setObstacleDensity( double density ) {
    m_OBS_DENSITY = std::min( std::max( 0.0, density ), 1.0 ); // 0<=density<=1
}

double World::getObstacleDensity() {
    return m_OBS_DENSITY;
}

void World::setWindowSize( int size ) {
    m_WIDTH = size;
    m_HEIGHT = m_WIDTH;
    m_OBS_SIZE = m_WIDTH/m_GRID_SIZE;
    m_WIDTH_OFFSET = -1.0*m_WIDTH/2;
    m_HEIGHT_OFFSET = -1.0*m_HEIGHT/2;

    m_M2P = m_OBS_SIZE;
    m_P2M = 1/m_M2P;
}

int World::getWindowSize() {
    return m_WIDTH;
}

void World::setGridSize( int size ) {
    m_GRID_SIZE = size;
    m_OBS_SIZE = m_WIDTH/m_GRID_SIZE;
    m_M2P = m_OBS_SIZE;
    m_P2M = 1/m_M2P;
}

int World::getGridSize() {
    return m_GRID_SIZE;
}

int World::run() {
    SDL_Init( SDL_INIT_EVERYTHING );
    //SDL_SetVideoMode( 640,480,32,SDL_OPENGL ); // SDL 1.2
    SDL_Window *window = SDL_CreateWindow( // SDL 2.0
        "RRT",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        m_WIDTH,
        m_HEIGHT,
        SDL_WINDOW_OPENGL
    );
    SDL_Renderer *renderer = SDL_CreateRenderer(
        window,
        -1,
        0
    );

    SDL_Event event;
    Uint32 start;
    bool running = true;
    int loop_count = 0;

    init();

    while( running ) {
        start = SDL_GetTicks();
        while( SDL_PollEvent( &event ) ) {
            switch( event.type ) {
            case SDL_QUIT:
                running=false;
                break;
            case SDL_KEYDOWN:
                switch( event.key.keysym.sym ) {
                case SDLK_ESCAPE:
                    running = false;
                    break;
                }
                break;
            //case SDL_MOUSEBUTTONDOWN:
                //startAgent( (event.button.x-m_WIDTH/2)*m_P2M, (event.button.y-m_HEIGHT/2)*m_P2M, m_OBS_SIZE*m_P2M );//-m_WIDTH/2, event.button.y-m_HEIGHT/2, m_OBS_SIZE );
            }
        }
        display();
        m_world->Step( 1.0/30.0, 5, 5 ); // update

        if( loop_count < 10 )
            loop_count++;
        else if( m_agent == nullptr )  // start rogue agent
            startAgent( -9*m_GRID_SIZE/20, -9*m_GRID_SIZE/20, 0, 0, 3 );


        SDL_GL_SwapWindow( window );

        if( 1000.0/30 > SDL_GetTicks()-start )
            SDL_Delay( 1000.0/30 - ( SDL_GetTicks() - start ) );

    }
    SDL_DestroyWindow( window );
    SDL_Quit();

    if( m_world != nullptr )
        delete m_world;
    if( m_agent != nullptr )
        delete m_agent;

    return EXIT_SUCCESS;
}

/* Protected member functions */

b2Body* World::addRect( double x, double y, double w, double h, int angle, bool dyn ) {
    // bodydef
    // world->CreateBody
    // fixturedef( shape )
    // body->CreateFixture
    b2BodyDef bodydef;
    bodydef.position.Set( x, y );
    if(dyn)
        bodydef.type = b2_dynamicBody;
    b2Body* body = m_world->CreateBody( &bodydef );
    body->SetTransform( bodydef.position, angle );

    b2PolygonShape shape;
    shape.SetAsBox( w/2, h/2 );

    b2FixtureDef fixturedef;
    fixturedef.shape = &shape;
    fixturedef.density = 1.0;
    fixturedef.filter.groupIndex = 2;
    body->CreateFixture( &fixturedef );

    return body;
}

void World::drawSquare( b2Vec2* points, b2Vec2 center, double angle, b2BodyType type ) {
    //std::cout<<"called drawSquare"<<std::endl;
    switch(type) {
        case b2_staticBody:
            glColor3f( 0.93, 0.93, 0.93 );
            //std::cout<<"printing static body"<<std::endl;
            break;

        case b2_dynamicBody:
            glColor3f( 0, 1, 1 );
            //std::cout<<"printing dynamic body"<<std::endl;
            break;
    }

    glPushMatrix();
        glTranslatef( center.x*m_M2P, center.y*m_M2P, 0 );
        glRotatef( angle*180/M_PI,0,0,1 );
        glBegin( GL_QUADS );
            for( int i=0; i<4; i++ )
                glVertex3f( points[i].x*m_M2P, points[i].y*m_M2P, -0.5 );
        glEnd();
    glPopMatrix();
}

void World::drawStart() {
    Point2 p = m_agent->getStartPoint();
    glColor3f( 0, 1, 1 );
    glPushMatrix();
        glTranslatef( p.x*m_M2P, p.y*m_M2P, 0 );
        //glRotatef( angle*180/M_PI,0,0,1 );
        glBegin( GL_QUADS );
            glVertex3f( (p.x*m_M2P+m_OBS_SIZE*2), p.y*m_M2P, 0.6 );
            glVertex3f( p.x*m_M2P, (p.y*m_M2P-m_OBS_SIZE*2), 0.6 );
            glVertex3f( (p.x*m_M2P-m_OBS_SIZE*2), p.y*m_M2P, 0.6 );
            glVertex3f( p.x*m_M2P, (p.y*m_M2P+m_OBS_SIZE*2), 0.6 );
        glEnd();
    glPopMatrix();
}

void World::drawGoal() {
    Point2 p = m_agent->getGoalPoint();
    glColor3f( 1, 0.63, 0.63 );
    glPushMatrix();
        glTranslatef( p.x*m_M2P, p.y*m_M2P, 0 );
        //glRotatef( angle*180/M_PI,0,0,1 );
        glBegin( GL_QUADS );
            glVertex3f( (p.x*m_M2P+m_OBS_SIZE*2), p.y*m_M2P, 0.6 );
            glVertex3f( p.x*m_M2P, (p.y*m_M2P-m_OBS_SIZE*2), 0.6 );
            glVertex3f( (p.x*m_M2P-m_OBS_SIZE*2), p.y*m_M2P, 0.6 );
            glVertex3f( p.x*m_M2P, (p.y*m_M2P+m_OBS_SIZE*2), 0.6 );
        glEnd();
    glPopMatrix();
}

void World::drawEdge( b2Vec2 v1, b2Vec2 v2, b2Vec2 center, bool isGoalPath ) {
    int lineWidth;
    if( isGoalPath ) {
        glColor3f( 1, 0.63, 0.63 );
        lineWidth=5;
    } else {
        glColor3f( 0, 1, 1 );
        lineWidth=2;
    }
    //std::cout<<"World::drawEdge: "<<v1.x<<" "<<v1.y<<", "<<v2.x<<" "<<v2.y<<std::endl;

    glPushMatrix();
        //glTranslatef( center.x*m_M2P, center.y*m_M2P, 0 );
        //glRotatef( angle*180/M_PI,0,0,1 );
        glLineWidth( lineWidth );
        glBegin( GL_LINES );
            glVertex3f( (center.x+v1.x)*m_M2P, (center.y+v1.y)*m_M2P, 0 );
            glVertex3f( (center.x+v2.x)*m_M2P, (center.y+v2.y)*m_M2P, 0 );
        glEnd();
    glPopMatrix();
}

void World::drawGoalPath() {
// draw shortest path
    for( auto it=m_agent->thePath.begin(), next=std::next(it,1); next!=m_agent->thePath.end(); it++, next++ ) {

        b2Vec2 p( it->point.x(), it->point.y() );
        b2Vec2 q( next->point.x(), next->point.y() );
        double avg_x = (p.x+q.x)/2;
        double avg_y = (p.y+q.y)/2;
        double delta_px = avg_x-p.x;
        double delta_py = avg_y-p.y;
        double delta_qx = avg_x-q.x;
        double delta_qy = avg_y-q.y;

        drawEdge(
            b2Vec2( delta_px, delta_py),
            b2Vec2( delta_qx, delta_qy),
            b2Vec2( avg_x, avg_y ),
            true
        );
    }
}

void World::display() {
    glClear( GL_COLOR_BUFFER_BIT );
    glLoadIdentity();
    b2Body* tmp = m_world->GetBodyList();
    b2Vec2 points[4];
    while( tmp ) {
        b2Shape* shape = tmp->GetFixtureList()->GetShape();
        switch( shape->GetType() ) {

        case b2Shape::Type::e_polygon:
            for( int i=0;i<4;i++ ) {
                points[i] = ((b2PolygonShape*)shape)->m_vertices[i];
            }
            drawSquare( points, tmp->GetWorldCenter(), tmp->GetAngle(), tmp->GetType() );
            break;

        case b2Shape::Type::e_edge:
            drawEdge( ((b2EdgeShape*)shape)->m_vertex1, ((b2EdgeShape*)shape)->m_vertex2, tmp->GetWorldCenter() );
            break;
        }
        tmp = tmp->GetNext();
    }
    if( m_agent != nullptr ) {
        drawStart();
        drawGoal();
        if( m_agent->pathExists() )
            drawGoalPath();
    }


}


void World::initGL() {
    glMatrixMode( GL_PROJECTION );
    glOrtho( -1*m_WIDTH/2, m_WIDTH/2, m_HEIGHT/2, -1*m_HEIGHT/2, -1, 1 ); // setup coordinate system
    glMatrixMode( GL_MODELVIEW );
    glClearColor( 0.13,0.13,0.13,1 );
}

void World::initWorld( double gx, double gy ) {
    m_world = new b2World( b2Vec2( gx,gy ) ); // x and y gravitational forces
    // create new map, addRect to add obstacles to world
    if( !m_map.m_obstacles.empty() )
        for( auto it = m_map.m_obstacles.begin(); it != m_map.m_obstacles.end(); it++ ) {
            addRect(
                it->first.x,
                it->first.y,
                m_OBS_SIZE*m_P2M,
                m_OBS_SIZE*m_P2M,
                false
            );
        }
}

void World::init() {
    initGL();
    initWorld(0,0);
}
