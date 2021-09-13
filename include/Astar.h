#ifndef ASTAR_H
#define ASTAR_H

//#include "../include/CgalComponents.h"
//#include "RRT.h"
#include "Point2.h"

namespace MAG {

    /*
     * Exception for termination of search.
     */
    struct found_goal {
        MAG::Vertex vertex;
    };

    template< class CostType >
    CostType distanceBetween( MAG::Point p, MAG::Point q ) {
        CostType res = CGAL::sqrt<CostType>( CGAL::squared_distance<K>( p, q ) );
        //std::cout<<"distance Between(p,q):"<<res<<std::endl;
        return res;
    }

    /*
     * Distance heuristic for search,
     * Euclidean implementation.
     */
    template< class GraphType, class CostType, class LocMap >
    class distance_heuristic : public boost::astar_heuristic<GraphType, CostType> {

        public:
            distance_heuristic( LocMap l, MAG::GraphVertex goal )
                : m_location(l), m_goal(goal)
            {

            }
            CostType operator()( MAG::GraphVertex u ) {
                return distanceBetween<CostType>( m_location[m_goal].point, m_location[u].point );
            }

        private:
            MAG::GraphVertex m_goal;
            LocMap m_location;
    };



    /*
     * Visitor that terminates when we find the goal.
     */
    template <class GVertex, class CostType, class LocMap>
    class astar_goal_visitor : public boost::default_astar_visitor {
        public:
            astar_goal_visitor( LocMap l, GVertex goal ) : m_goal(goal), m_location(l) {}

            template <class GraphType>
            void examine_vertex(GVertex u, GraphType& g) {
                //std::cout << m_location[u].point << std::endl;
                if( u == m_goal || distanceBetween<CostType>( m_location[m_goal].point, m_location[u].point ) < epsilon ) {
                    throw m_location[u];
//                    Vertex v = { m_location[u].point };
//                    throw v;
                }
            }
        private:
            GVertex m_goal;
            CostType epsilon = MAG::GOAL_EPSILON;
            LocMap m_location;
    };

}



#endif
