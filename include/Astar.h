#ifndef ASTAR_H
#define ASTAR_H

#include "CgalComponents.h"


namespace MAG {

    /*
     * Exception for termination of search.
     */
    struct found_goal {
        Vertex vertex;
    };

    template< class CostType >
    CostType distanceBetween( Point p, Point q ) {
        CostType dx = p.x() - q.x();
        CostType dy = p.y() - q.y();
        return std::sqrt( dx * dx + dy * dy );
    }

    /*
     * Distance heuristic for search,
     * Euclidean implementation.
     */
    template< class GraphType, class CostType, class LocMap >
    class distance_heuristic : public boost::astar_heuristic<GraphType, CostType> {

        public:
            distance_heuristic( LocMap l, GraphVertex goal )
                : m_location(l), m_goal(goal)
            {

            }
            CostType operator()( GraphVertex u ) {
                return distanceBetween<CostType>( m_location[m_goal].point, m_location[u].point );
            }

        private:
            GraphVertex m_goal;
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
                std::cout << m_location[u].point << std::endl;
                if( u == m_goal || distanceBetween<CostType>( m_location[m_goal].point, m_location[u].point ) < epsilon ) {
                    Vertex v = { m_location[u].point, u };
                    throw v;
                }
            }
        private:
            GVertex m_goal;
            CostType epsilon = 2;
            LocMap m_location;
    };

}



#endif
