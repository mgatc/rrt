#include "UnitDiskCoverCenters.h"
#include "CgalComponents.h"

UnitDiskCoverCenters::UnitDiskCoverCenters(vector<Point> &P, list<Point> &C) {

  DelaunayTriangulation T;
  bool isEmptyTriangulation = true;

  for( Point p : P ) {
        if(isEmptyTriangulation) {
          isEmptyTriangulation = false;
          C.push_back(p);
          T.insert(p);
        }
        else {
          Vertex_handle handleToTheNearestDiskCenter = T.nearest_vertex(p);
          if(squared_distance(p,handleToTheNearestDiskCenter->point()) > 1) {
            C.push_back(p);
            T.insert(p);
          }
        }
  }
  T.clear();
}

/*UnitDiskCoverCenters::UnitDiskCoverCenters(vector<Point> &P, vector<Point> &C) {
  for( Point p : P ) {
        bool isAlreadyCovered = false;
        for( Point c : C) {
            Circle_2<K> circle(c,1);
            if( circle.bounded_side(p) == ON_BOUNDED_SIDE || circle.bounded_side(p) == ON_BOUNDARY) {
                isAlreadyCovered = true;
                break;
            }
        }
        if( !isAlreadyCovered )
            C.push_back(p);
    }
}*/
