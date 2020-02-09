#include "UnitDiskCoverUnitGrid.h"
#include "CgalComponents.h"

struct pair_hash {
    inline size_t operator()(const pair<int,int> &v) const {
        return v.first*31+v.second;
    }
};

UnitDiskCoverUnitGrid::UnitDiskCoverUnitGrid(vector<Point> &P,list<Point> &C) {
   // C.clear();

    unordered_set< pair<int,int>, pair_hash> hashTableForDiskCenters;

    for( Point p : P ) {
            int col = floor(p.y()), row = floor(p.x());

            if( row % 2 == 0) {
                if( col % 2 != 0) {

                    Point c1(row+1,col), c2(row,col+1);

                    /* p belongs to c1 but not to c2*/
                    if(  !(squared_distance(p,c1)>1) && squared_distance(p,c2) > 1) {
                        auto search = hashTableForDiskCenters.find(make_pair(row+1,col));
                        if( search == hashTableForDiskCenters.end()) {
                             hashTableForDiskCenters.insert(pair<int,int>(row+1,col));
                             C.push_back(c1);
                        }

                    }
                    /* p belongs to c2 but not to c1*/
                    else if( squared_distance(p,c1)>1 && !(squared_distance(p,c2) > 1) ){
                        auto search = hashTableForDiskCenters.find(make_pair(row,col+1));
                        if( search == hashTableForDiskCenters.end()) {
                             hashTableForDiskCenters.insert(pair<int,int>(row,col+1));
                             C.push_back(c2);
                        }
                    }
                    /* p belongs to both of them */
                    else {
                        auto search1 = hashTableForDiskCenters.find(make_pair(row+1,col));
                        auto search2 = hashTableForDiskCenters.find(make_pair(row,col+1));

                        if( search1 == hashTableForDiskCenters.end() && search2 == hashTableForDiskCenters.end()) {
                             hashTableForDiskCenters.insert(pair<int,int>(row+1,col));
                             C.push_back(c1);
                        }
                    }
                }
                else
                {
                    Point c1(row,col), c2(row+1,col+1);

                    /* p belongs to c1 but not to c2*/
                   if( !(squared_distance(p,c1)>1) && squared_distance(p,c2)>1) {
                        auto search = hashTableForDiskCenters.find(make_pair(row,col));
                        if( search == hashTableForDiskCenters.end()) {
                             hashTableForDiskCenters.insert(pair<int,int>(row,col));
                             C.push_back(c1);
                        }

                    }
                    /* p belongs to c2 but not to c1*/
                   else if(squared_distance(p,c1)>1 && !(squared_distance(p,c2)>1) ){
                        auto search = hashTableForDiskCenters.find(make_pair(row+1,col+1));
                        if( search == hashTableForDiskCenters.end()) {
                             hashTableForDiskCenters.insert(pair<int,int>(row+1,col+1));
                             C.push_back(c2);
                        }
                    }
                    /* p belongs to both of them */
                    else {
                        auto search1 = hashTableForDiskCenters.find(make_pair(row,col));
                        auto search2 = hashTableForDiskCenters.find(make_pair(row+1,col+1));

                        if( search1 == hashTableForDiskCenters.end() && search2 == hashTableForDiskCenters.end()) {
                             hashTableForDiskCenters.insert(pair<int,int>(row,col));
                             C.push_back(c1);
                        }
                    }
                }
            }
            else {
                if( col % 2 != 0) {

                   Point c1(row,col), c2(row+1,col+1);

                    /* p belongs to c1 but not to c2*/
                   if( !(squared_distance(p,c1)>1) && squared_distance(p,c2)>1) {
                        auto search = hashTableForDiskCenters.find(make_pair(row,col));
                        if( search == hashTableForDiskCenters.end()) {
                             hashTableForDiskCenters.insert(pair<int,int>(row,col));
                             C.push_back(c1);
                        }

                    }
                    /* p belongs to c2 but not to c1*/
                    else if(squared_distance(p,c1)>1 && !(squared_distance(p,c2)>1) ){
                        auto search = hashTableForDiskCenters.find(make_pair(row+1,col+1));
                        if( search == hashTableForDiskCenters.end()) {
                             hashTableForDiskCenters.insert(pair<int,int>(row+1,col+1));
                             C.push_back(c2);
                        }
                    }
                    /* p belongs to both of them */
                    else {
                        auto search1 = hashTableForDiskCenters.find(make_pair(row,col));
                        auto search2 = hashTableForDiskCenters.find(make_pair(row+1,col+1));

                        if( search1 == hashTableForDiskCenters.end() && search2 == hashTableForDiskCenters.end()) {
                             hashTableForDiskCenters.insert(pair<int,int>(row,col));
                             C.push_back(c1);
                        }
                    }

                }
                else {
                    Point c1(row,col+1), c2(row+1,col);

                    /* p belongs to c1 but not to c2*/
                    if( !(squared_distance(p,c1)>1) && squared_distance(p,c2)>1) {
                        auto search = hashTableForDiskCenters.find(make_pair(row,col+1));
                        if( search == hashTableForDiskCenters.end()) {
                             hashTableForDiskCenters.insert(pair<int,int>(row,col+1));
                             C.push_back(c1);
                        }

                    }
                    /* p belongs to c2 but not to c1*/
                    else if(squared_distance(p,c1)>1 && !(squared_distance(p,c2)>1) ){
                        auto search = hashTableForDiskCenters.find(make_pair(row+1,col));
                        if( search == hashTableForDiskCenters.end()) {
                             hashTableForDiskCenters.insert(pair<int,int>(row+1,col));
                             C.push_back(c2);
                        }
                    }
                    /* p belongs to both of them */
                    else {
                        auto search1 = hashTableForDiskCenters.find(make_pair(row,col+1));
                        auto search2 = hashTableForDiskCenters.find(make_pair(row+1,col));

                        if( search1 == hashTableForDiskCenters.end() && search2 == hashTableForDiskCenters.end()) {
                             hashTableForDiskCenters.insert(pair<int,int>(row,col+1));
                             C.push_back(c1);
                        }
                    }

                }
            }
    }
    //hashTableForDiskCenters.clear();
}
