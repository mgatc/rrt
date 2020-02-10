#ifndef RRTPrinter_H
#define RRTPrinter_H

#include "CgalComponents.h"
#include "RRT_Tree.h"

#include<vector>

using namespace std;

class RRTPrinter
{
    private:
        RRT_Tree T;
        Point start;
        Point goal;
        string fileName;

    public:
        RRTPrinter( RRT_Tree tree, Point start, Point goal, string outputFileName );
        void displayPDF();
};

#endif
