#ifndef RRTPrinter_H
#define RRTPrinter_H

#include "CgalComponents.h"
#include "RRT.h"

#include<vector>

namespace MAG {
    class RRTPrinter {
        private:
            DelaunayTriangulation T;
            Point start;
            Point goal;
            string fileName;

        public:
            RRTPrinter( DelaunayTriangulation &tree, Point start, Point goal, string outputFileName );
            void displayPDF();
    };
}
#endif
