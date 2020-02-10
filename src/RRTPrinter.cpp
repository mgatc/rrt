#include "RRTPrinter.h"
#include <iostream>

using namespace std;

RRTPrinter::RRTPrinter( RRT_Tree tree, Point start, Point goal, string outputFileName ) :
    T( tree ), start( start ), goal( goal ), fileName( outputFileName ) {

}

double const radiusOfPoints = 1;

void RRTPrinter::displayPDF() {
    string fName = fileName + ".tex";
    FILE *fp = fopen(fName.c_str() ,"w");
    fprintf(fp,"\\documentclass{standalone} \n\\usepackage{tikz} \n \n\n\\begin{document}\n");
    fprintf(fp,"\n\n\n\\begin{tikzpicture}\n\n");

    for( Edge e : this->T.E ) {
        //fprintf( fp,"\\draw [fill=red,stroke=red] (%f,%f) circle [radius=%f];\n",e.p.x(),e.p.y(),radiusOfPoints );
        //fprintf( fp,"\\draw [fill=red,stroke=red] (%f,%f) circle [radius=%f];\n",e.q.x(),e.q.y(),radiusOfPoints );
        fprintf( fp,"\\draw (%f,%f) -- (%f,%f);",e.p.x(),e.p.y(),e.q.x(),e.q.y() );
    }
    // print start and goal
    fprintf( fp,"\\draw [fill=red,stroke=red] (%f,%f) circle [radius=%f];\n",start.x(),start.y(),radiusOfPoints );
    fprintf( fp,"\\draw [fill=green,stroke=green] (%f,%f) circle [radius=%f];\n",goal.x(),goal.y(),radiusOfPoints );

    fprintf(fp,"\n\n\\end{tikzpicture}");
    fprintf(fp,"\n\n\\end{document}");
    fclose(fp);

    cout << "\nOutput PDF generation started...\n";
    string command = "pdflatex " + fName + " > /dev/null";
    system(command.c_str());
    cout << "PDF generation terminated...\n";

   command = "atril " + fileName + ".pdf &";
   system(command.c_str());
}


