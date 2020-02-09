#ifndef UNITDISKCOVERPRINTER_H
#define UNITDISKCOVERPRINTER_H

#include "CgalComponents.h"

#include<vector>

using namespace std;

class UnitDiskCoverPrinter
{
    private:
        vector<Point> pointSet;
        list<Point> centersOfPlacedDisks;
        double r = 0.0;
        string fileName;

    public:
        UnitDiskCoverPrinter(vector<Point> &P, list<Point> &centersOfPlacedDisks, double radiusOfDisks, string outputFileName);
        void displayPDF();
};

#endif