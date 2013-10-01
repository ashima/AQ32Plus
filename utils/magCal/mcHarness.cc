#include <cstdio>
#include <iostream>
using namespace std;

#include "matDumb.h"

enum { ln_N = 1024 };

char ln[ln_N];
enum { X1_N = 3 };

extern matrix<double, X1_N, X1_N> mcCal;
extern matrix<double, X1_N, 1>    mcCentre;
extern matrix<double, X1_N, 1>    mcRadii;

void mcAddPoint(double x, double y, double z);
void mcCompute();

int main()
  {
  FILE *fp = fopen("mag_ac3.dat","r") ;
  float x,y,z;

  while (!feof(fp))
    {
    fgets(ln, ln_N, fp);
    sscanf(ln, "%f,%f,%f", &x,&y,&z);
    mcAddPoint(x,y,z);
    }

  mcCompute();
  cout << "cal = "     << mcCal <<endl;  
  cout << "mCentre = " << mcCentre <<endl;  
  cout << "radii = "   << mcRadii <<endl;  
  fclose(fp);
  }
