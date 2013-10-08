#include <cstdio>
#include <iostream>
using namespace std;

#include "matDumb.h"
#include "calibration/magCal.h"

enum { ln_N = 1024 };

char ln[ln_N];
//enum { X1_N = 3 };


void mcAddPoint(double x, double y, double z);
void mcCompute();
double mcOTOConditionNumber();

enum { conditionEveryN = 100 };

int main()
  {
  FILE *fp = stdin ; //fopen("mag_ac3.dat","r") ;
  float x,y,z;
  int i;
  double accOmega[ X4_N ];
  matrix<double, 5,3> calMat;

  mcInit(accOmega);

  for (i=0; !feof(fp); ++i )
    {
    fgets(ln, ln_N, fp);
    sscanf(ln, "%f,%f,%f", &x,&y,&z);
    mcAddPoint(accOmega,x,y,z);
    if (0 == i % conditionEveryN)
      cout << mcOTOConditionNumber(accOmega) << endl;
    }

  mcCompute((double*)&calMat(0,0),accOmega);
  cout << "cal = "     << calMat <<endl;
  fclose(fp);
  }
