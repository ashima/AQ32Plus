/**
  \file       magCal.h
  \brief      Magnetometer Calibration routines, to gather data and do an
              ellipsoidal fit.
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/AQ32Plus
  \remark     Ported for AQ32Plus.
*/

#include "localtypes.h"

#ifdef __cplusplus
extern "C" {
#endif

enum { X1_N = 3, X2_N = 9, X4_N = 34,
       calmat_N = X1_N*X1_N + X1_N + X1_N };

void mcInit(double [X4_N]);
void mcAddPoint(double[X4_N], double, double, double );
void mcCDeCal(floatXYZ_t*, double[X4_N] );
void mcCompute(double [calmat_N], double [X4_N] );
double mcOTOConditionNumber(double [X4_N] ) ;

#ifdef __cplusplus
}
#endif

