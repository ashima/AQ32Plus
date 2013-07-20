/**
  \file       state_constants.h
  \brief      Constants for Bispoke Kalman filter for height state from a BMP180.
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/AQ32Plus
  \remark     Ported for AQ32Plus.
*/


#define EX(X) ( 1.0 / (float_tt)(1ULL<<(X)))

float_tt c3 , c4 , b1_, c5 , c6 , mc_, md_, ps , x0 , x1 , x2 , y0_ , y1_ , y2 ;
float_tt z0 = (3791.0 - 0) / 1600.0 ;
float_tt z1 = 1.0 - 7357. * EX(20) ;
float_tt z2 = 3038.0 * 100. * EX(36) ;
float_tt P_0 = 101325.0 / 1.00134 ;
float_tt T_0 = 25.0 ;

float_tt rho = 5.;  //.255;
float_tt z_max = 44330.0;
float_tt scale = 0.01;

