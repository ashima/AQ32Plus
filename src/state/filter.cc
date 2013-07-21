/**
  \file       filter.c
  \brief      Bispoke Kalman filter for height state from a BMP180
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/AQ32Plus
  \remark     Ported for AQ32Plus.
*/

#include <inttypes.h>
#include <string>
#include <math.h>
using namespace std;

#include "filter.h"

#include "state_constants.h"
#include "model_eqs.c"

extern uint8_t overSamplingSetting;
extern int ac1 , ac2 , ac3;
extern unsigned int ac4 , ac5 , ac6 ;
extern int b1 , b2 , mb , mc , md ;


void filterSetParams()  //bmp_params_t &x)
 {
 c3  = 160.0 * EX(15) * (float_tt) ac3 ;
 c4  = 0.001 * EX(15) * (float_tt) ac4  ;
 b1_ = 160. * 160. * EX(30) * (float_tt) b1 ;
 c5  = (float_tt) ac5 * EX(15) / 160.0  ;
 c6  = (float_tt) ac6 ;
 mc_ = (float_tt) mc * (float_tt)(1<<11) / (160. * 160.) ;
 md_ = (float_tt) md / 160. ;
 ps  = 1.0 / (float_tt (1 << overSamplingSetting) ) ;
 x0 = (float_tt) ac1 ;
 x1 = (float_tt) ac2 * 160.0 * EX(13) ;
 x2 = (float_tt) b2  * 160.0 * 160. * EX(25) ;
 y0_ = c4 * (float_tt)(1<<15) ;
 y1_ = c4 * c3 ;
 y2 = c4 * b1_ ;
/*
 cout << " Params = \n" ;

 cout << "c3 =" << c3 << ", " ;
 cout << "c4 =" << c4 << ", " ;
 cout << "c5 =" << c5 << ", " ;
 cout << "c6 =" << c6 << ", " ;
 cout << "b1_ =" << b1_ << ", " ;
 cout << "mc_ =" << mc_ << ", " ;
 cout << "md_ =" << md_ << ", " ;
 cout << "ps =" << ps << ", " ;
 cout << "x0 =" << x0 << ", " ;
 cout << "x1 =" << x1 << ", " ;
 cout << "x2 =" << x2 << ", " ;
 cout << "y0_ =" << y0_ << ", " ;
 cout << "y1_ =" << y1_ << ", " ;
 cout << "y2 =" << y2 << ", " ;
 cout << "z0 =" << z0 << ", " ;
 cout << "z1 =" << z1 << ", " ;
 cout << "z2 =" << z2 << ", " ;
 cout << "\n";
*/
 }



template<typename T, uint NS>
void fwdP(matrix<T,NS,NS> &P,
matrix<T,NS,NS> &F,
matrix<T,NS,NS> &Q )
  {
  matrix<T,NS,NS> t(0.);

  m_mac(t,F,P);
  P = Q;
  m_macT(P,t,F);
  }

template<typename T, uint NS, uint NO>
void gainUpdate(matrix<T,NO,NS> H, matrix<T,NS,NS> &P, matrix<T,NS,1> &x, matrix<T,NO,1> &y,
matrix<T,NO,NO> &R)
  {
 /* J = H P
   S = R + J H'
   LDL' = S
   solve (LDL')K' = J
   x = x + K y 
   P = P - K'J
*/
  matrix<T, NS,NS > t(0.);
  matrix<T, NS,NO > K;
  matrix<T, NO,NS > j(0.), KT(0.);
  matrix<T, NO,NO > S,l;

  m_mac(j,H,P);
  S = R;
  m_macT(S,j,H);
  m_ldlT(l,S);
  m_solve_ldlT(KT,l,j);
  m_maTc(x,KT,y);
  m_maTc(t,KT,j);
  m_ncc(P,t);
  m_transpose(K,KT);
  }

// Filter specific ...

void make_QR(matrix<float_tt,ns,ns> &Q, matrix<float_tt,no,no> &R, 
  float_tt sigma_a, float_tt sigma_z_T, float_tt sigma_z_P,float_tt sigma_z_TP, float_tt dt)
  {
  float_tt dt2 = dt * dt;
  float_tt dt3_2 = dt2 * dt * 0.5;
  float_tt dt4_4 = dt2 * dt2 * 0.25;

  Q(0,0) = sigma_a * dt4_4;
  Q(0,1) = sigma_a * dt3_2;
  Q(1,0) = Q(0,1);
  Q(1,1) = sigma_a * dt2;

  Q(2,2) = sigma_a * dt4_4;
  Q(2,3) = sigma_a * dt3_2;
  Q(3,2) = Q(2,3);
  Q(3,3) = sigma_a * dt2;

  R(0,0) = sigma_z_T;
  R(1,1) = sigma_z_P;
  R(0,1) = sigma_z_TP;
  R(1,0) = sigma_z_TP;
  }


void filterInit(filter_t<float_tt,ns,no> &f , float_tt dt)
  {
  f.F(3,3) = f.F(2,2) = f.F(1,1) = f.F(0,0) = 1.0;
  f.F(0,1) = f.F(2,3) = dt;

  f.dF(0,1) = f.dF(2,3) = dt;

  f.x(2,0) = 230.234269; 
  f.x(0,0) = 28.941330;

  f.P(0,0) = f.P(1,1) = f.P(2,2) = f.P(3,3) = 1.0;

  make_QR(f.Q,f.R, 0.1, 28.0, 33.0, 1.0, dt);
  }

void filterStep( filter_t<float_tt,ns,no> &f, uint32_t theta, uint32_t phi)
  {
  matrix<float_tt,no,1>  y;
  matrix<float_tt,no,ns> h;

  m_mac(f.x,f.dF,f.x) ;
  fwdP(f.P,f.F,f.Q);

  y(0,0) = (float_t)theta - theta_T(f.x(0,0) );
  y(1,0) = (float_t)phi   - phi_zT(f.x(2,0), f.x(0,0) );
  h(0,0) = d_Ttheta_T( f.x(0,0) ) ;
  h(1,0) = d_Tphi_zT( f.x(2,0), f.x(0,0) );
  h(1,2) = d_zphi_zT( f.x(2,0), f.x(0,0) );
 
  h(0,1) = h(0,2) = h(0,3) = 
  h(1,1) = h(1,3) = 0.;
  gainUpdate(h, f.P, f.x, y, f.R);
  }

