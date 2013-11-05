/**
  \file       filter.c
  \brief      Bispoke Kalman filter for height state from a BMP180
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/AQ32Plus
  \remark     Ported for AQ32Plus.
*/
#include <stdio.h>
// #include <iostream>

#include <inttypes.h>
#include <math.h>
using namespace std;

#include "filter.h"

#include "state_constants.h"
#include "model_eqs.c"

extern uint8_t overSamplingSetting;
extern int ac1 , ac2 , ac3;
extern unsigned int ac4 , ac5 , ac6 ;
extern int b1 , b2 , mb , mc , md ;

void filterSetParams() throw()
 //bmp_params_t &x)
 {
 c3  = 160.0f * EX(15) * (float_tt) ac3 ;
 c4  = 0.001f * EX(15) * (float_tt) ac4  ;
 b1_ = 160.0f * 160.0f * EX(30) * (float_tt) b1 ;
 c5  = (float_tt) ac5 * EX(15) / 160.0f  ;
 c6  = (float_tt) ac6 ;
 mc_ = (float_tt) mc * (float_tt)(1<<11) / (160.0f * 160.0f) ;
 md_ = (float_tt) md / 160.0f ;
 ps  = 1.0f / (float_tt (1 << overSamplingSetting) ) ;
 x0 = (float_tt) ac1 ;
 x1 = (float_tt) ac2 * 160.0f * EX(13) ;
 x2 = (float_tt) b2  * 160.0f * 160.0f * EX(25) ;
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

float tOfDN(int ut)
  {
  float a_ = c5 * ( (float)ut - c6 );
  float t = a_ + mc_ / (a_ + md_);
  return t;
  }

float pOfDN(int up,float t)
  {
  float s = t - T_0;

  float x = ((x2  * s) + x1 ) * s + x0;
  float y = ((y2  * s) + y1_) * s + y0_;

  float q = ((float)up * ps - x ) / y  ;
  float p  = 100. * (((z2 * q) + z1) * q + z0) ;
  return p;
  }

float aOfp(float p)
  {
  float a = z_max * (1. -  pow( p / P_0, (1./rho) ) ) ;
  return a;
  }


template<typename T, uint NS>
void fwdP(matrix<T,NS,NS> &P,
matrix<T,NS,NS> &F,
matrix<T,NS,NS> &Q ) throw()
  {
  matrix<T,NS,NS> t(0.0f);

  m_mac(t,F,P);
  P = Q;
  m_macT(P,t,F);
  }

extern "C" uint32_t micros();
extern "C" void dump_filter();
extern uint32_t lt ;
extern int32_t filter_dt;

template<typename T, uint N>
void df_dt(matrix<T,N,1> &df, matrix<T,N,1> &f) 
  {
  df(0,0) = f(1,0);
  df(1,0) = f(2,0) - 0.4 * f(1,0);
  df(2,0) = 0.;
  }

template<typename T, uint NS, uint NO>
void filterFwd(filter_t<T,NS,NO> &f, matrix<T,NS,NS> &Q) throw()
  {
  uint32_t now = micros();
  float_tt dt;

  if (lt == 0)
    filter_dt = 1; // tiny tiny step.
  else
    filter_dt = (now - lt);

//  dt = filter_dt * 1.0e-6f;
  dt = 0.01 ;
  lt = now;
//printf ("dt = %f\n",dt );

  f.x(0,0) += dt * f.x(1,0); // forward BMP's temp.

  matrix<T,3,1> y0, y1, y2,y3, k1,k2,k3,k4;

  y0(0,0) = f.x(2,0);
  y0(1,0) = f.x(3,0);
  y0(2,0) = f.x(4,0);

  df_dt(k1, y0);

  y1.blit(k1);
  y1.scale(dt * 0.5);
  m_acc(y1, y0);

  df_dt(k2, y1);
 
  y2.blit(k2);
  y2.scale(dt * 0.5);
  m_acc(y2, y0);

  df_dt(k3, y2);

  y3.blit(k3);
  y3.scale(dt);
  m_acc(y3, y0);

  df_dt(k4, y3);

  k2.scale(2.0);
  k3.scale(2.0);

  m_acc(k1,k2);
  m_acc(k1,k3);
  m_acc(k1,k4);

  k1.scale(dt/6.0);
  f.x(2,0) += k1(0,0);
  f.x(3,0) += k1(1,0);
  f.x(4,0) += k1(2,0);

  fwdP(f.P,f.F,Q);

  m_symify(f.P);
  }


template<typename T, uint NS, uint NO>
void filterGainUpdate(matrix<T,NO,NS> H, matrix<T,NS,NS> &P,
                matrix<T,NS,1UL> &x, matrix<T,NO,1UL> &y,
                matrix<T,NO,NO> &R) throw()
  {
 /* J = H P
   S = R + J H'
   LDL' = S
   solve (LDL')K' = J
   x = x + K y 
   P = P - K'J
*/
  matrix<T, NS,NS > t(0.0f);
  matrix<T, NS,NO > K;
  matrix<T, NO,NS > j(0.0f), KT(0.0f);
  matrix<T, NO,NO > S,l;

  m_mac(j,H,P);
  S = R;
  m_macT(S,j,H);
  m_ldlT(l,S);
  m_solve_ldlT(KT,l,j);
  //KT.zeroise(1.0e-50);
  m_maTc(x,KT,y);
  m_maTc(t,KT,j);
  m_ncc(P,t);
  m_transpose(K,KT);
//cout << "K = " << K << " S = " << S << endl;
  }

// Filter specific ...

void make_Q(matrix<float_tt,ns,ns> &Q, float_tt dt) throw()
  {
#if 0
//origonal 
  float_tt dt2 = dt * dt;
  float_tt dt3_2 = dt2 * dt * 0.5f;
  float_tt dt4_4 = dt2 * dt2 * 0.25f;

  float_tt dt4 = dt2 * dt2;
  float_tt dt4 = dt2 * dt2;
 
  Q(0,0) = sigma_a * dt4_4;
  Q(0,1) = sigma_a * dt3_2;
  Q(1,0) = Q(0,1);
  Q(1,1) = sigma_a * dt2;
#endif
  float_tt dt2, dt3,dt4,dt5,dt6,dt7,dt8;

  dt8 = dt * ( dt7 = dt * ( dt6 = dt * ( dt5 = dt * 
  (dt4 = dt * ( dt3 = dt * ( dt2 = dt * dt ) )))));

  float_tt sigma_T  = 1.0e3f; 
  float_tt sigma_A  = 1.0e4f; 
// Pressure with acc
  Q(0,0) =          sigma_T * dt4 / 4.0;
  Q(1,0) = Q(0,1) = sigma_T * dt3 / 2.0;
  Q(1,1) =          sigma_T * dt2;

  Q(2,2) =          sigma_A * dt6 / 36.0; // dt8 / 576.0;
  Q(3,2) = Q(2,3) = sigma_A * dt5 / 12.0; // dt7 / 144.0;
  Q(4,2) = Q(2,4) = sigma_A * dt4 / 6.0 ; // dt6 / 48.0;

  Q(3,3) =          sigma_A * dt4 / 4.0 ; // dt6 / 36.0;
  Q(4,3) = Q(3,4) = sigma_A * dt3 / 2.0 ; // dt5 / 12.0;

  Q(4,4) =          sigma_A * dt2 ;       // dt4 / 4.0 ;

#if 0
//Simple pressure
  Q(0,0) =          dt4 / 4.0 ;
  Q(1,0) = Q(0,1) = dt3 / 2.0 ;
  Q(1,1) =          dt2 ;

  Q(2,2) =          dt4 / 4.0 ;
  Q(2,3) = Q(3,2) = dt3 / 2.0 ;
  Q(3,3) =          dt2 ;
#endif
  }

#if 0
void make_R( matrix<float_tt,no,no> &R, 
             float_tt sigma_z_T, float_tt sigma_z_P,float_tt sigma_z_TP) throw()
  {
  R(0,0) = sigma_z_T;
  R(1,1) = sigma_z_P;
  R(0,1) = sigma_z_TP;
  R(1,0) = sigma_z_TP;
  //R(2,2) = sigma_z_a;
  }
#endif

void make_F(matrix<float_tt,ns,ns> &F, float_tt dt) throw()
  {
  float_t dt2 = dt*dt /2.0;
  float_t dt3 = dt2 *dt /3.0 ;

/*      0    1    2     3     4        5

   0 |  1    dt   0     0     0        0  
   1 |  0    1    0     0     0        0  
   2 |  0    0    1    dt    dt^2 /2  dt^3 / 6
   3 |  0    0    0     1    dt       dt^2 /2  
   4 |  0    0    0     0     1       dt  
   5 |  0    0    0     0     0        1

*/ 

#if 1
//Pressure with acc
  for (uint32_t i = 0 ; i < ns ; ++i )
    F(i,i) = 1.0;

  F(0,1) = 
  F(2,3) = F(3,4) = dt;
  F(2,4) = dt2;
#endif

#if 0
//Simple Pressure
  F(0,0) = F(1,1) = F(2,2) = F(3,3) =  1.0 ;
  F(0,1) = F(2,3) = dt ;
#endif
  }


//  f.dF(0,1) = f.dF(2,3) = dt;

void filterInit(filter_t<float_tt,ns,no> &f , float_tt dt) throw()
  {
  f.x(2,0) = 230.234269f; 
  f.x(0,0) = 28.941330f;
#if 0
// Simple Pressure
  f.P(0,0) = f.P(2,2) = 100.0f;
  f.P(1,1) = f.P(3,3) = 0.1f;
#endif

#if 1
// Pressure with acc
float x = 1.0e2;
  f.P(0,0) = f.P(2,2) = x ;
  f.P(4,4) = x ;
  f.P(1,1) = f.P(3,3) = x * 0.01;
#endif

  make_F(f.F, 0.01);  
  //make_Q(f.Q, 0.001f, 0.01);

  //f.Rts(0,0) = 40.0f;
  //f.Rps(0,0) = 20.0f;
  //f.Ras(0,0) = 0.060f;
#if 0
  make_Q(f.Q, 1.0f, 0.01);
  f.Rts(0,0) = 10.00f;
  f.Rps(0,0) = 10.00f;
#endif
#if 1
  make_Q(f.Q, 0.01);
  //make_Q(f.Q, 1.0e-7f, 0.01);

// Cov of raw P vaules = 12532.0 
// Cov of raw T values = 8836.0
// Cov of raw A values = 0.91932
  f.Rts(0,0) =  8800.0;
  f.Rps(0,0) = 13000.0 * 1.0;
  f.Ras(0,0) =     0.92 * 1.0;
  //f.Rts(0,0) = 1.0e1f;
  //f.Rps(0,0) = 1.0e2f;
  //f.Ras(0,0) = 1.0e-2;
#endif
  }

#if 0
void filter_step_tp(filter_t<float_tt,ns,no> &f, uint32_t theta, uint32_t phi) throw()
  {
  matrix<float_tt,no,1UL>  y;
  matrix<float_tt,no,ns> h;

  filterFwd(f, f.Q); 

  y(0,0) = (float_t)theta - theta_T(f.x(0,0) );
  y(1,0) = (float_t)phi   - phi_zT(f.x(2,0), f.x(0,0) );
  h(0,0) = d_Ttheta_T( f.x(0,0) ) ;
  h(1,0) = d_Tphi_zT( f.x(2,0), f.x(0,0) );
  h(1,2) = d_zphi_zT( f.x(2,0), f.x(0,0) );
 
  h(0,1) = h(0,2) = h(0,3) = 
  h(1,1) = h(1,3) = 0.0f;
  filterGainUpdate(h, f.P, f.x, y, f.R);
  }
#endif

void filter_update_t( filter_t<float_tt,ns,no> &f, uint32_t theta) throw()
  {
  matrix<float_tt,1UL,1UL>  y;
  matrix<float_tt,1UL,ns> h(0.0f);

  y(0,0) = (float_t)theta - theta_T(f.x(0,0) );
  h(0,0) = d_Ttheta_T( f.x(0,0) ) ;
 
  filterGainUpdate(h, f.P, f.x, y, f.Rts);
  }

void filter_update_a( filter_t<float_tt,ns,no> &f, float_tt z_dot_dot) throw()
  {
#if 1
  float_tt zt = fabs(z_dot_dot);
  float_tt r = 1.;

  if ( zt > 0.25 )
    r = 10;
  else if ( zt > 1.0 )
    r = 100;
  else if ( zt > 5.0 )
    {
    f.x(3,0) = 0.;
    return;
    }

  matrix<float_tt,1UL,1UL>  y, R;
  matrix<float_tt,1UL,ns> h(0.0f);

  y(0,0) = z_dot_dot - f.x(4,0);
  h(0,4) = 1.0;

  R(0,0) = f.Ras(0,0) * r; 

  filterGainUpdate(h, f.P, f.x, y, R );
#endif
  }

void filter_update_p( filter_t<float_tt,ns,no> &f, uint32_t phi) throw()
  {
#if 1
  matrix<float_tt,1UL,1UL>  y;
  matrix<float_tt,1UL,ns> h(0.0f);

  y(0,0) = (float_t)phi   - phi_zT(f.x(2,0), f.x(0,0) );
  h(0,0) = d_Tphi_zT( f.x(2,0), f.x(0,0) );
  h(0,2) = d_zphi_zT( f.x(2,0), f.x(0,0) );
 
  filterGainUpdate(h, f.P, f.x, y, f.Rps);
#endif
  }

void filter_step(filter_t<float_tt,ns,no> &f) throw()
  {
  filterFwd(f, f.Q); 
  }

