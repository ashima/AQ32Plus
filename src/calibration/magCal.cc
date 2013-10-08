/**
  \file       magCal.cc
  \brief      Magnetometer Calibration routines, to gather data and do an
              ellipsoidal fit.
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/AQ32Plus
  \remark     Ported for AQ32Plus.
*/

#include <inttypes.h>
#include <math.h>

#include "matDumb.h"
#include "magCal.h"

typedef matrix<double, X1_N, 1>    VX1_t;
typedef matrix<double, X2_N, 1>    VX2_t;
typedef matrix<double, X4_N, 1>    VX4_t;

typedef matrix<double, X1_N, X1_N> MX1_t;
typedef matrix<double, X2_N, X2_N> MX2_t;

typedef struct {
  MX1_t K;
  VX1_t c;
  VX1_t iLambda;
  } calmat_t;

//matrix<double, X4_N, 1>    accOmega(0.0) ; // initilized to zero.
//MX1_t mcCal;
//VX1_t    mcCentre;
//VX1_t    mcRadii;

//void mcAccumulate( matrix<double, X4_N, 1> &, float,float,float);
void mcAccumulate( VX4_t &, float,float,float);

void mcCollectO( MX2_t &, VX2_t &, VX4_t &) ;
//void mcCollectO( MX2_t &, 
//                  VX2_t &,
//                  matrix<double, X4_N, 1> & );

void mcCollectA(  MX1_t &, VX1_t &, VX2_t & );
//void mcCollectA(  MX1_t &,
//                   VX1_t &,
//                   VX2_t & );

void mcColAndSolve( VX2_t &, VX4_t & );
//void mcColAndSolve( VX2_t &u,
//                    VX4_t &accOmega );

void mcDeEllipsoid( calmat_t &, VX2_t & );
//void mcDeEllipsoid( MX1_t &cal,
//                    VX1_t   &c,
//                    VX1_t   &iL,
//                    VX2_t   &u );


template<typename T>
T sqr(T x) { return x * x ; }

template<typename T>
void eigen(matrix<T,3,3> &Q, matrix<T,3,1> &L, matrix<T,3,3> &A)
  {
  T p,q,r, a00, a11 ;
  int i,j;
  matrix<T,3,3> B, C;
  const T pi23 = (3.14159265358979323846 * 2.0) / 3.0 ;

  q = m_trace(A)/3.0;
 
  for (i=0; i < 3 ; ++i)
  for (j=0; j < 3 ; ++j)
    B(i,j) = A(i,j) - (i==j ? q : 0 );

  C.fill(0.);
  m_mac(C, B, B);
  p = sqrt( m_trace(C) / 6.0) ;
  B.scale( 1.0 / p );
  r = 0.5 * m_det(B);

  if (r>1.0) r = 1.0;
  else if (r<-1.0) r = -1.0;

  T theta = acos(r) / 3.0;

  L(0,0) = q + p * 2.0 * cos ( theta + pi23 * 3.0 );
  L(1,0) = q + p * 2.0 * cos ( theta + pi23 * 1.0 );
  L(2,0) = q + p * 2.0 * cos ( theta + pi23 * 2.0 );

  T a10_21 = A(1,0)*A(2,1);
  T a20_01 = A(2,0)*A(0,1);
  T a10_01 = A(1,0)*A(0,1);
  for (i=0; i < 3; ++i)
    {
    a00 = A(0,0) - L(i,0);
    a11 = A(1,1) - L(i,0);

    r = sqr( Q(0,i) = a10_21 - A(2,0)*a11 )
      + sqr( Q(1,i) = a20_01 - A(2,1)*a00 )
      + sqr( Q(2,i) = a00*a11 - a10_01    ) ;

    r = 1.0 / sqrt(r);

    Q(0,i) *= r; 
    Q(1,i) *= r; 
    Q(2,i) *= r; 
    }
  }

void mcInit(double accOmega[X4_N])
  { // C entry
  ((VX4_t*)accOmega)->fill(0.);
  }

void mcAddPoint(double accOmega[X4_N], double x, double y, double z)
  { // C entry.
  mcAccumulate( *((VX4_t*)accOmega), x,y,z);
  }

void mcDeCal(VX1_t &zp, VX1_t &z, calmat_t &calmat)
  {
  VX1_t tmpz;

  m_sub(tmpz, z, calmat.c);
  zp.fill(0.);
  m_mac(zp, calmat.K, tmpz);
  }

void mcCDeCal(floatXYZ_t* v, double calmat[calmat_N] )
  { // C entry.
  matrix<double, 3, 1> z( (float*)v);

  mcDeCal(z,z, *((calmat_t*)calmat) );

  v->x = z(0,0);
  v->y = z(1,0);
  v->z = z(2,0);
  }

void mcCompute( double calmat[calmat_N], double accOmega[X4_N])
  { // C entry.
  VX2_t u;

  mcColAndSolve(u, *((VX4_t*)accOmega) );
  mcDeEllipsoid( *((calmat_t*)calmat), u);
 // mcDeEllipsoid(mcCal,mcCentre, mcRadii, u);
  }

void mcColAndSolve( VX2_t &u, VX4_t &accOmega )
  {
  MX2_t OmegaTOmega(0.);
  MX2_t LDL;
  VX2_t Omega1;
  mcCollectO( OmegaTOmega, Omega1, accOmega );

  m_ldlT( LDL, OmegaTOmega );
  m_solve_ldlT( u, LDL, Omega1 ); // return in u.
  }

template<typename T, unsigned int N>
T sym_norm(matrix<T,N,N> &x)
  {
  T a, m;
  m = 0.;
  for (int j = 0 ; j < (int)N ; ++j)
    {
    a = 0.;
    for (int i = 0 ; i < (int)N ; ++i)
      a += fabs( i<j ? x(i,j) : x(j,i) );
    if (a > m)
      m = a;
    }
  return m;
  }

double mcOTOConditionNumber(double accOmega[X4_N] ) 
  { // C entry.
    // I'm sure there should be a faster way to get
    // at a condition number without having to fully invert.
  MX2_t OmegaTOmega(0.), LDL(0.), invOTO(0.);
  VX2_t Omega1;

  mcCollectO( OmegaTOmega, Omega1, *((VX4_t*)accOmega) );
  m_ldlT( LDL, OmegaTOmega );
  m_inv_ldlT( invOTO, LDL ) ;

  return log(sym_norm(OmegaTOmega) * sym_norm(invOTO));
  }

void mcDeEllipsoid( calmat_t &calmat, VX2_t &u )
  {
  VX1_t  iL, L, tmpc, cp;
  MX1_t  Q, A, tmpS;
  matrix<double, 1, 1> r11 ;
  double r;
  int i;

  mcCollectA( A, cp, u );
  eigen( Q, L, A );
  
  for (i = 0; i < L.dim_1 ; ++i)
    iL(i,0) = 1.0 / L(i,0);

  m_maD ( tmpS, Q, iL );
  tmpc.fill(0.);
  m_maTc( tmpc,  Q,  cp );
  calmat.c.fill(0.);
  m_mac( calmat.c, tmpS, tmpc );
  calmat.c.scale(-1.0);   // return c

  tmpc.fill(0.);
  m_mac ( tmpc, A, calmat.c );
  r11.fill(0.);
  m_maTc( r11, calmat.c, tmpc ) ;

  r = 1.0 / ( r11(0,0) + 1.0) ;

  for (i = 0; i < L.dim_1 ; ++i)
    {
    L(i,0)  =  sqrt( r * L(i,0) );
    calmat.iLambda(i,0) =  1.0 / L(i,0);  // return iLambda
    }
  
  m_maD (tmpS, Q, L );
  calmat.K.fill(0.);
  m_macT(calmat.K,  tmpS, Q ); // return K
  }


