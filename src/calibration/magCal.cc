#include <inttypes.h>
#include <math.h>
#include <iostream>
using namespace std;

#include "matDumb.h"

enum { X1_N = 3, X2_N = 9, X4_N = 34 };

matrix<double, X4_N, 1>    accOmega ; // initilized to zero.
matrix<double, X1_N, X1_N> mcCal;
matrix<double, X1_N, 1>    mcCentre;
matrix<double, X1_N, 1>    mcRadii;

void mcAccumulate( matrix<double, X4_N, 1> &, float,float,float);
void mcCollectO( matrix<double, X2_N, X2_N> &, 
                  matrix<double, X2_N, 1> &,
                  matrix<double, X4_N, 1> & );
void mcCollectA(  matrix<double, X1_N, X1_N> &,
                   matrix<double, X1_N, 1> &,
                   matrix<double, X2_N, 1> & );

void mcColAndSolve( matrix<double, X2_N,1> &u,
                    matrix<double, X4_N,1> &accOmega );
void mcDeEllipsoid( matrix<double, X1_N, X1_N> &cal,
                    matrix<double, X1_N, 1>   &c,
                    matrix<double, X1_N, 1>   &iL,
                    matrix<double, X2_N, 1>   &u );


template<typename T>
T sqr(T x) { return x * x ; }

template<typename T>
void eigen(matrix<T,3,3> &Q, matrix<T,3,1> &L, matrix<T,3,3> &A)
  {
  T p,q,r, a00, a11 ;
  int i,j;
  matrix<T,3,3> B, C;
  const T pi23 = (M_PI * 2.0) / 3.0 ;

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

void mcAddPoint(double x, double y, double z)
  {
  mcAccumulate(accOmega, x,y,z);
  }

void mcDeCal(matrix<double, X1_N,1> &zp, matrix<double, X1_N,1> &z)
  {
  matrix<double, X1_N, 1> tmpz;

  m_sub(tmpz, z, mcCentre);
  zp.fill(0.);
  m_mac(zp, mcCal, tmpz);
  }

void mcCompute()
  {
  matrix<double, X2_N, 1 >   u;

  mcColAndSolve(u, accOmega);

  mcDeEllipsoid(mcCal,mcCentre, mcRadii, u);

  }

void mcColAndSolve( matrix<double, X2_N,1> &u,
                    matrix<double, X4_N,1> &accOmega )
  {
  matrix<double, X2_N, X2_N> OmegaTOmega;
  matrix<double, X2_N, X2_N> LDL;
  matrix<double, X2_N, 1      > Omega1;
  mcCollectO( OmegaTOmega, Omega1, accOmega );

  m_ldlT( LDL, OmegaTOmega );
  m_solve_ldlT( u, LDL, Omega1 ); // return in u.
  }

void mcDeEllipsoid( matrix<double, X1_N, X1_N> &cal,
                    matrix<double, X1_N, 1>   &c,
                    matrix<double, X1_N, 1>   &iL,
                    matrix<double, X2_N, 1>   &u )
  {
  matrix<double, X1_N, 1 >    L, tmpc, cp;
  matrix<double, X1_N, X1_N > Q, A, tmpS;
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
  c.fill(0.);
  m_mac ( c, tmpS, tmpc );
  c.scale(-1.0);   // return c

  tmpc.fill(0.);
  m_mac ( tmpc, A, c );
  m_maTc( r11, c, tmpc ) ;

  r = 1.0 / ( r11(0,0) + 1.0) ;

  for (i = 0; i < L.dim_1 ; ++i)
    {
    L(i,0)  =  sqrt( r * L(i,0) );
    iL(i,0) =  1.0 / L(i,0);  // return iL
    }
  
  m_maD (tmpS, Q,    L );
  cal.fill(0.);
  m_macT(cal,  tmpS, Q ); // return Cal
  }


