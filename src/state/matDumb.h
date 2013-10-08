/**
  \file       matDumb.h
  \brief      Simple 'Dumb' matrix ops that lets the compiler optimize well.
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/AQ32Plus
  \remark     Ported for AQ32Plus.
  \remark     The goal here is to produced well optimized, correct code; NOT
              to have a nicely wrapped operators that make expressions look
              nice! (but leave a lot of mess behind, and can't be unrolled or
              tiled). Much left to do.
*/

//#include <assert.h>

typedef unsigned int uint ;

template<typename T, uint N, uint M>
class matrix
  {
  T data[N][M];

public:
  enum { dim_1 = N, dim_2 = M, size = N*M };

  T &operator()(uint i, uint j)
    {
    // assert( i < N && j < M );
    return this->data[i][j];
    }

  const T &operator()(uint i, uint j) const
    {
    // assert( i < N && j < M );
    return this->data[i][j];
    }

  void blit(matrix<T,N,M>& s)
    {
    for (uint i = 0 ; i < N*M ; ++i)
      ((T*)data)[i] = ((T*)(s.data))[i];
    }

  void fill(T v)
    {
    for (uint i = 0 ; i < N*M ; ++i)
      ((T*)data)[i] = v;
    }

  void scale(T v)
    {
    for (uint i = 0 ; i < N*M ; ++i)
      ((T*)data)[i] *= v;
    }

  matrix()                 {};
  matrix(matrix<T,N,M> &s) { blit(s); }
  matrix(T v)              { fill(v); }
  template<typename S>
  matrix(S* p)
    {
    for (uint i = 0 ; i < N*M ; ++i)
      ((T*)data)[i] = (T)(p[i]);
    }

  matrix<T,N,M> &operator=(matrix<T,N,M>& s) { blit(s); return *this; }
  };

template<typename S, typename T, uint A, uint B>
S &operator<<(S &os, matrix<T,A,B> m)
  {
  os << "( " ;
  for (uint i=0; i < A ; ++i) 
    {
    os << "( " ;
    for (uint j = 0 ; j < B ; ++j )
      os << m(i,j) << ", " ;
    os << ")," ;
    }
  os << ")" ;
  return os;
  }

// d = s'
template<typename T, uint A, uint B>
void m_transpose( matrix<T, B, A> &d, matrix<T, A, B> &s ) {
  for (uint i = 0 ; i < A ; ++i )
    for (uint  j = 0 ; j < B ; ++j )
      d(j,i) = s(i,j);
  }

// d += a * b
template<typename T, uint A, uint B,uint C>
void m_mac(matrix<T,A,C> &d, matrix<T,A,B> &a, matrix<T,B,C> &b )
  {
  for (uint i = 0; i < A; ++i)
    for (uint j = 0; j < C; ++j )
      for (uint k = 0; k < B; ++k )
        d(i,j) = d(i,j) + a(i,k) * b(k,j);
  }

// d += a * diag(d)
template<typename T, uint A, uint B>
void m_maD(matrix<T,A,B> &d, matrix<T,A,B> &a, matrix<T,B,1> &b )
  {
  for (uint i = 0; i < A; ++i)
    for (uint j = 0; j < B; ++j )
      d(i,j) = a(i,j) * b(j,0);
  }

// d += a' * b
template<typename T, uint A, uint B,uint C>
void m_maTc(matrix<T,A,C> &d, matrix<T,B,A> &a, matrix<T,B,C> &b )
  {
  for (uint i=0; i < A ; ++i)
    for (uint  j = 0 ; j < C ; ++j )
      for (uint k = 0 ; k < B ; ++k )
        d(i,j) = d(i,j) + a(k,i) * b(k,j);
  }

// d += a * b'
template<typename T, uint A, uint B,uint C>
void m_macT(matrix<T,A,C> &d, matrix<T,A,B> &a, matrix<T,C,B> &b )
  {
  for (uint i=0; i < A ; ++i)
    for (uint  j = 0 ; j < C ; ++j )
      for (uint k = 0 ; k < B ; ++k )
        d(i,j) = d(i,j) + a(i,k) * b(j,k);
  }

// d = a + b
template<typename T, uint A, uint B>
void m_add(matrix<T,A,B> &d, matrix<T,A,B> &a, matrix<T,A,B> &b )
  {
  for (uint i=0; i < A ; ++i)
    for (uint  j = 0 ; j < B ; ++j )
        d(i,j) = a(i,j) + b(i,j);
  }

// d = a - b
template<typename T, uint A, uint B>
void m_sub(matrix<T,A,B> &d, matrix<T,A,B> &a, matrix<T,A,B> &b )
  {
  for (uint i=0; i < A ; ++i)
    for (uint  j = 0 ; j < B ; ++j )
        d(i,j) = a(i,j) - b(i,j);
  }

// d += a 
template<typename T, uint A, uint B>
void m_acc(matrix<T,A,B> &d, matrix<T,A,B> &a)
  {
  for (uint i=0; i < A ; ++i)
    for (uint  j = 0 ; j < B ; ++j )
        d(i,j) = d(i,j) + a(i,j);
  }

// d -= a
template<typename T, uint A, uint B>
void m_ncc(matrix<T,A,B> &d, matrix<T,A,B> &a)
  {
  for (uint i=0; i < A ; ++i)
    for (uint  j = 0 ; j < B ; ++j )
        d(i,j) = d(i,j) - a(i,j);
  }

// a = 0.5 * (a + a')
template<typename T, uint A>
void m_symify(matrix<T,A,A> &x)
  {
  uint32_t i,j;
  T a;
  for (i = 0; i < A-1 ; ++i)
  for (j = i+1; j < A ; ++j)
    {
    a = 0.5 * ( x(i,j) + x(j,i) );
    x(i,j) = x(j,i) = a;
    } 
  }

template<typename T, uint A>
T m_trace( matrix<T,A,A> &a )
  {
  T t = 0;

  for (uint i=0; i < A ; ++i)
    t += a(i,i);

  return t;
  }

// Determinates. Just 1x1, 2x2, 3x3 for now.
template<typename T>
T m_det( matrix<T,1,1> &a )
  {
  return a(0,0);
  }

template<typename T>
T m_det( matrix<T,2,2> &a )
  {
  return a(0,0)*a(1,1) - a(0,1)*a(1,0);
  }

template<typename T>
T m_det( matrix<T,3,3> &a )
  {
  return   a(0,0)*a(1,1)*a(2,2) - a(0,2)*a(1,1)*a(2,0)
         + a(0,1)*a(1,2)*a(2,0) - a(0,1)*a(1,0)*a(2,2)
         + a(0,2)*a(1,0)*a(2,1) - a(0,0)*a(1,2)*a(2,1);
  }

/* Find L D L' = A
 */
template<typename T, uint A>
void m_ldlT(matrix<T,A,A> &l, matrix<T,A,A> &m)
  {
  T a;
  l(0,0) = m(0,0);
  for (uint j=1; j < A ; ++j)
    l(j,0) = m(0,j) / l(0,0);
  for (uint i=1; i < A ; ++i)
    {
    a = m(i,i);
    for (uint k=0; k < i ; ++k)
      a -= l(i,k) * l(i,k) * l(k,k) ;
    l(i,i) = a;

    for (uint  j = i+1 ; j < A ; ++j )
      {
      a = m(i,j);
      for (uint k=0; k < i ; ++k)
        a -= l(i,k) * l(j,k) * l(k,k) ;
      l(j,i) = a / l(i,i);
      }
    }
  }

/*  solve (L D L') X  = Y
 */

template<typename T, uint N,uint M>
void m_solve_ldlT(matrix<T,N,M> &x, matrix<T,N,N> &l, matrix<T,N,M> &y)
  {
  T a;
  for (int j=0; j < (int)M; ++j )
    {
    x(0,j) = y(0,j);
    for (int i=1; i < (int)N; ++i)
      {
      a = y(i,j);
      for (int k=0; k < i ; ++k)
        a -= l(i,k) * x(k,j) ;
      x(i,j) = a;
      }
    x(N-1,j) = x(N-1,j) / l(N-1,N-1);
    for (int i=N-2; i >= 0; --i)
      {
      a = x(i,j) / l(i,i);
      for (int k=i+1; k < (int)N ; ++k)
        a -= l(k,i) * x(k,j) ;
      x(i,j) = a;
      }
    
    }
  }

template<typename T, uint N>
void m_inv_ldlT(matrix<T,N,N> &x, matrix<T,N,N> &l)
  {
  T a;
  for (int j=0; j < (int)N; ++j )
    {
    x(j,j) = 1.0;
    for (int i=j+1; i < (int)N; ++i)
      {
      a = 0.;
      for (int k=0; k < i ; ++k)
        a -= l(i,k) * x(k,j) ;
      x(i,j) = a;
      }
    x(N-1,j) = x(N-1,j) / l(N-1,N-1);
    for (int i=N-2; i >= j; --i)
      {
      a = x(i,j) / l(i,i);
      for (int k=i+1; k < (int)N ; ++k)
        a -= l(k,i) * x(k,j) ;
      x(i,j) = a;
      }
    for (int i = j-1; i>=0 ; --i)
      x(i,j) = x(j,i);
    }

  }
 
template<typename T, uint N>
void m_unpack_ld(matrix<T,N,N> &l, matrix<T,N,N> &d, matrix<T,N,N> &m)
  {
  T a;
  for (uint i=0; i < N; ++i )
    for (uint j=0; j < N; ++j)
      if (i == j)
        {
        d(i,j) = m(i,j);
        l(i,j) = 1.0;
        }
      else
        l(i,j) = m(i,j);
  }


