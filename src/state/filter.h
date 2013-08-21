/**
  \file       filter.h
  \brief      Bispoke Kalman filter for height state from a BMP180
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/AQ32Plus
  \remark     Ported for AQ32Plus.
*/

#ifndef H_FILTER_H
#define H_FILTER_H
#pragma once
#include <inttypes.h>
#include "matDumb.h"

template<typename T>
struct state_t
  {
  float pTemp;
  float pTempDot;
  matrix<T,4,3> x;
  state_t() : pTemp(0.0f), pTempDot(0.0f), x(0.0f) {};
  };

enum { e_x = 0 , e_y = 1, e_z = 2 };

template<typename T,unsigned int NS,unsigned int NO>
struct filter_t
  {
  enum nums_t { ns = NS, no = NO };

  //matrix<T,NS,1UL> x ;
  state_t<T> S;
  matrix<T,NS,NS> P, F, dF, Q ;
  //matrix<T,NO,NO> R ;
  matrix<T,1UL,1UL> Rts ;
  matrix<T,1UL,1UL> Rps ;
  matrix<T,1UL,1UL> Ras ;

  filter_t() : S(), P(0.0f), F(0.0f), dF(0.0f), Q(0.0f), // R(0.0f), 
    Rts(0.0f), Rps(0.0f), Ras(0.0f) {}
  };

typedef float float_tt ;
enum { ns = 6UL, no = 3UL};

//void filterStep( filter_t<float_tt,ns,no> &f, uint32_t theta, uint32_t phi);
//void filter_step_tp( filter_t<float_tt,ns,no> &f, uint32_t theta, uint32_t phi) throw();
void filter_update_t ( filter_t<float_tt,ns,no> &f, uint32_t theta) throw();
void filter_update_p ( filter_t<float_tt,ns,no> &f, uint32_t phi) throw();
void filter_update_a ( filter_t<float_tt,ns,no> &f, float_tt* acc) throw();
void filter_step     ( filter_t<float_tt,ns,no> &f ) throw();

void filterInit(filter_t<float_tt,ns,no> &f, float_tt dt) throw();
void filterSetParams() throw() ;
 //bmp_params_t &x);

// H_FILTER_H
#endif

