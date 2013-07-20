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

#include "matDumb.h"

struct bmp_params_t {
 int32_t ac1, ac2, ac3, ac4, ac5, ac6, b1, b2, mb, mc, md, oss ;
 } ;


template<typename T,int NS, int NO>
struct filter_t
  {
  enum nums_t { ns = NS, no = NO };

  matrix<T,NS,1> x ;
  matrix<T,NS,NS> P, F, dF;
  matrix<T,NS,NS> Q ;
  matrix<T,NO,NO> R  ;

  filter_t() : x(0.) , P(0.), F(0.), dF(0.), Q(0.), R(0.) {}
  };

typedef double float_tt ;
enum { ns = 4, no = 2};

void filter_step( filter_t<float_tt,ns,no> &f, uint32_t theta, uint32_t phi);
void filterInit(filter_t<float_tt,ns,no> &f , float_tt dt);
void set_params(bmp_params_t &x);




// H_FILTER_H
#endif

