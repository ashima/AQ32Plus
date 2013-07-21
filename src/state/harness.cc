/**
  \file       harness.c
  \brief      Bispoke Kalman filter for height state from a BMP180
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/AQ32Plus
  \remark     Ported for AQ32Plus.
*/

#include "harness.h"

#include "filter.h"

filter_t<float_tt,ns,no> filter;

void hsf_init(float dt)
  {
  filterSetParams();
  filterInit(filter, dt);
  }

extern long rawPressure;
extern long rawTemperature;

void hsf_step()
  {
  filterStep(filter,rawTemperature,rawPressure);
  }

float *hsf_getState()
  {
  return &(filter.x(0,0));
  }

