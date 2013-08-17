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

typedef filter_t<float_tt,ns,no> F;

float_tt filter[ sizeof(F)/sizeof(float_tt) ] ;
uint32_t lt = 0;
int32_t filter_dt;

void hsf_init()
  {
  float dt = 0.001f;
  filterSetParams();
  filterInit(*(F*)filter,dt);
  }

extern uint32_t rawPressure;
extern uint32_t rawTemperature;

void hsf_step_tp() { filter_step_tp(*(F*)filter, rawTemperature, rawPressure); }
void hsf_step_t()  { filter_step_t (*(F*)filter, rawTemperature); }
void hsf_step_p()  { filter_step_p (*(F*)filter, rawPressure); }

float *hsf_getState()
  {
  return &( (*(F*)(filter)).x(0,0));
  }

