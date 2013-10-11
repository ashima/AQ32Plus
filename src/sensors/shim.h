/**
  \file       shim.h
  \brief      Shim between flight code and sensors.
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/AQ32Plus
  \remark     Ported for AQ32Plus.
*/

void sreadAccel(floatXYZ_t *);
void sreadGyro(floatXYZ_t *);
void sreadMag(floatXYZ_t *);
void computeAccelOneG();

