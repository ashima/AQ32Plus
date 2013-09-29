/**
  \file       shim.c
  \brief      Shim between flight code and sensors.
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/AQ32Plus
  \remark     Ported for AQ32Plus.
  \remark     can i has c++ please.
  \remark     Vehicle to Board orientation transform hard coded for now. move it
              to an eeprom config *after* everything else checks out.
*/

#include "board.h"
#include "eMatAxisPerm.h"

// AQ's coordinate frame is defined as X+ forward, Y+ to the right, and Z+ Down.
// For out talon, the AC3 board is mounted upside down, so this lines up as an
// identity transorm. For the normal mount, the 180 degree rotation about X is
// just negating Y and Z.

//EMAT_DECLARE_PERM_EMAT( VehicleBoard, P_XYZ | P_PNN );
EMAT_DECLARE_PERM_EMAT( VehicleBoard, P_XYZ | P_PPP );

// This function assumes that the raw sensor values have just been read
// into global raw state variables by their appropriate functions.
void sreadAccel(floatXYZ_t *a)
  { // raw values and calibration offsets are in the board frame.
  float t = iRawAGTemp / 340.0f + 35.0f;

  float Tax = eepromConfig.accelTCBiasSlope[XAXIS] * t + eepromConfig.accelTCBiasIntercept[XAXIS];
  float Tay = eepromConfig.accelTCBiasSlope[YAXIS] * t + eepromConfig.accelTCBiasIntercept[YAXIS];
  float Taz = eepromConfig.accelTCBiasSlope[ZAXIS] * t + eepromConfig.accelTCBiasIntercept[ZAXIS];
  //Tax = Tay = Taz = 0. ;
  // Convert to vehicle frame. 
  EMAT_MUL_EMAT_VEC((*a), VehicleBoard, 
    ((floatXYZ_t){ ((float)iRawAcc.x - Tax) * ACCEL_SCALE_FACTOR ,
                   ((float)iRawAcc.y - Tay) * ACCEL_SCALE_FACTOR ,
                   ((float)iRawAcc.z - Taz) * ACCEL_SCALE_FACTOR } ) );

/*
  cliPrintF("T = %f\n", t);
  cliPrintF("m = [%f %f %f]\n", eepromConfig.accelTCBiasSlope[XAXIS], eepromConfig.accelTCBiasSlope[YAXIS], eepromConfig.accelTCBiasSlope[ZAXIS] );
  cliPrintF("c = [%f %f %f]\n", eepromConfig.accelTCBiasIntercept[XAXIS], eepromConfig.accelTCBiasIntercept[YAXIS], eepromConfig.accelTCBiasIntercept[ZAXIS] );
  cliPrintF("O = [%f %f %f]\n", Tax, Tay, Taz );
*/
  }

// This function assumes that the raw sensor values have just been read
// into global raw state variables by their appropriate functions.
void sreadGyro(floatXYZ_t *g)
  { // raw values and calibration offsets are in the board frame.
  float t = iRawAGTemp / 340.0f + 35.0f;

  float Tgx = eepromConfig.gyroTCBiasSlope[ROLL ]  * t + eepromConfig.gyroTCBiasIntercept[ROLL ];
  float Tgy = eepromConfig.gyroTCBiasSlope[PITCH]  * t + eepromConfig.gyroTCBiasIntercept[PITCH];
  float Tgz = eepromConfig.gyroTCBiasSlope[YAW  ]  * t + eepromConfig.gyroTCBiasIntercept[YAW  ];

  // Convert to vehicle frame. 
  EMAT_MUL_EMAT_VEC((*g), VehicleBoard, ((floatXYZ_t){ 
     ((float)iRawGyro.x - Tgx - gyroRTBias[XAXIS]) * GYRO_SCALE_FACTOR,
     ((float)iRawGyro.y - Tgy - gyroRTBias[YAXIS]) * GYRO_SCALE_FACTOR,
     ((float)iRawGyro.z - Tgz - gyroRTBias[ZAXIS]) * GYRO_SCALE_FACTOR }) );
  
  }

// This function assumes that the raw sensor values have just been read
// into global raw state variables by their appropriate functions.
void sreadMag(floatXYZ_t *m)
  { // raw values and calibration offsets are in the board frame.
  // Convert to vehicle frame. 
  EMAT_MUL_EMAT_VEC((*m),VehicleBoard, ((floatXYZ_t){
        (float)iRawMag.x * magScaleFactor[XAXIS] - eepromConfig.magBias[XAXIS],
        (float)iRawMag.y * magScaleFactor[YAXIS] - eepromConfig.magBias[YAXIS],
        (float)iRawMag.z * magScaleFactor[ZAXIS] - eepromConfig.magBias[ZAXIS] } ) );

  }

void computeAccelOneG()
  { // one-g measurement might not cancle with calibration for a given sensor.
  enum { 
    N = 1024,
    DELAY = (int)(1e6 / 1000), // 1Khz
    };

  floatXYZ_t s;
  floatXYZ_t g;
  float a = 0.;
  int i;
  g.x = g.y = g.z = 0.;

  mpu6000Calibrating = true;
  for (i=0; i < N ; ++i)
    {
    delayMicroseconds(DELAY);
    readMPU6000();
    sreadAccel(&s);
    a += sqrt( s.x * s.x + s.y * s.y + s.z * s.z );
    sreadGyro(&s);
    g.x += s.x;
    g.y += s.y;
    g.z += s.z;
    }

  delayMicroseconds(DELAY);
  mpu6000Calibrating = false;
  gyroRTBias[XAXIS]  = g.x / N; // roll
  gyroRTBias[YAXIS]  = g.y / N;  // pitch
  gyroRTBias[ZAXIS]  = g.z / N; // yaw

  accelOneG = a/ N;
  }

