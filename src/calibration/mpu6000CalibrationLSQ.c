/**
  \file       mpu6000CalibrationLSQ.c
  \brief      Least Squares fit calibration code for AQ32Plus.
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/AQ32Plus
  \remark     Ported for AQ32Plus.
*/

#include <inttypes.h>
#include "board.h"

enum { 
  N = 256*32,         // 32 seconds worth.
  DELAY = (int)(1e6 / 256),  // 256 Hz
  };

typedef struct {
  int32_t x,y,z;
  } xyz_t;

extern xyz_t iRawAcc, iRawGyro, iRawMag ;
extern int32_t iRawAGTemp;

static inline float sqr(float x) { return x*x; }

void mpu6000CalibrationLSQ(void)
  {
  xyz_t refAcc;
  int32_t refTemp;
  int i;

  float _n = 1.0 / (float)N;
  float r,temp;

  float t, tt, Stt;
  float ax, ay, az, axx, ayy, azz, atx, aty, atz;
  float gx, gy, gz, gxx, gyy, gzz, gtx, gty, gtz;
  float Saxx, Sayy, Sazz, Satx, Saty, Satz ;
  float Sgxx, Sgyy, Sgzz, Sgtx, Sgty, Sgtz ;
  float Tax, Tay, Taz,  Tgx, Tgy, Tgz;
  float Oax, Oay, Oaz,  Ogx, Ogy, Ogz;
  float Cax, Cay, Caz,  Cgx, Cgy, Cgz;

  t = tt =
  ax = ay = az = axx = ayy = azz = atx = aty = atz =
  gx = gy = gz = gxx = gyy = gzz = gtx = gty = gtz = 0.0;

  mpu6000Calibrating = true;

  cliPrint("\nStarting MPU9150 Calibration\n");

  delayMicroseconds(DELAY);

  // Grab a measurement to use as reference. The *only* purpose of the reference
  // is to make the numbers in use closer to zero to improve floating point
  // accuracy.
  readMPU6000();
  refAcc  = iRawAcc;
  refTemp = iRawAGTemp;
  temp    = (float)iRawAGTemp / 340.0 + 35 ;

  cliPrintF("RAW:  [%d %d %d], [%d %d %d] %d(%f)\n", 
       iRawAcc.x, iRawAcc.y, iRawAcc.z,
       iRawGyro.x, iRawGyro.y, iRawGyro.z,
       iRawAGTemp, temp );
  cliPrintF("RAW:  [%d %d %d], [%d %d %d] %d(%f)\n", 
       rawAccel[XAXIS],  rawAccel[YAXIS],  rawAccel[ZAXIS],
       rawGyro[XAXIS],  rawGyro[YAXIS],  rawGyro[ZAXIS],
       rawMPU6000Temperature.value, (float)rawMPU6000Temperature.value/340.0 + 35 );

  for (i=0; i < N ; i++)
    {
    delayMicroseconds(DELAY);

    // returns values in iRawAcc, iRawGyro, iRawMag, and iRawAGTemp;
    readMPU6000() ;
    iRawAcc.x -= refAcc.x;
    iRawAcc.y -= refAcc.y;
    iRawAcc.z -= refAcc.z;
    temp = (float)iRawAGTemp / 340.0 + 35 ;

    //iRawAGTemp  -= refTemp;
    //t += iRawAGTemp; tt += sqr(iRawAGTemp);
    t += temp; tt += sqr(temp);

    ax += iRawAcc.x;     axx += sqr(iRawAcc.x);  atx+= iRawAcc.x  * temp;
    ay += iRawAcc.y;     ayy += sqr(iRawAcc.y);  aty+= iRawAcc.y  * temp;
    az += iRawAcc.z;     azz += sqr(iRawAcc.z);  atz+= iRawAcc.z  * temp;

    gx += iRawGyro.x;    gxx += sqr(iRawGyro.x); gtx+= iRawGyro.x * temp;
    gy += iRawGyro.y;    gyy += sqr(iRawGyro.y); gty+= iRawGyro.y * temp;
    gz += iRawGyro.z;    gzz += sqr(iRawGyro.z); gtz+= iRawGyro.z * temp;
    }

  // Variances
  Stt  = tt  - t*t*_n;
  Saxx = axx - ax*ax*_n; Sayy = ayy - ay*ay*_n;  Sazz = azz - az*az*_n;
  Sgxx = gxx - gx*gx*_n; Sgyy = gyy - gy*gy*_n;  Sgzz = gzz - gz*gz*_n;

  // Covariances
  t *= _n ; // t becomes mean t.

  Satx = atx - t * ax;   Saty = aty - t * ay;    Satz = atz - t * az;
  Sgtx = gtx - t * gx;   Sgty = gty - t * gy;    Sgtz = gtz - t * gz;

  // Temp gradients
  Tax = Satx / Stt;      Tay = Saty / Stt;       Taz = Satz / Stt;
  Tgx = Sgtx / Stt;      Tgy = Sgty / Stt;       Tgz = Sgtz / Stt;

  // Offsets
  //t += refTemp;

  Oax = ax*_n - Tax * t + refAcc.x;
  Oay = ay*_n - Tay * t + refAcc.y;
  Oaz = az*_n - Taz * t + refAcc.z;

  Ogx = gx*_n - Tgx * t;
  Ogy = gy*_n - Tgy * t;
  Ogz = gz*_n - Tgz * t;

  // The acc offset should now contain both the 1g gravity measurement and
  // the bias. Assume the direction is totaly dominated by the 1g part and
  // subtract off 1g. (1g == 8192 for a +/- 4g range )
  r = 1.0 - 8192.0 / sqrt( sqr(Oax) + sqr(Oay) + sqr(Oaz) );
  Oax *= r;
  Oay *= r;
  Oaz *= r;

  // cor coeff.
  Cax = sqr( Satx ) / (Stt * Saxx);
  Cay = sqr( Saty ) / (Stt * Sayy);
  Caz = sqr( Satz ) / (Stt * Sazz);

  Cgx = sqr( Sgtx ) / (Stt * Sgxx);
  Cgy = sqr( Sgty ) / (Stt * Sgyy);
  Cgz = sqr( Sgtz ) / (Stt * Sgzz);

  cliPrintF("ACC:  [%f %f %f]*T + [%f %f %f] (%f %f %f)\n", 
    Tax, Tay, Taz, Oax, Oay, Oaz, Cax, Cay, Caz );
  cliPrintF("GYRO: [%f %f %f]*T + [%f %f %f] (%f %f %f)\n",
    Tgx, Tgy, Tgz, Ogx, Ogy, Ogz, Cgx, Cgy, Cgz );

  cliPrint("\nFinnished MPU9150 Calibration\n");
  mpu6000Calibrating = false;

  eepromConfig.accelTCBiasSlope[XAXIS]     = Tax ;
  eepromConfig.accelTCBiasSlope[YAXIS]     = Tay ;
  eepromConfig.accelTCBiasSlope[ZAXIS]     = Taz ;

  eepromConfig.accelTCBiasIntercept[XAXIS] = Oax ; 
  eepromConfig.accelTCBiasIntercept[YAXIS] = Oay ;
  eepromConfig.accelTCBiasIntercept[ZAXIS] = Oaz ;

  eepromConfig.gyroTCBiasSlope[ROLL ]      = Tgx ;
  eepromConfig.gyroTCBiasSlope[PITCH]      = Tgy ;
  eepromConfig.gyroTCBiasSlope[YAW  ]      = Tgz ;

  eepromConfig.gyroTCBiasIntercept[ROLL ]  = Ogx ;
  eepromConfig.gyroTCBiasIntercept[PITCH]  = Ogy ;
  eepromConfig.gyroTCBiasIntercept[YAW  ]  = Ogz ;

  cliPrint("\nMPU9150 Calibration Complete.\n\n");

  mpu6000Calibrating = false;
  }

