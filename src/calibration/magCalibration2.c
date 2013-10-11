/**
  \file       magCalibration2.c
  \brief      Wrapper to collect data for calibrating the magnetometer.
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/AQ32Plus
  \remark     Ported for AQ32Plus.
*/
#include "board.h"
#include "magCal.h"
#include "shim.h"

enum { sampleDelay = 100 , conditionEveryN = 10 } ;

uint8_t magCalibrating = false;

void magCalibration(I2C_TypeDef *I2Cx)
  {
  double accOmega[ X4_N ];
  int i;

  cliPrint( "\n\nCalibrate magnetometer\n\n"
            "Rotate around all axeses as equily as possible, and try to get\n"
            "the Condition Number as small as possible (<10 would be good)\n"
            "Take a couple of minutes.\n" 
            "Press any key to start collecting data.\n" );

  magCalibrating = true;

  while ( false == cliAvailable() );
  cliRead();

  cliPrint("Collecting. Press any key to stop.\n");

  mcInit(accOmega);

  for (i = 0 ; false == cliAvailable(); ++i )
    {
    delay(sampleDelay);

    readMPU6000();

    mcAddPoint(accOmega, iRawMag.x, iRawMag.y, iRawMag.z);

    if (0 == i % conditionEveryN )
      cliPrintF("%d Condition=%f\n",i, (float)mcOTOConditionNumber(accOmega) );
    }

  cliRead();

  mcCompute(eepromConfig.magCalMat, accOmega);

  cliPrint( "\nMagnetometer calibration done.\n" );
  magCalibrating = false;
  }

