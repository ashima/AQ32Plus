/*
  October 2012

  aq32Plus Rev -

  Copyright (c) 2012 John Ihlein.  All rights reserved.

  Open Source STM32 Based Multicopter Controller Software

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)S.O.H. Madgwick
  6)UAVX

  Designed to run on the AQ32 Flight Control Board

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

///////////////////////////////////////////////////////////////////////////////

#include "board.h"
#include "evr.h"
#include "batMon.h"
#include "harness.h"
#include "state/char_telem.h"

///////////////////////////////////////////////////////////////////////////////

__attribute__((__section__(".eeprom"), used)) const int8_t eepromArray[16384];

eepromConfig_t eepromConfig;

uint8_t        execUpCount = 0;

sensors_t      sensors;

heading_t      heading;

uint16_t       timerValue;

///////////////////////////////////////////////////////////////////////////////
void pushInitTelem()
  {
  extern uint8_t overSamplingSetting;
  extern int ac1 , ac2 , ac3;
  extern unsigned int ac4 , ac5 , ac6 ;
  extern int b1 , b2 , mb , mc , md ;

  union 
    {
    struct  __attribute__((__packed__))
      {
      int16_t ac1;
      int16_t ac2;
      int16_t ac3;
      uint16_t ac4;
      uint16_t ac5;
      uint16_t ac6;
      int16_t b1;
      int16_t b2;
      int16_t mb;
      int16_t mc;
      int16_t md;
      int16_t oss;
      };
    uint8_t c_ptr[1];
    } bmp = {{ ac1, ac2, ac3, ac4, ac5, ac6, b1, b2, mb, mc, md, overSamplingSetting }} ;

  evrPush(EVR_StartingMain,0);
  ctPushSMTB(ctIDHSFState, 4*(4), (uint8_t*) hsf_getState() );
  ctPushSMTB(ctIDBMP180Params, sizeof(bmp), bmp.c_ptr );
  }

uint32_t uart2TxBuffInUse(void);

extern uint32_t rawPressure, rawTemperature ;

extern uint16_t i2c2ErrorCount;
extern uint32_t u2TxOverflow ;

typedef union {
  struct __attribute__((__packed__)) {
    uint16_t t;
    int16_t dt; 
    } ;
  uint8_t c_ptr[1];
  } rawT_t;

typedef union {
  struct __attribute__((__packed__)) {
    uint32_t p;
    int16_t dt; 
    } ;
  uint8_t c_ptr[1];
  } rawP_t;



int main(void)
{
    #ifdef ASHIMACORE
    // A8, D7, D11, and E12 are tied to ground, so prevent their use
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    GPIO_PinLockConfig(GPIOA, GPIO_Pin_8);
    GPIO_PinLockConfig(GPIOD, GPIO_Pin_7 | GPIO_Pin_11);
    GPIO_PinLockConfig(GPIOE, GPIO_Pin_12);

    ///////////////////////////////////////////////////////////////////////////
    #endif

	uint32_t currentTime;

    systemInit();

    systemReady = true;
    delay(200);
    hsf_init();
    pushInitTelem();

    hsf_step();
    hsf_update_t();
    ctPushSMTB(ctIDTemperature, sizeof(rawT_t), ((rawT_t){{rawTemperature, filter_dt}}).c_ptr);
    ctPushSMTB(ctIDHSFState, 4*(4), (uint8_t*) hsf_getState() );

    hsf_step();
    hsf_update_p();
    ctPushSMTB(ctIDPressure, sizeof(rawP_t), ((rawP_t){{ rawPressure, filter_dt }}.c_ptr) );
    ctPushSMTB(ctIDHSFState, 4*(4), (uint8_t*) hsf_getState() );

    delay(10);

    while (1)
    {
        evrCheck();
        if (frame_50Hz)
        {
        	frame_50Hz = false;

        	currentTime      = micros();
			deltaTime50Hz    = currentTime - previous50HzTime;
			previous50HzTime = currentTime;

			processFlightCommands();
#ifdef HAVEOSD
			if (eepromConfig.osdEnabled)
			{
				if (eepromConfig.osdDisplayAlt)
				    displayAltitude(sensors.pressureAlt10Hz, 0.0f, DISENGAGED);

				if (eepromConfig.osdDisplayAH)
				    displayArtificialHorizon(sensors.attitude500Hz[ROLL], sensors.attitude500Hz[PITCH], flightMode);

				if (eepromConfig.osdDisplayAtt)
				    displayAttitude(sensors.attitude500Hz[ROLL], sensors.attitude500Hz[PITCH], flightMode);

				if (eepromConfig.osdDisplayHdg)
				    displayHeading(heading.mag);
			}
#endif

			executionTime50Hz = micros() - currentTime;
            ctPushSMTB(ctIDMotorCommands, sizeof(float)*4, (uint8_t*)motor );
        }

        ///////////////////////////////

        if (frame_10Hz)
        {
        	frame_10Hz = false;

        	currentTime      = micros();
			deltaTime10Hz    = currentTime - previous10HzTime;
			previous10HzTime = currentTime;

			if (newMagData == true)
			{
				sensors.mag10Hz[XAXIS] = eepromConfig.signMX *   (float)rawMag[XAXIS].value * magScaleFactor[XAXIS] - eepromConfig.magBias[XAXIS];
			    sensors.mag10Hz[YAXIS] = eepromConfig.signMY *   (float)rawMag[YAXIS].value * magScaleFactor[YAXIS] - eepromConfig.magBias[YAXIS];
			    sensors.mag10Hz[ZAXIS] = eepromConfig.signMZ * -((float)rawMag[ZAXIS].value * magScaleFactor[ZAXIS] - eepromConfig.magBias[ZAXIS]);

			    newMagData = false;
			    //magDataUpdate = true;
			}

        	d1Average = d1Sum / 10;
        	d1Sum = 0;
        	calculateTemperature();
        	calculatePressureAltitude();
                //hsf_step_tp();

                //float *st = hsf_getState();
                //cliPrintF( "%d %d %f %f %f %f\n", rawTemperature, 
                //      rawPressure, st[0], st[1], st[2], st[3] );

        	pressureAltValid = true;
#if !defined(NOGPS) && !defined(STVGPS)
        	switch (eepromConfig.gpsType)
			{
			    ///////////////////////

			    case NO_GPS:                // No GPS installed
			        break;

			    ///////////////////////

			    case MEDIATEK_3329_BINARY:  // MediaTek 3329 in binary mode
			    	decodeMediaTek3329BinaryMsg();
			    	break;

				///////////////////////

				case MEDIATEK_3329_NMEA:    // MediaTek 3329 in NMEA mode
				    decodeNMEAsentence();
	        	    break;

			    ///////////////////////

			    case UBLOX:                 // UBLOX in binary mode
			    	decodeUbloxMsg();
			    	break;

			    ///////////////////////
			}
#endif
        	cliCom();

        	rfCom();

            batMonTick();

            executionTime10Hz = micros() - currentTime;
              uint32_t tCnt = uart2TxBuffInUse() | ( u2TxOverflow << 16);
              ctPushSMTB( ctIDTelemTxBuffInUse,4, (uint8_t*)&tCnt );
        }

        ///////////////////////////////

        if (frame_500Hz)
        {
			frame_500Hz = false;

       	    currentTime       = micros();
       	    deltaTime500Hz    = currentTime - previous500HzTime;
       	    previous500HzTime = currentTime;

       	    TIM_Cmd(TIM10, DISABLE);
       	 	timerValue = TIM_GetCounter(TIM10);
       	 	TIM_SetCounter(TIM10, 0);
       	 	TIM_Cmd(TIM10, ENABLE);

       	 	dt500Hz = (float)timerValue * 0.0000005f;  // For integrations in 500 Hz loop

            computeMPU6000TCBias();
            /*
            sensorTemp1 = computeMPU6000SensorTemp();
            sensorTemp2 = sensorTemp1 * sensorTemp1;
            sensorTemp3 = sensorTemp2 * sensorTemp1;
            */
            sensors.accel500Hz[XAXIS] = eepromConfig.signAX *  ((float)accelSummedSamples500Hz[XAXIS] / 2.0f - accelTCBias[XAXIS]) * ACCEL_SCALE_FACTOR;
			sensors.accel500Hz[YAXIS] = eepromConfig.signAY * -((float)accelSummedSamples500Hz[YAXIS] / 2.0f - accelTCBias[YAXIS]) * ACCEL_SCALE_FACTOR;
			sensors.accel500Hz[ZAXIS] = eepromConfig.signAZ * -((float)accelSummedSamples500Hz[ZAXIS] / 2.0f - accelTCBias[ZAXIS]) * ACCEL_SCALE_FACTOR;
            /*
            sensors.accel500Hz[XAXIS] =  ((float)accelSummedSamples500Hz[XAXIS] / 2.0f  +
                                          eepromConfig.accelBiasP0[XAXIS]               +
                                          eepromConfig.accelBiasP1[XAXIS] * sensorTemp1 +
                                          eepromConfig.accelBiasP2[XAXIS] * sensorTemp2 +
                                          eepromConfig.accelBiasP3[XAXIS] * sensorTemp3 ) * ACCEL_SCALE_FACTOR;

			sensors.accel500Hz[YAXIS] = -((float)accelSummedSamples500Hz[YAXIS] / 2.0f  +
			                              eepromConfig.accelBiasP0[YAXIS]               +
			                              eepromConfig.accelBiasP1[YAXIS] * sensorTemp1 +
			                              eepromConfig.accelBiasP2[YAXIS] * sensorTemp2 +
			                              eepromConfig.accelBiasP3[YAXIS] * sensorTemp3 ) * ACCEL_SCALE_FACTOR;

			sensors.accel500Hz[ZAXIS] = -((float)accelSummedSamples500Hz[ZAXIS] / 2.0f  +
			                              eepromConfig.accelBiasP0[ZAXIS]               +
			                              eepromConfig.accelBiasP1[ZAXIS] * sensorTemp1 +
			                              eepromConfig.accelBiasP2[ZAXIS] * sensorTemp2 +
			                              eepromConfig.accelBiasP3[ZAXIS] * sensorTemp3 ) * ACCEL_SCALE_FACTOR;
            */
            sensors.gyro500Hz[ROLL ] = eepromConfig.signGX *  ((float)gyroSummedSamples500Hz[ROLL]  / 2.0f - gyroRTBias[ROLL ] - gyroTCBias[ROLL ]) * GYRO_SCALE_FACTOR;
			sensors.gyro500Hz[PITCH] = eepromConfig.signGY * -((float)gyroSummedSamples500Hz[PITCH] / 2.0f - gyroRTBias[PITCH] - gyroTCBias[PITCH]) * GYRO_SCALE_FACTOR;
            sensors.gyro500Hz[YAW  ] = eepromConfig.signGZ * -((float)gyroSummedSamples500Hz[YAW]   / 2.0f - gyroRTBias[YAW  ] - gyroTCBias[YAW  ]) * GYRO_SCALE_FACTOR;
            /*
            sensors.gyro500Hz[ROLL ] =  ((float)gyroSummedSamples500Hz[ROLL ] / 2.0f  +
                                         gyroBiasP0[ROLL ]                            +
                                         eepromConfig.gyroBiasP1[ROLL ] * sensorTemp1 +
                                         eepromConfig.gyroBiasP2[ROLL ] * sensorTemp2 +
                                         eepromConfig.gyroBiasP3[ROLL ] * sensorTemp3 ) * GYRO_SCALE_FACTOR;

			sensors.gyro500Hz[PITCH] = -((float)gyroSummedSamples500Hz[PITCH] / 2.0f  +
			                             gyroBiasP0[PITCH]                            +
			                             eepromConfig.gyroBiasP1[PITCH] * sensorTemp1 +
			                             eepromConfig.gyroBiasP2[PITCH] * sensorTemp2 +
			                             eepromConfig.gyroBiasP3[PITCH] * sensorTemp3 ) * GYRO_SCALE_FACTOR;

            sensors.gyro500Hz[YAW  ] = -((float)gyroSummedSamples500Hz[YAW]   / 2.0f  +
                                         gyroBiasP0[YAW  ]                            +
                                         eepromConfig.gyroBiasP1[YAW  ] * sensorTemp1 +
                                         eepromConfig.gyroBiasP2[YAW  ] * sensorTemp2 +
                                         eepromConfig.gyroBiasP3[YAW  ] * sensorTemp3 ) * GYRO_SCALE_FACTOR;
            */
            MargAHRSupdate( sensors.gyro500Hz[ROLL],   sensors.gyro500Hz[PITCH],  sensors.gyro500Hz[YAW],
                            sensors.accel500Hz[XAXIS], sensors.accel500Hz[YAXIS], sensors.accel500Hz[ZAXIS],
                            sensors.mag10Hz[XAXIS],    sensors.mag10Hz[YAXIS],    sensors.mag10Hz[ZAXIS],
                            eepromConfig.accelCutoff,
                            magDataUpdate,
                            dt500Hz );

            magDataUpdate = false;

            computeAxisCommands(dt500Hz);
            mixTable();
            writeServos();
            writeMotors();

       	    executionTime500Hz = micros() - currentTime;
		}

        ///////////////////////////////

        if (frame_100Hz)
        {
        	frame_100Hz = false;

            if ( 0 ==  frameCounter % COUNT_10HZ ) {
              ctPushSMTB(ctIDPressure, 3, (uint8_t*)&rawPressure);
              hsf_step();
              hsf_update_p();
              }
            else if ( 10 == frameCounter % COUNT_10HZ ) {
              hsf_step();
              hsf_update_t(); 
              ctPushSMTB(ctIDTemperature, sizeof(rawT_t), ((rawT_t){{rawTemperature, filter_dt}}).c_ptr);
RED_LED_TOGGLE;
              }
            else {
              hsf_step();
              hsf_update_p();
              ctPushSMTB(ctIDPressure, sizeof(rawP_t), ((rawP_t){{ rawPressure, filter_dt }}.c_ptr) );
              }

            ctPushSMTB(ctIDHSFState, sizeof(float)*(4), (uint8_t*) hsf_getState() );
        	currentTime       = micros();
			deltaTime100Hz    = currentTime - previous100HzTime;
			previous100HzTime = currentTime;

			TIM_Cmd(TIM11, DISABLE);
			timerValue = TIM_GetCounter(TIM11);
			TIM_SetCounter(TIM11, 0);
			TIM_Cmd(TIM11, ENABLE);

			dt100Hz = (float)timerValue * 0.0000005f;  // For integrations in 100 Hz loop

			sensors.accel100Hz[XAXIS] = eepromConfig.signAX *  ((float)accelSummedSamples100Hz[XAXIS] / 10.0f - accelTCBias[XAXIS]) * ACCEL_SCALE_FACTOR;
			sensors.accel100Hz[YAXIS] = eepromConfig.signAY * -((float)accelSummedSamples100Hz[YAXIS] / 10.0f - accelTCBias[YAXIS]) * ACCEL_SCALE_FACTOR;
			sensors.accel100Hz[ZAXIS] = eepromConfig.signAZ * -((float)accelSummedSamples100Hz[ZAXIS] / 10.0f - accelTCBias[ZAXIS]) * ACCEL_SCALE_FACTOR;

        	createRotationMatrix();
        	bodyAccelToEarthAccel();
        	vertCompFilter(dt100Hz);

            ctPushSMTB(ctIDWAcc100, sizeof(float)*3, (uint8_t*) &earthAxisAccels );
            //ctPushSMTB(ctIDComHeight, sizeof(float), (uint8_t*) &hEstimate );

        	if ( highSpeedTelem1Enabled == true )
            {
            	// 500 Hz Accels
            	telemetryPrintF("%9.4f, %9.4f, %9.4f\n", sensors.accel500Hz[XAXIS],
            	        			                     sensors.accel500Hz[YAXIS],
            	        			                     sensors.accel500Hz[ZAXIS]);
            }

            if ( highSpeedTelem2Enabled == true )
            {
            	// 500 Hz Gyros
            	telemetryPrintF("%9.4f, %9.4f, %9.4f\n", sensors.gyro500Hz[ROLL ],
            	        			                     sensors.gyro500Hz[PITCH],
            	        					             sensors.gyro500Hz[YAW  ]);
            }

            if ( highSpeedTelem3Enabled == true )
            {
            	// Roll Rate, Roll Rate Command
            	telemetryPrintF("%9.4f, %9.4f\n", sensors.gyro500Hz[ROLL],
            			                          rxCommand[ROLL]);
            }

            if ( highSpeedTelem4Enabled == true )
            {
            	// Pitch Rate, Pitch Rate Command
            	telemetryPrintF("%9.4f, %9.4f\n", sensors.gyro500Hz[PITCH],
            	            			          rxCommand[PITCH]);
            }

            if ( highSpeedTelem5Enabled == true )
            {
            	// Yaw Rate, Yaw Rate Command
            	telemetryPrintF("%9.4f, %9.4f\n", sensors.gyro500Hz[YAW],
            	            	                  rxCommand[YAW]);
            }

            if ( highSpeedTelem7Enabled == true )
            {
               	// Vertical Variables
                float *st = hsf_getState();
           /* 	telemetryPrintF("%9.4f, %9.4f, %9.4f, %9.4f\n", earthAxisAccels[ZAXIS],
            			                                        sensors.pressureAlt10Hz,
            			                                        hDotEstimate,
            			                                        hEstimate); */
            	telemetryPrintF("%f %f %f %f\n", 
                    st[0],st[1],st[2],st[3] );
            }

            executionTime100Hz = micros() - currentTime;
        }

        if (frame_50Hz)
        {
            if ( highSpeedTelem6Enabled == true )
            {
            float *st = hsf_getState();
                ;uint32_t mot_avg = (motor[0] + motor[1] + motor[2] + motor[3]) / 4;
                telemetryPrintF("%1.4f %1.4f %1.4f %1.4f  %1.0f %1.0f %1.0f %1.0f  %1d  %1.0f %1.0f %1.0f %1.0f  %1.2f %1.2f %1.2f %f %f %f %f\n",
                    q0, q1, q2, q3, 
                    rxCommand[ROLL], rxCommand[PITCH], rxCommand[YAW], rxCommand[THROTTLE], 
                    mot_avg,
                    motor[0]-mot_avg, motor[1]-mot_avg, motor[2]-mot_avg, motor[3]-mot_avg,
                    eepromConfig.PID[ROLL_RATE_PID].iTerm, eepromConfig.PID[PITCH_RATE_PID].iTerm, eepromConfig.PID[YAW_RATE_PID].iTerm,
                    st[0], st[1],st[2],st[3] );
            }
        }

        ///////////////////////////////

        if (frame_5Hz)
        {
        	frame_5Hz = false;

        	currentTime     = micros();
			deltaTime5Hz    = currentTime - previous5HzTime;
			previous5HzTime = currentTime;

			if (execUp == true)
			    BLUE_LED_TOGGLE;

        	executionTime5Hz = micros() - currentTime;
        }

        ///////////////////////////////

        if (frame_1Hz)
        {
        	frame_1Hz = false;

        	currentTime     = micros();
			deltaTime1Hz    = currentTime - previous1HzTime;
			previous1HzTime = currentTime;

			if (execUp == true)
			    GREEN_LED_TOGGLE;

			if (execUp == false)
			    execUpCount++;

			if ((execUpCount == 5) && (execUp == false))
			    execUp = true;

			executionTime1Hz = micros() - currentTime;
        }

        ////////////////////////////////
#ifndef NOGPS
void skytraqStepState(uint8_t c);

        while ( gpsAvailable() )
          skytraqStepState( gpsRead() );
#endif
    }

    ///////////////////////////////////////////////////////////////////////////
}

///////////////////////////////////////////////////////////////////////////////
