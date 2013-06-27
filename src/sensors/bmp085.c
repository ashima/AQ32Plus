/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.
 
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

#ifndef _AQ_BAROMETRIC_SENSOR_BMP085_
#define _AQ_BAROMETRIC_SENSOR_BMP085_

#include "bmp085.h"
#include "board.h"

// This sets up the BMP085 from Sparkfun
// Code from http://wiring.org.co/learning/libraries/bmp085.html
// Also made bug fixes based on BMP085 library from Jordi Munoz and Jose Julio

#define BMP085_I2C_ADDRESS 0x77

#define TEMPERATURE 0
#define PRESSURE 1
#define OVER_SAMPLING_SETTING 1 // use to be 3

uint8_t overSamplingSetting = OVER_SAMPLING_SETTING;
int ac1 = 0, ac2 = 0, ac3 = 0;
unsigned int ac4 = 0, ac5 = 0, ac6 = 0;
int b1 = 0, b2 = 0, mb = 0, mc = 0, md = 0;
long pressure = 0;
long rawPressure = 0, rawTemperature = 0;
uint8_t pressureCount = 0;
float pressureFactor = 1/5.255;
int isReadPressure = false;
// float rawPressure = 0;

uint32_t d1Average;
uint32_t d1Sum;
uint32andUint8_t d1;
uint8_t pressureAltValid;


uint8_t readByteI2C(I2C_TypeDef *I2Cx, uint8_t address)
{
  uint8_t data[1];

  i2cRead(I2Cx, BMP085_I2C_ADDRESS, address, 2, data);    // Request temperature read

  return data[0];
}

uint16_t readWordI2C(I2C_TypeDef *I2Cx, uint8_t address)
{
  uint8_t data[2];

  i2cRead(I2Cx, BMP085_I2C_ADDRESS, address, 2, data);    // Request temperature read

  return (data[0] << 8) | (data[1] << 0);
}

int16_t readShortI2C(I2C_TypeDef *I2Cx, uint8_t address)
{
  return (int16_t)readWordI2C(I2Cx, address);
}

void requestRawPressure(I2C_TypeDef *I2Cx) {
  i2cWrite(I2Cx, BMP085_I2C_ADDRESS, 0xF4, 0x34+(overSamplingSetting<<6));
}
  
uint32_t readRawPressure(I2C_TypeDef *I2Cx) {
  uint8_t data[3];

  i2cRead(I2Cx, BMP085_I2C_ADDRESS, 0xF6, 3, data);    // Request temperature read

  return (((unsigned long)data[0] << 16) | ((unsigned long)data[1] << 8) | ((unsigned long)data[2])) >> (8-overSamplingSetting);
}

void requestRawTemperature(I2C_TypeDef *I2Cx) {
  i2cWrite(I2Cx, BMP085_I2C_ADDRESS, 0xF4, 0x2E);
}
  
unsigned int readRawTemperature(I2C_TypeDef *I2Cx) {
  uint8_t data[2];

  i2cRead(I2Cx, BMP085_I2C_ADDRESS, 0xF6, 2, data);    // Request temperature read

  return ((unsigned long)data[0] << 8) | ((unsigned long)data[1] << 0);
}

// ***********************************************************
// Define all the virtual functions declared in the main class
// ***********************************************************
void initPressure(I2C_TypeDef *I2Cx)
{
  pressureAltValid = false;
  // oversampling setting
  // 0 = ultra low power
  // 1 = standard
  // 2 = high
  // 3 = ultra high resolution
  overSamplingSetting = OVER_SAMPLING_SETTING;
  pressure = 0;
  // baroGroundAltitude = 0;
  pressureFactor = 1/5.255;
    
  if (readByteI2C(I2Cx, BMP085_I2C_ADDRESS) == 0x55) {
	  //vehicleState |= BARO_DETECTED;
  }
  
  //i2cWrite(I2Cx, BMP085_I2C_ADDRESS, );
  //Wire.requestFrom(BMP085_I2C_ADDRESS, 22);
  uint8_t addr = 0xAA/2 - 1;
  ac1 = readShortI2C(I2Cx, ++addr * 2);
  ac2 = readShortI2C(I2Cx, ++addr * 2);
  ac3 = readShortI2C(I2Cx, ++addr * 2);
  ac4 = readWordI2C(I2Cx, ++addr * 2);
  ac5 = readWordI2C(I2Cx, ++addr * 2);
  ac6 = readWordI2C(I2Cx, ++addr * 2);
  b1 = readShortI2C(I2Cx, ++addr * 2);
  b2 = readShortI2C(I2Cx, ++addr * 2);
  mb = readShortI2C(I2Cx, ++addr * 2);
  mc = readShortI2C(I2Cx, ++addr * 2);
  md = readShortI2C(I2Cx, ++addr * 2);

  cliPrintF("BMP085 config values: %d %d %d %d %d %d %d %d %d %d %d %d\n", 
    ac1, ac2, ac3, ac4, ac5, ac6,
    b1, b2,
    mb, mc, md,
    overSamplingSetting);
  // requestRawTemperature(); // setup up next measure() for temperature
  // measureBaro();
  // delay(5); // delay for temperature
  // measureBaro();
  // delay(10); // delay for pressure
  // measureGroundBaro();
  // // check if measured ground altitude is valid
  // while (abs(baroRawAltitude - baroGroundAltitude) > 10) {
  //   delay(26);
  //   measureGroundBaro();
  // }
  // baroAltitude = baroGroundAltitude;
}
void readPressureRequestPressure(I2C_TypeDef *I2Cx)
{
  rawPressure = readRawPressure(I2Cx);

  d1.value = rawPressure; // WARN: not sure this is correct!
  requestRawPressure(I2Cx);  
}
void readPressureRequestTemperature(I2C_TypeDef *I2Cx)
{
  rawPressure = readRawPressure(I2Cx);

  d1.value = rawPressure; // WARN: not sure this is correct!
  requestRawTemperature(I2Cx);
}
void readTemperatureRequestPressure(I2C_TypeDef *I2Cx)
{
  rawTemperature = readRawTemperature(I2Cx);
  requestRawPressure(I2Cx);
}

void calculateTemperature() {};

void calculatePressureAltitude() {
  long x1, x2, x3, b3, b5, b6, p;
  unsigned long b4, b7;
  int32_t tmp;

  //calculate true temperature
  x1 = ((long)rawTemperature - ac6) * ac5 >> 15;
  x2 = ((long) mc << 11) / (x1 + md);
  b5 = x1 + x2;
  long temperature = ((b5 + 8) >> 4);

  // if (rawPressureSumCount == 0) { // it may occur at init time that no pressure has been read yet!
  //   return;
  // }
  // rawPressure = rawPressureSum / rawPressureSumCount;
  // rawPressureSum = 0.0;
  // rawPressureSumCount = 0;
  
  //calculate true pressure
  b6 = b5 - 4000;
  x1 = (b2 * (b6 * b6 >> 12)) >> 11; 
  x2 = ac2 * b6 >> 11;
  x3 = x1 + x2;
 
  // Real Bosch formula - b3 = ((((int32_t)ac1 * 4 + x3) << overSamplingSetting) + 2) >> 2;
  // The version below is the same, but takes less program space
  tmp = ac1;
  tmp = (tmp * 4 + x3) << overSamplingSetting;
  b3 = (tmp + 2) >> 2;
 
  x1 = ac3 * b6 >> 13;
  x2 = (b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (ac4 * (uint32_t) (x3 + 32768)) >> 15;
  b7 = ((uint32_t) rawPressure - b3) * (50000 >> overSamplingSetting);
  p = b7 < 0x80000000 ? (b7 << 1) / b4 : (b7 / b4) << 1;
    
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  pressure = (p + ((x1 + x2 + 3791) >> 4));
  
  //baroRawAltitude = 44330 * (1 - pow(pressure/101325.0, pressureFactor)); // returns absolute baroAltitude in meters
  sensors.pressureAlt10Hz = 44330 * (1 - pow(pressure/101325.0, pressureFactor)); // returns absolute baroAltitude in meters
  cliPrintF("%d %d %1.3f %d\n", rawTemperature, rawPressure, sensors.pressureAlt10Hz, temperature);
  // use calculation below in case you need a smaller binary file for CPUs having just 32KB flash ROM
  // baroRawAltitude = (101325.0-pressure)/4096*346;
  // = baroRawAltitude;//filterSmooth(baroRawAltitude, baroAltitude, baroSmoothFactor);
}

#endif
