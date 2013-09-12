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

///////////////////////////////////////////////////////////////////////////////
// MPU6000 Defines and Variables
///////////////////////////////////////////////////////////////////////////////

// AK-8975 (built into the MPU-9150 as an I2C device)
#define AK8975_ADDRESS_00         0x0C

#define AK8975_RA_WIA             0x00
  #define AK8975_WHO_AM_I         0x48
#define AK8975_RA_HXL             0x03
#define AK8975_RA_HXH             0x04
#define AK8975_RA_HYL             0x05
#define AK8975_RA_HYH             0x06
#define AK8975_RA_HZL             0x07
#define AK8975_RA_HZH             0x08

#define AK8975_RA_CNTL            0x0A
  #define AK8975_MODE_SINGLE        0x1

// Registers

#define MPU9150_RA_SMPLRT_DIV       0x19
#define MPU9150_RA_CONFIG           0x1A
  #define MPU9150_CFG_DLPF_CFG_BIT    0
  #define MPU9150_CFG_DLPF_CFG_LENGTH 3
    #define MPU9150_DLPF_BW_256         0x00
    #define MPU9150_DLPF_BW_188         0x01
    #define MPU9150_DLPF_BW_98          0x02
    #define MPU9150_DLPF_BW_42          0x03
    #define MPU9150_DLPF_BW_20          0x04
    #define MPU9150_DLPF_BW_10          0x05
    #define MPU9150_DLPF_BW_5           0x06
#define MPU9150_RA_GYRO_CONFIG      0x1B
  #define MPU9150_GCONFIG_FS_SEL_BIT      3
  #define MPU9150_GCONFIG_FS_SEL_LENGTH   2
    #define MPU9150_GYRO_FS_250         0x00
    #define MPU9150_GYRO_FS_500         0x01
    #define MPU9150_GYRO_FS_1000        0x02
    #define MPU9150_GYRO_FS_2000        0x03
#define MPU9150_RA_ACCEL_CONFIG     0x1C
  #define MPU9150_ACONFIG_AFS_SEL_BIT         3
  #define MPU9150_ACONFIG_AFS_SEL_LENGTH      2
    #define MPU9150_ACCEL_FS_2          0x00
    #define MPU9150_ACCEL_FS_4          0x01
    #define MPU9150_ACCEL_FS_8          0x02
    #define MPU9150_ACCEL_FS_16         0x03

#define MPU9150_RA_I2C_MST_CTRL     0x24
#define MPU9150_RA_I2C_SLV0_ADDR    0x25
  #define MPU9150_I2C_SLV_RW_BIT      7
  #define MPU9150_I2C_SLV_ADDR_BIT    0
  #define MPU9150_I2C_SLV_ADDR_LENGTH 7
#define MPU9150_RA_I2C_SLV0_REG     0x26
#define MPU9150_RA_I2C_SLV0_CTRL    0x27
  #define MPU9150_I2C_SLV_EN_BIT      7
  #define MPU9150_I2C_SLV_BYTE_SW_BIT 6
  #define MPU9150_I2C_SLV_REG_DIS_BIT 5
  #define MPU9150_I2C_SLV_GRP_BIT     4
  #define MPU9150_I2C_SLV_LEN_BIT     0
  #define MPU9150_I2C_SLV_LEN_LENGTH  4
#define MPU9150_RA_I2C_SLV1_ADDR    0x28
#define MPU9150_RA_I2C_SLV1_REG     0x29
#define MPU9150_RA_I2C_SLV1_CTRL    0x2A
#define MPU9150_RA_I2C_MST_STATUS   0x36
#define MPU9150_RA_INT_PIN_CFG      0x37
  #define MPU9150_INTCFG_I2C_BYPASS_EN_BIT    1

#define MPU9150_RA_ACCEL_XOUT_H     0x3B
#define MPU9150_RA_ACCEL_XOUT_L     0x3C
#define MPU9150_RA_ACCEL_YOUT_H     0x3D
#define MPU9150_RA_ACCEL_YOUT_L     0x3E
#define MPU9150_RA_ACCEL_ZOUT_H     0x3F
#define MPU9150_RA_ACCEL_ZOUT_L     0x40
#define MPU9150_RA_TEMP_OUT_H       0x41
#define MPU9150_RA_TEMP_OUT_L       0x42
#define MPU9150_RA_GYRO_XOUT_H      0x43
#define MPU9150_RA_GYRO_XOUT_L      0x44
#define MPU9150_RA_GYRO_YOUT_H      0x45
#define MPU9150_RA_GYRO_YOUT_L      0x46
#define MPU9150_RA_GYRO_ZOUT_H      0x47
#define MPU9150_RA_GYRO_ZOUT_L      0x48
#define MPU9150_RA_EXT_SENS_DATA_00 0x49
#define MPU9150_RA_EXT_SENS_DATA_01 0x4A
#define MPU9150_RA_EXT_SENS_DATA_02 0x4B
#define MPU9150_RA_EXT_SENS_DATA_03 0x4C
#define MPU9150_RA_EXT_SENS_DATA_04 0x4D
#define MPU9150_RA_EXT_SENS_DATA_05 0x4E

#define MPU9150_RA_I2C_SLV0_DO      0x63
#define MPU9150_RA_I2C_SLV1_DO      0x64

#define MPU9150_RA_I2C_MST_DELAY_CTRL   0x67

#define MPU9150_RA_USER_CTRL        0x6A
  #define MPU9150_USERCTRL_I2C_MST_EN_BIT         5
#define MPU9150_RA_PWR_MGMT_1       0x6B
  #define MPU9150_PWR1_DEVICE_RESET_BIT   7
  #define MPU9150_PWR1_SLEEP_BIT          6
  #define MPU9150_PWR1_CLKSEL_BIT         0
  #define MPU9150_PWR1_CLKSEL_LENGTH      3
    #define MPU9150_CLOCK_INTERNAL          0x00
    #define MPU9150_CLOCK_PLL_XGYRO         0x01
    #define MPU9150_CLOCK_PLL_YGYRO         0x02
    #define MPU9150_CLOCK_PLL_ZGYRO         0x03
    #define MPU9150_CLOCK_PLL_EXT32K        0x04
    #define MPU9150_CLOCK_PLL_EXT19M        0x05
    #define MPU9150_CLOCK_KEEP_RESET        0x07
#define MPU9150_RA_PWR_MGMT_2       0x6C

#define MPU9150_RA_WHO_AM_I         0x75
  #define MPU9150_WHO_AM_I_BIT        1
  #define MPU9150_WHO_AM_I_LENGTH     6


#define MPU9150_ADDR (0x68 + 1)
#define I2Cx I2C2
///////////////////////////////////////

float accelOneG = 9.8065;

int32_t accelSum100Hz[3] = { 0, 0, 0 };

int32_t accelSum500Hz[3] = { 0, 0, 0 };

int32_t accelSummedSamples100Hz[3];

int32_t accelSummedSamples500Hz[3];

float accelTCBias[3] = { 0.0f, 0.0f, 0.0f };

int16andUint8_t rawAccel[3];

///////////////////////////////////////

float gyroRTBias[3];

int32_t gyroSum500Hz[3] = { 0, 0, 0 };

int32_t gyroSummedSamples500Hz[3];

float gyroTCBias[3];

int16andUint8_t rawGyro[3];

///////////////////////////////////////

uint8_t mpu6000Calibrating = false;

float   mpu6000Temperature;

int16andUint8_t rawMPU6000Temperature;

///////////////////////////////////////////////////////////////////////////////
// MPU6000 Initialization
///////////////////////////////////////////////////////////////////////////////

void initMPU6000()//I2C_TypeDef *I2Cx)
{
    ///////////////////////////////////

    // chip reset
    i2cWrite(I2Cx, MPU9150_ADDR, MPU9150_RA_PWR_MGMT_1, 1 << MPU9150_PWR1_DEVICE_RESET_BIT);

    delay(150); // Startup time delay

    // wake up device and select GyroZ clock (better performance)
    i2cWrite(I2Cx, MPU9150_ADDR, MPU9150_RA_PWR_MGMT_1, MPU9150_CLOCK_PLL_ZGYRO << MPU9150_PWR1_CLKSEL_BIT);
    i2cWrite(I2Cx, MPU9150_ADDR, MPU9150_RA_PWR_MGMT_2, 0);

    // 100Hz sample, 42Hz low-pass filter, +/- 1000 deg/s, +/- 4g
    i2cWrite(I2Cx, MPU9150_ADDR, MPU9150_RA_SMPLRT_DIV, 9); // Sample rate = 1kHz / (1+9) = 100Hz
    i2cWrite(I2Cx, MPU9150_ADDR, MPU9150_RA_CONFIG, MPU9150_DLPF_BW_42 << MPU9150_CFG_DLPF_CFG_BIT);
    i2cWrite(I2Cx, MPU9150_ADDR, MPU9150_RA_GYRO_CONFIG, MPU9150_GYRO_FS_1000 << MPU9150_GCONFIG_FS_SEL_BIT);
    i2cWrite(I2Cx, MPU9150_ADDR, MPU9150_RA_ACCEL_CONFIG, MPU9150_ACCEL_FS_4 << MPU9150_ACONFIG_AFS_SEL_BIT);

    // disable I2C master, enable I2C bypass -- so we talk to the mag without needing the MPU-9150
    i2cWrite(I2Cx, MPU9150_ADDR, MPU9150_RA_USER_CTRL, 0 << MPU9150_USERCTRL_I2C_MST_EN_BIT);
    i2cWrite(I2Cx, MPU9150_ADDR, MPU9150_RA_INT_PIN_CFG, 1 << MPU9150_INTCFG_I2C_BYPASS_EN_BIT);

    delay(100);

    i2cWrite(I2Cx, MPU9150_ADDR, MPU9150_RA_INT_PIN_CFG, 0 << MPU9150_INTCFG_I2C_BYPASS_EN_BIT);

    // read mag x,y,z
    i2cWrite(I2Cx, MPU9150_ADDR, MPU9150_RA_I2C_SLV0_ADDR, (1 << MPU9150_I2C_SLV_RW_BIT) 
                                           | (AK8975_ADDRESS_00 << MPU9150_I2C_SLV_ADDR_BIT));
    i2cWrite(I2Cx, MPU9150_ADDR, MPU9150_RA_I2C_SLV0_REG, AK8975_RA_HXL);
    i2cWrite(I2Cx, MPU9150_ADDR, MPU9150_RA_I2C_SLV0_CTRL, (1 << MPU9150_I2C_SLV_EN_BIT) 
                                           | (1 << MPU9150_I2C_SLV_BYTE_SW_BIT) 
                                           | (1 << MPU9150_I2C_SLV_GRP_BIT) 
                                           | (sizeof(short)*3 << MPU9150_I2C_SLV_LEN_BIT));

    // tell the mag to take another sample
    i2cWrite(I2Cx, MPU9150_ADDR, MPU9150_RA_I2C_SLV1_ADDR, (0 << MPU9150_I2C_SLV_RW_BIT) 
                                           | (AK8975_ADDRESS_00 << MPU9150_I2C_SLV_ADDR_BIT));
    i2cWrite(I2Cx, MPU9150_ADDR, MPU9150_RA_I2C_SLV1_REG, AK8975_RA_CNTL);
    i2cWrite(I2Cx, MPU9150_ADDR, MPU9150_RA_I2C_SLV1_DO, AK8975_MODE_SINGLE);
    i2cWrite(I2Cx, MPU9150_ADDR, MPU9150_RA_I2C_SLV1_CTRL, (1 << MPU9150_I2C_SLV_EN_BIT) 
                                           | (sizeof(uint8_t) << MPU9150_I2C_SLV_LEN_BIT));

    // enable the slave I2C devices  
    i2cWrite(I2Cx, MPU9150_ADDR, MPU9150_RA_USER_CTRL, 1 << MPU9150_USERCTRL_I2C_MST_EN_BIT);    

    // HACK: this should be done somewhere else, and magScaleFactor should be defined locally,
    //       not in hmc5883.c/h
    magScaleFactor[0] = magScaleFactor[1] = magScaleFactor[2] = 1;

    // float min[3] = { -335, -170,  15 };
    // float max[3] = { -140,   10, 195 };
    float min[3] = { -1, -1, -1 };
    float max[3] = {  1,  1,  1 };
    int i;

    for (i = 0; i < 3; i++) {
        magScaleFactor[i] = 2.0 / (max[i] - min[i]);
        eepromConfig.magBias[i] = (max[i] + min[i]) / 2.0 * magScaleFactor[i];
    }

    computeMPU6000RTData();

}

///////////////////////////////////////////////////////////////////////////////
// Read MPU6000
///////////////////////////////////////////////////////////////////////////////

void readMPU6000()//I2C_TypeDef *I2Cx)
{
    enum { mpuBytes = (3+3+3+1) * 2 };
    //uint8_t buf[3*1*3*sizeof(int16_t)];
    uint8_t buf[ mpuBytes ];

    i2cRead(I2Cx, MPU9150_ADDR, MPU9150_RA_ACCEL_XOUT_H, mpuBytes, buf);

    int i = -1;

    rawAccel[XAXIS].bytes[1]       = buf[++i];
    rawAccel[XAXIS].bytes[0]       = buf[++i];
    rawAccel[YAXIS].bytes[1]       = buf[++i];
    rawAccel[YAXIS].bytes[0]       = buf[++i];
    rawAccel[ZAXIS].bytes[1]       = buf[++i];
    rawAccel[ZAXIS].bytes[0]       = buf[++i];

    rawMPU6000Temperature.bytes[1] = buf[++i];
    rawMPU6000Temperature.bytes[0] = buf[++i];

    rawGyro[ROLL ].bytes[1]        = buf[++i];
    rawGyro[ROLL ].bytes[0]        = buf[++i];
    rawGyro[PITCH].bytes[1]        = buf[++i];
    rawGyro[PITCH].bytes[0]        = buf[++i];
    rawGyro[YAW  ].bytes[1]        = buf[++i];
    rawGyro[YAW  ].bytes[0]        = buf[++i];

    // swap X and Y because the mag is not aligned with accel/gyro on 9150
    rawMag[YAXIS].bytes[1]         = buf[++i];
    rawMag[YAXIS].bytes[0]         = buf[++i];
    rawMag[XAXIS].bytes[1]         = buf[++i];
    rawMag[XAXIS].bytes[0]         = buf[++i];
    rawMag[ZAXIS].bytes[1]         = buf[++i];
    rawMag[ZAXIS].bytes[0]         = buf[++i];
}

///////////////////////////////////////////////////////////////////////////////
// Compute MPU6000 Runtime Data
///////////////////////////////////////////////////////////////////////////////

void computeMPU6000RTData(void)
{
    uint8_t  axis;
    uint16_t samples;

    float accelSum[3] = { 0.0f, 0.0f, 0.0f };
    float gyroSum[3]  = { 0.0f, 0.0f, 0.0f };

    mpu6000Calibrating = true;

    for (samples = 0; samples < 5000; samples++)
    {
        readMPU6000();

        computeMPU6000TCBias();

        accelSum[XAXIS] += (float)rawAccel[XAXIS].value - accelTCBias[XAXIS];
        accelSum[YAXIS] += (float)rawAccel[YAXIS].value - accelTCBias[YAXIS];
        accelSum[ZAXIS] += (float)rawAccel[ZAXIS].value - accelTCBias[ZAXIS];

        gyroSum[ROLL ]  += (float)rawGyro[ROLL ].value  - gyroTCBias[ROLL ];
        gyroSum[PITCH]  += (float)rawGyro[PITCH].value  - gyroTCBias[PITCH];
        gyroSum[YAW  ]  += (float)rawGyro[YAW  ].value  - gyroTCBias[YAW  ];

        delayMicroseconds(1000);
    }

    for (axis = 0; axis < 3; axis++)
    {
        accelSum[axis]   = accelSum[axis] / 5000.0f * ACCEL_SCALE_FACTOR;
        gyroRTBias[axis] = gyroSum[axis]  / 5000.0f;
    }

    accelOneG = sqrt(accelSum[XAXIS] * accelSum[XAXIS] +
                     accelSum[YAXIS] * accelSum[YAXIS] +
                     accelSum[ZAXIS] * accelSum[ZAXIS]);

    mpu6000Calibrating = false;
}

///////////////////////////////////////////////////////////////////////////////
// Compute MPU6000 Temperature Compensation Bias
///////////////////////////////////////////////////////////////////////////////

void computeMPU6000TCBias(void)
{
    mpu6000Temperature = (float) (rawMPU6000Temperature.value) / 340.0f + 35.0f;

    accelTCBias[XAXIS] = eepromConfig.accelTCBiasSlope[XAXIS] * mpu6000Temperature + eepromConfig.accelTCBiasIntercept[XAXIS];
    accelTCBias[YAXIS] = eepromConfig.accelTCBiasSlope[YAXIS] * mpu6000Temperature + eepromConfig.accelTCBiasIntercept[YAXIS];
    accelTCBias[ZAXIS] = eepromConfig.accelTCBiasSlope[ZAXIS] * mpu6000Temperature + eepromConfig.accelTCBiasIntercept[ZAXIS];

    gyroTCBias[ROLL ]  = eepromConfig.gyroTCBiasSlope[ROLL ]  * mpu6000Temperature + eepromConfig.gyroTCBiasIntercept[ROLL ];
    gyroTCBias[PITCH]  = eepromConfig.gyroTCBiasSlope[PITCH]  * mpu6000Temperature + eepromConfig.gyroTCBiasIntercept[PITCH];
    gyroTCBias[YAW  ]  = eepromConfig.gyroTCBiasSlope[YAW  ]  * mpu6000Temperature + eepromConfig.gyroTCBiasIntercept[YAW  ];
}

///////////////////////////////////////////////////////////////////////////////

