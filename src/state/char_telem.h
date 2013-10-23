/**
  \file       char_telem.h
  \brief      Simple base64 small packet telemetry.
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/AQ32Plus
  \remark     Ported for AQ32Plus.
  \remark     We really ought to make this a telem dict + autocodegen! -ijm
*/

#ifndef H_CHAR_TELEM_H
#define H_CHAR_TELEM_H

void ctPushB   (int32_t,  uint8_t*);
void ctPushSMTB(uint16_t, int32_t, uint8_t*);
void ctPushSMB (uint16_t, int32_t, uint8_t*);
void ctPushSB  (int32_t,  uint8_t*);

enum {
//Housekeeping
  ctHouseKeeping = 0x0,
  ctIDEVR              = 0x02 + ctHouseKeeping,
  ctIDTelemTxBuffInUse = 0x08 + ctHouseKeeping,

//Raw
  ctRaw = 0x20,
  ctIDPressure    = 0x00 + ctRaw,
  ctIDTemperature = 0x01 + ctRaw,
  ctIDGPSraw      = 0x02 + ctRaw,
  ctIDAcc         = 0x03 + ctRaw,

//Rawish
  ctRawish = 0x40,
  ctIDWAcc100       = 0x00 + ctRawish,
  ctIDMotorCommands = 0x01 + ctRawish,

// States
  ctState = 0x60,
  ctIDHSFState     = 0x00 + ctState,
  ctIDBMP180Params = 0x01 + ctState,
  ctIDComHeight    = 0x02 + ctState,

// endmarker
  ctID_EM = 0x7f
};

enum {
  wordArrayInxBITS = 5,
//Derived...
  ctIDTableSIZE = (ctID_EM & (ctID_EM-1) ? 1 : 0 ) + ctID_EM / 0x20,
  wordArrayInxSIZE = 1 << wordArrayInxBITS,
  wordArrayInxMASK = wordArrayInxSIZE -1,
  };

typedef union 
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
    } ctIDBMP180Params_t;

typedef union {
  struct __attribute__((__packed__)) {
    uint16_t t;
    int16_t dt; 
    } ;
  uint8_t c_ptr[1];
  } ctIDTemperature_t;

typedef union {
  struct __attribute__((__packed__)) {
    uint32_t p;
    int16_t dt; 
    } ;
  uint8_t c_ptr[1];
  } ctIDPressure_t;

typedef union {
  struct __attribute__((__packed__)) {
    int16_t T;
    int16_t x; 
    int16_t y; 
    int16_t z; 
    } ;
  int16_t v[1];
  uint8_t c_ptr[1];
  } ctIDAcc_t;

#endif
