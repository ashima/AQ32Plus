/**
  \file       char_telem.h
  \brief      Simple base64 small packet telemetry.
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/AQ32Plus
  \remark     Ported for AQ32Plus.
*/

#ifndef H_CHAR_TELEM_H
#define H_CHAR_TELEM_H

void ctPushB   (int32_t,  uint8_t*);
void ctPushSMTB(uint16_t, int32_t, uint8_t*);
void ctPushSMB (uint16_t, int32_t, uint8_t*);
void ctPushSB  (int32_t,  uint8_t*);

enum {
//Housekeeping
  ctIDEVR = 0x02,
  ctIDTelemTxBuffInUse = 0x08,

//Raw
  ctIDPressure = 0x10,
  ctIDTemperature = 0x11,
  ctIDGPSraw = 0x12,

//Rawish
  ctIDWAcc100 = 0x18,

// States
  ctIDHSFState = 0x20,
  ctIDBMP180Params = 0x21,
  ctIDComHeight = 0x22,
};

#endif
