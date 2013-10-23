/**
  \file       char_telem.c
  \brief      Simple base64 small packet telemetry.
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/AQ32Plus
  \remark     Ported for AQ32Plus.
*/

#include <inttypes.h>
#include <board.h>
#include "char_telem.h"

char base64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/" ;
char b64b[5]= "....";
#if 0
void ctPushB(int32_t s, uint8_t *c)
  {
  uint32_t x,y,z ;

// 765432 107654 321076 543210

  for ( ;s >0 ; s-=3)
    {
    x = c[0];
    y = ( s>1 ? c[1] : 0 );
    z = ( s>2 ? c[2] : 0 );

    b64b[0] = base64[  ( x >> 2 )               & 0x03f ];
    b64b[1] = base64[ (( x << 4 ) | ( y >> 4) ) & 0x03f ];
    b64b[2] = base64[ (( y << 2 ) | ( z >> 6) ) & 0x03f ];
    b64b[3] = base64[ z                         & 0x03f ];
    c += 3;
    telemetryPrint( b64b );
    }

  }
#endif
static const uint32_t ctChanEnable[ ctIDTableSIZE ] = {
  0x00000014,
  0x0000000f,
  0x00000001,
  0x00000007
  };

bool ctChanEnabled(uint16_t m)
  {
  return (true == highSpeedTelem1Enabled) && (m < ctID_EM) && 
         ((ctChanEnable[m >> wordArrayInxBITS] >> (m & wordArrayInxMASK))&1) ;
  }

void ctPushB(int32_t s, uint8_t *c)
  {
  uint32_t l;

  while ( s>0 )
    {
    l = c[0]<<16 | ( s > 1 ? c[1]<<8 : 0) | (s > 2 ? c[2] : 255 ) ;
    b64b[0] = base64[ (l >> 18 ) & 0x3f ];
    b64b[1] = base64[ (l >> 12 ) & 0x3f ];
    b64b[2] = base64[ (l >> 6 ) & 0x3f ];
    b64b[3] = base64[ (l >> 0 ) & 0x3f ];
    b64b[4] = '\0';
    c += 3;
    s -= 3;
    telemetryPrint( b64b );
    }
  }

// >>2     <<4    <<2 

/* Functions to push channels with start markers.
 */
void ctPushSB(int32_t s, uint8_t *c)
  {
  telemetryWrite( '$' );
  ctPushB(s, c);
  }

void ctPushSMB(uint16_t mid,int32_t s, uint8_t *c)
  {
  uint32_t m;
  if ( ctChanEnabled(mid) )
    {
    m = mid | (c[0] << 16);
    ctPushSB(3, (uint8_t*)&m);
    ctPushB(s-1, c+1);
    }
  }

void ctPushSMTB(uint16_t mid,int32_t s, uint8_t *c)
  {
  union {
    struct __attribute__((__packed__)) {
      uint16_t m ;
      uint32_t t ;
      };
    uint8_t c[1];
    } msgx;

  if ( ctChanEnabled(mid) )
    {
    msgx.t = micros();
    msgx.m = mid;
    ctPushSB(6, msgx.c);
    ctPushB(s, c);
    }
  }
