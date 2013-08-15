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
  m = mid | (c[0] << 16);
  ctPushSB(3, (uint8_t*)&m);
  ctPushB(s-1, c+1);
  }

  union {
    struct __attribute__((__packed__)) {
      uint16_t m ;
      uint32_t t ;
      uint8_t b[1];
      };
    uint8_t c[128];
    } msgx;

void ctPushSMTB(uint16_t mid,int32_t s, uint8_t *c)
  {
  int32_t i;

  msgx.m = mid;
  msgx.t = micros();
  for (i=0; i< s; ++i)
    msgx.b[i] = c[i]; 
  ctPushSB(s+6, msgx.c);
  }
#if 0
void ctPushSMTB(uint16_t mid,int32_t s, uint8_t *c)
  {
  uint8_t xb[6];

  uint32_t t = micros();
if (mid == 17)
  t = 0x12345678;
  xb[0] = (mid >> 0) & 0xff;
  xb[1] = (mid >> 16 ) & 0xff;
  xb[2] = (t >> 0  ) & 0xff;
  xb[3] = (t >> 8  ) & 0xff;
  xb[4] = (t >> 16 ) & 0xff;
  xb[5] = (t >> 24 ) & 0xff;

  ctPushSB(6, xb);
  ctPushB(s, c);

 /* 
  msgx.t = micros();
  msgx.m = mid;
  ctPushSB(6, msgx.c);
  ctPushB(s, c);
*/
  }
#endif
