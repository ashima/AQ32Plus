/**
  \file       skyTraq.c
  \brief      packet reading interface to Sky Traq Venus 6.
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/AQ32Plus
  \remark     Ported for AQ32Plus.
*/

#include <inttypes.h>
#include "board.h"
#include "gps/skyTraq.h"
#include "state/char_telem.h"

enum { stBuffMax = 128 };
 
uint32_t stState = 0, stNoise = 0;
uint8_t buff[stBuffMax];
int stBuffInx, stLength;
uint8_t stChk;

void skytraqNEMA(uint8_t *buff);
void skytraqBinPkt(uint8_t *buff, int stLength);
void skytraqPktError();

void skytraqStepState(uint8_t c)
  {
  switch(stState)
    {
    case 0 : // start
      stBuffInx = 0;
      buff[stBuffInx] = '\0';

      if ( c == 36 )
        stState = 1;
      else if ( c == 0xA0 )
        stState = 3;
      else
        stNoise++;
      break;

    case 1 : // readString
      if (c == 0x0D )
        stState = 2;
      else
        buff[stBuffInx++] = c; 
      break;

    case 2 : // endString
      if (c == 0x0A)
        {
        stState = 0;
        buff[stBuffInx++] = '\0'; 
        skytraqNEMA(buff);
        }
      else
        {
        stState = 0;
        skytraqPktError();
        }
      break;

    case 3 : // SearchingA1
      if (c == 0xA1)
        stState = 4;
      else
        {
        stState = 0;
        stNoise += 2;
        }
      break;

    case 4 : // ReadPL0
      stLength = c << 8;
      stState = 5 ;
      break;

    case 5 : // ReadPL1
      stLength |= c;
      stChk = 0;
      stState = 6;
      break;

    case 6 : // ReadPayload
      buff[stBuffInx++] = c;
      stChk = stChk ^ c;
      if (stBuffInx >= stLength )
        stState = 7;
      break;

    case 7 : // ReadChk
      stChk = stChk ^ c;
      if (stChk == 0 )
        stState = 8;
      else
        {
        stState = 0;
        skytraqPktError();
        }
      break;

    case 8 : // ReadEnd0
      if (c == 0x0D )
        stState = 9;
      else
        {
        stState = 0;
        skytraqPktError();
        }
      break;

    case 9 : // ReadEnd1
      if (c == 0x0A)
        {
        stState = 0;
        skytraqBinPkt(buff, stLength);
        }
      else
        {
        stState = 0;
        skytraqPktError();
        }
      break;
    }
  }

void skytraqPktError()
  {
  }

void skytraqNEMA(uint8_t *buff)
  {
  }

void skytraqNavigationDataPkt( message_0xA8_t* p );

void skytraqBinPkt(uint8_t *p, int stLength)
  {
  BLUE_LED_TOGGLE;
  switch ( p[0] )
    {
    case 0xA8 : skytraqNavigationDataPkt( (message_0xA8_t*)p ); break;
    }
  }

void skytraqNavigationDataPkt( message_0xA8_t* p )
  {
  union {
    struct  __attribute__((__packed__))  {
      uint8_t fm;
      uint8_t sv;
      uint32_t lat;
      uint32_t lon;
      uint32_t x;
      uint32_t y;
      uint32_t z;
      uint32_t u;
      uint32_t v;
      uint32_t w;
    };
    uint8_t c[1];
  } x;
  RED_LED_TOGGLE;

  x.fm = p->fix_mode;
  x.sv = p->number_of_sv_in_fix;
  x.lat = p->latitude ;
  x.lon = p->longitude;
  x.x = p->ecef_x;
  x.y = p->ecef_y;
  x.z = p->ecef_z;
  x.u = p->ecef_vx;
  x.v = p->ecef_vy;
  x.w = p->ecef_vz;
  
  ctPushSMTB(ctIDGPSraw, sizeof(x), x.c);

  }
