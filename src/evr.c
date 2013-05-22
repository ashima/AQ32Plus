/**
  \brief      EVR (EVent Report).
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/AQ32Plus
  \remark     Ported for AQ32Plus.
*/

#include <inttypes.h>
#include "stm32f4xx.h"

#include "evr.h"

enum evrConstants
   { 
   evrBuffMAXbits = 4,
   evrListenerMAX = 8,
   evrBuffMAX     = 1<<evrBuffMAXbits,
   evrBuffMASK    = evrBuffMAX-1
   } ;

static evr_t          evrRingBuff [evrBuffMAX];
static evrListener_fp evrListeners[evrListenerMAX];

static uint32_t  evrHead = 0;
static uint32_t  evrTail = 0;
static uint32_t  evrListenerTop = 0;

void evrPush(uint16_t evr, uint16_t reason)
  {
  evr_t e = { millis(), evr, reason };

  __disable_irq();
  evrRingBuff[ evrHead ] = e;
  evrHead = (evrHead +1) & evrBuffMASK;
  __enable_irq();
  }

int evrRegisterListener(evrListener_fp f)
  {
  int r = evrListenerTop < evrListenerMAX ;
  if (r)
    evrListeners[evrListenerTop++] = f ;
  else
    evrPush(EVR_OutOfListeners,0);
  return !r;
  }

void evrBroadcast(evr_t e)
  {
  int i;
  for (i=0; i < evrListenerTop; ++i)
    (*(evrListeners[i]))(e);
  }

void evrCheck()
  {
  while (evrHead != evrTail)
    {
    evrBroadcast( evrRingBuff[evrTail] );
    evrTail = (evrTail +1) & evrBuffMASK;
    }
  }

void evrHistory(evrListener_fp f)
  {
  uint32_t i = evrHead;
  do
    {
    (*f)(evrRingBuff[i]); 
    i = (i + 1) & evrBuffMASK;
    } while ( i != evrTail ) ;
  }

const char *evrToStrSeverity(uint32_t evr)
  {
  return evrSeverity[ ( evr >> 14 &3) ];
  }

const char *evrToStr(uint32_t evr)
  {
  uint32_t s = (evr >> 14 & 3),
           i = evr & 0x3fff;
  if (i < sizeof(evrStringTable[s]) )
    return evrStringTable[s][i];
  else
    return (const char*)0;
  }

