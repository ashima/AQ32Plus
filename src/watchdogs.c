/**
  \brief      Watch Dog Timers.
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/embedded-STM32F-lib
  \remark     Ported for AQ32Plus.
*/

#include <inttypes.h>
#include "watchdogs.h"

extern uint32_t sysTickUptime;

typedef struct {
  uint32_t ticks;     /*!< System ticks at last reset. */
  uint32_t timeout;   /*!< Number of ticks after which to timeout */
  timeout_fp func;    /*!< Function to call on timeout */
  } watchDogs_t;

enum { watchDogNUM = 16 } ;

static watchDogs_t watchDog[watchDogNUM];
static uint32_t    watchDogTop   = 0;
static uint32_t    watchDogTicks = 0;

/*
  \brief          Resgister and initialize a watch dog timer.
  \param hnd      Output; Pointer to storage for new timer's handle
  \param timeout  Input: Trigger function after 'timeout' ticks.
  \param func     Input: function to call on timeout.
 */
int watchDogRegister(uint32_t* hnd, uint32_t timeout, timeout_fp func)
  {
  if (watchDogTop < watchDogNUM)
    {
    watchDog[watchDogTop].ticks   = watchDogTicks;
    watchDog[watchDogTop].timeout = timeout;
    watchDog[watchDogTop].func    = func;

    *hnd = watchDogTop++;
    return 0;
    }
  else
    return 1;
  }

/*
  \brief  watch dog tick function.
 */
void watchDogsTick()
  {
  int i;

  ++watchDogTicks;

  for ( i = 0 ; i < watchDogTop ; ++i )
    if (watchDogTicks - watchDog[i].ticks > watchDog[i].timeout )
      (*(watchDog[i].func))();
  //assert(watchDogTop==i);
  }

/*
  \brief      Temporarily disable a timer. 
  \param hnd  Input: Handle of timer to disable.
 */
void watchDogDisable(uint32_t hnd)
  {
  watchDog[hnd].ticks = 0xFFFFFFFF;
  }

/*
  \brief      Reset watch dog timer, to postpone timeout.
  \param hnd  Input: Handle of timer to reset.
 */
void watchDogReset(uint32_t hnd)
  {
  watchDog[hnd].ticks = watchDogTicks;
  }
