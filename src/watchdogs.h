/**
  \brief      Watch Dog Timers.
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/AQ32Plus
  \remark     Ported for AQ32Plus.
*/

typedef void (*timeout_fp)(void); /*!< prototype for timeout functions */

int  watchDogRegister(uint32_t*, uint32_t, timeout_fp,int);
void watchDogsTick();
void watchDogDisable(uint32_t);
