/**
  \brief      EVR (EVent Report).
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/AQ32Plus
  \remark     Ported for AQ32Plus.
*/


typedef struct
  {
  uint32_t time;   /*!< Time of event */
  uint16_t evr;    /*! The event to report */
  uint16_t reason; /*!< optional event parameter (reason) if any */
  } evr_t;

typedef void (*evrListener_fp)(evr_t); /*!< prototype for listener functions */
typedef const char* constStrArr_t[];

enum { evrTypesNUM = 4 } ;

extern const char*  evrSeverity[evrTypesNUM];
extern const char** evrStringTable[evrTypesNUM];

void evrPush(uint16_t evr, uint16_t reason);
int evrRegisterListener(evrListener_fp f);
void evrCheck();
void evrHistory(evrListener_fp);
const char* evrToString(uint32_t);
/*
 Evr Lists for Information, Warnings, Errors and Fatal event reports.
 */
enum evrInfoList {
  EVR_NoEvrHere    = 0,
  EVR_NormalReset,
  EVR_StartingMain,
  };

enum evrWarnList {
  EVR_AbnormalReset = 0x4000,
  };

enum evrErrorList {
  EVR_OutOfListeners = 0x8000,
  EVR_FailRegisterWatchdogFrameReset,
  EVR_FailRegisterWatchdogFrameLost,
  EVR_RxFrameLost,
  };

//enum evrFatalList {
// EVR_  = 0xC000
//  };


