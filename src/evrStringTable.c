/**
  \brief      EVR (EVent Report) strings tables.
  \remark     Ported for AQ32Plus.
*/


#include <inttypes.h>
#include "evr.h"

const char* evrSeverity[evrTypesNUM] =
  {
  "Information", "Warning", "Error", "Fatal"
  };

constStrArr_t evrInfo = {
    "None",
    "Normal Reset",
    "Starting Main Loop"
    };
constStrArr_t evrWarn = {
    "Abnormal Reset",
    };
constStrArr_t evrError = {
    "Out of EVR Listener slots",
    "Failed to register Frame Reset Watchdog",
    "Failed to register Frame Lost Watchdog",
    "Recieve pilot command frame timeout!",
    };
constStrArr_t evrFatal = {
    ""
    };

const char **evrStringTable[] = {evrInfo,evrWarn,evrError,evrFatal} ;

