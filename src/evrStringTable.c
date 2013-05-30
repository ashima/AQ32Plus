/**
  \brief      EVR (EVent Report) strings tables.
  \remark     Ported for AQ32Plus.
*/


#include <inttypes.h>
#include "evr.h"


constStrArr_t evrInfo = {
    "None",
    "Normal Reset",
    "Starting Main Loop"
    };
constStrArr_t evrWarn = {
    "Abnormal Reset",
    "Battery Low",
    "Battery Very Low",
    };
constStrArr_t evrError = {
    "Out of EVR Listener slots",
    "Failed to register Frame Reset Watchdog",
    "Failed to register Frame Lost Watchdog",
    "Recieve pilot command frame timeout!",
    "Battery dangeriously Low!",
    "Flash CRC failed! (use eeprom cli to override)",
    "Flash erase failed.",
    "Flash programming failed."
    };
constStrArr_t evrFatal = {
    "",
    };

const evrStringTable_t evrStringTable[evrTypesNUM] =
  {
    { "Information", evrInfo,  sizeof(evrInfo)/sizeof(char*)  },
    { "Warning",     evrWarn,  sizeof(evrWarn)/sizeof(char*)  },
    { "Error",       evrError, sizeof(evrError)/sizeof(char*) },
    { "Fatal",       evrFatal, sizeof(evrFatal)/sizeof(char*) }
  };


