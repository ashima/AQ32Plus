/**
  \file       usb_usr_support.c
  \brief      USB support functions
  \copyright  Copyright (C) 2013 Ashima Research. All rights reserved. This
              file distributed under the MIT Expat License. See LICENSE file.
              https://github.com/ashima/AQ32Plus
  \remark     Ported for AQ32Plus.
*/

#include "usb_core.h"
#include "usb_dcd.h"

/**
  \brief   Disconnect and reconnect the USB port. Should cause the host end
           to reinitialize its connection to this device.
  \param   pdev : USB device to dis- and re- connect.
*/
void USB_Support_DisReConnect(USB_OTG_CORE_HANDLE *pdev)
  {
  DCD_DevDisconnect(pdev);
  delay(200);
  DCD_DevConnect(pdev);
  }

