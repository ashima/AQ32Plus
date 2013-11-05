/*
  October 2012

  aq32Plus Rev -

  Copyright (c) 2012 John Ihlein.  All rights reserved.

  Open Source STM32 Based Multicopter Controller Software

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)S.O.H. Madgwick
  6)UAVX

  Designed to run on the AQ32 Flight Control Board

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

///////////////////////////////////////////////////////////////////////////////

#include "board.h"
#include "evr.h"
#include "char_telem.h"

///////////////////////////////////////////////////////////////////////////////

/*
    DMA UART routines idea lifted from AutoQuad
    Copyright © 2011  Bill Nesbitt
*/

///////////////////////////////////////////////////////////////////////////////
// USART6 Defines and Variables
///////////////////////////////////////////////////////////////////////////////

// Usart 6 on PC6,7 using Stream 2,7,5 TX and 2,2,5 RX
#define USART6_TX_PIN        GPIO_Pin_6
#define USART6_RX_PIN        GPIO_Pin_7
#define USART6_GPIO          GPIOC
#define USART6_TX_PINSOURCE  GPIO_PinSource6
#define USART6_RX_PINSOURCE  GPIO_PinSource7

#define USART6_BUFFER_SIZE    2048

enum {
  u6TxBuffBITS = 8, // 256

  // Derived constants
  u6TxBuffMAX  = 1 << u6TxBuffBITS,
  u6TxBuffMASK = u6TxBuffMAX -1
  };

// Receive buffer, circular DMA
uint8_t tx6Buffer[u6TxBuffMAX];
uint32_t u6TxOverflow = 0;

volatile uint8_t rx6Buffer[USART6_BUFFER_SIZE];
uint32_t rx6DMAPos = 0;

volatile uint16_t tx6BufferTail = 0;
volatile uint16_t tx6BufferHead = 0;

volatile uint8_t  tx6DmaEnabled = false;

///////////////////////////////////////////////////////////////////////////////
// USART6 Transmit via DMA
///////////////////////////////////////////////////////////////////////////////
uint32_t usart6TxBuffInUse(void)
  {
  int x = (int)tx6BufferHead -(int) tx6BufferTail;
  if (x < 0) 
    x += u6TxBuffMAX;
  x += DMA_GetCurrDataCounter(DMA2_Stream7);
  return x;
  }

static void usart6TxDMA(void)
{
  uint32_t hd = tx6BufferHead;
	if ((tx6DmaEnabled == true) || (hd == tx6BufferTail))  // Ignore call if already active or no new data in buffer
    	return;
    
    DMA2_Stream7->M0AR = (uint32_t)&tx6Buffer[tx6BufferTail];
    if (hd > tx6BufferTail)
    {
	    DMA_SetCurrDataCounter(DMA2_Stream7, hd - tx6BufferTail);
	    tx6BufferTail = hd;
    }
    else
    {
	    DMA_SetCurrDataCounter(DMA2_Stream7, u6TxBuffMAX - tx6BufferTail);
	    tx6BufferTail = 0;
    }

    tx6DmaEnabled = true;

    DMA_Cmd(DMA2_Stream7, ENABLE);
}

///////////////////////////////////////////////////////////////////////////////
// USART6 TX Complete Interrupt Handler
///////////////////////////////////////////////////////////////////////////////

void DMA2_Stream7_IRQHandler(void)
{
    DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);

    tx6DmaEnabled = false;

    usart6TxDMA();
}
///////////////////////////////////////////////////////////////////////////////
// GPS Initialization
///////////////////////////////////////////////////////////////////////////////

void gpsInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef   DMA_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    GPIO_StructInit(&GPIO_InitStructure);
    USART_StructInit(&USART_InitStructure);
    DMA_StructInit(&DMA_InitStructure);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,  ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,   ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = USART6_TX_PIN |USART6_RX_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    GPIO_PinAFConfig(USART6_GPIO, USART6_TX_PINSOURCE, GPIO_AF_USART6);
    GPIO_PinAFConfig(USART6_GPIO, USART6_RX_PINSOURCE, GPIO_AF_USART6);

    GPIO_Init(USART6_GPIO, &GPIO_InitStructure);

    // DMA TX Interrupt
    NVIC_InitStructure.NVIC_IRQChannel                   = DMA2_Stream7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

    NVIC_Init(&NVIC_InitStructure);

    USART_InitStructure.USART_BaudRate            = 38400;
  //USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  //USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  //USART_InitStructure.USART_Parity              = USART_Parity_No;
  //USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    USART_Init(USART6, &USART_InitStructure);

    // Receive DMA into a circular buffer

    DMA_DeInit(DMA2_Stream2);

    DMA_InitStructure.DMA_Channel            = DMA_Channel_5;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART6->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)rx6Buffer;
  //DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize         = USART6_BUFFER_SIZE;
  //DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  //DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  //DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;
  //DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
  //DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;
  //DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  //DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;

    DMA_Init(DMA2_Stream2, &DMA_InitStructure);

    DMA_Cmd(DMA2_Stream2, ENABLE);

    USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);

    rx6DMAPos = DMA_GetCurrDataCounter(DMA2_Stream2);

    // Transmit DMA
    DMA_DeInit(DMA2_Stream7);

  //DMA_InitStructure.DMA_Channel            = DMA_Channel_5;
  //DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART6->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)tx6Buffer;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize         = u6TxBuffMAX;
  //DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  //DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  //DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  //DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
  //DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;
  //DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
  //DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;
  //DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  //DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;

    DMA_Init(DMA2_Stream7, &DMA_InitStructure);

    DMA_SetCurrDataCounter(DMA2_Stream7, 0);

    DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);

    USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);

    USART_Cmd(USART6, ENABLE);
}

///////////////////////////////////////////////////////////////////////////////
// GPS Available
///////////////////////////////////////////////////////////////////////////////

uint16_t gpsAvailable(void)
{
    return (DMA_GetCurrDataCounter(DMA2_Stream2) != rx6DMAPos) ? true : false;
}

///////////////////////////////////////////////////////////////////////////////
// GPS Clear Buffer
///////////////////////////////////////////////////////////////////////////////

void gpsClearBuffer(void)
{
    rx6DMAPos = DMA_GetCurrDataCounter(DMA2_Stream2);
}

///////////////////////////////////////////////////////////////////////////////
// GPS Number of Characters Available
///////////////////////////////////////////////////////////////////////////////

uint16_t gpsNumCharsAvailable(void)
{
	int32_t number;

	number = rx6DMAPos - DMA_GetCurrDataCounter(DMA2_Stream2);

	if (number >= 0)
	    return (uint16_t)number;
	else
	    return (uint16_t)(USART6_BUFFER_SIZE + number);
}

///////////////////////////////////////////////////////////////////////////////
// GPS Read
///////////////////////////////////////////////////////////////////////////////

uint8_t gpsRead(void)
{
    uint8_t ch;

    ch = rx6Buffer[USART6_BUFFER_SIZE - rx6DMAPos];
    // go back around the buffer
    if (--rx6DMAPos == 0)
	    rx6DMAPos = USART6_BUFFER_SIZE;

    return ch;
}

///////////////////////////////////////////////////////////////////////////////
// GPS Read Poll
///////////////////////////////////////////////////////////////////////////////

uint8_t gpsReadPoll(void)
{
    while (!gpsAvailable()); // wait for some bytes
    return gpsRead();
}

///////////////////////////////////////////////////////////////////////////////
// GPS Write
///////////////////////////////////////////////////////////////////////////////
inline void u6PutChar(uint8_t x)
  {
    tx6Buffer[tx6BufferHead] = x;
    tx6BufferHead = (tx6BufferHead + 1) & u6TxBuffMASK;
  }

void gpsWrite(uint8_t ch)
{
    if ( usart6TxBuffInUse() < u6TxBuffMASK) {
        u6PutChar(ch);
    //tx6Buffer[tx6BufferHead] = ch;
    //tx6BufferHead = (tx6BufferHead + 1) % USART6_BUFFER_SIZE;

        usart6TxDMA();
        }
    else
        u6TxOverflow++;
}

///////////////////////////////////////////////////////////////////////////////
// GPS Print
///////////////////////////////////////////////////////////////////////////////

void gpsPrint(char *str)
{
    uint32_t n = usart6TxBuffInUse();

    while (*str && n < u6TxBuffMASK) 
    {
        u6PutChar(*str++);
    	n++;
    	//tx6Buffer[tx6BufferHead] = *str++;
    	//tx6BufferHead = (tx6BufferHead + 1) % USART6_BUFFER_SIZE;
    }

    if (*str)
      u6TxOverflow++;

	usart6TxDMA();
}

///////////////////////////////////////////////////////////////////////////////
// Telemetry Print Formatted - Print formatted string to Telemetry Port
// From Ala42
///////////////////////////////////////////////////////////////////////////////

void gpsPrintF(const char * fmt, ...)
{
	char buf[256];

	va_list  vlist;
	va_start (vlist, fmt);

	vsnprintf(buf, sizeof(buf), fmt, vlist);
	gpsPrint(buf);
	va_end(vlist);
}

///////////////////////////////////////////////////////////////////////////////
