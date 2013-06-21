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

///////////////////////////////////////////////////////////////////////////////

/*
    DMA UART routines idea lifted from AutoQuad
    Copyright © 2011  Bill Nesbitt
*/

///////////////////////////////////////////////////////////////////////////////
// UART4 Defines and Variables
///////////////////////////////////////////////////////////////////////////////

#define UART4_TX_PIN        GPIO_Pin_5
#define UART4_RX_PIN        GPIO_Pin_6
#define UART4_GPIO          GPIOD
#define UART4_TX_PINSOURCE  GPIO_PinSource5
#define UART4_RX_PINSOURCE  GPIO_PinSource6

#define UART4_BUFFER_SIZE    2048

// Receive buffer, circular DMA
volatile uint8_t rx4Buffer[UART4_BUFFER_SIZE];
uint32_t rx4DMAPos = 0;

volatile uint8_t tx4Buffer[UART4_BUFFER_SIZE];
volatile uint16_t tx4BufferTail = 0;
volatile uint16_t tx4BufferHead = 0;

volatile uint8_t  tx4DmaEnabled = false;

///////////////////////////////////////////////////////////////////////////////
// UART4 Transmit via DMA
///////////////////////////////////////////////////////////////////////////////

static void uart4TxDMA(void)
{
	if ((tx4DmaEnabled == true) || (tx4BufferHead == tx4BufferTail))  // Ignore call if already active or no new data in buffer
    	return;

    DMA1_Stream6->M0AR = (uint32_t)&tx4Buffer[tx4BufferTail];
    if (tx4BufferHead > tx4BufferTail)
    {
	    DMA_SetCurrDataCounter(DMA1_Stream6, tx4BufferHead - tx4BufferTail);
	    tx4BufferTail = tx4BufferHead;
    }
    else
    {
	    DMA_SetCurrDataCounter(DMA1_Stream6, UART4_BUFFER_SIZE - tx4BufferTail);
	    tx4BufferTail = 0;
    }

    tx4DmaEnabled = true;

    DMA_Cmd(DMA1_Stream6, ENABLE);
}

///////////////////////////////////////////////////////////////////////////////
// UART4 TX Complete Interrupt Handler
///////////////////////////////////////////////////////////////////////////////

void DMA1_Stream4_IRQHandler(void)
{
    DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF6);

    tx4DmaEnabled = false;

    uart4TxDMA();
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

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,  ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,   ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = UART4_TX_PIN | UART4_RX_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    GPIO_PinAFConfig(UART4_GPIO, UART4_TX_PINSOURCE, GPIO_AF_UART4);
    GPIO_PinAFConfig(UART4_GPIO, UART4_RX_PINSOURCE, GPIO_AF_UART4);

    GPIO_Init(UART4_GPIO, &GPIO_InitStructure);

    // DMA TX Interrupt
    NVIC_InitStructure.NVIC_IRQChannel                   = DMA1_Stream4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

    NVIC_Init(&NVIC_InitStructure);

    USART_InitStructure.USART_BaudRate            = eepromConfig.gpsBaudRate;
  //USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  //USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  //USART_InitStructure.USART_Parity              = USART_Parity_No;
  //USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
  //USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    USART_Init(UART4, &USART_InitStructure);

    // Receive DMA into a circular buffer

    DMA_DeInit(DMA1_Stream2);

    DMA_InitStructure.DMA_Channel            = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)rx4Buffer;
  //DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize         = UART4_BUFFER_SIZE;
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

    DMA_Init(DMA1_Stream2, &DMA_InitStructure);

    DMA_Cmd(DMA1_Stream2, ENABLE);

    USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);

    rx4DMAPos = DMA_GetCurrDataCounter(DMA1_Stream2);

    // Transmit DMA
    DMA_DeInit(DMA1_Stream4);

  //DMA_InitStructure.DMA_Channel            = DMA_Channel_4;
  //DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)tx4Buffer;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
  //DMA_InitStructure.DMA_BufferSize         = UART_BUFFER_SIZE;
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

    DMA_Init(DMA1_Stream4, &DMA_InitStructure);

    DMA_SetCurrDataCounter(DMA1_Stream4, 0);

    DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);

    USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);

    USART_Cmd(UART4, ENABLE);
}

///////////////////////////////////////////////////////////////////////////////
// GPS Available
///////////////////////////////////////////////////////////////////////////////

uint16_t gpsAvailable(void)
{
    return (DMA_GetCurrDataCounter(DMA1_Stream5) != rx4DMAPos) ? true : false;
}

///////////////////////////////////////////////////////////////////////////////
// GPS Clear Buffer
///////////////////////////////////////////////////////////////////////////////

void gpsClearBuffer(void)
{
    rx4DMAPos = DMA_GetCurrDataCounter(DMA1_Stream5);
}

///////////////////////////////////////////////////////////////////////////////
// GPS Number of Characters Available
///////////////////////////////////////////////////////////////////////////////

uint16_t gpsNumCharsAvailable(void)
{
	int32_t number;

	number = rx4DMAPos - DMA_GetCurrDataCounter(DMA1_Stream5);

	if (number >= 0)
	    return (uint16_t)number;
	else
	    return (uint16_t)(UART4_BUFFER_SIZE + number);
}

///////////////////////////////////////////////////////////////////////////////
// GPS Read
///////////////////////////////////////////////////////////////////////////////

uint8_t gpsRead(void)
{
    uint8_t ch;

    ch = rx4Buffer[UART4_BUFFER_SIZE - rx4DMAPos];
    // go back around the buffer
    if (--rx4DMAPos == 0)
	    rx4DMAPos = UART4_BUFFER_SIZE;

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

void gpsWrite(uint8_t ch)
{
    tx4Buffer[tx4BufferHead] = ch;
    tx4BufferHead = (tx4BufferHead + 1) % UART4_BUFFER_SIZE;

    uart4TxDMA();
}

///////////////////////////////////////////////////////////////////////////////
// GPS Print
///////////////////////////////////////////////////////////////////////////////

void gpsPrint(char *str)
{
    while (*str)
    {
    	tx4Buffer[tx4BufferHead] = *str++;
    	tx4BufferHead = (tx4BufferHead + 1) % UART4_BUFFER_SIZE;
    }

	uart4TxDMA();
}

///////////////////////////////////////////////////////////////////////////////
