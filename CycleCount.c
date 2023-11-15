/*
 * CycleCount.c
 *
 *  Created on: 9 нояб. 2023 г.
 *      Author: Slava
 */


#include "CycleCount.h"

//****************************************************************************

volatile unsigned int *DWT_CYCCNT   = (volatile unsigned int *)0xE0001004;
volatile unsigned int *DWT_CONTROL  = (volatile unsigned int *)0xE0001000;
volatile unsigned int *DWT_LAR      = (volatile unsigned int *)0xE0001FB0;
volatile unsigned int *SCB_DHCSR    = (volatile unsigned int *)0xE000EDF0;
volatile unsigned int *SCB_DEMCR    = (volatile unsigned int *)0xE000EDFC;
volatile unsigned int *ITM_TER      = (volatile unsigned int *)0xE0000E00;
volatile unsigned int *ITM_TCR      = (volatile unsigned int *)0xE0000E80;

//****************************************************************************

static int Debug_ITMDebug = 0;
volatile unsigned int StartCnt = 0;
volatile unsigned int   RemainCycles = 0;

volatile unsigned int   CyclesArr[24] = {0};
volatile unsigned int    posCyclesArr;


//****************************************************************************

void EnableTiming(void)
{
  if ((*SCB_DHCSR & 1) && (*ITM_TER & 1)) // Enabled?
    Debug_ITMDebug = 1;

  *SCB_DEMCR |= 0x01000000;
  *DWT_LAR = 0xC5ACCE55; // enable access
  *DWT_CYCCNT = 0; // reset the counter
  *DWT_CONTROL |= 1 ; // enable the counter

   posCyclesArr = 0;
}
//****************************************************************************

void CalcCycles_MS()
{
for (int i = 0; i < posCyclesArr; i++) {
	 float f = CyclesArr[i];
	 CyclesArr[i] = (u32)(f*1000.0/SystemCoreClock);
}

}

//****************************************************************************

void DelayCycle(uint32_t cycles)
{
  uint32_t start = *DWT_CYCCNT;

  while((*DWT_CYCCNT - start) < cycles);
}

//****************************************************************************
