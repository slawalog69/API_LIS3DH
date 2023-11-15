/*
 * CycleCount.h
 *
 *  Created on: 9 нояб. 2023 г.
 *      Author: Slava
 */

#ifndef CYCLECOUNT_H_
#define CYCLECOUNT_H_

#include "main.h"
#include "cmsis_os.h"


extern volatile unsigned int StartCnt;
extern volatile unsigned int *DWT_CYCCNT;
extern volatile unsigned int   RemainCycles;
extern volatile unsigned int   CyclesArr[24];
extern volatile unsigned int    posCyclesArr;
extern void CalcCycles_MS();

#define startCycle() StartCnt = *DWT_CYCCNT;
#define GetCycles()  RemainCycles = ((*DWT_CYCCNT) - StartCnt)
#define AddCycles() GetCycles();CyclesArr[posCyclesArr] = RemainCycles;posCyclesArr++
#define ClrCycles() CalcCycles_MS();posCyclesArr = 0
void EnableTiming(void);
#endif /* CYCLECOUNT_H_ */
