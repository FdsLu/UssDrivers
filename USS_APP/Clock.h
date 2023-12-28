/******************************************************************************
;       Program		: Clock.h
;       Function	: Declare CPU Clock Function & Variable
;       Chip		: Infineon TC397
;       Clock		: Internal SYSPLL 300MHz
;       Date		: 2023 / 12 / 28
;       Author		: Fenderson Lu
******************************************************************************/
#ifndef __CLOCK_H__
#define __CLOCK_H__
//---------------------------- Include Library ------------------------------//
#include "IfxCpu.h"
//---------------------------- Define Constant ------------------------------//
#define		UNIT_MHZ	1000000
//---------------------------- Declare Function -----------------------------// 
extern void Clock_Init(void);
extern void Clock_Cpu1_Init(void);
extern uint32 Clock_Cpu_Freq_Get(const IfxCpu_ResourceCpu cpu);
#endif




