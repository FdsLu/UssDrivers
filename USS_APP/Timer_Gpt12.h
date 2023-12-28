/******************************************************************************
;       Program		: Timer_Gpt12.h
;       Function	: Declare Timer GPT12 Function
;       Chip		: Infineon TC397
;       Clock		: Internal SYSPLL 300MHz
;       Date		: 2023 / 12 / 26
;       Author		: Fenderson Lu
******************************************************************************/
#ifndef __TIMER_GPT12_H__
#define __TIMER_GPT12_H__
//---------------------------- Library Support --------------------------------//
#include "IfxGpt12.h"
//---------------------------- Define Constant --------------------------------//
#define ISR_PROVIDER_GPT12_TIMER    IfxSrc_Tos_cpu0		// Interrupt provider
#define RELOAD_VALUE                97u					// Reload value to have an interrupt each 1ms(97u), 500ms(48828u)

#define 	INT_TIME				1000				// Tick timer in 1000us period
#define 	TIME_100MS				(int)(100000/INT_TIME)
#define 	TIME_1S					(int)(1000000/INT_TIME)
#define 	TIME_3S					(int)(3000000/INT_TIME)
#define 	TIME_9S					(int)(9000000/INT_TIME)

#define		COUNTING_DONE			0
#define		DEFAULT_COUNT			0

IFX_INTERRUPT(interruptGpt12, 0, CONFIG_PRIORITY_6);
//---------------------------- Declare Function -------------------------------//
extern void Timer_Gpt12_Init(void);
extern void Timer_Gpt12_VTOS(void);
extern void Timer_Gpt12_VirtualTimer_Clear(void);
extern uint32 Timer_Gpt12_RealTimer_Get(void);
#endif

