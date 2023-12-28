/******************************************************************************
;       Program		: Timer_Gtm.h
;       Function	: Declare GTM ATOM Timer Function
;       Chip		: Infineon TC397
;       Clock		: Internal SYSPLL 300MHz
;       Date		: 2023 / 12 / 26
;       Author		: Fenderson Lu
******************************************************************************/
#ifndef __TIMER_GTM_H__
#define __TIMER_GTM_H__
//---------------------------- Library Support --------------------------------//
#include "IfxGtm_Atom_Timer.h"
//---------------------------- Define Constant --------------------------------//
#define		ISR_PRIORITY_ATOM		1					// Interrupt priority number                                    
#define		ATOM_FREQ				1000000.0f			// ATOM frequency 1MHz (1us)                                               
#define		CMU_FREQ				200000000.0f		// CMU clock frequency 200MHz   
//---------------------------- Declare Function -------------------------------//
extern void Timer_Gtm_Init(void);
extern uint32 Timer_Gtm_RealTimer_Get(void);
extern void Timer_Gtm_RealTimer_Clear(void);
#endif

