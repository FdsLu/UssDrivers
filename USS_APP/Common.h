/******************************************************************************
;       Program		: Common.h
;       Function	: Declare Common Function & Variable
;       Chip		: Infineon TC397
;       Clock		: Internal SYSPLL 300MHz
;       Date		: 2023 / 12 / 28
;       Author		: Fenderson Lu
******************************************************************************/
#ifndef __COMMON_H__
#define __COMMON_H__
//---------------------------- Support Library ------------------------------//
#include "Bsp.h"			// Time delay function
//---------------------------- Define Constant ------------------------------//
enum TimeUnit
{
	UNIT_MILLI,
	UNIT_MICRO
};
//---------------------------- Declare Function -----------------------------//
extern void Common_Delay(uint8 u8DelayUnit, uint32 u32DelayTime);
extern boolean Common_Down_Counter(uint32 *u32CountValues);
extern boolean Common_Debounce_Counter(uint32 *u32CountValues);
extern void Common_Buffer_Clear(uint32 *pu32Buffer, uint32 u32Length);
#endif




