/******************************************************************************
;       Program		:	Common.c
;       Function	:	The Main Program
;       Chip		:	Infineon TC397
;       Clock		:	Internal Clock 300MHz
;       Date		:	2023 / 12 / 5
;       Author		:	Fenderson Lu
******************************************************************************/
//---------------------------- Include File ---------------------------------//
#include "Cpu0_Main.h"
#include "Common.h"
//---------------------------- Declare Global Variable ----------------------//
//---------------------------- Start Program --------------------------------//
/******************************************************************************
;       Function Name			:	void Common_Delay(Ifx_STM *pDelayUnit, uint32 u32DelayTime)
;       Function Description	:	Tiem delay function.
;       Parameters				:	[u8DelayUnit] - Delay time unit selection
;												  (1) UNIT_MINISECOND : Time unit is millisecond
;												  (2) UNIT_MICRO : Time unit is microsecond
;									[u32DelayTime] - Timer counter
;       Return Values			:	void
;		Source ID				:	
******************************************************************************/
void Common_Delay(uint8 u8DelayUnit, uint32 u32DelayTime)
{
	switch(u8DelayUnit)
	{
		case UNIT_MILLI:
			waitTime(IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, u32DelayTime));
		break;
		case UNIT_MICRO:
			waitTime(IfxStm_getTicksFromMicroseconds(BSP_DEFAULT_TIMER, u32DelayTime)); 
		break;
	}	
}
//---------------------------------------------------------------------------//
boolean Common_Down_Counter(uint32 *u32CountValues)
{
	boolean bFlag;

	if(*u32CountValues > 0U)
	{
		*u32CountValues -= (uint32)(CNT_REDUCE);
		if(*u32CountValues != (uint32)(CNT_DONE))
		{
			bFlag = FALSE;
		}
		else
		{
			bFlag = TRUE;
		}			
	}
	else
	{
		bFlag = FALSE;
	}

	(void)(u32CountValues);
	return bFlag;
}
//---------------------------------------------------------------------------//
boolean Common_Debounce_Counter(uint32 *u32CountValues)
{
	boolean bFlag;
	
	if(*u32CountValues == (uint32)(CNT_DONE))
	{
		bFlag = TRUE;
	}
	else
	{		
		bFlag = FALSE;
		*u32CountValues -= (uint32)(CNT_REDUCE);
	}			

	(void)(u32CountValues);
	return bFlag;
}
//---------------------------------------------------------------------------//
void Common_Buffer_Clear(uint32 *pu32Buffer, uint32 u32Length)
{
    uint32 u32Index;
	
    for(u32Index = 0; u32Index < u32Length; u32Index++)
        pu32Buffer[u32Index] = VAL_INIT;
}
//---------------------------------------------------------------------------//

