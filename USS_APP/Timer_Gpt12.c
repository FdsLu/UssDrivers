/******************************************************************************
;       Program		: Timer_Gpt12.c
;       Function	: Use GPT12 timer to gerernal vistual timer
;       Chip		: Infineon TC397
;       Clock		: Internal SYSPLL 300MHz
;       Date		: 2023 / 12 / 26
;       Author		: Fenderson Lu
;       Describe	: 
******************************************************************************/
//---------------------------- Include File ---------------------------------//
#include "Cpu0_Main.h"
#include "Timer_Gpt12.h" 
#include "TasksApp.h"
#include "PinIO.h"
//---------------------------- Declare External Var -------------------------//
static uint32 gu32VirtualTimer, gu32RealTimer;
static uint32 gu32VTimer1msCnt;
//---------------------------- Start Program --------------------------------//
void interruptGpt12(void)
{
    gu32RealTimer++;	
}
//---------------------------------------------------------------------------//
void Timer_Gpt12_Init(void)
{ 	
    /* Initialize the GPT12 module */
    IfxGpt12_enableModule(&MODULE_GPT120);                                          /* Enable the GPT12 module      */
    IfxGpt12_setGpt1BlockPrescaler(&MODULE_GPT120, IfxGpt12_Gpt1BlockPrescaler_16); /* Set GPT1 block prescaler     */

    /* Initialize the Timer T3 */
    IfxGpt12_T3_setMode(&MODULE_GPT120, IfxGpt12_Mode_timer);                       /* Set T3 to timer mode         */
    IfxGpt12_T3_setTimerDirection(&MODULE_GPT120, IfxGpt12_TimerDirection_down);    /* Set T3 count direction       */
    IfxGpt12_T3_setTimerPrescaler(&MODULE_GPT120, IfxGpt12_TimerInputPrescaler_64); /* Set T3 input prescaler       */
    IfxGpt12_T3_setTimerValue(&MODULE_GPT120, RELOAD_VALUE);                        /* Set T3 start value           */

    /* Initialize the Timer T2 */
    IfxGpt12_T2_setMode(&MODULE_GPT120, IfxGpt12_Mode_reload);                      /* Set T2 to reload mode        */
    IfxGpt12_T2_setReloadInputMode(&MODULE_GPT120, IfxGpt12_ReloadInputMode_bothEdgesTxOTL); /* Set reload trigger  */
    IfxGpt12_T2_setTimerValue(&MODULE_GPT120, RELOAD_VALUE);                        /* Set T2 reload value          */

    /* Initialize the interrupt */
    volatile Ifx_SRC_SRCR *src = IfxGpt12_T3_getSrc(&MODULE_GPT120);                /* Get the interrupt address    */
    IfxSrc_init(src, ISR_PROVIDER_GPT12_TIMER, CONFIG_PRIORITY_6); 			        /* Initialize service request   */
    IfxSrc_enable(src);                                                             /* Enable GPT12 interrupt       */

    IfxGpt12_T3_run(&MODULE_GPT120, IfxGpt12_TimerRun_start);                       /* Start the timer */ 

    gu32VirtualTimer = gu32RealTimer = gu32VTimer1msCnt = 0;
}
//---------------------------------------------------------------------------//
void Timer_Gpt12_VTOS(void)
{
	if(gu32VirtualTimer != gu32RealTimer)
	{ 
		(gu32VirtualTimer)++;
		(gu32VTimer1msCnt)++;
		TasksApp_1ms();
	
		if((gu32VTimer1msCnt % (uint32)(TIME_100MS)) == (uint32)(COUNTING_DONE))
		{
			TasksApp_100ms();
		}

		if((gu32VTimer1msCnt % (uint32)(TIME_1S)) == (uint32)(COUNTING_DONE))
		{
			TasksApp_1s();
		}

		if((gu32VTimer1msCnt % (uint32)(TIME_3S)) == (uint32)(COUNTING_DONE))
		{
			TasksApp_3s();
		}		
		
		if((gu32VTimer1msCnt % (uint32)(TIME_9S)) == (uint32)(COUNTING_DONE))
		{
			gu32VTimer1msCnt = (uint32)(DEFAULT_COUNT);
		}
	}	
}
//---------------------------------------------------------------------------//
void Timer_Gpt12_VirtualTimer_Clear(void) 
{
	gu32VirtualTimer = gu32RealTimer;
}
//---------------------------------------------------------------------------//
uint32 Timer_Gpt12_RealTimer_Get(void) 
{
	return(gu32RealTimer);
}
//---------------------------------------------------------------------------//

