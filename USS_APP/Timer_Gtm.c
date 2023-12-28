/******************************************************************************
;       Program		: Timer_Gtm.c
;       Function	: Use GTM ATOM timer to gerernal vistual timer
;       Chip		: Infineon TC397
;       Clock		: Internal SYSPLL 300MHz
;       Date		: 2023 / 12 / 26
;       Author		: Fenderson Lu
;       Describe	: Timer base is 1us.
******************************************************************************/
//---------------------------- Include File ---------------------------------//
#include "Cpu0_Main.h"
#include "Timer_Gtm.h" 
#include "TasksApp.h"
#include "PinIO.h"
//---------------------------- Declare External Var -------------------------//
IfxGtm_Atom_Timer g_timerDriver;	// ATOM driver 
uint32 gu32GtmRealTimer;
//---------------------------- Start Program --------------------------------//
IFX_INTERRUPT(interruptHandlerGtmAtom, 0, ISR_PRIORITY_ATOM);			// Macro to define the Interrupt Service Routine
void interruptHandlerGtmAtom(void)
{
    gu32GtmRealTimer++;
    IfxGtm_Atom_Timer_acknowledgeTimerIrq(&g_timerDriver);				// Reset the timer event            
}
//---------------------------------------------------------------------------//
void Timer_Gtm_Init(void)
{ 	
	IfxGtm_enable(&MODULE_GTM);                                                 // Enable GTM                       

	IfxGtm_Atom_Timer_Config timerConfig;                                       // Timer configuration structure    
	IfxGtm_Atom_Timer_initConfig(&timerConfig, &MODULE_GTM);                    // Initialize default parameters    

	timerConfig.atom = IfxGtm_Atom_0;                                           // Select the ATOM_0                
	timerConfig.timerChannel = IfxGtm_Atom_Ch_0;                                // Select channel 0                 
	timerConfig.clock = IfxGtm_Cmu_Clk_0;                                       // Select the CMU clock 0           
	timerConfig.base.frequency = ATOM_FREQ;                                     // Set timer frequency              
	timerConfig.base.isrPriority = ISR_PRIORITY_ATOM;                           // Set interrupt priority           
	timerConfig.base.isrProvider = IfxSrc_Tos_cpu0;                             // Set interrupt provider           
	
	IfxGtm_Cmu_setClkFrequency(&MODULE_GTM, IfxGtm_Cmu_Clk_0, CMU_FREQ);        // Set the clock frequency          
	IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_CLK0);                // Enable the CMU clock 0           
	IfxGtm_Atom_Timer_init(&g_timerDriver, &timerConfig);                       // Initialize the ATOM              

	IfxGtm_Atom_Timer_run(&g_timerDriver);                                      // Start the ATOM                   
}
//---------------------------------------------------------------------------//
uint32 Timer_Gtm_RealTimer_Get(void) 
{
	return(gu32GtmRealTimer);
}
//---------------------------------------------------------------------------//
void Timer_Gtm_RealTimer_Clear(void)
{
    gu32GtmRealTimer = 0;
}
//---------------------------------------------------------------------------//

