/******************************************************************************
;       Program		:	Cpu0_Main.c
;       Function	:	The Main Program
;       Chip		:	Infineon TC397
;       Clock		:	Internal Clock 300MHz
;       Date		:	2024 / 1 / 4
;       Author		:	Fenderson Lu
******************************************************************************/
//---------------------------- Include File ---------------------------------//
#include "Cpu0_Main.h"
#include "Clock.h"
#include "TasksApp.h"
#include "Timer_Gtm.h"
#include "PinIO.h"
#include "Common.h"
#include "UssDrivers.h"
#include "Timer_Gpt12.h"
//---------------------------- Declare Global Variable ----------------------//
//---------------------------- Start Program --------------------------------//
void core0_main(void)
{
	Clock_Init();
	Timer_Gtm_Init();	
	Timer_Gpt12_Init();
	PinIO_Init();
	UssDrivers_Init();	
	
	Asclin9_Uart_Init();

	#if (EVB_DEMO == FALSE)
		PinIO_Pwr_Sequence();
	#endif

	Timer_Gtm_RealTimer_Clear();
	Timer_Gpt12_VirtualTimer_Clear();
	
	#if DEBUG_UART
		DEBUG_MSG_VAL([Sys] Clk(MHz) = , Clock_Cpu_Freq_Get(IfxCpu_ResourceCpu_0)/UNIT_MHZ, SHOW_DEC);
		DEBUG_MSG([Sys] Init OK); 
	#endif
	
	while(1)
	{		
		Timer_Gpt12_VTOS();
		UssDrivers_Rx_Data_Parse(UssDrivers_RxIsrFinishFlag_Get());
	}
}
//---------------------------------------------------------------------------//























