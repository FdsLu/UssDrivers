/******************************************************************************
;       Program		:	Clock.c
;       Function	:	The Main Program
;       Chip		:	Infineon TC397
;       Clock		:	Internal SYSPLL 300MHz
;       Date		:	2023 / 11 / 2
;       Author		:	Fenderson Lu
******************************************************************************/
//---------------------------- Include File ---------------------------------//
#include "Cpu0_Main.h"
#include "Clock.h"
//---------------------------- Declare Global Variable ----------------------//
IFX_ALIGN(4) IfxCpu_syncEvent g_cpuSyncEvent = 0;
//---------------------------- Start Program --------------------------------//
void Clock_Init(void)
{
    IfxCpu_enableInterrupts();
    
    // WATCHDOG0 AND SAFETY WATCHDOG ARE DISABLED HERE, Enable the watchdogs and service them periodically if it is required
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

    // Wait for CPU sync event 
    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 1);
}
//---------------------------------------------------------------------------//
void Clock_Cpu1_Init(void)
{
    IfxCpu_enableInterrupts();
    
    // !!WATCHDOG1 IS DISABLED HERE!! Enable the watchdog and service it periodically if it is required
   
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    
    /* Wait for CPU sync event */
    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 1);

}

//---------------------------------------------------------------------------//
uint32 Clock_Cpu_Freq_Get(const IfxCpu_ResourceCpu cpu)
{
	return IfxScuCcu_getCpuFrequency(cpu);
}
//---------------------------------------------------------------------------//

