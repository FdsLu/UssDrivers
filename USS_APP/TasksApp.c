/******************************************************************************
;       Program		: 	TasksApp.c
;       Function	: 	Simulate TASK function
;       Chip		: 	Infineon TC397
;       Clock		: 	Internal SYSPLL 300MHz
;       Date		: 	2023 / 12 / 28
;       Author		: 	Fenderson Lu
;       Describe	: 	
******************************************************************************/
//--------------------- Include File ----------------------------------//
#include "Cpu0_Main.h"
#include "TasksApp.h"
#include "UssDrivers.h"
#include <stdio.h>
//--------------------- Declare Global Variable -----------------------//
//const uint8 gu8McuVer[SIZE_MCU_VER] __attribute__((section(".mcu_version"))) = {0x56, 0x31, 0x2E, 0x30, 0x30, 0x2E, 0x30, 0x31};			// MCU Version is V0.00.01
//const uint8 gu8McuChecksum[SIZE_MCU_CHKSUM] __attribute__((section(".mcu_checksum"))) = {0x0F, 0xB7, 0x6F, 0x1D};
//--------------------- Start Program ---------------------------------//
void TasksApp_1ms(void)
{


}
//---------------------------------------------------------------------//
void TasksApp_100ms(void)
{	

}
//---------------------------------------------------------------------//
void TasksApp_1s(void)
{
	#if EVB_DEMO
	IfxPort_togglePin(portLED_D107, pinLED_D107); 		// Toggle LED state
	#endif
	//UssDrivers_Cmds_Transmit(EX_CMDS_RECEIVE_B);
	//UssDrivers_ThresSetup_Para_Write(USS_ID_IO1_TXRX_FLC, &gtUssThresSetupPara[0]);
	//UssDrivers_Calib_Write(USS_ID_IO1_TXRX_FLC, &gtUssCalibWritePara[0]);

	//DEBUG_MSG(TASK_1S);
	//DEBUG_MSG_VAL(Values = , 6, SHOW_DEC);
	//DEBUG_MSG_VALS(Values = , &gu32Val[0], 3);
}
//---------------------------------------------------------------------//
void TasksApp_3s(void)
{
	#if EVB_DEMO
	UssDrivers_Cmds_Transmit(USS_ID_IO1_TXRX_FLC, EX_CMDS_SEND_A);
	#else
	UssDrivers_Cmds_Transmit(USS_ID_IO2_TXRX_FLM, EX_CMDS_READ_STATUS);
	#endif
}
//---------------------------------------------------------------------//

