/******************************************************************************
;       Program		: TasksApp.h
;       Function	: Declare TasksApp Function & Variable
;       Chip		: Infineon TC397
;       Clock		: Internal SYSPLL 300MHz
;       Date		: 2023 / 1 / 2
;       Author		: Fenderson Lu
******************************************************************************/
#ifndef __TASKSAPP_H__
#define __TASKSAPP_H__
//---------------------------- Support Library ------------------------------//
//---------------------------- Support Function -----------------------------//
#define		ECU_VER					"ECU VER : V0.0.1\r"
#define		ECU_TEST				"ECU VER : T00\r"
//---------------------------- Define Constant ------------------------------//
#define		EVB_DEMO		FALSE
#define		DEBUG_UART		FALSE

#define		VAL_INIT			0

enum GpioDirection{
	GPIO_INPUT = 0,
	GPIO_OUTPUT = 1
};

enum ModeSelect{
	MODE_DISABLE = 0,
	MODE_ENABLE = 1
};

enum CounterPara{
	CNT_DONE = 0U,
	CNT_REDUCE = 1U
};

enum InformationSize{
	SIZE_MCU_VER = 8,
	SIZE_MCU_CHKSUM = 4
};
//---------------------------- Declare Function -----------------------------// 
extern void TasksApp_1ms(void);
extern void TasksApp_100ms(void);
extern void TasksApp_1s(void);
extern void TasksApp_3s(void);
#endif




