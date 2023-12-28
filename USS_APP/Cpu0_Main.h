/******************************************************************************
;       Program		: Cpu0_Main.h
;       Function	: Declare CPU0 Main Function & Variable
;       Chip		: Infineon TC397
;       Clock		: Internal SYSPLL 300MHz
;       Date		: 2023 / 12 / 4
;       Author		: Fenderson Lu
******************************************************************************/
#ifndef __CPU0_MAIN_H__
#define __CPU0_MAIN_H__
//---------------------------- Include Library ------------------------------//
#include "Ifx_Types.h"
#include "IfxScuWdt.h"

#include "String.h"

#include "Asclin9_Uart.h"
#include "PinIO.h"
#include "Common.h"
#include "TasksApp.h"
//---------------------------- Define Constant ------------------------------//
//ISR Priority
#define	CONFIG_PRIORITY_0			0x00		// No interrupt function
#define	CONFIG_PRIORITY_1			0x01		// Priority lowest
#define	CONFIG_PRIORITY_2			0x02
#define	CONFIG_PRIORITY_3			0x03
#define	CONFIG_PRIORITY_4			0x04
#define	CONFIG_PRIORITY_5			0x05
#define	CONFIG_PRIORITY_6			0x06
#define	CONFIG_PRIORITY_7			0x07
#define	CONFIG_PRIORITY_8			0x08
#define	CONFIG_PRIORITY_9			0x09
#define	CONFIG_PRIORITY_10			0x0A		// Priority high, highest=0xFF	
#define	CONFIG_PRIORITY_11			0x0B
#define	CONFIG_PRIORITY_12			0x0C
#define	CONFIG_PRIORITY_13			0x0D
#define	CONFIG_PRIORITY_14			0x0E
#define	CONFIG_PRIORITY_15			0x0F
#define	CONFIG_PRIORITY_16			0x10
#define	CONFIG_PRIORITY_17			0x11
#define	CONFIG_PRIORITY_18			0x12
#define	CONFIG_PRIORITY_19			0x13
#define	CONFIG_PRIORITY_20			0x14

#define	CONFIG_PRIORITY_40			0x28

#define		SHIFT_BIT_0			0
#define		SHIFT_BIT_1			1
#define		SHIFT_BIT_2			2
#define		SHIFT_BIT_3			3
#define		SHIFT_BIT_4			4
#define		SHIFT_BIT_5			5
#define		SHIFT_BIT_6			6
#define		SHIFT_BIT_7			7
#define		SHIFT_BIT_8			8
#define		SHIFT_BIT_9			9
#define		SHIFT_BIT_10		10
#define		SHIFT_BIT_11		11
#define		SHIFT_BIT_12		12
#define		SHIFT_BIT_13		13
#define		SHIFT_BIT_14		14
#define		SHIFT_BIT_15		15
#define		SHIFT_BIT_16		16
#define		SHIFT_BIT_17		17
#define		SHIFT_BIT_18		18
#define		SHIFT_BIT_19		19
#define		SHIFT_BIT_20		20
#define		SHIFT_BIT_21		21
#define		SHIFT_BIT_22		22
#define		SHIFT_BIT_23		23
#define		SHIFT_BIT_24		24
#define		SHIFT_BIT_25		25
#define		SHIFT_BIT_26		26
#define		SHIFT_BIT_27		27
#define		SHIFT_BIT_28		28
#define		SHIFT_BIT_29		29
#define		SHIFT_BIT_30		30
#define		SHIFT_BIT_31		31
#define		SHIFT_BIT_32		32
//---------------------------- Declare Function -----------------------------// 
#endif




