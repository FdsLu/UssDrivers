/******************************************************************************
;       Program		: Asclin9_Uart.h
;       Function	: Declare ASCLIN9 UART Function & Variable
;       Chip		: Infineon TC397
;       Clock		: Internal SYSPLL 300MHz
;       Date		: 2023 / 12 / 7
;       Author		: Fenderson Lu
******************************************************************************/
#ifndef __ASCLIN_UART_H__
#define __ASCLIN_UART_H__
//---------------------------- Support Library ------------------------------//
#include "IfxAsclin_Asc.h"
#include "IfxCpu_Irq.h"
//---------------------------- Define Constant ------------------------------//
#define		BAUDRATE_UART_9					115200					// UART baud rate in bit/s
#define		PRINTFS							30

#define		PRIORITY_ASCLIN9_UART_RX		CONFIG_PRIORITY_18
#define		PRIORITY_ASCLIN9_UART_TX		CONFIG_PRIORITY_19

enum UartSize{
	SIZE_UART9_RX = 64,
	SIZE_UART9_TX = 64,
	SIZE_UART9_STR = 13,		// Size of the string
    SIZE_SHOW_VALUES = 6,
    SIZE_SHOW_HEX = 10
};

enum ShowMode{
    SHOW_HEX,
    SHOW_DEC
};

enum DecNum{
	DEC_VAL_H,
	DEC_VAL_M0,
	DEC_VAL_M1,
	DEC_VAL_M2,
	DEC_VAL_M3,
	DEC_VAL_L
};

enum HexNum{
    HEX_HEAD_1,
    HEX_HEAD_2,
    HEX_VAL_H,
    HEX_VAL_M0,
    HEX_VAL_M1,
    HEX_VAL_M2,
    HEX_VAL_M3,
    HEX_VAL_M4,
    HEX_VAL_M5,
    HEX_VAL_L
};

enum Uart0RxAscii{
    ASCII_A = 0x37,
    ASCII_ZERO = 0x30,
    ASCII_X = 0x78,
    ASCII_CAPS = 0x37U,
    NUM_A = 0x0A
};
//---------------------------- MARCO Function -------------------------------//
#define	DEBUG_MSG(String)						Asclin9_Messages_Show(""#String"\r")
#define	DEBUG_MSG_VAL(String, Value, Mode)		Asclin9_Messages_Value_Show(""#String"", Value, Mode)
#define	DEBUG_MSG_VALS(String, Value, Len)		Asclin9_Meg_Values_Show(""#String"", Value, Len)
//---------------------------- Declare Function -----------------------------// 
extern void Asclin9_Uart_Init(void);
extern void Asclin9_Messages_Show(char* String);		
extern void Asclin9_Messages_Value_Show(char* String, uint32 Value, uint8 u8Mode);
extern void Asclin9_Meg_Values_Show(char* String, uint32 *pu32Value, uint32 u32Len);
#endif




