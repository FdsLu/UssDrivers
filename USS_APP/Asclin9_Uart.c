/******************************************************************************
;       Program		: 	Asclin9_Uart.c
;       Function	: 	Declare ASCLIN9 UART function
;       Chip		: 	Infineon TC397
;       Clock		: 	Internal SYSPLL 300MHz
;       Date		: 	2023 / 12 / 7
;       Author		: 	Fenderson Lu
;       Describe	: 	Baudrate is 115200 bps
;						UART9 TX = ASCLIN9_UART_TX_P14.7 
;						UART9 RX = ASCLIN9_UART_RX_P14.9						
******************************************************************************/
//--------------------- Include File ----------------------------------//
#include "Cpu0_Main.h"
#include "Asclin9_Uart.h"
//--------------------- Declare Global Variable -----------------------//
/* Declaration of the ASC handle */
static IfxAsclin_Asc g_ascHandle;

/* Declaration of the FIFOs parameters */
static uint8 g_ascTxBuffer[SIZE_UART9_TX + sizeof(Ifx_Fifo) + 8];
static uint8 g_ascRxBuffer[SIZE_UART9_RX + sizeof(Ifx_Fifo) + 8];
static uint8 gu8ShowValuesBuffer[SIZE_SHOW_HEX] = {0};

uint8 gu8PrintfsBuffer[PRINTFS] = {0};
uint8 gu8PrintfsFlag;
Ifx_SizeT PrintfsLen, HexLen = SIZE_SHOW_HEX, ValuesLen;

static uint8 gu8ShowValueIndex;

uint8 g_txData[] = "";
uint8 g_rxData[SIZE_UART9_STR] = {''};
Ifx_SizeT g_count = sizeof(g_txData);	// Size of the message
//--------------------- Start Program ---------------------------------//
IFX_INTERRUPT(asclin1TxISR, 0, PRIORITY_ASCLIN9_UART_TX);
void asclin1TxISR(void)
{
    IfxAsclin_Asc_isrTransmit(&g_ascHandle);
}
//---------------------------------------------------------------------//
IFX_INTERRUPT(asclin1RxISR, 0, PRIORITY_ASCLIN9_UART_RX);
void asclin1RxISR(void)
{
    IfxAsclin_Asc_isrReceive(&g_ascHandle);
}
//---------------------------------------------------------------------//
void Asclin9_Uart_Init(void)
{   
	IfxAsclin_Asc_Config ascConfig;

	IfxAsclin_Asc_initModuleConfig(&ascConfig, portUART_9);		 /* Initialize an instance of IfxAsclin_Asc_Config with default values */

	/* Set the desired baud rate */
	ascConfig.baudrate.baudrate = BAUDRATE_UART_9;

	/* ISR priorities and interrupt target */
	ascConfig.interrupt.txPriority = PRIORITY_ASCLIN9_UART_TX;
	ascConfig.interrupt.rxPriority = PRIORITY_ASCLIN9_UART_RX;
	ascConfig.interrupt.typeOfService = IfxCpu_Irq_getTos(IfxCpu_getCoreIndex());

	/* FIFO configuration */
	ascConfig.txBuffer = &g_ascTxBuffer;
	ascConfig.txBufferSize = SIZE_UART9_TX;
	ascConfig.rxBuffer = &g_ascRxBuffer;
	ascConfig.rxBufferSize = SIZE_UART9_RX;

	/* Pin configuration */
	const IfxAsclin_Asc_Pins pins =
	{
		NULL_PTR,       IfxPort_InputMode_pullUp,     /* CTS pin not used */
		&pinUART_9_RX,   IfxPort_InputMode_pullUp,     /* RX pin           */
		NULL_PTR,       IfxPort_OutputMode_pushPull,  /* RTS pin not used */
		&pinUART_9_TX,   IfxPort_OutputMode_pushPull,  /* TX pin           */
		IfxPort_PadDriver_cmosAutomotiveSpeed1
	};
	ascConfig.pins = &pins;

	IfxAsclin_Asc_initModule(&g_ascHandle, &ascConfig); /* Initialize module with above parameters */
}
//---------------------------------------------------------------------//
void Asclin9_Messages_Show(char* String)
{
	while(*String)												// Compute String length
	{
	  gu8PrintfsBuffer[PrintfsLen++] = *String;
	  String++;
	  if (PrintfsLen >= (PRINTFS-1))
	  {
	    gu8PrintfsBuffer[PRINTFS-1] = 0;
	    break;
	  }
	}

	IfxAsclin_Asc_write(&g_ascHandle, &gu8PrintfsBuffer[0], &PrintfsLen, TIME_INFINITE);
	PrintfsLen = 0;	
}
//---------------------------------------------------------------------//
void Asclin9_Values_To_ASCII(uint32 u32Value, uint8 u8Mode)
{
    switch(u8Mode)
    {
        case SHOW_HEX:
            /* Hex show range are 32-bits */
            gu8ShowValuesBuffer[HEX_HEAD_1] = (uint8)(ASCII_ZERO);    // show head "0"
            gu8ShowValuesBuffer[HEX_HEAD_2] = (uint8)(ASCII_X);       // show head "x"

            /* 8 digit hex */
            gu8ShowValuesBuffer[HEX_VAL_H] = (uint8)((u32Value & 0xF0000000U) >> (uint32)(SHIFT_BIT_28));
            gu8ShowValuesBuffer[HEX_VAL_M0] = (uint8)((u32Value & 0x0F000000U) >> (uint32)(SHIFT_BIT_24));
            gu8ShowValuesBuffer[HEX_VAL_M1] = (uint8)((u32Value & 0x00F00000U) >> (uint32)(SHIFT_BIT_20));
            gu8ShowValuesBuffer[HEX_VAL_M2] = (uint8)((u32Value & 0x000F0000U) >> (uint32)(SHIFT_BIT_16));
            gu8ShowValuesBuffer[HEX_VAL_M3] = (uint8)((u32Value & 0x0000F000U) >> (uint32)(SHIFT_BIT_12));
            gu8ShowValuesBuffer[HEX_VAL_M4] = (uint8)((u32Value & 0x00000F00U) >> (uint32)(SHIFT_BIT_8));
            gu8ShowValuesBuffer[HEX_VAL_M5] = (uint8)((u32Value & 0x000000F0U) >> (uint32)(SHIFT_BIT_4));
            gu8ShowValuesBuffer[HEX_VAL_L] = (uint8)(u32Value & 0x0000000FU);

            for(uint32 u32Index=0; u32Index<(uint32)(SIZE_SHOW_HEX); u32Index++)
            {
                if(u32Index >= HEX_VAL_H)       // HEX_VAL_H is set print location, 0x"0"...0, "0" = HEX_VAL_H
                {
                    if(gu8ShowValuesBuffer[u32Index] >= NUM_A)
                    {
                        gu8ShowValuesBuffer[u32Index] += (uint8)(ASCII_A);        // show ASCII string "A ~ Z"
                    }
                    else
                    {
                        gu8ShowValuesBuffer[u32Index] += (uint8)(ASCII_ZERO);     // show ASICC string "0" ~ "9"
                    }
                }
            }
           
			IfxAsclin_Asc_write(&g_ascHandle, &gu8ShowValuesBuffer[0], &HexLen, TIME_INFINITE);
        break;
        case SHOW_DEC:
            // Define Value is uint32_t (0~4294967295), Show range are 0 ~ 999999
            gu8ShowValuesBuffer[DEC_VAL_H] = (uint8)(u32Value / 100000U) | (uint8)(ASCII_ZERO);
            gu8ShowValuesBuffer[DEC_VAL_M0] = (uint8)((u32Value % 100000U) / 10000U) | (uint8)(ASCII_ZERO);
            gu8ShowValuesBuffer[DEC_VAL_M1] = (uint8)((u32Value % 10000U) / 1000U) | (uint8)(ASCII_ZERO);
            gu8ShowValuesBuffer[DEC_VAL_M2] = (uint8)((u32Value % 1000U) / 100U) | (uint8)(ASCII_ZERO);
            gu8ShowValuesBuffer[DEC_VAL_M3] = (uint8)((u32Value % 100U) / 10U) | (uint8)(ASCII_ZERO);
            gu8ShowValuesBuffer[DEC_VAL_L] = (uint8)(u32Value % 10U) | (uint8)(ASCII_ZERO);

            for(gu8ShowValueIndex=0; gu8ShowValueIndex<SIZE_SHOW_VALUES; gu8ShowValueIndex++)
            {
                if(gu8ShowValuesBuffer[gu8ShowValueIndex] != ASCII_ZERO)
                    break;
            }

            if((SIZE_SHOW_VALUES - gu8ShowValueIndex) != 0)
            {
				ValuesLen = SIZE_SHOW_VALUES - gu8ShowValueIndex;
				IfxAsclin_Asc_write(&g_ascHandle, &gu8ShowValuesBuffer[gu8ShowValueIndex], &ValuesLen, TIME_INFINITE);
            }
            else
            {
                ValuesLen = 1;
                IfxAsclin_Asc_write(&g_ascHandle, &gu8ShowValuesBuffer[DEC_VAL_L], &ValuesLen, TIME_INFINITE);	 // Values are zero.
            }
        break;
    }
}
//---------------------------------------------------------------------------//
// Debug Messages and Value Function
// Description  : Asclin9_Messages_Value_Show("Values = ", 5, SHOW_DEC)
//---------------------------------------------------------------------------//
void Asclin9_Messages_Value_Show(char* String, uint32 Value, uint8 u8Mode)
{
    Asclin9_Messages_Show(String);
   	Asclin9_Values_To_ASCII(Value, u8Mode);
    Asclin9_Messages_Show("\r");
}

//---------------------------------------------------------------------------//
// Debug Messages and Values Function
// Description  : Asclin9_Meg_Values_Show("Values = ", &u32Buffer[0], 3)
//---------------------------------------------------------------------------//
void Asclin9_Meg_Values_Show(char* String, uint32 *pu32Value, uint32 u32Len)
{
	uint32 u32Index;

	Asclin9_Messages_Show(String);
	for(u32Index=0; u32Index<u32Len; u32Index++)
	{
		Asclin9_Values_To_ASCII((uint32)(pu32Value[u32Index]), SHOW_DEC);
		Asclin9_Messages_Show(", ");
	}
   
    Asclin9_Messages_Show("\r");
}

//---------------------------------------------------------------------//

/*void send_receive_ASCLIN_UART_message(void)
{
    IfxAsclin_Asc_write(&g_ascHandle, g_txData, &g_count, TIME_INFINITE);   // Transmit data via TX 
    IfxAsclin_Asc_read(&g_ascHandle, g_rxData, &g_count, TIME_INFINITE);    // Receive data via RX  
}*/




