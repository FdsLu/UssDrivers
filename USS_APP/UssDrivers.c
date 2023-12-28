/******************************************************************************
;       Program		:	UssDrivers.c
;       Function	:	USS (Ultrasonic Sensor System) Function
;       Chip		:	Infineon TC397
;       Clock		:	Internal Clock 300MHz
;       Date		:	2023 / 12 / 28
;       Author		:	Fenderson Lu
;       Describe	: 	USS_TX_1 = USS_IO_TX1 (P15.2)
;						USS_RX_1 = USS_IO_RX1 (P15.8)
;						USS_TX_2 = USS_IO_TX2 (P15.3)
;						USS_RX_2 = USS_IO_RX2 (P20.9)
;
;
;						USS RX data reception is MSB to LSB. 
******************************************************************************/
//---------------------------- Include File ---------------------------------//
#include "Cpu0_Main.h"
#include "PinIO.h"
#include "UssDrivers.h"
//---------------------------- Declare Global Variable ----------------------//
Uss_Thres_Data_t gtUssThresSetupPara[SIZE_USS_SENSOR];
Uss_Calib_Data_t gtUssCalibWritePara[SIZE_USS_SENSOR];
Uss_Rx_Ack_Data_t gtUssRxAckData[SIZE_USS_SENSOR];
uint32 gu32UssRxAckLen;
uint32 gu32DetectTimeLen;

uint8 gu8ThresSetupTxBuff[SIZE_THRES_SETUP];
uint8 gu8CalibWriteTxBuff[SIZE_CALIB_WRITE];

uint32 gu32InvertHighPulseTemp[SIZE_USS_RX_RAW];

uint32 gu32TimeTagTemp[SIZE_TIME_TAG];

uint32 gu32TxFilterLen;
uint32 gu32UssRxBitsTemp[SIZE_USS_RX];
boolean gbUssRxFinishFlag, gbUssRxAckEnFlag;
boolean gbAckRecFinishFlag = FALSE;



Uss_Exchange_Cmds gu8UssTxCmd;
Uss_Sensor_Id_t tUssSensorId;


Ifx_P *gtUssIoPort[SIZE_USS_SENSOR]={	portUSS_IO_TX1, portUSS_IO_TX2, portUSS_IO_TX3, portUSS_IO_TX4, portUSS_IO_TX5, 
										portUSS_IO_TX6, portUSS_IO_TX7, portUSS_IO_TX8, portUSS_IO_TX9, portUSS_IO_TX10,
										portUSS_IO_TX11, portUSS_IO_TX12};
uint8 gu8UssIoPin[SIZE_USS_SENSOR]={	pinUSS_IO_TX1, pinUSS_IO_TX2, pinUSS_IO_TX3, pinUSS_IO_TX4, pinUSS_IO_TX5,
										pinUSS_IO_TX6, pinUSS_IO_TX7, pinUSS_IO_TX8, pinUSS_IO_TX9, pinUSS_IO_TX10,
										pinUSS_IO_TX11, pinUSS_IO_TX12};

uint16 gu16SendMask;
//------------------- gu16SendMask -----------------------------//
// Bit15	: Reserved
// Bit14	: Reserved 
// Bit13	: Reserved
// Bit12	: Reserved
// Bit11	: 1 = USS rlc sensor is called send command. 
// Bit10	: 1 = USS rlm sensor is called send command. 
// Bit9		: 1 = USS rrm sensor is called send command. 
// Bit8		: 1 = USS rrc sensor is called send command. 
// Bit7		: 1 = USS frc sensor is called send command. 
// Bit6		: 1 = USS frm sensor is called send command. 
// Bit5		: 1 = USS flm sensor is called send command. 
// Bit4		: 1 = USS flc sensor is called send command. 
// Bit3		: 1 = USS rrs sensor is called send command. 
// Bit2		: 1 = USS frs sensor is called send command. 
// Bit1		: 1 = USS rls sensor is called send command. 
// Bit0		: 1 = USS fls sensor is called send command. 
//--------------------------------------------------------------//

uint16 gu16RecMask;
//------------------- gu16RecMask ------------------------------//
// Bit15	: Reserved
// Bit14	: Reserved 
// Bit13	: Reserved
// Bit12	: Reserved
// Bit11	: 1 = USS rlc sensor is called receive command. 
// Bit10	: 1 = USS rlm sensor is called receive command. 
// Bit9		: 1 = USS rrm sensor is called receive command. 
// Bit8		: 1 = USS rrc sensor is called receive command. 
// Bit7		: 1 = USS frc sensor is called receive command. 
// Bit6		: 1 = USS frm sensor is called receive command. 
// Bit5		: 1 = USS flm sensor is called receive command. 
// Bit4		: 1 = USS flc sensor is called receive command. 
// Bit3		: 1 = USS rrs sensor is called receive command. 
// Bit2		: 1 = USS frs sensor is called receive command. 
// Bit1		: 1 = USS rls sensor is called receive command. 
// Bit0		: 1 = USS fls sensor is called receive command. 
//--------------------------------------------------------------//





#if EVB_DEMO
uint8 gu8LowPulseTemp[SIZE_USS_RX_RAW];
#endif
//---------------------------- Start Program --------------------------------//
void UssDrivers_Init(void)
{	
	// Default Values
	for(uint32 u32SensorIndex=USS_ID_IO1_TXRX_FLC; u32SensorIndex<USS_ID_IO12_TXRX_RRS; u32SensorIndex++)
	{
		gtUssThresSetupPara[u32SensorIndex].u8Thresscale_Rec = 0x01;
		gtUssThresSetupPara[u32SensorIndex].u8Atg_Alpha = 0x00;
		gtUssThresSetupPara[u32SensorIndex].u8Atg_Tau = 0x01;
		gtUssThresSetupPara[u32SensorIndex].u8Atg_Cfg = 0x03;
		gtUssThresSetupPara[u32SensorIndex].u8Thsft_Cfg = 0x00;
		gtUssThresSetupPara[u32SensorIndex].u8Thval[NUM_THVAL_2] = 0x0F; 
		gtUssThresSetupPara[u32SensorIndex].u8Thval[NUM_THVAL_3] = 0x0F;
		gtUssThresSetupPara[u32SensorIndex].u8Thval[NUM_THVAL_4] = 0x0F;
		gtUssThresSetupPara[u32SensorIndex].u8Thval[NUM_THVAL_5] = 0x0F;
		gtUssThresSetupPara[u32SensorIndex].u8Thval[NUM_THVAL_6] = 0x0A;
		gtUssThresSetupPara[u32SensorIndex].u8Thval[NUM_THVAL_7] = 0x0A;
		gtUssThresSetupPara[u32SensorIndex].u8Thval[NUM_THVAL_8] = 0x05;
		gtUssThresSetupPara[u32SensorIndex].u8Thval[NUM_THVAL_9] = 0x05;
		gtUssThresSetupPara[u32SensorIndex].u8Thval[NUM_THVAL_10] = 0x00;
		gtUssThresSetupPara[u32SensorIndex].u8Thval[NUM_THVAL_11] = 0x00;
		gtUssThresSetupPara[u32SensorIndex].u8Thval[NUM_THVAL_12] = 0x00;
		gtUssThresSetupPara[u32SensorIndex].u8Thval[NUM_THVAL_13] = 0x00;
	}	

	gtUssCalibWritePara[0].u8F_Drv = 0x02;
	gtUssCalibWritePara[0].u8I_Drv = 0x02;
	gtUssCalibWritePara[0].u8G_Ana = 0x02;
	gtUssCalibWritePara[0].u8G_Dig = 0x02;
	gtUssCalibWritePara[0].u8Customer_Bits = 0x02;
	gtUssCalibWritePara[0].u8Osc_Trim = 0x02;
}
//---------------------------------------------------------------------------//
uint8 UssDrivers_ParityBit_Calculate(ParityChkMode_t tMode, uint32 u32Values)
{
    uint32 u32Parity;

	if(tMode == MODE_PC_ODD)
	{
		u32Parity = MODE_PC_ODD;
	}
	else
	{
		u32Parity = MODE_PC_EVEN;
	}
	
    while(u32Values != 0) 
	{
        u32Parity ^= u32Values;
        u32Values >>= SHIFT_BIT_1;
    }
	
    return (u32Parity & PARITY_BIT);
}
//---------------------------------------------------------------------------//
void UssDrivers_IO_Symbol_Signal(Uss_Sensor_Id_t tSensorMask, uint8 u8Symbol)		
{	
	switch(u8Symbol)
	{	
		// MCU output signals need invert for TDCU HW design
		case IO_SYM_T_D:
			#if EVB_DEMO
			IfxPort_setPinHigh(gtUssIoPort[tSensorMask], gu8UssIoPin[tSensorMask]);
			#else
			IfxPort_setPinLow(gtUssIoPort[tSensorMask], gu8UssIoPin[tSensorMask]);
			#endif
			Common_Delay(UNIT_MICRO, (uint32)(SYM_TIME_T_D * TIME_GAP));		
		break;
		case IO_SYM_T_SND:
			#if EVB_DEMO
			IfxPort_setPinLow(gtUssIoPort[tSensorMask], gu8UssIoPin[tSensorMask]);
			#else
			IfxPort_setPinHigh(gtUssIoPort[tSensorMask], gu8UssIoPin[tSensorMask]);
			#endif
			Common_Delay(UNIT_MICRO, (uint32)(SYM_TIME_T_SND * TIME_GAP));		
		break;
		case IO_SYM_T_REC:
			#if EVB_DEMO
			IfxPort_setPinLow(gtUssIoPort[tSensorMask], gu8UssIoPin[tSensorMask]);
			#else
			IfxPort_setPinHigh(gtUssIoPort[tSensorMask], gu8UssIoPin[tSensorMask]);
			#endif
			Common_Delay(UNIT_MICRO, (uint32)(SYM_TIME_T_REC * TIME_GAP));		
		break;
		case IO_SYM_T_MEAS:
			#if EVB_DEMO
			IfxPort_setPinLow(gtUssIoPort[tSensorMask], gu8UssIoPin[tSensorMask]);
			#else
			IfxPort_setPinHigh(gtUssIoPort[tSensorMask], gu8UssIoPin[tSensorMask]);
			#endif
			Common_Delay(UNIT_MICRO, (uint32)(SYM_TIME_T_MEAS * TIME_GAP));	
		break;	
		case IO_SYM_T_CMD:
			#if EVB_DEMO
			IfxPort_setPinLow(gtUssIoPort[tSensorMask], gu8UssIoPin[tSensorMask]);
			#else
			IfxPort_setPinHigh(gtUssIoPort[tSensorMask], gu8UssIoPin[tSensorMask]);
			#endif
			Common_Delay(UNIT_MICRO, (uint32)(SYM_TIME_T_CMD * TIME_GAP));		
		break;
		case IO_SYM_T_BIT0_LOG_0:
			#if EVB_DEMO
			IfxPort_setPinLow(gtUssIoPort[tSensorMask], gu8UssIoPin[tSensorMask]);
			Common_Delay(UNIT_MICRO, (uint32)(SYM_TIME_T_BIT0 * TIME_GAP));		
			IfxPort_setPinHigh(gtUssIoPort[tSensorMask], gu8UssIoPin[tSensorMask]);					
			Common_Delay(UNIT_MICRO, (uint32)((SYM_TIME_T_BIT - SYM_TIME_T_BIT0) * TIME_GAP));			
			#else
			IfxPort_setPinHigh(gtUssIoPort[tSensorMask], gu8UssIoPin[tSensorMask]);
			Common_Delay(UNIT_MICRO, (uint32)(SYM_TIME_T_BIT0 * TIME_GAP));		
			IfxPort_setPinLow(gtUssIoPort[tSensorMask], gu8UssIoPin[tSensorMask]);					
			Common_Delay(UNIT_MICRO, (uint32)((SYM_TIME_T_BIT - SYM_TIME_T_BIT0) * TIME_GAP));				
			#endif
		break;
		case IO_SYM_T_BIT1_LOG_1:
			#if EVB_DEMO
			IfxPort_setPinLow(gtUssIoPort[tSensorMask], gu8UssIoPin[tSensorMask]);
			Common_Delay(UNIT_MICRO, (uint32)(SYM_TIME_T_BIT1 * TIME_GAP));								
			IfxPort_setPinHigh(gtUssIoPort[tSensorMask], gu8UssIoPin[tSensorMask]);	
			Common_Delay(UNIT_MICRO, (uint32)((SYM_TIME_T_BIT - SYM_TIME_T_BIT1) * TIME_GAP));	
			#else
			IfxPort_setPinHigh(gtUssIoPort[tSensorMask], gu8UssIoPin[tSensorMask]);
			Common_Delay(UNIT_MICRO, (uint32)(SYM_TIME_T_BIT1 * TIME_GAP));								
			IfxPort_setPinLow(gtUssIoPort[tSensorMask], gu8UssIoPin[tSensorMask]);	
			Common_Delay(UNIT_MICRO, (uint32)((SYM_TIME_T_BIT - SYM_TIME_T_BIT1) * TIME_GAP));		
			#endif
		break;		
	}
}
//---------------------------------------------------------------------------//
void UssDrivers_Cmds_Transmit(Uss_Sensor_Id_t tSensorMask, Uss_Exchange_Cmds u8Cmd)
{
	uint32 u32TxIndex = VAL_INIT, u32MaskIndex = VAL_INIT, u32TsByteIndex = VAL_INIT;

	gu8UssTxCmd = u8Cmd;
	tUssSensorId = tSensorMask;

	switch(u8Cmd)
	{
		case EX_CMDS_SEND_A:		// ACK
			#if DEBUG_UART
				DEBUG_MSG([UssTx] SEND_A);
			#endif
			
			gbUssRxAckEnFlag = TRUE;			
			
			#if EVB_DEMO
			gu32DetectTimeLen = 2320;						// 2ms for test
			gu32UssRxAckLen = ACK_BITS_LEN_SEND_A;		
			#else			
			gu32DetectTimeLen = 2320;						//FDS: need link to API?????
			gu32UssRxAckLen = ACK_BITS_LEN_SEND_A;
			#endif

			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_SND);

			#if EVB_DEMO
			// Sensor ACK				
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_D);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);			
			#endif

			
		break;
		case EX_CMDS_RECEIVE_A:		// ACK
			#if DEBUG_UART
				DEBUG_MSG([UssTx] RECEIVE_A);
			#endif
			
			gbUssRxAckEnFlag = TRUE;

			#if EVB_DEMO
			gu32DetectTimeLen = 2320;						// 2ms for test
			gu32UssRxAckLen = ACK_BITS_LEN_RECEIVE_A;		
			#else			
			gu32DetectTimeLen = 2320;						//FDS: need link to API?????
			gu32UssRxAckLen = ACK_BITS_LEN_RECEIVE_A;
			#endif		
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_REC);			
		break;
		case EX_CMDS_SEND_B:		// ACK
			#if DEBUG_UART
				DEBUG_MSG([UssTx] SEND_B);
			#endif
			
			gbUssRxAckEnFlag = TRUE;

			#if EVB_DEMO
			gu32DetectTimeLen = 2320;						// 2ms for test
			gu32UssRxAckLen = ACK_BITS_LEN_SEND_B;		
			#else			
			gu32DetectTimeLen = 2320;						//FDS: need link to API?????
			gu32UssRxAckLen = ACK_BITS_LEN_SEND_B;
			#endif	

			
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_MEAS);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_D);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);
		break;
		case EX_CMDS_RECEIVE_B:		// ACK
			#if DEBUG_UART
				DEBUG_MSG([UssTx] RECEIVE_B);
			#endif
			
			gbUssRxAckEnFlag = TRUE;

			#if EVB_DEMO
			gu32DetectTimeLen = 2320;						// 2ms for test
			gu32UssRxAckLen = ACK_BITS_LEN_RECEIVE_B;		
			#else			
			gu32DetectTimeLen = 2320;						//FDS: need link to API?????
			gu32UssRxAckLen = ACK_BITS_LEN_RECEIVE_B;
			#endif

			
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_MEAS);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_D);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);			
		break;	
		case EX_CMDS_SEND_C:		// ACK	
			#if DEBUG_UART
				DEBUG_MSG([UssTx] SEND_C);
			#endif
			
			gbUssRxAckEnFlag = TRUE;


			#if EVB_DEMO
			gu32DetectTimeLen = 2320;						// 2ms for test
			gu32UssRxAckLen = ACK_BITS_LEN_SEND_C;		
			#else			
			gu32DetectTimeLen = 2320;						//FDS: need link to API?????
			gu32UssRxAckLen = ACK_BITS_LEN_SEND_C;
			#endif
			
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_MEAS);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_D);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);			
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);			
		break;
		case EX_CMDS_RECEIVE_C:		// ACK
			#if DEBUG_UART
				DEBUG_MSG([UssTx] RECEIVE_C);
			#endif
			
			gbUssRxAckEnFlag = TRUE;


			#if EVB_DEMO
			gu32DetectTimeLen = 2320;						// 2ms for test
			gu32UssRxAckLen = ACK_BITS_LEN_RECEIVE_C;		
			#else			
			gu32DetectTimeLen = 2320;						//FDS: need link to API?????
			gu32UssRxAckLen = ACK_BITS_LEN_RECEIVE_C;
			#endif

			
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_MEAS);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_D);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);			
		break;
		case EX_CMDS_THRES_SETUP:
			#if DEBUG_UART
				DEBUG_MSG([UssTx] THRES_SETUP);
			#endif
			
			u32TxIndex = LEN_THRES_SETUP_BITS;
			u32MaskIndex = SHIFT_BIT_6;
		
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_CMD);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_D);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);

			while(u32TxIndex--)		// MSB to LSB
			{
				if((gu8ThresSetupTxBuff[TS_BYTE9 - u32TsByteIndex] & (REG_BIT_7 >> u32MaskIndex)) == REG_BIT_7 >> u32MaskIndex)
				{
					// Send logic '1' signal
					UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);			
				}
				else
				{
					// Send logic '0' signal
					UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);	
				}

				if(u32TxIndex % UNIT_A_U8)
				{
					u32MaskIndex++;
				}
				else
				{					
					u32TsByteIndex++;
					u32MaskIndex = VAL_INIT;
				}
			}	
		break;
		case EX_CMDS_READ_THRES_SETUP:		// ACK
			#if DEBUG_UART
				DEBUG_MSG([UssTx] READ_THRES_SETUP);
			#endif
			
			gbUssRxAckEnFlag = TRUE;
			gu32TxFilterLen = LEN_FILTER_READ_THRES_SETUP;
			gu32UssRxAckLen = gu32TxFilterLen + ACK_BITS_LEN_READ_THRES_SETUP;
			
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_CMD);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_D);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);	
		break;
		case EX_CMDS_MEAS_SETUP:		
			// Reserved		
		break;
		case EX_CMDS_READ_MEAS_SETUP:		// ACK	
			#if DEBUG_UART
				DEBUG_MSG([UssTx] READ_MEAS_SETUP);
			#endif
			
			gbUssRxAckEnFlag = TRUE;
			gu32TxFilterLen = LEN_FILTER_READ_MEAS_SETUP;
			gu32UssRxAckLen = gu32TxFilterLen + ACK_BITS_LEN_READ_MEAS_SETUP;
			
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_CMD);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_D);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);	
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
		break;
		case EX_CMDS_READ_STATUS:			// ACK	
			#if DEBUG_UART
				DEBUG_MSG([UssTx] READ_STATUS);
			#endif
			
			gbUssRxAckEnFlag = TRUE;
			gu32TxFilterLen = LEN_FILTER_READ_STATUS;
			gu32UssRxAckLen = gu32TxFilterLen + ACK_BITS_LEN_READ_STATUS;
			
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_CMD);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_D);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);	

			#if EVB_DEMO
			// Sensor ACK
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);
			#endif		
			
		break;
		case EX_CMDS_CAL_PULSES:
			#if DEBUG_UART
				DEBUG_MSG([UssTx] CAL_PULSES);
			#endif
			
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_CMD);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_D);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);		
		break;
		case EX_CMDS_READ_TEMP:			// ACK
			#if DEBUG_UART
				DEBUG_MSG([UssTx] READ_TEMP);
			#endif
			
			gbUssRxAckEnFlag = TRUE;
			gu32TxFilterLen = LEN_FILTER_READ_TEMP;
			gu32UssRxAckLen = gu32TxFilterLen + ACK_BITS_LEN_READ_TEMP;
			
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_CMD);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_D);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);	
		break;
		case EX_CMDS_ENVELOPE_SEND_A:
			#if DEBUG_UART
				DEBUG_MSG([UssTx] ENVELOPE_SEND_A);
			#endif
			
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_CMD);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_D);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_D);
		break;
		case EX_CMDS_ENVELOPE_REC_A:
			#if DEBUG_UART
				DEBUG_MSG([UssTx] ENVELOPE_REC_A);
			#endif
			
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_CMD);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_D);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_D);
		break;
		case EX_CMDS_CALIB_WRITE:
			#if DEBUG_UART
				DEBUG_MSG([UssTx] CALIB_WRITE);
			#endif
			
			u32TxIndex = LEN_CALIB_WRITE_BITS;
			u32MaskIndex = SHIFT_BIT_6;
		
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_CMD);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_D);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);

			while(u32TxIndex--)		// MSB to LSB
			{
				if((gu8CalibWriteTxBuff[CW_BYTE4 - u32TsByteIndex] & (REG_BIT_7 >> u32MaskIndex)) == REG_BIT_7 >> u32MaskIndex)
				{
					// Send logic '1' signal
					UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);			
				}
				else
				{
					// Send logic '0' signal
					UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);	
				}

				if(u32TxIndex % UNIT_A_U8)
				{
					u32MaskIndex++;
				}
				else
				{					
					u32TsByteIndex++;
					u32MaskIndex = VAL_INIT;
				}
			}		
		break;
		case EX_CMDS_CALIB_READ:	// ACK
			#if DEBUG_UART
				DEBUG_MSG([UssTx] CALIB_READ);
			#endif
			
			gbUssRxAckEnFlag = TRUE;
			gu32TxFilterLen = LEN_FILTER_CALIB_READ;
			gu32UssRxAckLen = gu32TxFilterLen + ACK_BITS_LEN_CALIB_READ;
			
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_CMD);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_D);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);	
		break;
		case EX_CMDS_EE_COPY:	
			#if DEBUG_UART
				DEBUG_MSG([UssTx] EE_COPY);
			#endif
			
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_CMD);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_D);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);		
		break;
		case EX_CMDS_EE_READ:		// ACK
			#if DEBUG_UART
				DEBUG_MSG([UssTx] EE_READ);
			#endif
			
			gbUssRxAckEnFlag = TRUE;
			gu32TxFilterLen = LEN_FILTER_EE_READ;
			gu32UssRxAckLen = gu32TxFilterLen + ACK_BITS_LEN_EE_READ;
			
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_CMD);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_D);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
		break;
		case EX_CMDS_READ_ID:		// ACK
			#if DEBUG_UART
				DEBUG_MSG([UssTx] READ_ID);
			#endif
			
			gbUssRxAckEnFlag = TRUE;
			gu32TxFilterLen = LEN_FILTER_READ_ID;
			gu32UssRxAckLen = gu32TxFilterLen + ACK_BITS_LEN_READ_ID;
			
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_CMD);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_D);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);	
		break;
		case EX_CMDS_STANDBY:
			#if DEBUG_UART
				DEBUG_MSG([UssTx] STANDBY);
			#endif
			
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_CMD);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_D);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);	
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
		break;
		case EX_CMDS_WAKE_UP:
			#if DEBUG_UART
				DEBUG_MSG([UssTx] WAKE_UP);
			#endif
			
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_CMD);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_D);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);			
		break;		
	}
}
//---------------------------------------------------------------------------//
static uint8 UssDrivers_Parity_Check(ParityChk_Num_t tParityNum)
{
	uint16 u16UssParityBitCal = INIT_PARITY_BITS;

	switch(tParityNum)
	{
		case PC_PARITY_0:		// Bit 5 ~ Bit 18
			u16UssParityBitCal |= (gu8ThresSetupTxBuff[TS_BYTE0] & (MASK_B0_THRES_SCALE_REC_L | MASK_B0_THRES_SCALE_REC_H | MASK_B0_ATG_ALPHA)) >> SHIFT_BIT_5;
		  	u16UssParityBitCal |= (gu8ThresSetupTxBuff[TS_BYTE1] & (MASK_B1_ATG_TAU_L | MASK_B1_ATG_TAU_H | MASK_B1_ATG_CFG_L | MASK_B1_ATG_CFG_H | 
		  															MASK_B1_THSFT_CFG_L | MASK_B1_THSFT_CFG_H | MASK_B1_THVAL13_BIT_0 | MASK_B1_THVAL13_BIT_1)) << SHIFT_BIT_3;
			u16UssParityBitCal |= (gu8ThresSetupTxBuff[TS_BYTE2] & (MASK_B2_THVAL13_BIT_2 | MASK_B2_THVAL13_BIT_3 | MASK_B2_THVAL13_BIT_4)) << SHIFT_BIT_11;
		break;
		case PC_PARITY_1:		// Bit 19 ~ Bit 32
			u16UssParityBitCal |= (gu8ThresSetupTxBuff[TS_BYTE2] & (MASK_B2_THVAL12_BIT_0 | MASK_B2_THVAL12_BIT_1 | MASK_B2_THVAL12_BIT_2 | MASK_B2_THVAL12_BIT_3 |
																	MASK_B2_THVAL12_BIT_4)) >> SHIFT_BIT_3;
			u16UssParityBitCal |= (gu8ThresSetupTxBuff[TS_BYTE3] & (MASK_B3_THVAL11_BIT_0 | MASK_B3_THVAL11_BIT_1 | MASK_B3_THVAL11_BIT_2 | MASK_B3_THVAL11_BIT_3 |
																	MASK_B3_THVAL11_BIT_4 | MASK_B3_THVAL10_BIT_0 | MASK_B3_THVAL10_BIT_1 | MASK_B3_THVAL10_BIT_2)) << SHIFT_BIT_5;
			u16UssParityBitCal |= (gu8ThresSetupTxBuff[TS_BYTE4] & MASK_B4_THVAL10_BIT_3) << SHIFT_BIT_13;
		break;
		case PC_PARITY_2:		// Bit 33 ~ Bit 46
			u16UssParityBitCal |= (gu8ThresSetupTxBuff[TS_BYTE4] & (MASK_B4_THVAL10_BIT_4 | MASK_B4_THVAL9_BIT_0 | MASK_B4_THVAL9_BIT_1 | MASK_B4_THVAL9_BIT_2 |
																	MASK_B4_THVAL9_BIT_3 | MASK_B4_THVAL9_BIT_4 | MASK_B4_THVAL8_BIT_0)) >> SHIFT_BIT_1;
			u16UssParityBitCal |= (gu8ThresSetupTxBuff[TS_BYTE5] & (MASK_B5_THVAL8_BIT_1 | MASK_B5_THVAL8_BIT_2 | MASK_B5_THVAL8_BIT_3 | MASK_B5_THVAL8_BIT_4 |
																	MASK_B5_THVAL7_BIT_0 | MASK_B5_THVAL7_BIT_1 | MASK_B5_THVAL7_BIT_2)) << SHIFT_BIT_1;
		break;
		case PC_PARITY_3:		// Bit 47) ~ Bit 60
			u16UssParityBitCal |= (gu8ThresSetupTxBuff[TS_BYTE5] & MASK_B5_THVAL7_BIT_3) >> SHIFT_BIT_7;
			u16UssParityBitCal |= (gu8ThresSetupTxBuff[TS_BYTE6] & (MASK_B6_THVAL7_BIT_4 | MASK_B6_THVAL6_BIT_0 | MASK_B6_THVAL6_BIT_1 | MASK_B6_THVAL6_BIT_2 |
																	MASK_B6_THVAL6_BIT_3 | MASK_B6_THVAL6_BIT_4 | MASK_B6_THVAL5_BIT_0 | MASK_B6_THVAL5_BIT_1)) << SHIFT_BIT_7;
			u16UssParityBitCal |= (gu8ThresSetupTxBuff[TS_BYTE7] & (MASK_B7_THVAL5_BIT_2 | MASK_B7_THVAL5_BIT_3 | MASK_B7_THVAL5_BIT_4 | MASK_B7_THVAL4_BIT_0 |
																	MASK_B7_THVAL4_BIT_1)) << SHIFT_BIT_9;
		break;
		case PC_PARITY_4:		// Bit 61 ~ Bit 73
			u16UssParityBitCal |= (gu8ThresSetupTxBuff[TS_BYTE7] & (MASK_B7_THVAL4_BIT_2 | MASK_B7_THVAL4_BIT_3 | MASK_B7_THVAL4_BIT_4)) >> SHIFT_BIT_5;
			u16UssParityBitCal |= (gu8ThresSetupTxBuff[TS_BYTE8] & (MASK_B8_THVAL3_BIT_0 | MASK_B8_THVAL3_BIT_1 | MASK_B8_THVAL3_BIT_2 | MASK_B8_THVAL3_BIT_3 |
																	MASK_B8_THVAL3_BIT_4 | MASK_B8_THVAL2_BIT_0 | MASK_B8_THVAL2_BIT_1 | MASK_B8_THVAL2_BIT_2)) << SHIFT_BIT_5;
			u16UssParityBitCal |= (gu8ThresSetupTxBuff[TS_BYTE9] & (MASK_B9_THVAL2_BIT_3 | MASK_B9_THVAL2_BIT_4)) << SHIFT_BIT_11;
		break;
	}

	return UssDrivers_ParityBit_Calculate(MODE_PC_EVEN, u16UssParityBitCal);
}
//---------------------------------------------------------------------------//
Func_Status_t UssDrivers_ThresSetup_Para_Write(Uss_Sensor_Id_t tSensorMask, Uss_Thres_Data_t *tThresSetupPara)
{
	// Set Thresscale_Rec config
	gu8ThresSetupTxBuff[TS_BYTE0] = (gu8ThresSetupTxBuff[TS_BYTE0] & (~(MASK_B0_THRES_SCALE_REC_L | MASK_B0_THRES_SCALE_REC_H))) | ((tThresSetupPara->u8Thresscale_Rec & MASK_DATA_THRES_SCALE_REC) << SHIFT_BIT_5);

	// Set Atg_Alpha config
	gu8ThresSetupTxBuff[TS_BYTE0] = (gu8ThresSetupTxBuff[TS_BYTE0] & (~MASK_B0_ATG_ALPHA)) | ((tThresSetupPara->u8Atg_Alpha & MASK_DATA_ATG_ALPHA) << SHIFT_BIT_7);
	
	// Set Atg_Tau config
	gu8ThresSetupTxBuff[TS_BYTE1] = (gu8ThresSetupTxBuff[TS_BYTE1] & (~(MASK_B1_ATG_TAU_L | MASK_B1_ATG_TAU_H))) | (tThresSetupPara->u8Atg_Tau & MASK_DATA_ATG_TAU);

	// Set Atg_Cfg config
	gu8ThresSetupTxBuff[TS_BYTE1] = (gu8ThresSetupTxBuff[TS_BYTE1] & (~(MASK_B1_ATG_CFG_L | MASK_B1_ATG_CFG_H))) | ((tThresSetupPara->u8Atg_Cfg & MASK_DATA_ATG_CFG) << SHIFT_BIT_2);

	// Set Thsft_Cfg config
	gu8ThresSetupTxBuff[TS_BYTE1] = (gu8ThresSetupTxBuff[TS_BYTE1] & (~(MASK_B1_THSFT_CFG_L | MASK_B1_THSFT_CFG_H))) | ((tThresSetupPara->u8Thsft_Cfg & MASK_DATA_THSFT_CFG) << SHIFT_BIT_4);

	// Set THVAL13 config, thval[11] = THVAL13
	gu8ThresSetupTxBuff[TS_BYTE1] = (gu8ThresSetupTxBuff[TS_BYTE1] & (~(MASK_B1_THVAL13_BIT_0 | MASK_B1_THVAL13_BIT_1))) | ((tThresSetupPara->u8Thval[NUM_THVAL_13] & MASK_DATA_THVAL) << SHIFT_BIT_6);
	gu8ThresSetupTxBuff[TS_BYTE2] = (gu8ThresSetupTxBuff[TS_BYTE2] & (~(MASK_B2_THVAL13_BIT_2 | MASK_B2_THVAL13_BIT_3 | MASK_B2_THVAL13_BIT_4))) | ((tThresSetupPara->u8Thval[NUM_THVAL_13] & MASK_DATA_THVAL) >> SHIFT_BIT_2);
	
	// Set THVAL12 config, thval[10] = THVAL12
	gu8ThresSetupTxBuff[TS_BYTE2] = (gu8ThresSetupTxBuff[TS_BYTE2] & (~(MASK_B2_THVAL12_BIT_0 | MASK_B2_THVAL12_BIT_1 | MASK_B2_THVAL12_BIT_2 | MASK_B2_THVAL12_BIT_3 | MASK_B2_THVAL12_BIT_4))) | ((tThresSetupPara->u8Thval[NUM_THVAL_12] & MASK_DATA_THVAL) << SHIFT_BIT_3);
	
	// Set THVAL11 config, thval[9] = THVAL11
	gu8ThresSetupTxBuff[TS_BYTE3] = (gu8ThresSetupTxBuff[TS_BYTE3] & (~(MASK_B3_THVAL11_BIT_0 | MASK_B3_THVAL11_BIT_1 | MASK_B3_THVAL11_BIT_2 | MASK_B3_THVAL11_BIT_3 | MASK_B3_THVAL11_BIT_4))) | (tThresSetupPara->u8Thval[NUM_THVAL_11] & MASK_DATA_THVAL);

	// Set THVAL10 config
	gu8ThresSetupTxBuff[TS_BYTE3] = (gu8ThresSetupTxBuff[TS_BYTE3] & (~(MASK_B3_THVAL10_BIT_0 | MASK_B3_THVAL10_BIT_1 | MASK_B3_THVAL10_BIT_2))) | ((tThresSetupPara->u8Thval[NUM_THVAL_10] & MASK_DATA_THVAL) << SHIFT_BIT_5);
	gu8ThresSetupTxBuff[TS_BYTE4] = (gu8ThresSetupTxBuff[TS_BYTE4] & (~(MASK_B4_THVAL10_BIT_3 | MASK_B4_THVAL10_BIT_4))) | ((tThresSetupPara->u8Thval[NUM_THVAL_10] & MASK_DATA_THVAL) >> SHIFT_BIT_3);

	// Set THVAL9 config
	gu8ThresSetupTxBuff[TS_BYTE4] = (gu8ThresSetupTxBuff[TS_BYTE4] & (~(MASK_B4_THVAL9_BIT_0 | MASK_B4_THVAL9_BIT_1 | MASK_B4_THVAL9_BIT_2 | MASK_B4_THVAL9_BIT_3 | MASK_B4_THVAL9_BIT_4))) | ((tThresSetupPara->u8Thval[NUM_THVAL_9] & MASK_DATA_THVAL) << SHIFT_BIT_2);

	// Set THVAL8 config
	gu8ThresSetupTxBuff[TS_BYTE4] = (gu8ThresSetupTxBuff[TS_BYTE4] & (~MASK_B4_THVAL8_BIT_0)) | ((tThresSetupPara->u8Thval[NUM_THVAL_8] & MASK_DATA_THVAL) << SHIFT_BIT_7);
	gu8ThresSetupTxBuff[TS_BYTE5] = (gu8ThresSetupTxBuff[TS_BYTE5] & (~(MASK_B5_THVAL8_BIT_1 | MASK_B5_THVAL8_BIT_2 | MASK_B5_THVAL8_BIT_3 | MASK_B5_THVAL8_BIT_4))) | ((tThresSetupPara->u8Thval[NUM_THVAL_8] & MASK_DATA_THVAL) >> SHIFT_BIT_1);

	// Set THVAL7 config
	gu8ThresSetupTxBuff[TS_BYTE5] = (gu8ThresSetupTxBuff[TS_BYTE5] & (~(MASK_B5_THVAL7_BIT_0 | MASK_B5_THVAL7_BIT_1 | MASK_B5_THVAL7_BIT_2 | MASK_B5_THVAL7_BIT_3))) | ((tThresSetupPara->u8Thval[NUM_THVAL_7] & MASK_DATA_THVAL) << SHIFT_BIT_4);
	gu8ThresSetupTxBuff[TS_BYTE6] = (gu8ThresSetupTxBuff[TS_BYTE6] & (~MASK_B6_THVAL7_BIT_4)) | ((tThresSetupPara->u8Thval[NUM_THVAL_7] & MASK_DATA_THVAL) >> SHIFT_BIT_4);

	// Set THVAL6 config
	gu8ThresSetupTxBuff[TS_BYTE6] = (gu8ThresSetupTxBuff[TS_BYTE6] & (~(MASK_B6_THVAL6_BIT_0 | MASK_B6_THVAL6_BIT_1 | MASK_B6_THVAL6_BIT_2 | MASK_B6_THVAL6_BIT_3 | MASK_B6_THVAL6_BIT_4))) | ((tThresSetupPara->u8Thval[NUM_THVAL_6] & MASK_DATA_THVAL) << SHIFT_BIT_1);

	// Set THVAL5 config
	gu8ThresSetupTxBuff[TS_BYTE6] = (gu8ThresSetupTxBuff[TS_BYTE6] & (~(MASK_B6_THVAL5_BIT_0 | MASK_B6_THVAL5_BIT_1))) | ((tThresSetupPara->u8Thval[NUM_THVAL_5] & MASK_DATA_THVAL) << SHIFT_BIT_6);
	gu8ThresSetupTxBuff[TS_BYTE7] = (gu8ThresSetupTxBuff[TS_BYTE7] & (~(MASK_B7_THVAL5_BIT_2 | MASK_B7_THVAL5_BIT_3 | MASK_B7_THVAL5_BIT_4))) | ((tThresSetupPara->u8Thval[NUM_THVAL_5] & MASK_DATA_THVAL) >> SHIFT_BIT_2);

	// Set THVAL4 config
	gu8ThresSetupTxBuff[TS_BYTE7] = (gu8ThresSetupTxBuff[TS_BYTE7] & (~(MASK_B7_THVAL4_BIT_0 | MASK_B7_THVAL4_BIT_1 | MASK_B7_THVAL4_BIT_2 | MASK_B7_THVAL4_BIT_3 | MASK_B7_THVAL4_BIT_4))) | ((tThresSetupPara->u8Thval[NUM_THVAL_4] & MASK_DATA_THVAL) << SHIFT_BIT_3);

	// Set THVAL3 config
	gu8ThresSetupTxBuff[TS_BYTE8] = (gu8ThresSetupTxBuff[TS_BYTE8] & (~(MASK_B8_THVAL3_BIT_0 | MASK_B8_THVAL3_BIT_1 | MASK_B8_THVAL3_BIT_2 | MASK_B8_THVAL3_BIT_3 | MASK_B8_THVAL3_BIT_4))) | (tThresSetupPara->u8Thval[NUM_THVAL_3] & MASK_DATA_THVAL);

	// Set THVAL2 config, thval[0] = THVAL2
	gu8ThresSetupTxBuff[TS_BYTE8] = (gu8ThresSetupTxBuff[TS_BYTE8] & (~(MASK_B8_THVAL2_BIT_0 | MASK_B8_THVAL2_BIT_1 | MASK_B8_THVAL2_BIT_2))) | ((tThresSetupPara->u8Thval[NUM_THVAL_2] & MASK_DATA_THVAL) << SHIFT_BIT_5);
	gu8ThresSetupTxBuff[TS_BYTE9] = (gu8ThresSetupTxBuff[TS_BYTE9] & (~(MASK_B9_THVAL2_BIT_3 | MASK_B9_THVAL2_BIT_4))) | ((tThresSetupPara->u8Thval[NUM_THVAL_2] & MASK_DATA_THVAL) >> SHIFT_BIT_3);

	// Calculate parity bit 0
	gu8ThresSetupTxBuff[TS_BYTE0] = (gu8ThresSetupTxBuff[TS_BYTE0] & (~MASK_B0_PARITY_0)) | (UssDrivers_Parity_Check(PC_PARITY_0) << SHIFT_BIT_4);

	// Calculate parity bit 1
	gu8ThresSetupTxBuff[TS_BYTE0] = (gu8ThresSetupTxBuff[TS_BYTE0] & (~MASK_B0_PARITY_1)) | (UssDrivers_Parity_Check(PC_PARITY_1) << SHIFT_BIT_3);
	
	// Calculate parity bit 2
	gu8ThresSetupTxBuff[TS_BYTE0] = (gu8ThresSetupTxBuff[TS_BYTE0] & (~MASK_B0_PARITY_2)) | (UssDrivers_Parity_Check(PC_PARITY_2) << SHIFT_BIT_2);

	// Calculate parity bit 3
	gu8ThresSetupTxBuff[TS_BYTE0] = (gu8ThresSetupTxBuff[TS_BYTE0] & (~MASK_B0_PARITY_3)) | (UssDrivers_Parity_Check(PC_PARITY_3) << SHIFT_BIT_1);

	// Calculate parity bit 4
	gu8ThresSetupTxBuff[TS_BYTE0] = (gu8ThresSetupTxBuff[TS_BYTE0] & (~MASK_B0_PARITY_4)) | UssDrivers_Parity_Check(PC_PARITY_4);

	// Send parameters to sensor
	UssDrivers_Cmds_Transmit(tSensorMask, EX_CMDS_THRES_SETUP);	

    return FUNC_SUCCESS;		
}
//---------------------------------------------------------------------------//
Func_Status_t UssDrivers_Calib_Write(Uss_Sensor_Id_t tSensorMask, Uss_Calib_Data_t *tCalibWritePara)
{
	// Set F_DRV config
	gu8CalibWriteTxBuff[CW_BYTE0] = (gu8CalibWriteTxBuff[CW_BYTE0] & (~(MASK_B0_F_DRV_BIT_0 | MASK_B0_F_DRV_BIT_1 | MASK_B0_F_DRV_BIT_2 | MASK_B0_F_DRV_BIT_3 |
																		MASK_B0_F_DRV_BIT_4 | MASK_B0_F_DRV_BIT_5 | MASK_B0_F_DRV_BIT_6 | MASK_B0_F_DRV_BIT_7))) | (gtUssCalibWritePara->u8F_Drv & MASK_DATA_F_DRV);

	// Set I_DRV config
	gu8CalibWriteTxBuff[CW_BYTE1] = (gu8CalibWriteTxBuff[CW_BYTE1] & (~(MASK_B1_I_DRV_BIT_0 | MASK_B1_I_DRV_BIT_1 | MASK_B1_I_DRV_BIT_2 | MASK_B1_I_DRV_BIT_3 | 
																		MASK_B1_I_DRV_BIT_4))) | (gtUssCalibWritePara->u8I_Drv & MASK_DATA_I_DRV);

	// Set G_ANA config
	gu8CalibWriteTxBuff[CW_BYTE1] = (gu8CalibWriteTxBuff[CW_BYTE1] & (~(MASK_B1_G_ANA_BIT_0 | MASK_B1_G_ANA_BIT_1 | MASK_B1_G_ANA_BIT_2))) | ((gtUssCalibWritePara->u8G_Ana & MASK_DATA_G_ANA) << SHIFT_BIT_5);

	// Set G_DIG config
	gu8CalibWriteTxBuff[CW_BYTE2] = (gu8CalibWriteTxBuff[CW_BYTE2] & (~(MASK_B2_G_DIG_BIT_0 | MASK_B2_G_DIG_BIT_1 | MASK_B2_G_DIG_BIT_2 | MASK_B2_G_DIG_BIT_3 |
																		MASK_B2_G_DIG_BIT_4 | MASK_B2_G_DIG_BIT_5 | MASK_B2_G_DIG_BIT_6))) | (gtUssCalibWritePara->u8G_Dig & MASK_DATA_G_DIG);

	// Set CUSTOMER_BITS config
	gu8CalibWriteTxBuff[CW_BYTE2] = (gu8CalibWriteTxBuff[CW_BYTE2] & (~(MASK_B2_CUSTOMER_BITS_BIT_0))) | ((gtUssCalibWritePara->u8Customer_Bits & MASK_DATA_CUSTOMER_BITS) << SHIFT_BIT_7);
	gu8CalibWriteTxBuff[CW_BYTE3] = (gu8CalibWriteTxBuff[CW_BYTE3] & (~(MASK_B3_CUSTOMER_BITS_BIT_1 | MASK_B3_CUSTOMER_BITS_BIT_2 | MASK_B3_CUSTOMER_BITS_BIT_3 | MASK_B3_CUSTOMER_BITS_BIT_4 |
																		MASK_B3_CUSTOMER_BITS_BIT_5 | MASK_B3_CUSTOMER_BITS_BIT_6))) | ((gtUssCalibWritePara->u8Customer_Bits & MASK_DATA_CUSTOMER_BITS) >> SHIFT_BIT_1);

	// Set OSC_TRIM config
	gu8CalibWriteTxBuff[CW_BYTE3] = (gu8CalibWriteTxBuff[CW_BYTE3] & (~(MASK_B3_OSC_TRIM_BIT_0 | MASK_B3_OSC_TRIM_BIT_1))) | ((gtUssCalibWritePara->u8Osc_Trim & MASK_DATA_OSC_TRIM) << SHIFT_BIT_6);
	gu8CalibWriteTxBuff[CW_BYTE4] = (gu8CalibWriteTxBuff[CW_BYTE4] & (~(MASK_B4_OSC_TRIM_BIT_2 | MASK_B4_OSC_TRIM_BIT_3))) | ((gtUssCalibWritePara->u8Osc_Trim & MASK_DATA_OSC_TRIM) >> SHIFT_BIT_2);

	// Send parameters to sensor
	UssDrivers_Cmds_Transmit(tSensorMask, EX_CMDS_CALIB_WRITE);

	return FUNC_SUCCESS;		
}
//---------------------------------------------------------------------------//

//int read_status(uint8 sensor_idx, uint16 *status_data);
Func_Status_t UssDrivers_Status_Read(Uss_Sensor_Id_t tSensorMask, uint16 *u16StatusData)
{
    Func_Status_t Status = FUNC_FAIL;
	
	if(gbAckRecFinishFlag == TRUE)
	{
		*u16StatusData = (uint16)(gtUssRxAckData[tSensorMask].u32ReadStatus);
		gbAckRecFinishFlag = FALSE;
		Status = FUNC_SUCCESS;
	}
	else
	{
		Status = FUNC_FAIL;
	}	

	return Status;
}


void UssDrivers_Cmds_SendRecC_Send(uint16 u16Mask, Uss_Cmds_SendRec u8SendRecCmd)
{
	switch(u8SendRecCmd)
	{
		case CMDS_SEND_X:
			if((u16Mask & 0x0001) == MASK_IO1_FLC)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO1_TXRX_FLC, EX_CMDS_SEND_C);		
			}
			if((u16Mask & 0x0002) == MASK_IO2_FLM)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO2_TXRX_FLM, EX_CMDS_SEND_C);		
			}
			if((u16Mask & 0x0004) == MASK_IO3_FRC)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO3_TXRX_FRC, EX_CMDS_SEND_C);		
			}
			if((u16Mask & 0x0008) == MASK_IO4_FRM)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO4_TXRX_FRM, EX_CMDS_SEND_C);		
			}
			if((u16Mask & 0x0010) == MASK_IO5_RLC)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO5_TXRX_RLC, EX_CMDS_SEND_C);		
			}
			if((u16Mask & 0x0020) == MASK_IO6_RLM)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO6_TXRX_RLM, EX_CMDS_SEND_C);		
			}
			if((u16Mask & 0x0040) == MASK_IO7_RRC)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO7_TXRX_RRC, EX_CMDS_SEND_C);		
			}
			if((u16Mask & 0x0080) == MASK_IO8_RRM)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO8_TXRX_RRM, EX_CMDS_SEND_C);		
			}
			if((u16Mask & 0x0100) == MASK_IO9_FLS)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO9_TXRX_FLS, EX_CMDS_SEND_C);		
			}
			if((u16Mask & 0x0200) == MASK_IO10_FRS)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO10_TXRX_FRS, EX_CMDS_SEND_C);		
			}
			if((u16Mask & 0x0400) == MASK_IO11_RLS)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO11_TXRX_RLS, EX_CMDS_SEND_C);		
			}
			if((u16Mask & 0x0800) == MASK_IO12_RRS)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO12_TXRX_RRS, EX_CMDS_SEND_C);		
			}			
		break;
		case CMDS_REC_X:
		break;
	}

}

//int uss_detect(uint8 mode,uint16 send_mask, uint16 receive_mask, uint16 detect_time);
Func_Status_t UssDrivers_Uss_Detect(Uss_Detect_Mode_t tMode, uint16 u16SendMask, uint16 u16RecMask, uint16 u16DetTime)
{
	gu16SendMask = u16SendMask;
	gu16RecMask = u16RecMask;
	gu32DetectTimeLen = u16DetTime;
	(void)(gu32DetectTimeLen);

	switch(tMode)
	{
		case MODE_SEND_REC_A:
		break;
		case MODE_SEND_REC_B:
		break;
		case MODE_SEND_REC_C:
			UssDrivers_Cmds_SendRecC_Send(gu16SendMask, CMDS_SEND_X);
			UssDrivers_Cmds_SendRecC_Send(gu16RecMask, CMDS_REC_X);

		
		break;
		case MODE_ENVELOPE:
		break;
	}

	return FUNC_SUCCESS;
}

//int read_bilateral_time(uint8 sensor_idx, uin32 *bilateral_time);
















//---------------------------------------------------------------------------//
void UssDrivers_Rx_Data_Store(Uss_Sensor_Id_t tSensorMask, Uss_Exchange_Cmds u8Cmd)
{
	switch(u8Cmd)
	{
		case EX_CMDS_SEND_A:
			for(uint32 u32Index=0; u32Index<gu32UssRxAckLen; u32Index++)
			{
				gtUssRxAckData[tSensorMask].u32SendAT[u32Index] = gu32TimeTagTemp[u32Index];
			}
			
			#if DEBUG_UART
				DEBUG_MSG([UssRx] SEND_A OK);				
			#endif
		break;
		case EX_CMDS_RECEIVE_A:
			for(uint32 u32Index=0; u32Index<gu32UssRxAckLen; u32Index++)
			{
				gtUssRxAckData[tSensorMask].u32ReceiveAT[u32Index] = gu32TimeTagTemp[u32Index];
			}	
			
			#if DEBUG_UART
				DEBUG_MSG([UssRx] RECEIVE_A OK);
			#endif
		break;
		case EX_CMDS_SEND_B:
			for(uint32 u32Index=0; u32Index<gu32UssRxAckLen; u32Index++)
			{
				gtUssRxAckData[tSensorMask].u32SendBT[u32Index] = gu32TimeTagTemp[u32Index];
			}	
			
			#if DEBUG_UART
				DEBUG_MSG([UssRx] SEND_B OK);
			#endif
		break;
		case EX_CMDS_RECEIVE_B:
			for(uint32 u32Index=0; u32Index<gu32UssRxAckLen; u32Index++)
			{
				gtUssRxAckData[tSensorMask].u32ReceiveBT[u32Index] = gu32TimeTagTemp[u32Index];
			}	
			
			#if DEBUG_UART
				DEBUG_MSG([UssRx] RECEIVE_B OK);
			#endif
		break;
		case EX_CMDS_SEND_C:
			for(uint32 u32Index=0; u32Index<gu32UssRxAckLen; u32Index++)
			{
				gtUssRxAckData[tSensorMask].u32SendCT[u32Index] = gu32TimeTagTemp[u32Index];
			}	

			#if DEBUG_UART
				DEBUG_MSG([UssRx] SEND_C OK);
			#endif
		break;
		case EX_CMDS_RECEIVE_C:
			for(uint32 u32Index=0; u32Index<gu32UssRxAckLen; u32Index++)
			{
				gtUssRxAckData[tSensorMask].u32ReceiveCT[u32Index] = gu32TimeTagTemp[u32Index];
			}	

			#if DEBUG_UART
				DEBUG_MSG([UssRx] RECEIVE_C OK);
			#endif
		break;		
		case EX_CMDS_THRES_SETUP:
			// Reserved, no ACK
		break;
		case EX_CMDS_READ_THRES_SETUP:		
			gtUssRxAckData[tSensorMask].u32ReadThresSetup[0] = (gtUssRxAckData[tSensorMask].u32ReadThresSetup[0] & (~UNIT_U32)) | gu32UssRxBitsTemp[0];	
			gtUssRxAckData[tSensorMask].u32ReadThresSetup[1] = (gtUssRxAckData[tSensorMask].u32ReadThresSetup[1] & (~UNIT_U32)) | gu32UssRxBitsTemp[1];	
			gtUssRxAckData[tSensorMask].u32ReadThresSetup[2] = (gtUssRxAckData[tSensorMask].u32ReadThresSetup[2] & (~UNIT_U32)) | gu32UssRxBitsTemp[2]; 

			#if DEBUG_UART
				DEBUG_MSG([UssRx] READ_THRES_SETUP OK);
			#endif
		break;
		case EX_CMDS_MEAS_SETUP:
			// Reserved, no ACK
		break;
		case EX_CMDS_READ_MEAS_SETUP:
			gtUssRxAckData[tSensorMask].u32ReadMeasSetup[0] = (gtUssRxAckData[tSensorMask].u32ReadMeasSetup[0] & (~UNIT_U32)) | gu32UssRxBitsTemp[0];
			gtUssRxAckData[tSensorMask].u32ReadMeasSetup[1] = (gtUssRxAckData[tSensorMask].u32ReadMeasSetup[1] & (~UNIT_U32)) | gu32UssRxBitsTemp[1];

			#if DEBUG_UART
				DEBUG_MSG([UssRx] READ_MEAS_SETUP OK);
			#endif
		break;
		case EX_CMDS_READ_STATUS:
			gtUssRxAckData[tSensorMask].u32ReadStatus = (gtUssRxAckData[tSensorMask].u32ReadStatus & (~UNIT_U32)) | gu32UssRxBitsTemp[0];

			#if DEBUG_UART
				DEBUG_MSG([UssRx] READ_STATUS OK);
			#endif
		break;
		case EX_CMDS_CAL_PULSES:
			// Reserved, no ACK
		break;
		case EX_CMDS_READ_TEMP:
			gtUssRxAckData[tSensorMask].u32ReadTemp = (gtUssRxAckData[tSensorMask].u32ReadTemp & (~UNIT_U32)) | gu32UssRxBitsTemp[0];

			#if DEBUG_UART
				DEBUG_MSG([UssRx] READ_TEMP OK);
			#endif
		break;
		case EX_CMDS_ENVELOPE_SEND_A:
			// Reserved, no ACK
		break;
		case EX_CMDS_ENVELOPE_REC_A:
			// Reserved, no ACK
		break;
		case EX_CMDS_CALIB_WRITE:
			// Reserved, no ACK
		break;
		case EX_CMDS_CALIB_READ:
			gtUssRxAckData[tSensorMask].u32CalibRead[0] = (gtUssRxAckData[tSensorMask].u32CalibRead[0] & (~UNIT_U32)) | gu32UssRxBitsTemp[0];
			gtUssRxAckData[tSensorMask].u32CalibRead[1] = (gtUssRxAckData[tSensorMask].u32CalibRead[1] & (~UNIT_U32)) | gu32UssRxBitsTemp[1];

			#if DEBUG_UART
				DEBUG_MSG([UssRx] CALIB_READ OK);
			#endif
		break;
		case EX_CMDS_EE_COPY:
			// Reserved, no ACK
		break;
		case EX_CMDS_EE_READ:
			gtUssRxAckData[tSensorMask].u32EeRead[0] = (gtUssRxAckData[tSensorMask].u32EeRead[0] & (~UNIT_U32)) | gu32UssRxBitsTemp[0];
			gtUssRxAckData[tSensorMask].u32EeRead[1] = (gtUssRxAckData[tSensorMask].u32EeRead[1] & (~UNIT_U32)) | gu32UssRxBitsTemp[1];

			#if DEBUG_UART
				DEBUG_MSG([UssRx] EE_READ OK);
			#endif
		break;
		case EX_CMDS_READ_ID:
			gtUssRxAckData[tSensorMask].u32ReadId = (gtUssRxAckData[tSensorMask].u32ReadId & (~UNIT_U32)) | gu32UssRxBitsTemp[0];

			#if DEBUG_UART
				DEBUG_MSG([UssRx] READ_ID OK);
			#endif
		break;
		case EX_CMDS_STANDBY:
			// Reserved, no ACK
		break;
		case EX_CMDS_WAKE_UP:
			// Reserved, no ACK
		break;			
	}
}
//---------------------------------------------------------------------------//
void UssDrivers_Rx_Data_Parse(boolean bFlag)
{
	uint32 u32Index = VAL_INIT;
	uint32 u32UnitShift = VAL_INIT; 
	uint32 u32IndexTemp = VAL_INIT;
	uint32 u32BitShift = VAL_INIT;
	
	if(bFlag == TRUE)	
	{
		if((gu8UssTxCmd >= EX_CMDS_SEND_A) && (gu8UssTxCmd <= EX_CMDS_RECEIVE_C))
		{	
			// Record tag time
			UssDrivers_Rx_Data_Store(tUssSensorId, gu8UssTxCmd);
			memset(&gu32TimeTagTemp, VAL_INIT, sizeof(gu32TimeTagTemp));							// Clear buffer
			//Common_Buffer_Clear(&gu32TimeTagTemp[0], sizeof(gu32TimeTagTemp)/sizeof(uint32));		// Clear buffer

			gu32DetectTimeLen = VAL_INIT;
			//gu32UssRxAckLen = VAL_INIT;
		}
		else
		{
			// Capture time transfer to logic value 
			for(u32Index = VAL_INIT; u32Index < (gu32UssRxAckLen - gu32TxFilterLen); u32Index++)
			{
				u32UnitShift = u32Index / UNIT_A_U32;
				u32IndexTemp = (gu32UssRxAckLen - u32Index - INIT_START) / UNIT_A_U32;
				u32BitShift = gu32UssRxAckLen - INIT_START - gu32TxFilterLen - u32Index - (UNIT_A_U32 * u32UnitShift);

				#if EVB_DEMO
					if((gu8LowPulseTemp[u32Index + gu32TxFilterLen] >= SYM_RX_T_BIT1_MIN) && (gu8LowPulseTemp[u32Index + gu32TxFilterLen] < SYM_RX_T_BIT1_MAX))
					{				
						gu32UssRxBitsTemp[u32IndexTemp] = (gu32UssRxBitsTemp[u32IndexTemp]) | (USS_SIGNAL_1 << u32BitShift);
					}
					else if((gu8LowPulseTemp[u32Index + gu32TxFilterLen] >= SYM_RX_T_BIT0_MIN) && (gu8LowPulseTemp[u32Index + gu32TxFilterLen] < SYM_RX_T_BIT0_MAX))
					{
						gu32UssRxBitsTemp[u32IndexTemp] = (gu32UssRxBitsTemp[u32IndexTemp]) | (USS_SIGNAL_0 << u32BitShift) ;
					}
					else
					{		
						#if DEBUG_UART
							DEBUG_MSG([UssRx] BIT_FAIL); 
						#endif
					}	
				#else				
					if((gu32InvertHighPulseTemp[u32Index + gu32TxFilterLen] >= SYM_RX_T_BIT1_MIN) && (gu32InvertHighPulseTemp[u32Index + gu32TxFilterLen] < SYM_RX_T_BIT1_MAX))
					{				
						gu32UssRxBitsTemp[u32IndexTemp] = (gu32UssRxBitsTemp[u32IndexTemp]) | (USS_SIGNAL_1 << u32BitShift);
					}
					else if((gu32InvertHighPulseTemp[u32Index + gu32TxFilterLen] >= SYM_RX_T_BIT0_MIN) && (gu32InvertHighPulseTemp[u32Index + gu32TxFilterLen] < SYM_RX_T_BIT0_MAX))
					{
						gu32UssRxBitsTemp[u32IndexTemp] = (gu32UssRxBitsTemp[u32IndexTemp]) | (USS_SIGNAL_0 << u32BitShift) ;
					}
					else
					{		
						#if DEBUG_UART
							DEBUG_MSG([UssRx] BIT_FAIL); 
						#endif
					}					
				#endif
			}

			UssDrivers_Rx_Data_Store(tUssSensorId, gu8UssTxCmd);
			
			#if EVB_DEMO
			memset(&gu8LowPulseTemp, VAL_INIT, sizeof(gu8LowPulseTemp));	// Clear buffer
			#else
			memset(&gu32InvertHighPulseTemp, VAL_INIT, sizeof(gu32InvertHighPulseTemp));	// Clear buffer
			#endif
		}		
		
		gbUssRxFinishFlag = FALSE;
	}
}
//---------------------------------------------------------------------------//
Uss_Exchange_Cmds UssDrivers_UssTxCmd_Get(void)
{
	return gu8UssTxCmd;
}
//---------------------------------------------------------------------------//
void UssDrivers_UssTxCmd_Set(uint8 u8Cmd)
{
	gu8UssTxCmd = u8Cmd;
}
//---------------------------------------------------------------------------//
boolean UssDrivers_UssRxAckEnFlag_Get(void)
{
	return gbUssRxAckEnFlag;
}
//---------------------------------------------------------------------------//
void UssDrivers_UssRxAckEnFlag_Set(boolean bFlagSwitch)
{
	gbUssRxAckEnFlag = bFlagSwitch;
}
//---------------------------------------------------------------------------//
boolean UssDrivers_RxIsrFinishFlag_Get(void)
{
	return gbUssRxFinishFlag;
}
//---------------------------------------------------------------------------//
void UssDrivers_RxIsrFinishFlag_Set(boolean bFlagStatus)
{
	gbUssRxFinishFlag = bFlagStatus;
}
//---------------------------------------------------------------------------//
uint32 UssDrivers_RxAckLen_Get(void)
{
	return gu32UssRxAckLen;
}
//---------------------------------------------------------------------------//
void UssDrivers_RxAckLen_Set(uint32 u32AckLen)
{
	gu32UssRxAckLen = u32AckLen;
}
//---------------------------------------------------------------------------//
boolean UssDrivers_AckRecFinishFlag_Get(void)
{
	return gbAckRecFinishFlag;
}
//---------------------------------------------------------------------------//
void UssDrivers_AckRecFinishFlag_Set(boolean bSwitch)
{
	gbAckRecFinishFlag = bSwitch;
}
//---------------------------------------------------------------------------//
uint32 UssDrivers_DetectTimeLen_Get(void)
{
	return gu32DetectTimeLen;
}
//---------------------------------------------------------------------------//
void UssDrivers_DetectTimeLen_Set(uint32 u32DetTLen)
{
	gu32DetectTimeLen = u32DetTLen;
}
//---------------------------------------------------------------------------//


