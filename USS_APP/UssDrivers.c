/******************************************************************************
;       Program		:	UssDrivers.c
;       Function	:	USS (Ultrasonic Sensor System) Function
;       Chip		:	Infineon TC397
;       Clock		:	Internal Clock 300MHz
;       Date		:	2024 / 1 / 9
;       Author		:	Fenderson Lu & Jim
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
Uss_Exchange_Cmds gu8UssTxCmd;
Uss_Sensor_Id_t tUssSensorId;
Uss_Thres_Data_t gtUssThresSetupPara[SIZE_USS_SENSOR];
Uss_Meas_Data_t gtUssMeasData[SIZE_USS_SENSOR];
Uss_Calib_Data_t gtUssCalibWritePara[SIZE_USS_SENSOR];
Uss_Rx_Ack_Data_t gtUssRxAckData[SIZE_USS_SENSOR];
uint32 gu32UssRxBitsTemp[SIZE_USS_RX];
uint32 gu32InvertHighPulseTemp[SIZE_ISR_RX_RAW];
uint32 gu32TimeTagTemp[SIZE_TIME_TAG];
uint8 gu8ThresSetupTxBuff[SIZE_THRES_SETUP];
uint8 gu8MeasWriteTxBuff[SIZE_MEAS_WRITE];
uint8 gu8CalibWriteTxBuff[SIZE_CALIB_WRITE];
uint32 gu32UssRxAckLen;
uint32 gu32DetectTimeLen;
uint32 gu32TxFilterLen;
uint32 gu32RxTagTCnt;
boolean gbIsrRxFinishFlag, gbUssRxAckEnFlag;
boolean gbAckRecFinishFlag = FALSE;

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
// Bit11	: 1 = USS IO12 rrs sensor is called transmitted command. 
// Bit10	: 1 = USS IO11 rls sensor is called transmitted command. 
// Bit9		: 1 = USS IO10 frs sensor is called transmitted command. 
// Bit8		: 1 = USS IO9 fls sensor is called transmitted command. 
// Bit7		: 1 = USS IO8 rrm sensor is called transmitted command. 
// Bit6		: 1 = USS IO7 rrc sensor is called transmitted command. 
// Bit5		: 1 = USS IO6 rlm sensor is called transmitted command. 
// Bit4		: 1 = USS IO5 rlc sensor is called transmitted command. 
// Bit3		: 1 = USS IO4 frm sensor is called transmitted command. 
// Bit2		: 1 = USS IO3 frc sensor is called transmitted command. 
// Bit1		: 1 = USS IO2 flm sensor is called transmitted command. 
// Bit0		: 1 = USS IO1 flc sensor is called transmitted command. 
//--------------------------------------------------------------//

uint16 gu16RecMask;
//------------------- gu16RecMask ------------------------------//
// Bit15	: Reserved
// Bit14	: Reserved 
// Bit13	: Reserved
// Bit12	: Reserved
// Bit11	: 1 = USS IO12 rrs sensor is called receivable command. 
// Bit10	: 1 = USS IO11 rls sensor is called receivable command. 
// Bit9		: 1 = USS IO10 frs sensor is called receivable command. 
// Bit8		: 1 = USS IO9 fls sensor is called receivable command. 
// Bit7		: 1 = USS IO8 rrm sensor is called receivable command. 
// Bit6		: 1 = USS IO7 rrc sensor is called receivable command. 
// Bit5		: 1 = USS IO6 rlm sensor is called receivable command. 
// Bit4		: 1 = USS IO5 rlc sensor is called receivable command. 
// Bit3		: 1 = USS IO4 frm sensor is called receivable command. 
// Bit2		: 1 = USS IO3 frc sensor is called receivable command. 
// Bit1		: 1 = USS IO2 flm sensor is called receivable command. 
// Bit0		: 1 = USS IO1 flc sensor is called receivable command. 
//--------------------------------------------------------------//

#if EVB_DEMO
uint32 gu32LowPulseTemp[SIZE_ISR_RX_RAW];
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

/* USS_MEAS_DATA */
		gtUssMeasData[u32SensorIndex].u8filter_cfg = 0x00;
		gtUssMeasData[u32SensorIndex].u8noise_cfg = 0x02;
		gtUssMeasData[u32SensorIndex].u8stc_start = 0x00;
		gtUssMeasData[u32SensorIndex].u8stc_cfg = 0x02;
		gtUssMeasData[u32SensorIndex].u8epd = 0x01;
		gtUssMeasData[u32SensorIndex].u8ftc = 0x00;
		gtUssMeasData[u32SensorIndex].u8nftg = 0x01;
		gtUssMeasData[u32SensorIndex].u8rt_cfg = 0x00;
		gtUssMeasData[u32SensorIndex].u8echo_deb = 0x00;
		gtUssMeasData[u32SensorIndex].u8thresscale_c = 0x02;
		gtUssMeasData[u32SensorIndex].u8tmeas_c = 0x06;
		gtUssMeasData[u32SensorIndex].u8npulses_c = 0x05;
		gtUssMeasData[u32SensorIndex].u8thresscale_b = 0x02;
		gtUssMeasData[u32SensorIndex].u8tmeas_b = 0x00;
		gtUssMeasData[u32SensorIndex].u8npulses_b = 0x01;
		gtUssMeasData[u32SensorIndex].u8thresscale_a = 0x02;
		gtUssMeasData[u32SensorIndex].u8tmeas_a = 0x02;
		gtUssMeasData[u32SensorIndex].u8npulses_a = 0x03;
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
			Common_Delay(UNIT_MICRO, (uint32)(SYM_TIME_T_SND * TIME_GAP));
			IfxPort_setPinHigh(gtUssIoPort[tSensorMask], gu8UssIoPin[tSensorMask]);
			#else
			IfxPort_setPinHigh(gtUssIoPort[tSensorMask], gu8UssIoPin[tSensorMask]);
			Common_Delay(UNIT_MICRO, (uint32)(SYM_TIME_T_SND * TIME_GAP));		
			IfxPort_setPinLow(gtUssIoPort[tSensorMask], gu8UssIoPin[tSensorMask]);			
			#endif
		break;
		case IO_SYM_T_REC:
			#if EVB_DEMO
			IfxPort_setPinLow(gtUssIoPort[tSensorMask], gu8UssIoPin[tSensorMask]);
			Common_Delay(UNIT_MICRO, (uint32)(SYM_TIME_T_REC * TIME_GAP));	
			IfxPort_setPinHigh(gtUssIoPort[tSensorMask], gu8UssIoPin[tSensorMask]);
			#else
			IfxPort_setPinHigh(gtUssIoPort[tSensorMask], gu8UssIoPin[tSensorMask]);
			Common_Delay(UNIT_MICRO, (uint32)(SYM_TIME_T_REC * TIME_GAP));	
			IfxPort_setPinLow(gtUssIoPort[tSensorMask], gu8UssIoPin[tSensorMask]);
			#endif
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
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_REC);			
		break;
		case EX_CMDS_SEND_B:		// ACK
			#if DEBUG_UART
				DEBUG_MSG([UssTx] SEND_B);
			#endif
			
			gbUssRxAckEnFlag = TRUE;
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
			u32TxIndex = LEN_MEAS_WRITE_BITS;
			u32MaskIndex = SHIFT_BIT_1;
		
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_CMD);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_D);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT1_LOG_1);

			while(u32TxIndex--)		// MSB to LSB
			{
				if((gu8MeasWriteTxBuff[MEAS_BYTE4 - u32TsByteIndex] & (REG_BIT_7 >> u32MaskIndex)) == REG_BIT_7 >> u32MaskIndex)
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


			#if EVB_DEMO
			// Sensor ACK				
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
			UssDrivers_IO_Symbol_Signal(tSensorMask, IO_SYM_T_BIT0_LOG_0);			
			#endif

			
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
uint8 UssDrivers_Meas_Parity_Check(ParityChk_Num_MEAS_t tParityNum)
{
	uint16 u16UssParityBitCal = INIT_PARITY_BITS;

	switch(tParityNum)
	{
		case PC_MEAS_PARITY_0:
			u16UssParityBitCal |= (gu8MeasWriteTxBuff[MEAS_BYTE0] & (MASK_MEAS_B0_FILTER_CFG | MASK_MEAS_B0_NOISE_CFG | MASK_MEAS_B0_STC_START)) >> SHIFT_BIT_3;
			u16UssParityBitCal |= (gu8MeasWriteTxBuff[MEAS_BYTE1] & (MASK_MEAS_B1_ECHO_DEB | MASK_MEAS_B1_EPD | MASK_MEAS_B1_FTC | MASK_MEAS_B1_NFTG | MASK_MEAS_B1_RT_CFG | MASK_MEAS_B1_STC_CFG | MASK_MEAS_B1_THRESSCALE_C_L)) << SHIFT_BIT_5;
			u16UssParityBitCal |= (gu8MeasWriteTxBuff[MEAS_BYTE2] & MASK_MEAS_B2_THRESSCALE_C_H) << SHIFT_BIT_13;
		break;
		case PC_MEAS_PARITY_1:
			u16UssParityBitCal |= (gu8MeasWriteTxBuff[MEAS_BYTE2] & (MASK_MEAS_B2_NPULSES_C | MASK_MEAS_B2_THRESSCALE_B_L | MASK_MEAS_B2_TMEAS_C)) >> SHIFT_BIT_1;
			u16UssParityBitCal |= (gu8MeasWriteTxBuff[MEAS_BYTE3] & (MASK_MEAS_B3_NPULSES_B | MASK_MEAS_B3_TMEAS_B | MASK_MEAS_B3_THRESSCALE_B_H)) << SHIFT_BIT_7;
		break;
		case PC_MEAS_PARITY_2:
			u16UssParityBitCal |= (gu8MeasWriteTxBuff[MEAS_BYTE3] & MASK_MEAS_B3_THRESSCALE_A_L) >> SHIFT_BIT_7;
			u16UssParityBitCal |= (gu8MeasWriteTxBuff[MEAS_BYTE4] & (MASK_MEAS_B4_NPULSES_A | MASK_MEAS_B4_THRESSCALE_A_H | MASK_MEAS_B4_TMEAS_A)) << SHIFT_BIT_1;
		break;
	}

	return UssDrivers_ParityBit_Calculate(MODE_PC_EVEN, u16UssParityBitCal);
}
//---------------------------------------------------------------------------//
Func_Status_t UssDrivers_ThresSetup_Para_Write(Uss_Sensor_Id_t tSensorMask, Uss_Thres_Data_t *tThresSetupPara)
{
	// Set Thresscale_Rec config
	gu8ThresSetupTxBuff[TS_BYTE0] = (gu8ThresSetupTxBuff[TS_BYTE0] & (~(MASK_B0_THRES_SCALE_REC_L | MASK_B0_THRES_SCALE_REC_H))) | (((*tThresSetupPara).u8Thresscale_Rec & MASK_DATA_THRES_SCALE_REC) << SHIFT_BIT_5);

	// Set Atg_Alpha config
	gu8ThresSetupTxBuff[TS_BYTE0] = (gu8ThresSetupTxBuff[TS_BYTE0] & (~MASK_B0_ATG_ALPHA)) | (((*tThresSetupPara).u8Atg_Alpha & MASK_DATA_ATG_ALPHA) << SHIFT_BIT_7);
	
	// Set Atg_Tau config
	gu8ThresSetupTxBuff[TS_BYTE1] = (gu8ThresSetupTxBuff[TS_BYTE1] & (~(MASK_B1_ATG_TAU_L | MASK_B1_ATG_TAU_H))) | ((*tThresSetupPara).u8Atg_Tau & MASK_DATA_ATG_TAU);

	// Set Atg_Cfg config
	gu8ThresSetupTxBuff[TS_BYTE1] = (gu8ThresSetupTxBuff[TS_BYTE1] & (~(MASK_B1_ATG_CFG_L | MASK_B1_ATG_CFG_H))) | (((*tThresSetupPara).u8Atg_Cfg & MASK_DATA_ATG_CFG) << SHIFT_BIT_2);

	// Set Thsft_Cfg config
	gu8ThresSetupTxBuff[TS_BYTE1] = (gu8ThresSetupTxBuff[TS_BYTE1] & (~(MASK_B1_THSFT_CFG_L | MASK_B1_THSFT_CFG_H))) | (((*tThresSetupPara).u8Thsft_Cfg & MASK_DATA_THSFT_CFG) << SHIFT_BIT_4);

	// Set THVAL13 config, thval[11] = THVAL13
	gu8ThresSetupTxBuff[TS_BYTE1] = (gu8ThresSetupTxBuff[TS_BYTE1] & (~(MASK_B1_THVAL13_BIT_0 | MASK_B1_THVAL13_BIT_1))) | (((*tThresSetupPara).u8Thval[NUM_THVAL_13] & MASK_DATA_THVAL) << SHIFT_BIT_6);
	gu8ThresSetupTxBuff[TS_BYTE2] = (gu8ThresSetupTxBuff[TS_BYTE2] & (~(MASK_B2_THVAL13_BIT_2 | MASK_B2_THVAL13_BIT_3 | MASK_B2_THVAL13_BIT_4))) | (((*tThresSetupPara).u8Thval[NUM_THVAL_13] & MASK_DATA_THVAL) >> SHIFT_BIT_2);
	
	// Set THVAL12 config, thval[10] = THVAL12
	gu8ThresSetupTxBuff[TS_BYTE2] = (gu8ThresSetupTxBuff[TS_BYTE2] & (~(MASK_B2_THVAL12_BIT_0 | MASK_B2_THVAL12_BIT_1 | MASK_B2_THVAL12_BIT_2 | MASK_B2_THVAL12_BIT_3 | MASK_B2_THVAL12_BIT_4))) | (((*tThresSetupPara).u8Thval[NUM_THVAL_12] & MASK_DATA_THVAL) << SHIFT_BIT_3);
	
	// Set THVAL11 config, thval[9] = THVAL11
	gu8ThresSetupTxBuff[TS_BYTE3] = (gu8ThresSetupTxBuff[TS_BYTE3] & (~(MASK_B3_THVAL11_BIT_0 | MASK_B3_THVAL11_BIT_1 | MASK_B3_THVAL11_BIT_2 | MASK_B3_THVAL11_BIT_3 | MASK_B3_THVAL11_BIT_4))) | ((*tThresSetupPara).u8Thval[NUM_THVAL_11] & MASK_DATA_THVAL);

	// Set THVAL10 config
	gu8ThresSetupTxBuff[TS_BYTE3] = (gu8ThresSetupTxBuff[TS_BYTE3] & (~(MASK_B3_THVAL10_BIT_0 | MASK_B3_THVAL10_BIT_1 | MASK_B3_THVAL10_BIT_2))) | (((*tThresSetupPara).u8Thval[NUM_THVAL_10] & MASK_DATA_THVAL) << SHIFT_BIT_5);
	gu8ThresSetupTxBuff[TS_BYTE4] = (gu8ThresSetupTxBuff[TS_BYTE4] & (~(MASK_B4_THVAL10_BIT_3 | MASK_B4_THVAL10_BIT_4))) | (((*tThresSetupPara).u8Thval[NUM_THVAL_10] & MASK_DATA_THVAL) >> SHIFT_BIT_3);

	// Set THVAL9 config
	gu8ThresSetupTxBuff[TS_BYTE4] = (gu8ThresSetupTxBuff[TS_BYTE4] & (~(MASK_B4_THVAL9_BIT_0 | MASK_B4_THVAL9_BIT_1 | MASK_B4_THVAL9_BIT_2 | MASK_B4_THVAL9_BIT_3 | MASK_B4_THVAL9_BIT_4))) | (((*tThresSetupPara).u8Thval[NUM_THVAL_9] & MASK_DATA_THVAL) << SHIFT_BIT_2);

	// Set THVAL8 config
	gu8ThresSetupTxBuff[TS_BYTE4] = (gu8ThresSetupTxBuff[TS_BYTE4] & (~MASK_B4_THVAL8_BIT_0)) | (((*tThresSetupPara).u8Thval[NUM_THVAL_8] & MASK_DATA_THVAL) << SHIFT_BIT_7);
	gu8ThresSetupTxBuff[TS_BYTE5] = (gu8ThresSetupTxBuff[TS_BYTE5] & (~(MASK_B5_THVAL8_BIT_1 | MASK_B5_THVAL8_BIT_2 | MASK_B5_THVAL8_BIT_3 | MASK_B5_THVAL8_BIT_4))) | (((*tThresSetupPara).u8Thval[NUM_THVAL_8] & MASK_DATA_THVAL) >> SHIFT_BIT_1);

	// Set THVAL7 config
	gu8ThresSetupTxBuff[TS_BYTE5] = (gu8ThresSetupTxBuff[TS_BYTE5] & (~(MASK_B5_THVAL7_BIT_0 | MASK_B5_THVAL7_BIT_1 | MASK_B5_THVAL7_BIT_2 | MASK_B5_THVAL7_BIT_3))) | (((*tThresSetupPara).u8Thval[NUM_THVAL_7] & MASK_DATA_THVAL) << SHIFT_BIT_4);
	gu8ThresSetupTxBuff[TS_BYTE6] = (gu8ThresSetupTxBuff[TS_BYTE6] & (~MASK_B6_THVAL7_BIT_4)) | (((*tThresSetupPara).u8Thval[NUM_THVAL_7] & MASK_DATA_THVAL) >> SHIFT_BIT_4);

	// Set THVAL6 config
	gu8ThresSetupTxBuff[TS_BYTE6] = (gu8ThresSetupTxBuff[TS_BYTE6] & (~(MASK_B6_THVAL6_BIT_0 | MASK_B6_THVAL6_BIT_1 | MASK_B6_THVAL6_BIT_2 | MASK_B6_THVAL6_BIT_3 | MASK_B6_THVAL6_BIT_4))) | (((*tThresSetupPara).u8Thval[NUM_THVAL_6] & MASK_DATA_THVAL) << SHIFT_BIT_1);

	// Set THVAL5 config
	gu8ThresSetupTxBuff[TS_BYTE6] = (gu8ThresSetupTxBuff[TS_BYTE6] & (~(MASK_B6_THVAL5_BIT_0 | MASK_B6_THVAL5_BIT_1))) | (((*tThresSetupPara).u8Thval[NUM_THVAL_5] & MASK_DATA_THVAL) << SHIFT_BIT_6);
	gu8ThresSetupTxBuff[TS_BYTE7] = (gu8ThresSetupTxBuff[TS_BYTE7] & (~(MASK_B7_THVAL5_BIT_2 | MASK_B7_THVAL5_BIT_3 | MASK_B7_THVAL5_BIT_4))) | (((*tThresSetupPara).u8Thval[NUM_THVAL_5] & MASK_DATA_THVAL) >> SHIFT_BIT_2);

	// Set THVAL4 config
	gu8ThresSetupTxBuff[TS_BYTE7] = (gu8ThresSetupTxBuff[TS_BYTE7] & (~(MASK_B7_THVAL4_BIT_0 | MASK_B7_THVAL4_BIT_1 | MASK_B7_THVAL4_BIT_2 | MASK_B7_THVAL4_BIT_3 | MASK_B7_THVAL4_BIT_4))) | (((*tThresSetupPara).u8Thval[NUM_THVAL_4] & MASK_DATA_THVAL) << SHIFT_BIT_3);

	// Set THVAL3 config
	gu8ThresSetupTxBuff[TS_BYTE8] = (gu8ThresSetupTxBuff[TS_BYTE8] & (~(MASK_B8_THVAL3_BIT_0 | MASK_B8_THVAL3_BIT_1 | MASK_B8_THVAL3_BIT_2 | MASK_B8_THVAL3_BIT_3 | MASK_B8_THVAL3_BIT_4))) | ((*tThresSetupPara).u8Thval[NUM_THVAL_3] & MASK_DATA_THVAL);

	// Set THVAL2 config, thval[0] = THVAL2
	gu8ThresSetupTxBuff[TS_BYTE8] = (gu8ThresSetupTxBuff[TS_BYTE8] & (~(MASK_B8_THVAL2_BIT_0 | MASK_B8_THVAL2_BIT_1 | MASK_B8_THVAL2_BIT_2))) | (((*tThresSetupPara).u8Thval[NUM_THVAL_2] & MASK_DATA_THVAL) << SHIFT_BIT_5);
	gu8ThresSetupTxBuff[TS_BYTE9] = (gu8ThresSetupTxBuff[TS_BYTE9] & (~(MASK_B9_THVAL2_BIT_3 | MASK_B9_THVAL2_BIT_4))) | (((*tThresSetupPara).u8Thval[NUM_THVAL_2] & MASK_DATA_THVAL) >> SHIFT_BIT_3);

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
Func_Status_t UssDrivers_Meas_Para_Write(Uss_Sensor_Id_t tSensorMask, Uss_Meas_Data_t *tMeasPara)
{
	// Set Filter_cfg config
	gu8MeasWriteTxBuff[MEAS_BYTE0] = (gu8MeasWriteTxBuff[MEAS_BYTE0] & (~MASK_MEAS_B0_FILTER_CFG)) | (((*tMeasPara).u8filter_cfg << SHIFT_BIT_3) & MASK_MEAS_B0_FILTER_CFG);

	// Set Noise_cfg config
	gu8MeasWriteTxBuff[MEAS_BYTE0] = (gu8MeasWriteTxBuff[MEAS_BYTE0] & (~MASK_MEAS_B0_NOISE_CFG)) | (((*tMeasPara).u8noise_cfg << SHIFT_BIT_4) & MASK_MEAS_B0_NOISE_CFG);

	// Set Stc_start config
	gu8MeasWriteTxBuff[MEAS_BYTE0] = (gu8MeasWriteTxBuff[MEAS_BYTE0] & (~MASK_MEAS_B0_STC_START)) | (((*tMeasPara).u8stc_start << SHIFT_BIT_6) & MASK_MEAS_B0_STC_START);

	// Set Stc_cfg config
	gu8MeasWriteTxBuff[MEAS_BYTE1] = (gu8MeasWriteTxBuff[MEAS_BYTE1] & (~MASK_MEAS_B1_STC_CFG)) | ((*tMeasPara).u8stc_cfg & MASK_MEAS_B1_STC_CFG);

	// Set Epd config
	gu8MeasWriteTxBuff[MEAS_BYTE1] = (gu8MeasWriteTxBuff[MEAS_BYTE1] & (~MASK_MEAS_B1_EPD)) | (((*tMeasPara).u8epd << SHIFT_BIT_2) & MASK_MEAS_B1_EPD);

	// Set Ftc config
	gu8MeasWriteTxBuff[MEAS_BYTE1] = (gu8MeasWriteTxBuff[MEAS_BYTE1] & (~MASK_MEAS_B1_FTC)) | (((*tMeasPara).u8ftc << SHIFT_BIT_3) & MASK_MEAS_B1_FTC);

	// Set Nftg config
	gu8MeasWriteTxBuff[MEAS_BYTE1] = (gu8MeasWriteTxBuff[MEAS_BYTE1] & (~MASK_MEAS_B1_NFTG)) | (((*tMeasPara).u8nftg << SHIFT_BIT_4) & MASK_MEAS_B1_NFTG);

	// Set Rt_cfg config
	gu8MeasWriteTxBuff[MEAS_BYTE1] = (gu8MeasWriteTxBuff[MEAS_BYTE1] & (~MASK_MEAS_B1_RT_CFG)) | (((*tMeasPara).u8rt_cfg << SHIFT_BIT_5) & MASK_MEAS_B1_RT_CFG);

	// Set Echo_deb config
	gu8MeasWriteTxBuff[MEAS_BYTE1] = (gu8MeasWriteTxBuff[MEAS_BYTE1] & (~MASK_MEAS_B1_ECHO_DEB)) | (((*tMeasPara).u8echo_deb << SHIFT_BIT_6) & MASK_MEAS_B1_ECHO_DEB);

	// Set Thresscale_c config
	gu8MeasWriteTxBuff[MEAS_BYTE1] = (gu8MeasWriteTxBuff[MEAS_BYTE1] & (~MASK_MEAS_B1_THRESSCALE_C_L)) | ((uint8)((*tMeasPara).u8thresscale_c << SHIFT_BIT_7) & MASK_MEAS_B1_THRESSCALE_C_L);
	gu8MeasWriteTxBuff[MEAS_BYTE2] = (gu8MeasWriteTxBuff[MEAS_BYTE2] & (~MASK_MEAS_B2_THRESSCALE_C_H)) | (((*tMeasPara).u8thresscale_c >> SHIFT_BIT_1) & MASK_MEAS_B2_THRESSCALE_C_H);

	// Set TMEAS_C config
	gu8MeasWriteTxBuff[MEAS_BYTE2] = (gu8MeasWriteTxBuff[MEAS_BYTE2] & (~MASK_MEAS_B2_TMEAS_C)) | (((*tMeasPara).u8tmeas_c << SHIFT_BIT_1) & MASK_MEAS_B2_TMEAS_C);

	// Set NPULSES_C config
	gu8MeasWriteTxBuff[MEAS_BYTE2] = (gu8MeasWriteTxBuff[MEAS_BYTE2] & (~MASK_MEAS_B2_NPULSES_C)) | (((*tMeasPara).u8npulses_c << SHIFT_BIT_4) & MASK_MEAS_B2_NPULSES_C);

	// Set THRESSCALE_B config
	gu8MeasWriteTxBuff[MEAS_BYTE2] = (gu8MeasWriteTxBuff[MEAS_BYTE2] & (~MASK_MEAS_B2_THRESSCALE_B_L)) | ((uint8)((*tMeasPara).u8thresscale_b << SHIFT_BIT_7) & MASK_MEAS_B2_THRESSCALE_B_L);
	gu8MeasWriteTxBuff[MEAS_BYTE3] = (gu8MeasWriteTxBuff[MEAS_BYTE3] & (~MASK_MEAS_B3_THRESSCALE_B_H)) | (((*tMeasPara).u8thresscale_b >> SHIFT_BIT_1) & MASK_MEAS_B3_THRESSCALE_B_H);

	// Set TMEAS_B config
	gu8MeasWriteTxBuff[MEAS_BYTE3] = (gu8MeasWriteTxBuff[MEAS_BYTE3] & (~MASK_MEAS_B3_TMEAS_B)) | (((*tMeasPara).u8tmeas_b << SHIFT_BIT_1) & MASK_MEAS_B3_TMEAS_B);

	// Set NPULSES_B config
	gu8MeasWriteTxBuff[MEAS_BYTE3] = (gu8MeasWriteTxBuff[MEAS_BYTE3] & (~MASK_MEAS_B3_NPULSES_B)) | (((*tMeasPara).u8npulses_b << SHIFT_BIT_4) & MASK_MEAS_B3_NPULSES_B);

	// Set THRESSCALE_A config
	gu8MeasWriteTxBuff[MEAS_BYTE3] = (gu8MeasWriteTxBuff[MEAS_BYTE3] & (~MASK_MEAS_B3_THRESSCALE_A_L)) | ((uint8)((*tMeasPara).u8thresscale_a << SHIFT_BIT_7) & MASK_MEAS_B3_THRESSCALE_A_L);
	gu8MeasWriteTxBuff[MEAS_BYTE4] = (gu8MeasWriteTxBuff[MEAS_BYTE4] & (~MASK_MEAS_B4_THRESSCALE_A_H)) | (((*tMeasPara).u8thresscale_a >> SHIFT_BIT_1) & MASK_MEAS_B4_THRESSCALE_A_H);

	// Set TMEAS_A config
	gu8MeasWriteTxBuff[MEAS_BYTE4] = (gu8MeasWriteTxBuff[MEAS_BYTE4] & (~MASK_MEAS_B4_TMEAS_A)) | (((*tMeasPara).u8tmeas_a << SHIFT_BIT_1) & MASK_MEAS_B4_TMEAS_A);

	// Set NPULSES_A config
	gu8MeasWriteTxBuff[MEAS_BYTE4] = (gu8MeasWriteTxBuff[MEAS_BYTE4] & (~MASK_MEAS_B4_NPULSES_A)) | (((*tMeasPara).u8npulses_a << SHIFT_BIT_4) & MASK_MEAS_B4_NPULSES_A);

	// Calculate parity bit 0
	gu8MeasWriteTxBuff[MEAS_BYTE0] = (gu8MeasWriteTxBuff[MEAS_BYTE0] & (~MASK_MEAS_B0_PARITY_0)) | (UssDrivers_Meas_Parity_Check(PC_MEAS_PARITY_0) << SHIFT_BIT_2);

	// Calculate parity bit 1
	gu8MeasWriteTxBuff[MEAS_BYTE0] = (gu8MeasWriteTxBuff[MEAS_BYTE0] & (~MASK_MEAS_B0_PARITY_1)) | (UssDrivers_Meas_Parity_Check(PC_MEAS_PARITY_1) << SHIFT_BIT_1);

	// Calculate parity bit 2
	gu8MeasWriteTxBuff[MEAS_BYTE0] = (gu8MeasWriteTxBuff[MEAS_BYTE0] & (~MASK_MEAS_B0_PARITY_2)) | UssDrivers_Meas_Parity_Check(PC_MEAS_PARITY_2);

	// Send parameters to sensor
	UssDrivers_Cmds_Transmit(tSensorMask, EX_CMDS_MEAS_SETUP);	

    return FUNC_SUCCESS;		
}

//---------------------------------------------------------------------------//
Func_Status_t UssDrivers_Calib_Write(Uss_Sensor_Id_t tSensorMask, Uss_Calib_Data_t *tCalibWritePara)
{
	// Set F_DRV config
	gu8CalibWriteTxBuff[CW_BYTE0] = (gu8CalibWriteTxBuff[CW_BYTE0] & (~(MASK_B0_F_DRV_BIT_0 | MASK_B0_F_DRV_BIT_1 | MASK_B0_F_DRV_BIT_2 | MASK_B0_F_DRV_BIT_3 |
																		MASK_B0_F_DRV_BIT_4 | MASK_B0_F_DRV_BIT_5 | MASK_B0_F_DRV_BIT_6 | MASK_B0_F_DRV_BIT_7))) | ((*tCalibWritePara).u8F_Drv & MASK_W_F_DRV);

	// Set I_DRV config
	gu8CalibWriteTxBuff[CW_BYTE1] = (gu8CalibWriteTxBuff[CW_BYTE1] & (~(MASK_B1_I_DRV_BIT_0 | MASK_B1_I_DRV_BIT_1 | MASK_B1_I_DRV_BIT_2 | MASK_B1_I_DRV_BIT_3 | 
																		MASK_B1_I_DRV_BIT_4))) | ((*tCalibWritePara).u8I_Drv & MASK_W_I_DRV);

	// Set G_ANA config
	gu8CalibWriteTxBuff[CW_BYTE1] = (gu8CalibWriteTxBuff[CW_BYTE1] & (~(MASK_B1_G_ANA_BIT_0 | MASK_B1_G_ANA_BIT_1 | MASK_B1_G_ANA_BIT_2))) | (((*tCalibWritePara).u8G_Ana & MASK_W_G_ANA) << SHIFT_BIT_5);

	// Set G_DIG config
	gu8CalibWriteTxBuff[CW_BYTE2] = (gu8CalibWriteTxBuff[CW_BYTE2] & (~(MASK_B2_G_DIG_BIT_0 | MASK_B2_G_DIG_BIT_1 | MASK_B2_G_DIG_BIT_2 | MASK_B2_G_DIG_BIT_3 |
																		MASK_B2_G_DIG_BIT_4 | MASK_B2_G_DIG_BIT_5 | MASK_B2_G_DIG_BIT_6))) | ((*tCalibWritePara).u8G_Dig & MASK_W_G_DIG);

	// Set CUSTOMER_BITS config
	gu8CalibWriteTxBuff[CW_BYTE2] = (gu8CalibWriteTxBuff[CW_BYTE2] & (~(MASK_B2_CUSTOMER_BITS_BIT_0))) | (((*tCalibWritePara).u8Customer_Bits & MASK_W_CUSTOMER_BITS) << SHIFT_BIT_7);
	gu8CalibWriteTxBuff[CW_BYTE3] = (gu8CalibWriteTxBuff[CW_BYTE3] & (~(MASK_B3_CUSTOMER_BITS_BIT_1 | MASK_B3_CUSTOMER_BITS_BIT_2 | MASK_B3_CUSTOMER_BITS_BIT_3 | MASK_B3_CUSTOMER_BITS_BIT_4 |
																		MASK_B3_CUSTOMER_BITS_BIT_5 | MASK_B3_CUSTOMER_BITS_BIT_6))) | (((*tCalibWritePara).u8Customer_Bits & MASK_W_CUSTOMER_BITS) >> SHIFT_BIT_1);

	// Set OSC_TRIM config
	gu8CalibWriteTxBuff[CW_BYTE3] = (gu8CalibWriteTxBuff[CW_BYTE3] & (~(MASK_B3_OSC_TRIM_BIT_0 | MASK_B3_OSC_TRIM_BIT_1))) | (((*tCalibWritePara).u8Osc_Trim & MASK_W_OSC_TRIM) << SHIFT_BIT_6);
	gu8CalibWriteTxBuff[CW_BYTE4] = (gu8CalibWriteTxBuff[CW_BYTE4] & (~(MASK_B4_OSC_TRIM_BIT_2 | MASK_B4_OSC_TRIM_BIT_3))) | (((*tCalibWritePara).u8Osc_Trim & MASK_W_OSC_TRIM) >> SHIFT_BIT_2);

	// Send parameters to sensor
	UssDrivers_Cmds_Transmit(tSensorMask, EX_CMDS_CALIB_WRITE);

	return FUNC_SUCCESS;		
}
//---------------------------------------------------------------------------//
Func_Status_t UssDrivers_Status_Read(Uss_Sensor_Id_t tSensorMask)
{
	// Sensor ACK need about 2ms
	UssDrivers_Cmds_Transmit(tSensorMask, EX_CMDS_READ_STATUS);
	Common_Delay(UNIT_MILLI, 2);	

	return FUNC_SUCCESS;
}
//---------------------------------------------------------------------------//
Func_Status_t UssDrivers_Status_Data_Get(Uss_Sensor_Id_t tSensorMask, uint16 *u16StatusData)
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
//---------------------------------------------------------------------------//
void UssDrivers_Cmds_SedRecEnv_Send(uint16 u16Mask, Uss_Cmds_SendRecEnv tCmds)
{
	switch(tCmds)
	{
		case CMDS_SEND_A:
			if((u16Mask & 0x0001) == MASK_IO1_FLC)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO1_TXRX_FLC, EX_CMDS_SEND_A);		
			}
			if((u16Mask & 0x0002) == MASK_IO2_FLM)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO2_TXRX_FLM, EX_CMDS_SEND_A);		
			}
			if((u16Mask & 0x0004) == MASK_IO3_FRC)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO3_TXRX_FRC, EX_CMDS_SEND_A);		
			}
			if((u16Mask & 0x0008) == MASK_IO4_FRM)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO4_TXRX_FRM, EX_CMDS_SEND_A);		
			}
			if((u16Mask & 0x0010) == MASK_IO5_RLC)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO5_TXRX_RLC, EX_CMDS_SEND_A);		
			}
			if((u16Mask & 0x0020) == MASK_IO6_RLM)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO6_TXRX_RLM, EX_CMDS_SEND_A);		
			}
			if((u16Mask & 0x0040) == MASK_IO7_RRC)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO7_TXRX_RRC, EX_CMDS_SEND_A);		
			}
			if((u16Mask & 0x0080) == MASK_IO8_RRM)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO8_TXRX_RRM, EX_CMDS_SEND_A);		
			}
			if((u16Mask & 0x0100) == MASK_IO9_FLS)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO9_TXRX_FLS, EX_CMDS_SEND_A);		
			}
			if((u16Mask & 0x0200) == MASK_IO10_FRS)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO10_TXRX_FRS, EX_CMDS_SEND_A);		
			}
			if((u16Mask & 0x0400) == MASK_IO11_RLS)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO11_TXRX_RLS, EX_CMDS_SEND_A);		
			}
			if((u16Mask & 0x0800) == MASK_IO12_RRS)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO12_TXRX_RRS, EX_CMDS_SEND_A);		
			}			
		break;
		case CMDS_SEND_B:
			if((u16Mask & 0x0001) == MASK_IO1_FLC)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO1_TXRX_FLC, EX_CMDS_SEND_B);		
			}
			if((u16Mask & 0x0002) == MASK_IO2_FLM)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO2_TXRX_FLM, EX_CMDS_SEND_B);		
			}
			if((u16Mask & 0x0004) == MASK_IO3_FRC)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO3_TXRX_FRC, EX_CMDS_SEND_B);		
			}
			if((u16Mask & 0x0008) == MASK_IO4_FRM)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO4_TXRX_FRM, EX_CMDS_SEND_B);		
			}
			if((u16Mask & 0x0010) == MASK_IO5_RLC)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO5_TXRX_RLC, EX_CMDS_SEND_B);		
			}
			if((u16Mask & 0x0020) == MASK_IO6_RLM)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO6_TXRX_RLM, EX_CMDS_SEND_B);		
			}
			if((u16Mask & 0x0040) == MASK_IO7_RRC)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO7_TXRX_RRC, EX_CMDS_SEND_B);		
			}
			if((u16Mask & 0x0080) == MASK_IO8_RRM)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO8_TXRX_RRM, EX_CMDS_SEND_B);		
			}
			if((u16Mask & 0x0100) == MASK_IO9_FLS)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO9_TXRX_FLS, EX_CMDS_SEND_B);		
			}
			if((u16Mask & 0x0200) == MASK_IO10_FRS)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO10_TXRX_FRS, EX_CMDS_SEND_B);		
			}
			if((u16Mask & 0x0400) == MASK_IO11_RLS)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO11_TXRX_RLS, EX_CMDS_SEND_B);		
			}
			if((u16Mask & 0x0800) == MASK_IO12_RRS)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO12_TXRX_RRS, EX_CMDS_SEND_B);		
			}	
		break;
		case CMDS_SEND_C:
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
		case CMDS_REC_A:
			if((u16Mask & 0x0001) == MASK_IO1_FLC)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO1_TXRX_FLC, EX_CMDS_RECEIVE_A);		
			}
			if((u16Mask & 0x0002) == MASK_IO2_FLM)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO2_TXRX_FLM, EX_CMDS_RECEIVE_A);		
			}
			if((u16Mask & 0x0004) == MASK_IO3_FRC)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO3_TXRX_FRC, EX_CMDS_RECEIVE_A);		
			}
			if((u16Mask & 0x0008) == MASK_IO4_FRM)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO4_TXRX_FRM, EX_CMDS_RECEIVE_A);		
			}
			if((u16Mask & 0x0010) == MASK_IO5_RLC)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO5_TXRX_RLC, EX_CMDS_RECEIVE_A);		
			}
			if((u16Mask & 0x0020) == MASK_IO6_RLM)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO6_TXRX_RLM, EX_CMDS_RECEIVE_A);		
			}
			if((u16Mask & 0x0040) == MASK_IO7_RRC)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO7_TXRX_RRC, EX_CMDS_RECEIVE_A);		
			}
			if((u16Mask & 0x0080) == MASK_IO8_RRM)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO8_TXRX_RRM, EX_CMDS_RECEIVE_A);		
			}
			if((u16Mask & 0x0100) == MASK_IO9_FLS)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO9_TXRX_FLS, EX_CMDS_RECEIVE_A);		
			}
			if((u16Mask & 0x0200) == MASK_IO10_FRS)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO10_TXRX_FRS, EX_CMDS_RECEIVE_A);		
			}
			if((u16Mask & 0x0400) == MASK_IO11_RLS)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO11_TXRX_RLS, EX_CMDS_RECEIVE_A);		
			}
			if((u16Mask & 0x0800) == MASK_IO12_RRS)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO12_TXRX_RRS, EX_CMDS_RECEIVE_A);		
			}					
		break;
		case CMDS_REC_B:
			if((u16Mask & 0x0001) == MASK_IO1_FLC)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO1_TXRX_FLC, EX_CMDS_RECEIVE_B);		
			}
			if((u16Mask & 0x0002) == MASK_IO2_FLM)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO2_TXRX_FLM, EX_CMDS_RECEIVE_B);		
			}
			if((u16Mask & 0x0004) == MASK_IO3_FRC)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO3_TXRX_FRC, EX_CMDS_RECEIVE_B);		
			}
			if((u16Mask & 0x0008) == MASK_IO4_FRM)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO4_TXRX_FRM, EX_CMDS_RECEIVE_B);		
			}
			if((u16Mask & 0x0010) == MASK_IO5_RLC)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO5_TXRX_RLC, EX_CMDS_RECEIVE_B);		
			}
			if((u16Mask & 0x0020) == MASK_IO6_RLM)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO6_TXRX_RLM, EX_CMDS_RECEIVE_B);		
			}
			if((u16Mask & 0x0040) == MASK_IO7_RRC)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO7_TXRX_RRC, EX_CMDS_RECEIVE_B);		
			}
			if((u16Mask & 0x0080) == MASK_IO8_RRM)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO8_TXRX_RRM, EX_CMDS_RECEIVE_B);		
			}
			if((u16Mask & 0x0100) == MASK_IO9_FLS)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO9_TXRX_FLS, EX_CMDS_RECEIVE_B);		
			}
			if((u16Mask & 0x0200) == MASK_IO10_FRS)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO10_TXRX_FRS, EX_CMDS_RECEIVE_B);		
			}
			if((u16Mask & 0x0400) == MASK_IO11_RLS)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO11_TXRX_RLS, EX_CMDS_RECEIVE_B);		
			}
			if((u16Mask & 0x0800) == MASK_IO12_RRS)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO12_TXRX_RRS, EX_CMDS_RECEIVE_B);		
			}				
		break;		
		case CMDS_REC_C:
			if((u16Mask & 0x0001) == MASK_IO1_FLC)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO1_TXRX_FLC, EX_CMDS_RECEIVE_C);		
			}
			if((u16Mask & 0x0002) == MASK_IO2_FLM)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO2_TXRX_FLM, EX_CMDS_RECEIVE_C);		
			}
			if((u16Mask & 0x0004) == MASK_IO3_FRC)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO3_TXRX_FRC, EX_CMDS_RECEIVE_C);		
			}
			if((u16Mask & 0x0008) == MASK_IO4_FRM)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO4_TXRX_FRM, EX_CMDS_RECEIVE_C);		
			}
			if((u16Mask & 0x0010) == MASK_IO5_RLC)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO5_TXRX_RLC, EX_CMDS_RECEIVE_C);		
			}
			if((u16Mask & 0x0020) == MASK_IO6_RLM)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO6_TXRX_RLM, EX_CMDS_RECEIVE_C);		
			}
			if((u16Mask & 0x0040) == MASK_IO7_RRC)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO7_TXRX_RRC, EX_CMDS_RECEIVE_C);		
			}
			if((u16Mask & 0x0080) == MASK_IO8_RRM)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO8_TXRX_RRM, EX_CMDS_RECEIVE_C);		
			}
			if((u16Mask & 0x0100) == MASK_IO9_FLS)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO9_TXRX_FLS, EX_CMDS_RECEIVE_C);		
			}
			if((u16Mask & 0x0200) == MASK_IO10_FRS)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO10_TXRX_FRS, EX_CMDS_RECEIVE_C);		
			}
			if((u16Mask & 0x0400) == MASK_IO11_RLS)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO11_TXRX_RLS, EX_CMDS_RECEIVE_C);		
			}
			if((u16Mask & 0x0800) == MASK_IO12_RRS)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO12_TXRX_RRS, EX_CMDS_RECEIVE_C);		
			}								
		break;
		case CMDS_ENVELOPE_SEND_A:
			if((u16Mask & 0x0001) == MASK_IO1_FLC)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO1_TXRX_FLC, EX_CMDS_ENVELOPE_SEND_A);		
			}
			if((u16Mask & 0x0002) == MASK_IO2_FLM)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO2_TXRX_FLM, EX_CMDS_ENVELOPE_SEND_A);		
			}
			if((u16Mask & 0x0004) == MASK_IO3_FRC)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO3_TXRX_FRC, EX_CMDS_ENVELOPE_SEND_A);		
			}
			if((u16Mask & 0x0008) == MASK_IO4_FRM)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO4_TXRX_FRM, EX_CMDS_ENVELOPE_SEND_A);		
			}
			if((u16Mask & 0x0010) == MASK_IO5_RLC)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO5_TXRX_RLC, EX_CMDS_ENVELOPE_SEND_A);		
			}
			if((u16Mask & 0x0020) == MASK_IO6_RLM)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO6_TXRX_RLM, EX_CMDS_ENVELOPE_SEND_A);		
			}
			if((u16Mask & 0x0040) == MASK_IO7_RRC)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO7_TXRX_RRC, EX_CMDS_ENVELOPE_SEND_A);		
			}
			if((u16Mask & 0x0080) == MASK_IO8_RRM)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO8_TXRX_RRM, EX_CMDS_ENVELOPE_SEND_A);		
			}
			if((u16Mask & 0x0100) == MASK_IO9_FLS)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO9_TXRX_FLS, EX_CMDS_ENVELOPE_SEND_A);		
			}
			if((u16Mask & 0x0200) == MASK_IO10_FRS)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO10_TXRX_FRS, EX_CMDS_ENVELOPE_SEND_A);		
			}
			if((u16Mask & 0x0400) == MASK_IO11_RLS)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO11_TXRX_RLS, EX_CMDS_ENVELOPE_SEND_A);		
			}
			if((u16Mask & 0x0800) == MASK_IO12_RRS)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO12_TXRX_RRS, EX_CMDS_ENVELOPE_SEND_A);		
			}						
		break;
		case CMDS_ENVELOPE_REC_A:
			if((u16Mask & 0x0001) == MASK_IO1_FLC)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO1_TXRX_FLC, EX_CMDS_ENVELOPE_REC_A);		
			}
			if((u16Mask & 0x0002) == MASK_IO2_FLM)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO2_TXRX_FLM, EX_CMDS_ENVELOPE_REC_A);		
			}
			if((u16Mask & 0x0004) == MASK_IO3_FRC)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO3_TXRX_FRC, EX_CMDS_ENVELOPE_REC_A);		
			}
			if((u16Mask & 0x0008) == MASK_IO4_FRM)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO4_TXRX_FRM, EX_CMDS_ENVELOPE_REC_A);		
			}
			if((u16Mask & 0x0010) == MASK_IO5_RLC)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO5_TXRX_RLC, EX_CMDS_ENVELOPE_REC_A);		
			}
			if((u16Mask & 0x0020) == MASK_IO6_RLM)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO6_TXRX_RLM, EX_CMDS_ENVELOPE_REC_A);		
			}
			if((u16Mask & 0x0040) == MASK_IO7_RRC)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO7_TXRX_RRC, EX_CMDS_ENVELOPE_REC_A);		
			}
			if((u16Mask & 0x0080) == MASK_IO8_RRM)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO8_TXRX_RRM, EX_CMDS_ENVELOPE_REC_A);		
			}
			if((u16Mask & 0x0100) == MASK_IO9_FLS)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO9_TXRX_FLS, EX_CMDS_ENVELOPE_REC_A);		
			}
			if((u16Mask & 0x0200) == MASK_IO10_FRS)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO10_TXRX_FRS, EX_CMDS_ENVELOPE_REC_A);		
			}
			if((u16Mask & 0x0400) == MASK_IO11_RLS)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO11_TXRX_RLS, EX_CMDS_ENVELOPE_REC_A);		
			}
			if((u16Mask & 0x0800) == MASK_IO12_RRS)
			{
				UssDrivers_Cmds_Transmit(USS_ID_IO12_TXRX_RRS, EX_CMDS_ENVELOPE_REC_A);		
			}								
		break;
	}
}
//---------------------------------------------------------------------------//
Func_Status_t UssDrivers_SndRecEnv_Detect(Uss_Detect_Mode_t tMode, uint16 u16SendMask, uint16 u16RecMask, uint16 u16DetTime_us)
{
	gu16SendMask = u16SendMask;
	gu16RecMask = u16RecMask;
	gu32DetectTimeLen = u16DetTime_us;
	(void)(gu32DetectTimeLen);

	switch(tMode)
	{
		case MODE_SEND_REC_A:
			// Sensor ACK need about 17ms
			UssDrivers_Cmds_SedRecEnv_Send(gu16SendMask, CMDS_SEND_A);
			Common_Delay(UNIT_MILLI, 17);	

			// Sensor ACK need about 17ms
			UssDrivers_Cmds_SedRecEnv_Send(gu16RecMask, CMDS_REC_A);	
			Common_Delay(UNIT_MILLI, 17);	
		break;
		case MODE_SEND_REC_B:
			// Sensor ACK need about 11ms
			UssDrivers_Cmds_SedRecEnv_Send(gu16SendMask, CMDS_SEND_B);
			Common_Delay(UNIT_MILLI, 11);

			// Sensor ACK need about 11ms
			UssDrivers_Cmds_SedRecEnv_Send(gu16RecMask, CMDS_REC_B);	
			Common_Delay(UNIT_MILLI, 11);
		break;
		case MODE_SEND_REC_C:
			// Sensor ACK need about 37ms
			UssDrivers_Cmds_SedRecEnv_Send(gu16SendMask, CMDS_SEND_C);
			Common_Delay(UNIT_MILLI, 37);

			// Sensor ACK need about 37ms
			UssDrivers_Cmds_SedRecEnv_Send(gu16RecMask, CMDS_REC_C);	
			Common_Delay(UNIT_MILLI, 37);
		break;
		case MODE_ENVELOPE:
			UssDrivers_Cmds_SedRecEnv_Send(gu16SendMask, CMDS_ENVELOPE_SEND_A);
			Common_Delay(UNIT_MILLI, 1);
			UssDrivers_Cmds_SedRecEnv_Send(gu16RecMask, CMDS_ENVELOPE_REC_A);
			Common_Delay(UNIT_MILLI, 1);
		break;
	}

	return FUNC_SUCCESS;
}
//---------------------------------------------------------------------------//
Func_Status_t UssDrivers_Bilat_Get(Uss_Sensor_Id_t tSensorMask, Uss_Cmds_SendRecEnv tCmd, uint32 *u32BilateralT)
{
    Func_Status_t Status = FUNC_FAIL;
	
	if(gbAckRecFinishFlag == TRUE)
	{
	switch(tCmd)
	{
		case CMDS_SEND_A:
			for(uint32 u32Index=0; u32Index<SIZE_SND_REC; u32Index++)
				u32BilateralT[u32Index] = gtUssRxAckData[tSensorMask].u32SendAT[u32Index];			
		break;
		case CMDS_SEND_B:
			for(uint32 u32Index=0; u32Index<SIZE_SND_REC; u32Index++)
				u32BilateralT[u32Index] = gtUssRxAckData[tSensorMask].u32SendBT[u32Index];	
		break;
		case CMDS_SEND_C:
			for(uint32 u32Index=0; u32Index<SIZE_SND_REC; u32Index++)
				u32BilateralT[u32Index] = gtUssRxAckData[tSensorMask].u32SendCT[u32Index];	
		break;
		case CMDS_REC_A:
			for(uint32 u32Index=0; u32Index<SIZE_SND_REC; u32Index++)
				u32BilateralT[u32Index] = gtUssRxAckData[tSensorMask].u32ReceiveAT[u32Index];				
		break;
		case CMDS_REC_B:
			for(uint32 u32Index=0; u32Index<SIZE_SND_REC; u32Index++)
				u32BilateralT[u32Index] = gtUssRxAckData[tSensorMask].u32ReceiveBT[u32Index];			
		break;
		case CMDS_REC_C:
			for(uint32 u32Index=0; u32Index<SIZE_SND_REC; u32Index++)
				u32BilateralT[u32Index] = gtUssRxAckData[tSensorMask].u32ReceiveCT[u32Index];			
		break;
		case CMDS_ENVELOPE_SEND_A:
			// Reserved, No ACK		
		break;
		case CMDS_ENVELOPE_REC_A:
			// Reserved, No ACK
		break;		
	}
	
		gbAckRecFinishFlag = FALSE;
		Status = FUNC_SUCCESS;
	}
	else
	{
		Status = FUNC_FAIL;
	}	

	return Status;
}
//---------------------------------------------------------------------------//
Func_Status_t UssDrivers_Sensors_Temp_Read(Uss_Sensor_Id_t tSensorMask)
{
	// Sensor ACK need about 2ms
	UssDrivers_Cmds_Transmit(tSensorMask, EX_CMDS_READ_TEMP);
	Common_Delay(UNIT_MILLI, 2);

	return FUNC_SUCCESS;
}
//---------------------------------------------------------------------------//
Func_Status_t UssDrivers_Temperature_Get(Uss_Sensor_Id_t tSensorMask, uint16 *u16Temp)
{
    Func_Status_t Status = FUNC_FAIL;
	
	if(gbAckRecFinishFlag == TRUE)
	{
	*u16Temp = gtUssRxAckData[tSensorMask].u16ReadTemp;

		gbAckRecFinishFlag = FALSE;
		Status = FUNC_SUCCESS;
	}
	else
	{
		Status = FUNC_FAIL;
	}	

	return Status;
}
//---------------------------------------------------------------------------//
Func_Status_t UssDrivers_Sensors_Calib_Read(Uss_Sensor_Id_t tSensorMask)
{
	// Sensor ACK need about 6ms
	UssDrivers_Cmds_Transmit(tSensorMask, EX_CMDS_CALIB_READ);
	Common_Delay(UNIT_MILLI, 6);

	return FUNC_SUCCESS;
}
//---------------------------------------------------------------------------//
Func_Status_t UssDrivers_Calib_Get(Uss_Sensor_Id_t tSensorMask, Uss_Calib_Data_t *tCalibData)
{
    Func_Status_t Status = FUNC_FAIL;
	
	if(gbAckRecFinishFlag == TRUE)
	{
	(*tCalibData).u8F_Drv = (uint8)(gtUssRxAckData[tSensorMask].u32CalibRead[0] & MASK_R_F_DRV);
	(*tCalibData).u8I_Drv = (uint8)((gtUssRxAckData[tSensorMask].u32CalibRead[0] & MASK_R_I_DRV) >> SHIFT_BIT_8);
	(*tCalibData).u8G_Ana = (uint8)((gtUssRxAckData[tSensorMask].u32CalibRead[0] & MASK_R_G_ANA) >> SHIFT_BIT_13);
	(*tCalibData).u8G_Dig = (uint8)((gtUssRxAckData[tSensorMask].u32CalibRead[0] & MASK_R_G_DIG) >> SHIFT_BIT_16);
	(*tCalibData).u8Customer_Bits = (uint8)((gtUssRxAckData[tSensorMask].u32CalibRead[0] & MASK_R_CUSTOMER_BITS) >> SHIFT_BIT_23);
	(*tCalibData).u8Osc_Trim = (uint8)(((gtUssRxAckData[tSensorMask].u32CalibRead[0] & MASK_R_OSC_TRIM_L) >> SHIFT_BIT_30) | 
								((gtUssRxAckData[tSensorMask].u32CalibRead[1] & MASK_R_OSC_TRIM_H) << SHIFT_BIT_2));
	
		gbAckRecFinishFlag = FALSE;
		Status = FUNC_SUCCESS;
	}
	else
	{
		Status = FUNC_FAIL;
	}	

	return Status;
}
//---------------------------------------------------------------------------//
Func_Status_t UssDrivers_Sensors_EEPROM_Read(Uss_Sensor_Id_t tSensorMask)		
{
	// Sensor ACK need about 16ms
	UssDrivers_Cmds_Transmit(tSensorMask, EX_CMDS_EE_READ);	
	Common_Delay(UNIT_MILLI, 16);

	return FUNC_SUCCESS;
}
//---------------------------------------------------------------------------//
Func_Status_t UssDrivers_EEPROM_Data_Get(Uss_Sensor_Id_t tSensorMask, Uss_Calib_Data_t *tEepromData)
{	
    Func_Status_t Status = FUNC_FAIL;
	
	if(gbAckRecFinishFlag == TRUE)
	{
	// the third packets (EEPROM_data + VPROM_Bit)
	(*tEepromData).u8Vprog_Status = (uint8)(gtUssRxAckData[tSensorMask].u32EeRead[0] & MASK_R_VPROM_STATUS);
	(*tEepromData).u8F_Drv = (uint8)((gtUssRxAckData[tSensorMask].u32EeRead[0] & (MASK_R_F_DRV << 1)) >> SHIFT_BIT_1);
	(*tEepromData).u8I_Drv = (uint8)((gtUssRxAckData[tSensorMask].u32EeRead[0] & (MASK_R_I_DRV << 1)) >> SHIFT_BIT_9);
	(*tEepromData).u8G_Ana = (uint8)((gtUssRxAckData[tSensorMask].u32EeRead[0] & (MASK_R_G_ANA << 1)) >> SHIFT_BIT_14);
	(*tEepromData).u8G_Dig = (uint8)((gtUssRxAckData[tSensorMask].u32EeRead[0] & (MASK_R_G_DIG << 1)) >> SHIFT_BIT_17);
	(*tEepromData).u8Customer_Bits = (uint8)((gtUssRxAckData[tSensorMask].u32EeRead[0] & (MASK_R_CUSTOMER_BITS << 1)) >> SHIFT_BIT_24);
	(*tEepromData).u8Osc_Trim = (uint8)(((gtUssRxAckData[tSensorMask].u32EeRead[0] & MASK_R_EE_OSC_TRIM_L) >> SHIFT_BIT_31) | 
								((gtUssRxAckData[tSensorMask].u32EeRead[1] & MASK_R_EE_OSC_TRIM_H) << SHIFT_BIT_1));
		
		gbAckRecFinishFlag = FALSE;
		Status = FUNC_SUCCESS;
	}
	else
	{
		Status = FUNC_FAIL;
	}	

	return Status;
}
//---------------------------------------------------------------------------//
Func_Status_t UssDrivers_Meas_Setup_Read(Uss_Sensor_Id_t tSensorMask)
{
	// Sensor ACK need about 6ms    
	UssDrivers_Cmds_Transmit(tSensorMask, EX_CMDS_READ_MEAS_SETUP);
	Common_Delay(UNIT_MILLI, 6);

	return FUNC_SUCCESS;
}
//---------------------------------------------------------------------------//
Func_Status_t UssDrivers_Meas_Setup_Get(Uss_Sensor_Id_t tSensorMask, Uss_Meas_Data_t *tMeasPara) 
{
    Func_Status_t Status = FUNC_FAIL;
	
	if(gbAckRecFinishFlag == TRUE)
	{
	(*tMeasPara).u8npulses_a = (uint8)((gtUssRxAckData[tSensorMask].u32ReadMeasSetup[1] & MASK_READMEAS_B1_NPULSES_A) >> SHIFT_BIT_1);
	(*tMeasPara).u8tmeas_a = (uint8)((gtUssRxAckData[tSensorMask].u32ReadMeasSetup[1] & MASK_READMEAS_B1_TMEAS_A_H) |
								((gtUssRxAckData[tSensorMask].u32ReadMeasSetup[0] & MASK_READMEAS_B0_TMEAS_A_L) >> SHIFT_BIT_30));
	(*tMeasPara).u8thresscale_a = (uint8)(((gtUssRxAckData[tSensorMask].u32ReadMeasSetup[0] & MASK_READMEAS_B0_THRESSCALE_A) >> SHIFT_BIT_28));
	(*tMeasPara).u8npulses_b = (uint8)((gtUssRxAckData[tSensorMask].u32ReadMeasSetup[0] & MASK_READMEAS_BO_NPULSES_B) >> SHIFT_BIT_25);
	(*tMeasPara).u8tmeas_b = (uint8)((gtUssRxAckData[tSensorMask].u32ReadMeasSetup[0] & MASK_READMEAS_B0_TMEAS_B) >> SHIFT_BIT_22);
	(*tMeasPara).u8thresscale_b = (uint8)((gtUssRxAckData[tSensorMask].u32ReadMeasSetup[0] & MASK_READMEAS_B0_THRESSCALE_B) >> SHIFT_BIT_20);
	(*tMeasPara).u8npulses_c = (uint8)((gtUssRxAckData[tSensorMask].u32ReadMeasSetup[0] & MASK_READMEAS_BO_NPULSES_C) >> SHIFT_BIT_17);
	(*tMeasPara).u8tmeas_c = (uint8)((gtUssRxAckData[tSensorMask].u32ReadMeasSetup[0] & MASK_READMEAS_B0_TMEAS_C) >> SHIFT_BIT_14);
	(*tMeasPara).u8thresscale_c = (uint8)((gtUssRxAckData[tSensorMask].u32ReadMeasSetup[0] & MASK_READMEAS_B0_THRESSCALE_C) >> SHIFT_BIT_12);
	(*tMeasPara).u8echo_deb = (uint8)((gtUssRxAckData[tSensorMask].u32ReadMeasSetup[0] & MASK_READMEAS_B0_ECHO_DEB) >> SHIFT_BIT_11);
	(*tMeasPara).u8rt_cfg = (uint8)((gtUssRxAckData[tSensorMask].u32ReadMeasSetup[0] & MASK_READMEAS_B0_RT_CFG) >> SHIFT_BIT_10);
	(*tMeasPara).u8nftg = (uint8)((gtUssRxAckData[tSensorMask].u32ReadMeasSetup[0] & MASK_READMEAS_B0_NFTG) >> SHIFT_BIT_9);
	(*tMeasPara).u8ftc = (uint8)((gtUssRxAckData[tSensorMask].u32ReadMeasSetup[0] & MASK_READMEAS_B0_FTC) >> SHIFT_BIT_8);
 	(*tMeasPara).u8epd = (uint8)((gtUssRxAckData[tSensorMask].u32ReadMeasSetup[0] & MASK_READMEAS_B0_EPD) >> SHIFT_BIT_7);
	(*tMeasPara).u8stc_cfg = (uint8)((gtUssRxAckData[tSensorMask].u32ReadMeasSetup[0] & MASK_READMEAS_BO_STC_CFG) >> SHIFT_BIT_5);
	(*tMeasPara).u8stc_start = (uint8)((gtUssRxAckData[tSensorMask].u32ReadMeasSetup[0] & MASK_READMEAS_B0_STC_START) >> SHIFT_BIT_3);
	(*tMeasPara).u8noise_cfg = (uint8)((gtUssRxAckData[tSensorMask].u32ReadMeasSetup[0] & MASK_READMEAS_B0_NOISE_CFG) >> SHIFT_BIT_1);
	(*tMeasPara).u8filter_cfg = (uint8)((gtUssRxAckData[tSensorMask].u32ReadMeasSetup[0] & MASK_READMEAS_B0_FILTER_CFG));

		gbAckRecFinishFlag = FALSE;
		Status = FUNC_SUCCESS;
	}
	else
	{
		Status = FUNC_FAIL;
	}	

	return Status;
}
//---------------------------------------------------------------------------//
Func_Status_t UssDrivers_Sensors_Thres_Read(Uss_Sensor_Id_t tSensorMask)		
{
	// Sensor ACK need about 11ms
	UssDrivers_Cmds_Transmit(tSensorMask, EX_CMDS_READ_THRES_SETUP);
	Common_Delay(UNIT_MILLI, 11);	

	return FUNC_SUCCESS;
}
//---------------------------------------------------------------------------//
Func_Status_t UssDrivers_Thres_Para_Get(Uss_Sensor_Id_t tSensorMask, Uss_Thres_Data_t *tThresholdData)
{
    Func_Status_t Status = FUNC_FAIL;
	
	if(gbAckRecFinishFlag == TRUE)
	{
	(*tThresholdData).u8Thval[NUM_THVAL_2] = (uint8)(gtUssRxAckData[tSensorMask].u32ReadThresSetup[2] & MASK_R_B2_THVAL_2);
	(*tThresholdData).u8Thval[NUM_THVAL_3] = (uint8)((gtUssRxAckData[tSensorMask].u32ReadThresSetup[1] & MASK_R_B1_THVAL_3) >> SHIFT_BIT_27);
	(*tThresholdData).u8Thval[NUM_THVAL_4] = (uint8)((gtUssRxAckData[tSensorMask].u32ReadThresSetup[1] & MASK_R_B1_THVAL_4) >> SHIFT_BIT_22);
	(*tThresholdData).u8Thval[NUM_THVAL_5] = (uint8)((gtUssRxAckData[tSensorMask].u32ReadThresSetup[1] & MASK_R_B1_THVAL_5) >> SHIFT_BIT_17);
	(*tThresholdData).u8Thval[NUM_THVAL_6] = (uint8)((gtUssRxAckData[tSensorMask].u32ReadThresSetup[1] & MASK_R_B1_THVAL_6) >> SHIFT_BIT_12);
	(*tThresholdData).u8Thval[NUM_THVAL_7] = (uint8)((gtUssRxAckData[tSensorMask].u32ReadThresSetup[1] & MASK_R_B1_THVAL_7) >> SHIFT_BIT_7);
	(*tThresholdData).u8Thval[NUM_THVAL_8] = (uint8)((gtUssRxAckData[tSensorMask].u32ReadThresSetup[1] & MASK_R_B1_THVAL_8) >> SHIFT_BIT_2);
	(*tThresholdData).u8Thval[NUM_THVAL_9] = (uint8)(((gtUssRxAckData[tSensorMask].u32ReadThresSetup[1] & MASK_R_B1_THVAL_9_H) << SHIFT_BIT_3) |
												((gtUssRxAckData[tSensorMask].u32ReadThresSetup[0] & MASK_R_B0_THVAL_9_L) >> SHIFT_BIT_29));
	(*tThresholdData).u8Thval[NUM_THVAL_10] = (uint8)((gtUssRxAckData[tSensorMask].u32ReadThresSetup[0] & MASK_R_B0_THVAL_10) >> SHIFT_BIT_24);
	(*tThresholdData).u8Thval[NUM_THVAL_11] = (uint8)((gtUssRxAckData[tSensorMask].u32ReadThresSetup[0] & MASK_R_B0_THVAL_11) >> SHIFT_BIT_19);	
	(*tThresholdData).u8Thval[NUM_THVAL_12] = (uint8)((gtUssRxAckData[tSensorMask].u32ReadThresSetup[0] & MASK_R_B0_THVAL_12) >> SHIFT_BIT_14);
	(*tThresholdData).u8Thval[NUM_THVAL_13] = (uint8)((gtUssRxAckData[tSensorMask].u32ReadThresSetup[0] & MASK_R_B0_THVAL_13) >> SHIFT_BIT_9);
	(*tThresholdData).u8Thsft_Cfg = (uint8)((gtUssRxAckData[tSensorMask].u32ReadThresSetup[0] & MASK_R_B0_THSFT_CFG) >> SHIFT_BIT_7);
	(*tThresholdData).u8Atg_Cfg = (uint8)((gtUssRxAckData[tSensorMask].u32ReadThresSetup[0] & MASK_R_B0_ATG_CFG) >> SHIFT_BIT_5);
	(*tThresholdData).u8Atg_Tau = (uint8)((gtUssRxAckData[tSensorMask].u32ReadThresSetup[0] & MASK_R_B0_ATG_TAU) >> SHIFT_BIT_3);
	(*tThresholdData).u8Atg_Alpha = (uint8)((gtUssRxAckData[tSensorMask].u32ReadThresSetup[0] & MASK_R_B0_ATG_ALPHA) >> SHIFT_BIT_2);
	(*tThresholdData).u8Thresscale_Rec = (uint8)(gtUssRxAckData[tSensorMask].u32ReadThresSetup[0] & MASK_R_B0_THRESSCALE_REC);

		gbAckRecFinishFlag = FALSE;
		Status = FUNC_SUCCESS;
	}
	else
	{
		Status = FUNC_FAIL;
	}	

	return Status;
}
//---------------------------------------------------------------------------//
Func_Status_t UssDrivers_EEPROM_Copy(Uss_Sensor_Id_t tSensorMask)
{
	UssDrivers_Cmds_Transmit(tSensorMask, EX_CMDS_EE_COPY);	

	return FUNC_SUCCESS;
}
//---------------------------------------------------------------------------//
void UssDrivers_Rx_Data_Store(Uss_Sensor_Id_t tSensorMask, Uss_Exchange_Cmds u8Cmd)
{
	switch(u8Cmd)
	{
		case EX_CMDS_SEND_A:
			for(uint32 u32Index=0; u32Index<gu32RxTagTCnt; u32Index++)
			{
				gtUssRxAckData[tSensorMask].u32SendAT[u32Index] = gu32TimeTagTemp[u32Index];
			}
			
			gbAckRecFinishFlag = TRUE;
			
			#if DEBUG_UART
				DEBUG_MSG([UssRx] SEND_A OK);				
			#endif
		break;
		case EX_CMDS_RECEIVE_A:
			for(uint32 u32Index=0; u32Index<gu32RxTagTCnt; u32Index++)
			{
				gtUssRxAckData[tSensorMask].u32ReceiveAT[u32Index] = gu32TimeTagTemp[u32Index];
			}	
			
			gbAckRecFinishFlag = TRUE;
			
			#if DEBUG_UART
				DEBUG_MSG([UssRx] RECEIVE_A OK);
			#endif
		break;
		case EX_CMDS_SEND_B:
			for(uint32 u32Index=0; u32Index<gu32RxTagTCnt; u32Index++)
			{
				gtUssRxAckData[tSensorMask].u32SendBT[u32Index] = gu32TimeTagTemp[u32Index];
			}	
			
			gbAckRecFinishFlag = TRUE;
			
			#if DEBUG_UART
				DEBUG_MSG([UssRx] SEND_B OK);
			#endif
		break;
		case EX_CMDS_RECEIVE_B:
			for(uint32 u32Index=0; u32Index<gu32RxTagTCnt; u32Index++)
			{
				gtUssRxAckData[tSensorMask].u32ReceiveBT[u32Index] = gu32TimeTagTemp[u32Index];
			}	
			
			gbAckRecFinishFlag = TRUE;
			
			#if DEBUG_UART
				DEBUG_MSG([UssRx] RECEIVE_B OK);
			#endif
		break;
		case EX_CMDS_SEND_C:
			for(uint32 u32Index=0; u32Index<gu32RxTagTCnt; u32Index++)
			{
				gtUssRxAckData[tSensorMask].u32SendCT[u32Index] = gu32TimeTagTemp[u32Index];
			}	

			gbAckRecFinishFlag = TRUE;

			#if DEBUG_UART
				DEBUG_MSG([UssRx] SEND_C OK);
			#endif
		break;
		case EX_CMDS_RECEIVE_C:
			for(uint32 u32Index=0; u32Index<gu32RxTagTCnt; u32Index++)
			{
				gtUssRxAckData[tSensorMask].u32ReceiveCT[u32Index] = gu32TimeTagTemp[u32Index];
			}	

			gbAckRecFinishFlag = TRUE;

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

			gbAckRecFinishFlag = TRUE;

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

			gbAckRecFinishFlag = TRUE;

			#if DEBUG_UART
				DEBUG_MSG([UssRx] READ_MEAS_SETUP OK);
			#endif
		break;
		case EX_CMDS_READ_STATUS:
			gtUssRxAckData[tSensorMask].u32ReadStatus = (gtUssRxAckData[tSensorMask].u32ReadStatus & (~UNIT_U32)) | gu32UssRxBitsTemp[0];

			gbAckRecFinishFlag = TRUE;
		
			#if DEBUG_UART
				DEBUG_MSG([UssRx] READ_STATUS OK);
			#endif
		break;
		case EX_CMDS_CAL_PULSES:
			// Reserved, no ACK
		break;
		case EX_CMDS_READ_TEMP:
			gtUssRxAckData[tSensorMask].u16ReadTemp = (gtUssRxAckData[tSensorMask].u16ReadTemp & (~UNIT_U16)) | (uint16)(gu32UssRxBitsTemp[0]);

			gbAckRecFinishFlag = TRUE;

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

			gbAckRecFinishFlag = TRUE;

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
			gtUssRxAckData[tSensorMask].u32EeRead[2] = (gtUssRxAckData[tSensorMask].u32EeRead[2] & (~UNIT_U32)) | gu32UssRxBitsTemp[2];
			gtUssRxAckData[tSensorMask].u32EeRead[3] = (gtUssRxAckData[tSensorMask].u32EeRead[3] & (~UNIT_U32)) | gu32UssRxBitsTemp[3];

			gbAckRecFinishFlag = TRUE;

			#if DEBUG_UART
				DEBUG_MSG([UssRx] EE_READ OK);
			#endif
		break;
		case EX_CMDS_READ_ID:
			gtUssRxAckData[tSensorMask].u32ReadId = (gtUssRxAckData[tSensorMask].u32ReadId & (~UNIT_U32)) | gu32UssRxBitsTemp[0];

			gbAckRecFinishFlag = TRUE;

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
	uint32 u32AckTotalLen = VAL_INIT;
	uint32 u32ShiftTotalLen = VAL_INIT;
	
	if(bFlag == TRUE)	
	{
		if((gu8UssTxCmd >= EX_CMDS_SEND_A) && (gu8UssTxCmd <= EX_CMDS_RECEIVE_C))
		{	
			// Record tag time
			gu32UssRxAckLen = gu32RxTagTCnt;
			UssDrivers_Rx_Data_Store(tUssSensorId, gu8UssTxCmd);
			memset(&gu32TimeTagTemp, VAL_INIT, sizeof(gu32TimeTagTemp));							// Clear buffer
			//Common_Buffer_Clear(&gu32TimeTagTemp[0], sizeof(gu32TimeTagTemp)/sizeof(uint32));		// Clear buffer

			gu32DetectTimeLen = VAL_INIT;
			gu32UssRxAckLen = VAL_INIT;
		}
		else
		{
			// Capture time transfer to logic value 
			for(u32Index = VAL_INIT; u32Index < (gu32UssRxAckLen - gu32TxFilterLen); u32Index++)
			{
				u32AckTotalLen = gu32UssRxAckLen - gu32TxFilterLen;
				u32UnitShift = (u32AckTotalLen - u32Index) / UNIT_A_U32;				
				u32ShiftTotalLen = u32AckTotalLen - u32Index - (UNIT_A_U32 * u32UnitShift);
				u32IndexTemp = (u32AckTotalLen - u32Index) / UNIT_A_U32;
				u32BitShift = u32ShiftTotalLen - ZERO_START;

				#if EVB_DEMO
					if((gu32LowPulseTemp[u32Index + gu32TxFilterLen] >= SYM_RX_T_BIT1_MIN) && (gu32LowPulseTemp[u32Index + gu32TxFilterLen] < SYM_RX_T_BIT1_MAX))
					{				
						gu32UssRxBitsTemp[u32IndexTemp] = (gu32UssRxBitsTemp[u32IndexTemp]) | (USS_SIGNAL_1 << u32BitShift);
					}
					else if((gu32LowPulseTemp[u32Index + gu32TxFilterLen] >= SYM_RX_T_BIT0_MIN) && (gu32LowPulseTemp[u32Index + gu32TxFilterLen] < SYM_RX_T_BIT0_MAX))
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
			memset(&gu32LowPulseTemp, VAL_INIT, sizeof(gu32LowPulseTemp));	// Clear buffer
			#else
			memset(&gu32InvertHighPulseTemp, VAL_INIT, sizeof(gu32InvertHighPulseTemp));	// Clear buffer
			#endif
		}		
		
		gbIsrRxFinishFlag = FALSE;
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
boolean UssDrivers_IsrRxFinishFlag_Get(void)
{
	return gbIsrRxFinishFlag;
}
//---------------------------------------------------------------------------//
void UssDrivers_IsrRxFinishFlag_Set(boolean bFlagStatus)
{
	gbIsrRxFinishFlag = bFlagStatus;
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
uint16 UssDrivers_SendMask_Get(void)
{
	return gu16SendMask;
}
//---------------------------------------------------------------------------//
void UssDrivers_SendMask_Set(uint16 u16SndMask)
{
	gu16SendMask = u16SndMask;
}
//---------------------------------------------------------------------------//
uint16 UssDrivers_RecMask_Get(void)
{
	return gu16RecMask;
}
//---------------------------------------------------------------------------//
void UssDrivers_RecMask_Set(uint16 u16RecMask)
{
	gu16RecMask = u16RecMask;
}
//---------------------------------------------------------------------------//
uint32 UssDrivers_RxTagTCnt_Get(void)
{
	return gu32RxTagTCnt;
}
//---------------------------------------------------------------------------//
void UssDrivers_RxTagTCnt_Set(uint32 u32Cnt)
{
	gu32RxTagTCnt = u32Cnt;
}
//---------------------------------------------------------------------------//


