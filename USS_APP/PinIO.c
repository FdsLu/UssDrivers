/******************************************************************************
;       Program		: 	PinIO.c
;       Function	: 	GPIO sensing
;       Chip		: 	Infineon TC397
;       Clock		: 	Internal SYSPLL 300MHz
;       Date		: 	2023 / 1 / 2
;       Author		: 	Fenderson Lu
;       Describe	: 	ERU interrupt pins
;						(1) USS_IO_RX1 = 
;						(2) USS_IO_RX2 = P.20.9
;
******************************************************************************/
//---------------------------- Include File ----------------------------------//
#include "Cpu0_Main.h"
#include "UssDrivers.h"
#include "Timer_Gtm.h"
//---------------------------- Declare Global Variable -----------------------//
ERUconfig gtEruIsrConfig; 
uint32 u32InvertHighStartT, u32InvertHighEndT, u32InvertHighTotalT;
uint32 u32PreLowStartT;
uint32 gu32TagStartT, gu32PreTagStartT, gu32TagEndT, gu32PreTagStartT2, gu32TagTotalT;
uint8 gu8RxAckIndex;
uint32 gu32StartTimeTag = MODE_ENABLE;
uint32 gu32PwrOnDebounceTime = INIT_ISR;
#if EVB_DEMO
uint32 u32LowStartT, u32LowEndT, u32LowTotalT;
#endif
//---------------------------- Start Program ---------------------------------//
IFX_INTERRUPT(SCUERU_Int0_Handler, 0, ISR_ERU_PRIORITY);
void SCUERU_Int0_Handler(void)
{
	#if (EVB_DEMO == FALSE)
	if(Common_Debounce_Counter(&gu32PwrOnDebounceTime) == TRUE)
	{
	#endif
		if((UssDrivers_RxIsrFinishFlag_Get() == FALSE) && (UssDrivers_UssRxAckEnFlag_Get() == TRUE))
		{
			if((UssDrivers_UssTxCmd_Get() >= EX_CMDS_SEND_A) && (UssDrivers_UssTxCmd_Get() <= EX_CMDS_RECEIVE_C))
			{
				// tag pulses timing
				if(Common_Down_Counter(&gu32StartTimeTag))
				{
					Timer_Gtm_RealTimer_Clear();
					gu32TagStartT = Timer_Gtm_RealTimer_Get();
					gu32PreTagStartT = gu32TagStartT;		
					gu32TimeTagTemp[gu8RxAckIndex++] = gu32TagStartT - gu32PreTagStartT;
				}
				else
				{
					gu32TagStartT = Timer_Gtm_RealTimer_Get();
					gu32TimeTagTemp[gu8RxAckIndex++] = gu32TagStartT - gu32PreTagStartT;
					gu32TagTotalT = gu32TagStartT - gu32PreTagStartT;
				}
				
				// Receive data finish check
				if(gu32TagTotalT >= UssDrivers_DetectTimeLen_Get())				
				{
					UssDrivers_RxTagTCnt_Set(gu8RxAckIndex-1);
					gu32TagTotalT = 0;
					gu8RxAckIndex = 0;
					gu32TagStartT = 0;
					gu32PreTagStartT = 0;
					gu32StartTimeTag = MODE_ENABLE;
					UssDrivers_RxIsrFinishFlag_Set(TRUE);
				}	
			}
			else
			{
				// Other commands, calculate low pulses time length
				#if EVB_DEMO
					if(IfxPort_getPinState(portUSS_ISR_TEST, pinUSS_ISR_TEST) == PIN_LEVEL_LOW)
					{		
						u32LowStartT = Timer_Gtm_RealTimer_Get();		// Record trigger start time	
					}
					else if(IfxPort_getPinState(portUSS_ISR_TEST, pinUSS_ISR_TEST) == PIN_LEVEL_HIGH)
					{			
						u32LowEndT = Timer_Gtm_RealTimer_Get(); 		// Record trigger end time
						u32LowTotalT = u32LowEndT - u32LowStartT; 
						
						if(gu8RxAckIndex < UssDrivers_RxAckLen_Get())
						{
							gu8LowPulseTemp[gu8RxAckIndex++] = (uint8)u32LowTotalT;
						}
						else
						{
							gu8RxAckIndex = 0;
							UssDrivers_RxIsrFinishFlag_Set(TRUE);
						}
						
						u32LowTotalT = 0;
						u32LowStartT = 0;
						u32LowEndT = 0; 			
					}	
				#else			
					if(IfxPort_getPinState(portUSS_IO_RX2, pinUSS_IO_RX2) == PIN_LEVEL_HIGH)
					{		
						u32InvertHighStartT = Timer_Gtm_RealTimer_Get();	// Record trigger start time	
					}
					else if(IfxPort_getPinState(portUSS_IO_RX2, pinUSS_IO_RX2) == PIN_LEVEL_LOW)
					{			
						u32InvertHighEndT = Timer_Gtm_RealTimer_Get();		// Record trigger end time
						u32InvertHighTotalT = u32InvertHighEndT - u32InvertHighStartT; 
						
						if(gu8RxAckIndex < UssDrivers_RxAckLen_Get())
						{
							gu32InvertHighPulseTemp[gu8RxAckIndex++] = u32InvertHighTotalT;
						}
						else
						{
							gu8RxAckIndex = 0;
							UssDrivers_RxIsrFinishFlag_Set(TRUE);
						}
						
						u32InvertHighTotalT = 0;
						u32InvertHighStartT = 0;
						u32InvertHighEndT = 0;				
					}		
				#endif
			}		
			
			IfxScuEru_clearEventFlag(gtEruIsrConfig.inputChannel);	
		}
	#if (EVB_DEMO == FALSE)	
	}
	#endif
}
//-------------------------------------------------------------------------//
void PinIO_Init(void)
{
	#if EVB_DEMO
	IfxPort_setPinModeOutput(portLED_D107, pinLED_D107, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);		// Initialize the LED

	IfxPort_setPinHigh(portUSS_IO_TX1, pinUSS_IO_TX1);
	#endif

	IfxPort_setPinModeOutput(portUSS_IO_TX1, pinUSS_IO_TX1, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
	IfxPort_setPinModeOutput(portUSS_IO_TX2, pinUSS_IO_TX2, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);

	IfxPort_setPinModeOutput(portPOWER_EN_5141, pinPOWER_EN_5141, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);	
	IfxPort_setPinModeOutput(portUSS_PWR_EN_TC397, pinUSS_PWR_EN_TC397, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);

	PinIO_PeripheralsAndERU_Init();
}
//-------------------------------------------------------------------------//
void PinIO_PeripheralsAndERU_Init(void)
{
	#if EVB_DEMO
	gtEruIsrConfig.reqPin = &reqUSS_ISR_TEST;	// Select external request pin     
	#else
	gtEruIsrConfig.reqPin = &reqUSS_IO_RX2;	// Select external request pin 
	#endif
	IfxScuEru_initReqPin(gtEruIsrConfig.reqPin, IfxPort_InputMode_pullDown);	// Initialize this pin with pull-down enabled, This function will also configure the input multiplexers of the ERU (Register EXISx)
    
	gtEruIsrConfig.inputChannel = (IfxScuEru_InputChannel) gtEruIsrConfig.reqPin->channelId;		// Determine input channel depending on input pin 

	// Input channel configuration 
	IfxScuEru_enableRisingEdgeDetection(gtEruIsrConfig.inputChannel);          // Interrupt triggers on, rising edge (Register RENx) and  
	IfxScuEru_enableFallingEdgeDetection(gtEruIsrConfig.inputChannel);         // on falling edge (Register FENx)  

	// Signal destination 
	gtEruIsrConfig.outputChannel = IfxScuEru_OutputChannel_0;                  // OGU channel 0 
	// Event from input ETL0 triggers output OGU0 (signal TRx0) 
	gtEruIsrConfig.triggerSelect = IfxScuEru_InputNodePointer_0;
	
	IfxScuEru_enableTriggerPulse(gtEruIsrConfig.inputChannel);					// Connecting Matrix, Event Trigger Logic ETL block, Enable generation of trigger event (Register EIENx) 
	IfxScuEru_connectTrigger(gtEruIsrConfig.inputChannel, gtEruIsrConfig.triggerSelect);	// Determination of output channel for trigger event (Register INPx) 
	IfxScuEru_setInterruptGatingPattern(gtEruIsrConfig.outputChannel, IfxScuEru_InterruptGatingPattern_alwaysActive);	// Configure Output channels, OutputGating Unit OGU (Register IGPy) 
	
	gtEruIsrConfig.src = &MODULE_SRC.SCU.SCUERU[(int) gtEruIsrConfig.outputChannel % 4];	// Service request configuration, Get source pointer depending on outputChannel (SRC_SCUERU0 for outputChannel0) 


	IfxScuEru_enableInputFilter(IfxScuEru_InputFilterRequestSelection_7A);		// enable reqUSS_IO_RX2 pin filter
		
 	IfxSrc_init(gtEruIsrConfig.src, IfxSrc_Tos_cpu0, ISR_ERU_PRIORITY);
	IfxSrc_enable(gtEruIsrConfig.src);
}
//-------------------------------------------------------------------------//
void PinIO_Pwr_Sequence(void)
{
	IfxPort_setPinHigh(portPOWER_EN_5141, pinPOWER_EN_5141);
	IfxPort_setPinHigh(portUSS_PWR_EN_TC397, pinUSS_PWR_EN_TC397);
}
//-------------------------------------------------------------------------//





