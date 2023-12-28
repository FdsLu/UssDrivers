/******************************************************************************
;       Program		: PinIO.h
;       Function	: Declare GPIO Sensing & Variable
;       Chip		: Infineon TC397
;       Clock		: Internal SYSPLL 300MHz
;       Date		: 2023 / 12 / 21
;       Author		: Fenderson Lu
******************************************************************************/
#ifndef __PINIO_H__
#define __PINIO_H__
//---------------------------- Library Support ------------------------------//
#include "IfxPort.h"
#include "IfxAsclin_Asc.h"

// ERU
#include "IfxSrc.h"
#include "IfxScuEru.h"
//---------------------------- Declare Constant -----------------------------//
#define		ISR_ERU_PRIORITY	CONFIG_PRIORITY_40

#define		PIN_LEVEL_LOW		FALSE
#define		PIN_LEVEL_HIGH		TRUE

#define		LIMIT_VARIABLE			4294967295ul		// uint32 = 2^32
#define		THRESHOLD_TIME_RST		4294967000ul	

#define		INIT_ISR			1
//---------------------------- Support Function -----------------------------//
#define		portPOWER_EN_5141	&MODULE_P00
#define		pinPOWER_EN_5141	6

#define		portUSS_PWR_EN_TC397	&MODULE_P00
#define		pinUSS_PWR_EN_TC397		9


#define		portLED_D107		&MODULE_P13
#define		pinLED_D107			0

#define		portUART_9			&MODULE_ASCLIN9
#define		pinUART_9_TX		IfxAsclin9_TX_P14_7_OUT
#define		pinUART_9_RX		IfxAsclin9_RXD_P14_9_IN

#define		portUSS_IO_TX1		&MODULE_P15
#define		pinUSS_IO_TX1		2

#define		portUSS_IO_TX2		&MODULE_P15
#define		pinUSS_IO_TX2		3

#define		portUSS_IO_TX3		&MODULE_P15
#define		pinUSS_IO_TX3		4

#define		portUSS_IO_TX4		&MODULE_P15
#define		pinUSS_IO_TX4		5

#define		portUSS_IO_TX5		&MODULE_P15
#define		pinUSS_IO_TX5		7

#define		portUSS_IO_TX6		&MODULE_P15
#define		pinUSS_IO_TX6		8

#define		portUSS_IO_TX7		&MODULE_P20
#define		pinUSS_IO_TX7		0

#define		portUSS_IO_TX8		&MODULE_P20
#define		pinUSS_IO_TX8		1

#define		portUSS_IO_TX10		&MODULE_P20
#define		pinUSS_IO_TX10		3

#define		portUSS_IO_TX11		&MODULE_P20
#define		pinUSS_IO_TX11		6

#define		portUSS_IO_TX12		&MODULE_P20
#define		pinUSS_IO_TX12		7

#define		portUSS_IO_RX1		&MODULE_P20
#define		pinUSS_IO_RX1		8

#define		portUSS_IO_RX2		&MODULE_P20
#define		pinUSS_IO_RX2		9
#define		reqUSS_IO_RX2		IfxScu_REQ7A_P20_9_IN	//IfxScu_REQ3C_P02_0_IN	//IfxScu_REQ5A_P15_8_IN		

#define		portUSS_IO_RX3		&MODULE_P20
#define		pinUSS_IO_RX3		10

#define		portUSS_IO_RX4		&MODULE_P20
#define		pinUSS_IO_RX4		11

#define		portUSS_IO_RX5		&MODULE_P20
#define		pinUSS_IO_RX5		12

#define		portUSS_IO_RX6		&MODULE_P20
#define		pinUSS_IO_RX6		13

#define		portUSS_IO_RX7		&MODULE_P20
#define		pinUSS_IO_RX7		14

#define		portUSS_IO_RX8		&MODULE_P21
#define		pinUSS_IO_RX8		0

#define		portUSS_IO_RX9		&MODULE_P21
#define		pinUSS_IO_RX9		1

#define		portUSS_IO_RX10		&MODULE_P21
#define		pinUSS_IO_RX10		2

#define		portUSS_IO_RX11		&MODULE_P21
#define		pinUSS_IO_RX11		3

#define		portUSS_IO_RX12		&MODULE_P21
#define		pinUSS_IO_RX12		4

#define		portUSS_IO_TX9		&MODULE_P21
#define		pinUSS_IO_TX9		5

#define		portUSS_ISR_TEST	&MODULE_P02
#define		pinUSS_ISR_TEST		0
#define		reqUSS_ISR_TEST		IfxScu_REQ3C_P02_0_IN

//---------------------------- Declare Data Structure -----------------------//
typedef struct
{
    IfxScu_Req_In *reqPin;                      // External request pin                                             
    IfxScuEru_InputChannel inputChannel;        // Input channel EICRm depending on input pin                       
    IfxScuEru_InputNodePointer triggerSelect;   // Input node pointer                                               
    IfxScuEru_OutputChannel outputChannel;      // Output channel                                                   
    volatile Ifx_SRC_SRCR *src;                 // Service request register                                         
    volatile uint8 LEDstate;                    // Control parameter to set state of pin which triggers interrupt   
} ERUconfig;
//---------------------------- Extern Support -------------------------------//
extern ERUconfig gtEruIsrConfig;
//---------------------------- Declare Function -----------------------------// 
extern void PinIO_Init(void);
extern void PinIO_PeripheralsAndERU_Init(void);
extern void PinIO_Pwr_Sequence(void);
#endif




