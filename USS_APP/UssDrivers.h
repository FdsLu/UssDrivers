/******************************************************************************
;       Program		: UssDrivers.h
;       Function	: Declare USS Drivers Function & Variable
;       Chip		: Infineon TC397
;       Clock		: Internal SYSPLL 300MHz
;       Date		: 2024 / 1 / 9
;       Author		: Fenderson Lu & Jim
******************************************************************************/
#ifndef __USSDRIVERS_H__
#define __USSDRIVERS_H__
//---------------------------- Include Library ------------------------------//
//---------------------------- Declare Constant -----------------------------//
#define		TIME_GAP			0.96
#define		INIT_PARITY_BITS	0x0000
#define		UNIT_A_U8			8				// 8 bits
#define		UNIT_A_U32			32				// 32 bits
#define		UNIT_U32			0xFFFFFFFF	
#define     UNIT_U16            0xFFFF
#define		REG_BIT_7			0x80
#define		PARITY_BIT			0x01
#define		ZERO_START			1

// ThresDataReadMask
#define		MASK_R_B2_THVAL_2			0x1F
#define		MASK_R_B1_THVAL_3			0xF8000000ul
#define		MASK_R_B1_THVAL_4			0x07C00000ul
#define		MASK_R_B1_THVAL_5			0x003E0000ul
#define		MASK_R_B1_THVAL_6			0x0001F000ul
#define		MASK_R_B1_THVAL_7			0x00000F80ul
#define		MASK_R_B1_THVAL_8			0x0000007Cul
#define		MASK_R_B1_THVAL_9_H			0x00000003ul
#define		MASK_R_B0_THVAL_9_L			0xE0000000ul
#define		MASK_R_B0_THVAL_10			0x1F000000ul
#define		MASK_R_B0_THVAL_11			0x00F80000ul
#define		MASK_R_B0_THVAL_12			0x0007C000ul
#define		MASK_R_B0_THVAL_13			0x00003E00ul
#define		MASK_R_B0_THSFT_CFG			0x00000180ul
#define		MASK_R_B0_ATG_CFG			0x00000060ul
#define		MASK_R_B0_ATG_TAU			0x00000018ul
#define		MASK_R_B0_ATG_ALPHA			0x00000004ul
#define		MASK_R_B0_THRESSCALE_REC	0x00000003ul

// CalibDataReadMask
#define		MASK_R_VPROM_STATUS		0x1
#define		MASK_R_F_DRV			0x00000000FFul
#define		MASK_R_I_DRV			0x0000001F00ul
#define		MASK_R_G_ANA			0x000000D000ul
#define		MASK_R_G_DIG			0x00007F0000ul
#define		MASK_R_CUSTOMER_BITS	0x003F800000ul
#define		MASK_R_OSC_TRIM_L		0xC0000000ul
#define		MASK_R_OSC_TRIM_H		0x03
#define		MASK_R_EE_OSC_TRIM_L	0x10000000ul
#define		MASK_R_EE_OSC_TRIM_H	0x07

// MEASBitsReadMask
#define		MASK_READMEAS_B1_NPULSES_A		0x0e
#define		MASK_READMEAS_B1_TMEAS_A_H		0x01
#define		MASK_READMEAS_B0_TMEAS_A_L		0xC0000000ul
#define		MASK_READMEAS_B0_THRESSCALE_A	0x30000000ul
#define		MASK_READMEAS_BO_NPULSES_B		0x0e000000ul
#define		MASK_READMEAS_B0_TMEAS_B		0x01C00000ul
#define		MASK_READMEAS_B0_THRESSCALE_B	0x00300000ul
#define		MASK_READMEAS_BO_NPULSES_C		0x000E0000ul
#define		MASK_READMEAS_B0_TMEAS_C		0x0001C000ul
#define		MASK_READMEAS_B0_THRESSCALE_C	0x00003000ul
#define		MASK_READMEAS_B0_ECHO_DEB		0x00000800ul
#define		MASK_READMEAS_B0_RT_CFG			0x00000400ul
#define		MASK_READMEAS_B0_NFTG			0x00000200ul
#define		MASK_READMEAS_B0_FTC			0x00000100ul
#define		MASK_READMEAS_B0_EPD			0x00000080ul
#define		MASK_READMEAS_BO_STC_CFG		0x00000060ul
#define		MASK_READMEAS_B0_STC_START		0x00000018ul
#define		MASK_READMEAS_B0_NOISE_CFG		0x00000006ul
#define		MASK_READMEAS_B0_FILTER_CFG		0x00000001ul

enum UssRxSymbolSignalLimit{
	SYM_RX_T_BIT0_MIN = 93,
	SYM_RX_T_BIT0_MAX = 108,
	SYM_RX_T_BIT1_MIN = 43,
	SYM_RX_T_BIT1_MAX = 58
};

enum IoSymbolDelayTime{
	SYM_PARSE_T = 1,

	// Time unit is us
	SYM_TIME_T_D = 50,		// Level low 50us
	SYM_TIME_T_SND = 100,	// Level low 100us
	SYM_TIME_T_REC = 150,	// Level low 150us
	SYM_TIME_T_MEAS = 200,	// Level low 200us
	SYM_TIME_T_CMD = 250,	// Level low 250us
	SYM_TIME_T_BIT = 150,	// Level low 150us
	SYM_TIME_T_BIT0 = 100,	// Level low 100us
	SYM_TIME_T_BIT1 = 50	// Level low 50us
};

enum IoSymbolT{
	IO_SYM_T_D,
	IO_SYM_T_SND,
	IO_SYM_T_REC,
	IO_SYM_T_MEAS,
	IO_SYM_T_CMD,
	IO_SYM_T_BIT,
	IO_SYM_T_BIT0_LOG_0,
	IO_SYM_T_BIT1_LOG_1
};

enum UssCmdsSize{
	SIZE_THRES_SETUP = 10,		// TS_BYTEx
	SIZE_USS_SENSOR = 12,		// USS_SENSOR_SIDE_MASK_x
	SIZE_THRES_THVAL = 12,
	SIZE_CALIB_WRITE = 5,		// CW_BYTEx
	SIZE_MEAS_WRITE = 5,
	SIZE_ISR_RX_RAW = 120,
	SIZE_TIME_TAG = 60,
	SIZE_USS_RX = 4,
	SIZE_SND_REC = 128
};

enum UssCmdsLen{
	LEN_THRES_SETUP_BITS = 74,
	LEN_CALIB_WRITE_BITS = 34,
	LEN_MEAS_WRITE_BITS = 39,
	LEN_EE_READ = 35
};

enum UssCmdRxAckUint32Len{
	ACK_U32_LEN_SEND_A = 1,			// Sync bits (2-bits) + Status wrap up bits (10-bits), no include echo reporting length
	ACK_U32_LEN_RECEIVE_A = 1,		// Sync bits (2-bits) + Status wrap up bits (10-bits), no include echo reporting length
	ACK_U32_LEN_SEND_B = 1,			// Sync bits (2-bits) + Status wrap up bits (10-bits), no include echo reporting length
	ACK_U32_LEN_RECEIVE_B = 1,		// Sync bits (2-bits) + Status wrap up bits (10-bits), no include echo reporting length
	ACK_U32_LEN_SEND_C = 1,			// Sync bits (2-bits) + Status wrap up bits (10-bits), no include echo reporting length
	ACK_U32_LEN_RECEIVE_C = 1,		// Sync bits (2-bits) + Status wrap up bits (10-bits), no include echo reporting length
	ACK_U32_LEN_READ_THRES_SETUP = 3,	
	ACK_U32_LEN_READ_MEAS_SETUP = 2,
	ACK_U32_LEN_READ_STATUS = 1,
	ACK_U32_LEN_READ_TEMP = 1,
	ACK_U32_LEN_CALIB_READ = 2,		// EEPROM bits
	ACK_U32_LEN_EE_READ = 4,		// EEPROM bits + VPROG_STATUS bit (35 bits), 35bits*3 = 105bits (need 4 bytes)
	ACK_U32_LEN_READ_ID = 1
};

enum UssSignalPare{
	USS_SIGNAL_0 = 0x0,
	USS_SIGNAL_1 = 0x1
};

enum UssTxCmdsTimeTagFilter{
	LEN_FILTER_SEND_A = 1,
	LEN_FILTER_RECEIVE_A = 1,
	LEN_FILTER_SEND_B = 6,
	LEN_FILTER_RECEIVE_B = 6,
	LEN_FILTER_SEND_C = 6,
	LEN_FILTER_RECEIVE_C = 6
};

enum Uss_TxCmds_Filter_Len{
	LEN_FILTER_READ_THRES_SETUP = 5,		// T_CMD + T_D + 1111
	LEN_FILTER_READ_MEAS_SETUP = 5, 		// T_CMD + T_D + 0111
	LEN_FILTER_READ_STATUS = 5, 			// T_CMD + T_D + 0010
	LEN_FILTER_READ_TEMP = 5,				// T_CMD + T_D + 0100
	LEN_FILTER_CALIB_READ = 5, 				// T_CMD + T_D + 1001
	LEN_FILTER_EE_READ = 5,					// T_CMD + T_D + 1011
	LEN_FILTER_READ_ID = 5					// T_CMD + T_D + 1100	
};

enum Uss_Cmds_BitsAckLen{
	ACK_BITS_LEN_SEND_A = 31 + LEN_FILTER_SEND_A,				// Capture time tag, include  burst, echo 1/2 reporting and status wrap up bits
	ACK_BITS_LEN_RECEIVE_A = 29 + LEN_FILTER_RECEIVE_A,			// Capture time tag, include  echo 1/2 reporting to status wrap up bits
	ACK_BITS_LEN_SEND_B = 28 + LEN_FILTER_SEND_B,				// Capture time tag, include  burst, echo 1 reporting and status wrap up bits
	ACK_BITS_LEN_RECEIVE_B = 26 + LEN_FILTER_RECEIVE_B,			// Capture time tag, include echo 1 reporting and status wrap up bits
	ACK_BITS_LEN_SEND_C = 32 + LEN_FILTER_SEND_C,				// Capture time tag, include  burst, echo 1/2/3 reporting and status wrap up bits
	ACK_BITS_LEN_RECEIVE_C = 30 + LEN_FILTER_RECEIVE_C,			// Capture time tag, include  echo 1/2/3 reporting and status wrap up bits
	ACK_BITS_LEN_READ_THRES_SETUP = 69,	
	ACK_BITS_LEN_READ_MEAS_SETUP = 36,
	ACK_BITS_LEN_READ_STATUS = 10,
	ACK_BITS_LEN_READ_TEMP = 10,
	ACK_BITS_LEN_CALIB_READ = 34,			// EEPROM bits
	ACK_BITS_LEN_EE_READ = 105,		// (EEPROM bits + VPROG_STATUS bit) = 35 bits *3 = 105
	ACK_BITS_LEN_READ_ID = 24
} ;

enum UssSensorsMask{
	MASK_IO1_FLC = 0x0001,
	MASK_IO2_FLM = 0x0002,
	MASK_IO3_FRC = 0x0004,
	MASK_IO4_FRM = 0x0008,
	MASK_IO5_RLC = 0x0010,
	MASK_IO6_RLM = 0x0020,
	MASK_IO7_RRC = 0x0040,
	MASK_IO8_RRM = 0x0080,
	MASK_IO9_FLS = 0x0100,
	MASK_IO10_FRS = 0x0200,
	MASK_IO11_RLS = 0x0400,
	MASK_IO12_RRS = 0x0800
};
//---------------- THRES_SETUP Command --------------------//
//  Bytes of transmitted data for 'THRES_SETUP' command
enum ThresSetupPacketBytes{
	TS_BYTE0 = 0,
	TS_BYTE1 = 1,
	TS_BYTE2 = 2,
	TS_BYTE3 = 3,
	TS_BYTE4 = 4,
	TS_BYTE5 = 5,
	TS_BYTE6 = 6,
	TS_BYTE7 = 7,
	TS_BYTE8 = 8,
	TS_BYTE9 = 9
};

// Bits of transmitted data for 'THRES_SETUP' command
enum ThresSetupBitsMask{
	// TS_BYTE0
	MASK_B0_PARITY_4 			= 0x01,			
	MASK_B0_PARITY_3 			= 0x02,			
	MASK_B0_PARITY_2 			= 0x04,			
	MASK_B0_PARITY_1 			= 0x08,	
	MASK_B0_PARITY_0 			= 0x10,			
	MASK_B0_THRES_SCALE_REC_L	= 0x20,	
	MASK_B0_THRES_SCALE_REC_H	= 0x40,	
	MASK_B0_ATG_ALPHA 			= 0x80,	

	// TS_BYTE1
	MASK_B1_ATG_TAU_L		= 0x01,			
	MASK_B1_ATG_TAU_H		= 0x02,			
	MASK_B1_ATG_CFG_L		= 0x04,			
	MASK_B1_ATG_CFG_H		= 0x08,			
	MASK_B1_THSFT_CFG_L		= 0x10,		
	MASK_B1_THSFT_CFG_H		= 0x20,		
	MASK_B1_THVAL13_BIT_0	= 0x40,		
	MASK_B1_THVAL13_BIT_1	= 0x80,		

	// TS_BYTE2
	MASK_B2_THVAL13_BIT_2 = 0x01,		
	MASK_B2_THVAL13_BIT_3 = 0x02,		
	MASK_B2_THVAL13_BIT_4 = 0x04,		
	MASK_B2_THVAL12_BIT_0 = 0x08,		
	MASK_B2_THVAL12_BIT_1 = 0x10,		
	MASK_B2_THVAL12_BIT_2 = 0x20,		
	MASK_B2_THVAL12_BIT_3 = 0x40,		
	MASK_B2_THVAL12_BIT_4 = 0x80,

	// TS_BYTE3
	MASK_B3_THVAL11_BIT_0 = 0x01,		
	MASK_B3_THVAL11_BIT_1 = 0x02,		
	MASK_B3_THVAL11_BIT_2 = 0x04,		
	MASK_B3_THVAL11_BIT_3 = 0x08,		
	MASK_B3_THVAL11_BIT_4 = 0x10,			
	MASK_B3_THVAL10_BIT_0 = 0x20,		
	MASK_B3_THVAL10_BIT_1 = 0x40,		
	MASK_B3_THVAL10_BIT_2 = 0x80,		

	// TS_BYTE4
	MASK_B4_THVAL10_BIT_3	= 0x01,		
	MASK_B4_THVAL10_BIT_4	= 0x02,		
	MASK_B4_THVAL9_BIT_0	= 0x04,		
	MASK_B4_THVAL9_BIT_1	= 0x08,		
	MASK_B4_THVAL9_BIT_2	= 0x10,		
	MASK_B4_THVAL9_BIT_3	= 0x20,		
	MASK_B4_THVAL9_BIT_4	= 0x40,		
	MASK_B4_THVAL8_BIT_0	= 0x80,		

	// TS_BYTE5
	MASK_B5_THVAL8_BIT_1 = 0x01,		
	MASK_B5_THVAL8_BIT_2 = 0x02,		
	MASK_B5_THVAL8_BIT_3 = 0x04,		
	MASK_B5_THVAL8_BIT_4 = 0x08,		
	MASK_B5_THVAL7_BIT_0 = 0x10,		
	MASK_B5_THVAL7_BIT_1 = 0x20,		
	MASK_B5_THVAL7_BIT_2 = 0x40,		
	MASK_B5_THVAL7_BIT_3 = 0x80,		

	// TS_BYTE6
	MASK_B6_THVAL7_BIT_4 = 0x01,		
	MASK_B6_THVAL6_BIT_0 = 0x02,		
	MASK_B6_THVAL6_BIT_1 = 0x04,		
	MASK_B6_THVAL6_BIT_2 = 0x08,		
	MASK_B6_THVAL6_BIT_3 = 0x10,		
	MASK_B6_THVAL6_BIT_4 = 0x20,		
	MASK_B6_THVAL5_BIT_0 = 0x40,		
	MASK_B6_THVAL5_BIT_1 = 0x80,

	// TS_BYTE7
	MASK_B7_THVAL5_BIT_2 = 0x01,		
	MASK_B7_THVAL5_BIT_3 = 0x02,		
	MASK_B7_THVAL5_BIT_4 = 0x04,		
	MASK_B7_THVAL4_BIT_0 = 0x08,		
	MASK_B7_THVAL4_BIT_1 = 0x10,		
	MASK_B7_THVAL4_BIT_2 = 0x20,		
	MASK_B7_THVAL4_BIT_3 = 0x40,		
	MASK_B7_THVAL4_BIT_4 = 0x80,

	// TS_BYTE8
	MASK_B8_THVAL3_BIT_0 = 0x01,		
	MASK_B8_THVAL3_BIT_1 = 0x02,		
	MASK_B8_THVAL3_BIT_2 = 0x04,		
	MASK_B8_THVAL3_BIT_3 = 0x08,		
	MASK_B8_THVAL3_BIT_4 = 0x10,		
	MASK_B8_THVAL2_BIT_0 = 0x20,		
	MASK_B8_THVAL2_BIT_1 = 0x40,		
	MASK_B8_THVAL2_BIT_2 = 0x80,

	// TS_BYTE9
	MASK_B9_THVAL2_BIT_3 = 0x01,		
	MASK_B9_THVAL2_BIT_4 = 0x02	
};

enum ThresSetupDataStructureMask{
	MASK_DATA_THRES_SCALE_REC = 0x03,
	MASK_DATA_ATG_ALPHA = 0x01,
	MASK_DATA_ATG_TAU = 0x03,
	MASK_DATA_ATG_CFG = 0x03,
	MASK_DATA_THSFT_CFG = 0x03,
	MASK_DATA_THVAL = 0x1F
};

enum THVALNumber{
	NUM_THVAL_2 = 0,
	NUM_THVAL_3 = 1,
	NUM_THVAL_4 = 2,
	NUM_THVAL_5 = 3,
	NUM_THVAL_6 = 4,
	NUM_THVAL_7 = 5,
	NUM_THVAL_8 = 6,
	NUM_THVAL_9 = 7,
	NUM_THVAL_10 = 8,
	NUM_THVAL_11 = 9,
	NUM_THVAL_12 = 10,
	NUM_THVAL_13 = 11
};
//---------------- Meas_WRITE Command ---------------------//
//  Bytes of transmitted data for 'MEAS_WRITE' command
enum MEASPacketBytes{
	MEAS_BYTE0 = 0,
	MEAS_BYTE1 = 1,
	MEAS_BYTE2 = 2,
	MEAS_BYTE3 = 3,
	MEAS_BYTE4 = 4,
};
// Bits of transmitted data for 'MEAS_WRITE' command
enum MEASBitsMask{
	// MEAS_BYTE0
	MASK_MEAS_B0_PARITY_2			= 0x01, 		
	MASK_MEAS_B0_PARITY_1			= 0x02, 		
	MASK_MEAS_B0_PARITY_0			= 0x04, 		
	MASK_MEAS_B0_FILTER_CFG			= 0x08, 
	MASK_MEAS_B0_NOISE_CFG			= 0x30, 		
	MASK_MEAS_B0_STC_START      	= 0xC0, 
	// MEAS_BYTE1
	MASK_MEAS_B1_STC_CFG            = 0x03,
	MASK_MEAS_B1_EPD                = 0x04,
	MASK_MEAS_B1_FTC                = 0x08,
	MASK_MEAS_B1_NFTG               = 0x10,
	MASK_MEAS_B1_RT_CFG             = 0x20,
	MASK_MEAS_B1_ECHO_DEB           = 0x40,
	MASK_MEAS_B1_THRESSCALE_C_L     = 0x80,
	// MEAS_BYTE2
	MASK_MEAS_B2_THRESSCALE_C_H     = 0x01,
	MASK_MEAS_B2_TMEAS_C            = 0x0E,
	MASK_MEAS_B2_NPULSES_C          = 0x70,
	MASK_MEAS_B2_THRESSCALE_B_L     = 0x80,
	// MEAS_BYTE3
	MASK_MEAS_B3_THRESSCALE_B_H     = 0x01,
	MASK_MEAS_B3_TMEAS_B            = 0x0E,
	MASK_MEAS_B3_NPULSES_B          = 0x70,
	MASK_MEAS_B3_THRESSCALE_A_L     = 0x80,
	// MEAS_BYTE4
	MASK_MEAS_B4_THRESSCALE_A_H     = 0x01,
	MASK_MEAS_B4_TMEAS_A            = 0x0E,
	MASK_MEAS_B4_NPULSES_A          = 0X70
}; 
//---------------- CALIB_WRITE Command --------------------//
//  Bytes of transmitted data for 'CALIB_WRITE' command
enum CalibWritePacketBytes{
	CW_BYTE0 = 0,
	CW_BYTE1 = 1,
	CW_BYTE2 = 2,
	CW_BYTE3 = 3,
	CW_BYTE4 = 4
};

// Bits of transmitted data for 'CALIB_WRITE' command
enum CalibDataBitsMask{
	// CW_BYTE0
	MASK_B0_F_DRV_BIT_0 = 0x01,
	MASK_B0_F_DRV_BIT_1 = 0x02,
	MASK_B0_F_DRV_BIT_2 = 0x04,
	MASK_B0_F_DRV_BIT_3 = 0x08,
	MASK_B0_F_DRV_BIT_4 = 0x10,
	MASK_B0_F_DRV_BIT_5 = 0x20,
	MASK_B0_F_DRV_BIT_6 = 0x40,
	MASK_B0_F_DRV_BIT_7 = 0x80,

	// CW_BYTE1
	MASK_B1_I_DRV_BIT_0 = 0x01,
	MASK_B1_I_DRV_BIT_1 = 0x02,
	MASK_B1_I_DRV_BIT_2 = 0x04,
	MASK_B1_I_DRV_BIT_3 = 0x08,
	MASK_B1_I_DRV_BIT_4 = 0x10,
	MASK_B1_G_ANA_BIT_0 = 0x20,
	MASK_B1_G_ANA_BIT_1 = 0x40,
	MASK_B1_G_ANA_BIT_2 = 0x80,
	
	// CW_BYTE2
	MASK_B2_G_DIG_BIT_0 = 0x01,
	MASK_B2_G_DIG_BIT_1 = 0x02,
	MASK_B2_G_DIG_BIT_2 = 0x04,
	MASK_B2_G_DIG_BIT_3 = 0x08,
	MASK_B2_G_DIG_BIT_4 = 0x10,
	MASK_B2_G_DIG_BIT_5 = 0x20,
	MASK_B2_G_DIG_BIT_6 = 0x40,
	MASK_B2_CUSTOMER_BITS_BIT_0 = 0x80,
	
	// CW_BYTE3
	MASK_B3_CUSTOMER_BITS_BIT_1 = 0x01,
	MASK_B3_CUSTOMER_BITS_BIT_2 = 0x02,
	MASK_B3_CUSTOMER_BITS_BIT_3 = 0x04,
	MASK_B3_CUSTOMER_BITS_BIT_4 = 0x08,
	MASK_B3_CUSTOMER_BITS_BIT_5 = 0x10,
	MASK_B3_CUSTOMER_BITS_BIT_6 = 0x20,
	MASK_B3_OSC_TRIM_BIT_0 = 0x40,
	MASK_B3_OSC_TRIM_BIT_1 = 0x80,
	
	// CW_BYTE4
	MASK_B4_OSC_TRIM_BIT_2 = 0x01,
	MASK_B4_OSC_TRIM_BIT_3 = 0x02,
	MASK_B4_VPROG_STATUS = 0x04	
};

enum CalibDataWriteMask{
	MASK_W_F_DRV = 0xFF,
	MASK_W_I_DRV = 0x1F,
	MASK_W_G_ANA = 0x07,
	MASK_W_G_DIG = 0x7F,
	MASK_W_CUSTOMER_BITS = 0x7F,
	MASK_W_OSC_TRIM = 0x0F,
	MASK_W_VPROG_STATUS = 0x01
};
//---------------- SEND_x / RECEIVE_x Command --------------------//
enum SEND_A_AckBitsMask{
	// 1st echo height
	MASK_1ST_ECHO_HEIGHT_0		= 0x0001,
	MASK_1ST_ECHO_HEIGHT_1		= 0x0002,
	MASK_1ST_ECHO_HEIGHT_2		= 0x0004,
	MASK_1ST_ECHO_HEIGHT_3		= 0x0008,
	MASK_1ST_ECHO_HEIGHT_4		= 0x0010,
	MASK_1ST_ECHO_HEIGHT_5		= 0x0020,
	
	// FREQ_DEV
	MASK_FREQ_DEV_L				= 0x0040,
	MASK_FREQ_DEV_H				= 0x0080,
	
	// Threshold and profile status
	MASK_THRES_PROFILE_STATUS	= 0x0100,

	// Noise detect
	MASK_NOISE_DETECT			= 0x0200,
	
	// SYNC Bits
	MASK_SYNC_L					= 0x0400,
	MASK_SYNC_H 				= 0x0800
};
//---------------------------- Declare Typedef Data -------------------------//
typedef enum{
    EX_CMDS_SEND_A,
	EX_CMDS_RECEIVE_A,
	EX_CMDS_SEND_B,
	EX_CMDS_RECEIVE_B,
	EX_CMDS_SEND_C,
	EX_CMDS_RECEIVE_C,
	EX_CMDS_THRES_SETUP,
	EX_CMDS_READ_THRES_SETUP,
	EX_CMDS_MEAS_SETUP,
	EX_CMDS_READ_MEAS_SETUP,
	EX_CMDS_READ_STATUS,
	EX_CMDS_CAL_PULSES,
	EX_CMDS_READ_TEMP,
	EX_CMDS_ENVELOPE_SEND_A,
	EX_CMDS_ENVELOPE_REC_A,
	EX_CMDS_CALIB_WRITE,
	EX_CMDS_CALIB_READ,
	EX_CMDS_EE_COPY,
	EX_CMDS_EE_READ,
	EX_CMDS_READ_ID,
	EX_CMDS_STANDBY,
	EX_CMDS_WAKE_UP
} Uss_Exchange_Cmds;

typedef enum{
	USS_ID_IO1_TXRX_FLC = 0,
	USS_ID_IO2_TXRX_FLM = 1,
	USS_ID_IO3_TXRX_FRC = 2,
	USS_ID_IO4_TXRX_FRM = 3,
	USS_ID_IO5_TXRX_RLC = 4,
	USS_ID_IO6_TXRX_RLM = 5,
	USS_ID_IO7_TXRX_RRC = 6,
	USS_ID_IO8_TXRX_RRM = 7,
	USS_ID_IO9_TXRX_FLS = 8,
	USS_ID_IO10_TXRX_FRS = 9,
	USS_ID_IO11_TXRX_RLS = 10,
	USS_ID_IO12_TXRX_RRS = 11
} Uss_Sensor_Id_t;

typedef enum{
	MODE_PC_EVEN = 0,
	MODE_PC_ODD = 1
} ParityChkMode_t;

typedef enum{
	PC_PARITY_4 = 0,
	PC_PARITY_3 = 1,
	PC_PARITY_2 = 2,
	PC_PARITY_1 = 3,
	PC_PARITY_0 = 4
} ParityChk_Num_t;

typedef enum{
    PC_MEAS_PARITY_2 = 0,
    PC_MEAS_PARITY_1 = 1,
    PC_MEAS_PARITY_0 = 2
} ParityChk_Num_MEAS_t;

typedef enum{
	FUNC_SUCCESS = 0,
	FUNC_FAIL	= 1
} Func_Status_t;

typedef enum{
	MODE_SEND_REC_A = 0,
	MODE_SEND_REC_B = 1,
	MODE_SEND_REC_C = 2,
	MODE_ENVELOPE = 3
} Uss_Detect_Mode_t;

typedef enum{
	CMDS_SEND_A,
	CMDS_SEND_B,
	CMDS_SEND_C,
	CMDS_REC_A,
	CMDS_REC_B,
	CMDS_REC_C,
	CMDS_ENVELOPE_SEND_A,
	CMDS_ENVELOPE_REC_A	
} Uss_Cmds_SendRecEnv;

typedef struct
{
    uint8 u8Thval[SIZE_THRES_THVAL];
    uint8 u8Thsft_Cfg;
	uint8 u8Atg_Cfg;
	uint8 u8Atg_Tau;
	uint8 u8Atg_Alpha;
	uint8 u8Thresscale_Rec;
}Uss_Thres_Data_t;

typedef struct
{
	uint8 u8npulses_a;
	uint8 u8tmeas_a;
	uint8 u8thresscale_a;
	uint8 u8npulses_b;
	uint8 u8tmeas_b;
	uint8 u8thresscale_b;
	uint8 u8npulses_c;
	uint8 u8tmeas_c;
	uint8 u8thresscale_c;
	uint8 u8echo_deb;
	uint8 u8rt_cfg;
	uint8 u8nftg;
	uint8 u8ftc;
	uint8 u8epd;
	uint8 u8stc_cfg;
	uint8 u8stc_start;
	uint8 u8noise_cfg;
	uint8 u8filter_cfg;
}Uss_Meas_Data_t;

typedef struct
{
	uint8 u8Vprog_Status;
	uint8 u8Osc_Trim;
	uint8 u8Customer_Bits;
	uint8 u8G_Dig;
	uint8 u8G_Ana;
	uint8 u8I_Drv;
	uint8 u8F_Drv;
} Uss_Calib_Data_t;

typedef struct
{
	uint32 u32SendAT[SIZE_SND_REC];
	uint32 u32ReceiveAT[SIZE_SND_REC];
	uint32 u32SendBT[SIZE_SND_REC];
	uint32 u32ReceiveBT[SIZE_SND_REC];
	uint32 u32SendCT[SIZE_SND_REC];
	uint32 u32ReceiveCT[SIZE_SND_REC];
	uint32 u32ReadThresSetup[ACK_U32_LEN_READ_THRES_SETUP];
	uint32 u32ReadMeasSetup[ACK_U32_LEN_READ_MEAS_SETUP];
	uint32 u32ReadStatus;
	uint16 u16ReadTemp;
	uint32 u32CalibRead[ACK_U32_LEN_CALIB_READ];
	uint32 u32EeRead[ACK_U32_LEN_EE_READ];
	uint32 u32ReadId;	
} Uss_Rx_Ack_Data_t;
//--------------------------- Extern Support --------------------------------//
extern Uss_Thres_Data_t gtUssThresSetupPara[SIZE_USS_SENSOR];
extern Uss_Meas_Data_t gtUssMeasData[SIZE_USS_SENSOR]; 
extern Uss_Calib_Data_t gtUssCalibWritePara[SIZE_USS_SENSOR];
extern uint32 gu32InvertHighPulseTemp[SIZE_ISR_RX_RAW];
extern uint32 gu32TimeTagTemp[SIZE_TIME_TAG];
extern Ifx_P *gtUssIoPort[SIZE_USS_SENSOR];
extern uint8 gu8UssIoPin[SIZE_USS_SENSOR];
extern Uss_Rx_Ack_Data_t gtUssRxAckData[SIZE_USS_SENSOR];
#if EVB_DEMO
extern uint32 gu32LowPulseTemp[SIZE_ISR_RX_RAW];
#endif
//---------------------------- Declare Function -----------------------------// 
extern void UssDrivers_Init(void);
extern void UssDrivers_IO_Symbol_Signal(Uss_Sensor_Id_t tSensorMask, uint8 u8Symbol);
extern void UssDrivers_Cmds_Transmit(Uss_Sensor_Id_t tSensorMask, Uss_Exchange_Cmds u8Cmd);
extern Func_Status_t UssDrivers_ThresSetup_Para_Write(Uss_Sensor_Id_t tSensorMask, Uss_Thres_Data_t *tThresSetupPara);
extern Func_Status_t UssDrivers_Sensors_Thres_Read(Uss_Sensor_Id_t tSensorMask);
extern Func_Status_t UssDrivers_Thres_Para_Get(Uss_Sensor_Id_t tSensorMask, Uss_Thres_Data_t *tThresholdData);
extern Func_Status_t UssDrivers_Meas_Para_Write(Uss_Sensor_Id_t tSensorMask, Uss_Meas_Data_t *tThresSetupPara);
extern Func_Status_t UssDrivers_Meas_Setup_Read(Uss_Sensor_Id_t tSensorMask);
extern Func_Status_t UssDrivers_Meas_Setup_Get(Uss_Sensor_Id_t tSensorMask, Uss_Meas_Data_t *tMeasPara);
extern uint8 UssDrivers_ParityBit_Calculate(ParityChkMode_t tMode, uint32 u32Values);
extern Func_Status_t UssDrivers_Calib_Write(Uss_Sensor_Id_t tSensorMask, Uss_Calib_Data_t *tCalibWritePara);
extern void UssDrivers_Rx_Data_Parse(boolean bFlag);
extern boolean UssDrivers_RxIsrFinishFlag_Get(void);
extern void UssDrivers_RxIsrFinishFlag_Set(boolean bFlagStatus);
extern uint32 UssDrivers_RxAckLen_Get(void);
extern void UssDrivers_RxAckLen_Set(uint32 u32AckLen);
extern void UssDrivers_Rx_Data_Store(Uss_Sensor_Id_t tSensorMask, Uss_Exchange_Cmds u8Cmd);
extern Uss_Exchange_Cmds UssDrivers_UssTxCmd_Get(void);
extern void UssDrivers_UssTxCmd_Set(uint8 u8Cmd);
extern boolean UssDrivers_UssRxAckEnFlag_Get(void);
extern void UssDrivers_UssRxAckEnFlag_Set(boolean bFlagSwitch);
extern Func_Status_t UssDrivers_Status_Read(Uss_Sensor_Id_t tSensorMask, uint16 *u16StatusData);
extern boolean UssDrivers_AckRecFinishFlag_Get(void);
extern void UssDrivers_AckRecFinishFlag_Set(boolean bSwitch);
extern uint32 UssDrivers_DetectTimeLen_Get(void);
extern void UssDrivers_DetectTimeLen_Set(uint32 u32DetTLen);
extern uint16 UssDrivers_SendMask_Get(void);
extern void UssDrivers_SendMask_Set(uint16 u16SndMask);
extern uint16 UssDrivers_RecMask_Get(void);
extern void UssDrivers_RecMask_Set(uint16 u16RecMask);
extern Func_Status_t UssDrivers_SndRecEnv_Detect(Uss_Detect_Mode_t tMode, uint16 u16SendMask, uint16 u16RecMask, uint16 u16DetTime_us);
extern void UssDrivers_Cmds_SedRecEnv_Send(uint16 u16Mask, Uss_Cmds_SendRecEnv tCmds);
extern uint32 UssDrivers_RxTagTCnt_Get(void);
extern void UssDrivers_RxTagTCnt_Set(uint32 u32Cnt);
extern Func_Status_t UssDrivers_Bilat_Get(Uss_Sensor_Id_t tSensorMask, Uss_Cmds_SendRecEnv tCmd, uint32 *u32BilateralT);
extern Func_Status_t UssDrivers_Sensors_Temp_Read(Uss_Sensor_Id_t tSensorMask);
extern Func_Status_t UssDrivers_Temperature_Get(Uss_Sensor_Id_t tSensorMask, uint16 *u16Temp);
extern Func_Status_t UssDrivers_Sensors_Calib_Read(Uss_Sensor_Id_t tSensorMask);
extern Func_Status_t UssDrivers_Calib_Get(Uss_Sensor_Id_t tSensorMask, Uss_Calib_Data_t *tCalibData);
extern Func_Status_t UssDrivers_Sensors_EEPROM_Read(Uss_Sensor_Id_t tSensorMask);
extern Func_Status_t UssDrivers_EEPROM_Data_Get(Uss_Sensor_Id_t tSensorMask, Uss_Calib_Data_t *tEepromData);
extern Func_Status_t UssDrivers_EEPROM_Copy(Uss_Sensor_Id_t tSensorMask);
#endif




