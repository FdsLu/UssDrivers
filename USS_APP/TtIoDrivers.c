/******************************************************************************
;       Program		:	TtIoDrivers.c
;       Function	:	APIs transfer to TT API format
;       Chip		:	Infineon TC397
;       Clock		:	Internal Clock 300MHz
;       Date		:	2023 / 1 / 3
;       Author		:	Fenderson Lu & Jim
;		Description	:	
******************************************************************************/
//---------------------------- Include File ---------------------------------//
#include "Cpu0_Main.h"
#include "TtIoDrivers.h"
#include "UssDrivers.h"
//---------------------------- Declare Global Variable ----------------------//
//---------------------------- Start Program --------------------------------//
/******************************************************************************
;       Function Name			:	int write_threshold(Uss_Sensor_Id_t tSensorMask, Uss_Thres_Data_t *tThresSetupPara)
;       Function Description	:	Send and set threshold setup parameters
;       Parameters				:	[Uss_Sensor_Id_t tSensorMask]		- Sensors ID
;									[Uss_Thres_Data_t *tThresSetupPara]	- Threshold setup parameters
;       Return Values			:	0 = FUNC_SUCCESS
;		Description				:	Ex. write_threshold(USS_ID_IO1_TXRX_FLC, &gtUssThresSetupPara[0])
******************************************************************************/
int write_threshold(Uss_Sensor_Id_t tSensorMask, Uss_Thres_Data_t *tThresSetupPara)
{
	return (int)UssDrivers_ThresSetup_Para_Write(tSensorMask, tThresSetupPara);
}

/******************************************************************************
;       Function Name			:	int write_calibration(Uss_Sensor_Id_t tSensorMask, Uss_Calib_Data_t *tCalibWritePara)
;       Function Description	:	Send and set calibration parameters
;       Parameters				:	[Uss_Sensor_Id_t tSensorMask]		- Sensors ID
;									[Uss_Calib_Data_t *tCalibWritePara]	- Calibration write parameters
;       Return Values			:	0 = FUNC_SUCCESS
;		Description				:	Ex. write_calibration(USS_ID_IO1_TXRX_FLC, &gtUssCalibWritePara[0])
******************************************************************************/
int write_calibration(Uss_Sensor_Id_t tSensorMask, Uss_Calib_Data_t *tCalibWritePara)
{
	return (int)UssDrivers_Calib_Write(tSensorMask, tCalibWritePara);
}

/******************************************************************************
;       Function Name			:	int read_status(Uss_Sensor_Id_t tSensorMask, uint16 *u16StatusData)
;       Function Description	:	Read sensors status
;       Parameters				:	[Uss_Sensor_Id_t tSensorMask]	- Sensors ID
;									[uint16 *u16StatusData]			- USS status
;       Return Values			:	0 = Success
;									1 = Failure
;		Description				:	Ex. read_status(USS_ID_IO1_TXRX_FLC, &u16Status)
******************************************************************************/
int read_status(Uss_Sensor_Id_t tSensorMask, uint16 *u16StatusData)
{
	return (int)UssDrivers_Status_Read(tSensorMask, u16StatusData);
}

/******************************************************************************
;       Function Name           :   int write_meas(Uss_Sensor_Id_t sensor_mask, uss_meas_data_t meas_data[12])
;       Function Description    :
;       Parameters              :   [Uss_Sensor_Id_t tSensorMask]   - Sensors ID
;                                   [uint16 *u16StatusData]         - USS status
;       Return Values           :   0 = Success
;                                   1 = Failure
;       Description             :   Ex. read_status(USS_ID_IO1_TXRX_FLC, &u16Status)
******************************************************************************/
int write_meas(Uss_Sensor_Id_t sensor_mask, Uss_Meas_Data_t *tMeasSetupPara)
{
    return (int)UssDrivers_Meas_Para_Write(sensor_mask, tMeasSetupPara);
}

/******************************************************************************
;       Function Name			:	int uss_detect(Uss_Detect_Mode_t tMode, uint16 u16SendMask, uint16 u16RecMask, uint16 u16DetTime_us)
;       Function Description	:	Range measurement
;       Parameters				:	[Uss_Detect_Mode_t tMode]	- Command mode selection
;									[uint16 u16SendMask]		- Choose send sensor
;									[uint16 u16RecMask]			- Choose receive sensor
;									[uint16 u16DetTime_us]		- Sensor ACK time length
;       Return Values			:	0 = Success
;									1 = Failure
;		Description				:	Ex. uss_detect(MODE_SEND_REC_C, 0x0002, 0x0000, 36000);
******************************************************************************/
int uss_detect(Uss_Detect_Mode_t tMode, uint16 u16SendMask, uint16 u16RecMask, uint16 u16DetTime_us)
{
	return (int)UssDrivers_SndRecEnv_Detect(tMode, u16SendMask, u16RecMask, u16DetTime_us);
}

/******************************************************************************
;       Function Name			:	int read_bilateral_time(Uss_Sensor_Id_t tSensorMask, Uss_Cmds_SendRecEnv tCmd, uint32 *u32BilateralT)
;       Function Description	:	Read sensors bilateral time
;       Parameters				:	[Uss_Sensor_Id_t tSensorMask]	- Sensors ID
;									[Uss_Cmds_SendRecEnv tCmd]		- Choose command
;									[uint32 *u32BilateralT]			- read and store bilateral time of command
;       Return Values			:	0 = Success
;									1 = Failure
;		Description				:	Ex. read_bilateral_time(USS_ID_IO2_TXRX_FLM, CMDS_SEND_C, &gu32Buffer[0])
******************************************************************************/
int read_bilateral_time(Uss_Sensor_Id_t tSensorMask, Uss_Cmds_SendRecEnv tCmd, uint32 *u32BilateralT)
{
	return	(int)UssDrivers_Bilat_Get(tSensorMask, tCmd, u32BilateralT);
}

/******************************************************************************
;       Function Name			:	int get_temp(Uss_Sensor_Id_t tSensorMask)
;       Function Description	:	Read sensors temperature values
;       Parameters				:	[Uss_Sensor_Id_t tSensorMask] - Sensors ID
;       Return Values			:	0 = Success
;									1 = Failure
;		Description				:	Ex. get_temp(USS_ID_IO2_TXRX_FLM)
******************************************************************************/
int get_temp(Uss_Sensor_Id_t tSensorMask)
{
	return (int)UssDrivers_Sensors_Temp_Read(tSensorMask);
}

/******************************************************************************
;       Function Name			:	int read_temp(Uss_Sensor_Id_t tSensorMask, uint16 *u16Temp)
;       Function Description	:	Get sensors temperature values from the MCU memory
;       Parameters				:	[Uss_Sensor_Id_t tSensorMask]	- Sensors ID
;									[uint16 *u16Temp]				- Sensor temperature values
;       Return Values			:	0 = Success
;									1 = Failure
;		Description				:	Ex. read_temp(USS_ID_IO2_TXRX_FLM, &gu16Temperture)
******************************************************************************/
int read_temp(Uss_Sensor_Id_t tSensorMask, uint16 *u16Temp)
{
	return (int)UssDrivers_Temperature_Get(tSensorMask, u16Temp);
}
//---------------------------------------------------------------------------//























