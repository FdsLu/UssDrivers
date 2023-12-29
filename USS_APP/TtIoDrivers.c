/******************************************************************************
;       Program		:	TtIoDrivers.c
;       Function	:	APIs transfer to TT API format
;       Chip		:	Infineon TC397
;       Clock		:	Internal Clock 300MHz
;       Date		:	2023 / 12 / 21
;       Author		:	Fenderson Lu
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
;       Function Description	:	
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
;       Function Description	:	
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
;       Function Description	:	
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
;       Function Name           :   int write_meas(uint16 sensor_mask, uss_meas_data_t meas_data[12])
;       Function Description    :
;       Parameters              :   [Uss_Sensor_Id_t tSensorMask]   - Sensors ID
;                                   [uint16 *u16StatusData]         - USS status
;       Return Values           :   0 = Success
;                                   1 = Failure
;       Description             :   Ex. read_status(USS_ID_IO1_TXRX_FLC, &u16Status)
******************************************************************************/
int write_meas(uint16 sensor_mask, Uss_Meas_Data_t *tMeasSetupPara)
{
    return (int)UssDrivers_Meas_Para_Write(sensor_mask, tMeasSetupPara);
}

//---------------------------------------------------------------------------//























