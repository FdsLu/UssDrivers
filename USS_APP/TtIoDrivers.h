/******************************************************************************
;       Program		: TtIoDrivers.h
;       Function	: Declare TT APIs Format Function & Variable
;       Chip		: Infineon TC397
;       Clock		: Internal SYSPLL 300MHz
;       Date		: 2023 / 12 / 21
;       Author		: Fenderson Lu
******************************************************************************/
#ifndef __TTIODRIVERS_H__
#define __TTIODRIVERS_H__
//---------------------------- Include Library ------------------------------//
#include "UssDrivers.h"
//---------------------------- Define Constant ------------------------------//
//---------------------------- Declare Function -----------------------------// 
extern int write_threshold(Uss_Sensor_Id_t tSensorMask, Uss_Thres_Data_t *tThresSetupPara);
extern int write_calibration(Uss_Sensor_Id_t tSensorMask, Uss_Calib_Data_t *tCalibWritePara);
extern int read_status(Uss_Sensor_Id_t tSensorMask, uint16 *u16StatusData);
extern int write_meas(uint16 sensor_mask, Uss_Meas_Data_t *tMeasSetupPara);
#endif




