/*
*	imu_hal.h
*	Created on:2019.3.8
*	Author:liuzhao
*	
*/
#ifndef _IMU_HAL_H_
#define _IMU_HAL_H_

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define USE_MURUTA_IMU  1

#define IMU_DEV_SPI_BURST_MODE      0

//#define IMU_DEBUG

typedef struct imu_data{
	int    iAccel[3]; /*x,y,z*/
	int    iGyro[3];  /*x,y,z*/
	int    iMag[3];   /*x,y,z*/
	int    iTemp;
	double dAccel[3];                     //accelerometer  X-Y-Z output (m/s2)
	double dGyro[3];                      //gyroscope X-Y-Z output(rad/s)
	double dMag[3];                       //magnetometer
	double dTemp;
}imu_data_t;

typedef struct imu_cfg{
	int type;
	double bw;                              //bandwidth
	double dScaleAccel_32bit;               //accelerometer  
	double dScaleAccel_16bit;               //accelerometer  
	double dScaleGyro_32bit;                //gyroscope 
	double dScaleGyro_16bit;                //gyroscope 
	double dScaleMag_32bit;                 //magnetometer
	double dScaleMag_16bit;                 //magnetometer
	double dScaleTemp;  
}imu_cfg_t;

typedef enum imu_dev
{
	IMU_DEV_P2 = 0,	
	IMU_DEV_I80S,
}imu_dev_t;

typedef enum imu_type{
	IMU_TYPE_AUTO = 0,
	IMU_TYPE_ADIS16465_1,
	IMU_TYPE_ADIS16465_2,
	IMU_TYPE_ADIS16465_3,
	IMU_TYPE_IMU381ZA,
	IMU_TYPE_SCC2230,
	IMU_TYPE_SCR2100_X,
	IMU_TYPE_SCR2100_Y,
}imu_type_t;

int imu_init(int imu_type, int imu_dev, int bw);
int imu_reg_write(uint16_t addr,uint16_t data);
int imu_reg_read(uint16_t addr,uint16_t *data);

int imu_data_read(imu_data_t *stImuData);

#ifdef __cplusplus
}
#endif

#endif
