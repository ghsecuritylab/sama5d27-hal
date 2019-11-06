#ifndef _ADIS16465_H_
#define _ADIS16465_H_


#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "imu_hal.h"

/****************************************************************/
#ifdef __cplusplus
extern "C" {
#endif           	/*__cplusplus*/
/****************************************************************/

#define ADIS16465_DEV_ID			0x4051
#define ADIS16505_DEV_ID 			0x4079	
//------------ADIS16465 READ CMD----------------------------
#define ADIS16465_REG_DIAG_STAT			 0x0200
#define ADIS16465_REG_X_GYRO_LOW     0x0400
#define ADIS16465_REG_X_GYRO_OUT     0x0600
#define ADIS16465_REG_Y_GYRO_LOW     0x0800
#define ADIS16465_REG_Y_GYRO_OUT     0x0A00
#define ADIS16465_REG_Z_GYRO_LOW     0x0C00
#define ADIS16465_REG_Z_GYRO_OUT     0x0E00	
	
#define ADIS16465_REG_X_ACCL_LOW     0x1000
#define ADIS16465_REG_X_ACCL_OUT     0x1200
#define ADIS16465_REG_Y_ACCL_LOW     0x1400
#define ADIS16465_REG_Y_ACCL_OUT     0x1600
#define ADIS16465_REG_Z_ACCL_LOW     0x1800
#define ADIS16465_REG_Z_ACCL_OUT     0x1A00
	
#define ADIS16465_REG_TEMP_OUT 			 0x1C00

#define ADIS16465_REG_RANG_MDL			 0x5E00

#define ADIS16465_REG_PROD_ID 			 0x7200

#define ADIS16465_REG_FILT_CTRL			 0x5C00
#define ADIS16465_REG_UP_SCALE			 0x6200

//------------ADIS16465 WRITE CMD----------------------------

#define ADIS16465_REG_DEC_RATE		     0x6400

#define ADIS16465_REG_GLOB_CMD			 0x6800

#define ADIS16465_REG_SET_FILT_CMD_LOW  0xDC04
#define ADIS16465_REG_SET_FILT_CMD_HIGH 0xDD00

#define ADIS16465_REG_FILT_PARAM	0x0004

	
extern const imu_cfg_t imu_cfg_adis16465_1;
extern const imu_cfg_t imu_cfg_adis16465_2;
extern const imu_cfg_t imu_cfg_adis16465_3;
//------------Function----------------------------

int imu_cfg_init_adis16465(imu_cfg_t *pstImuCfg,int bw);
int imu_data_read_adis16465(imu_cfg_t *stImuCfg,imu_data_t *stImuData,int opt);



/****************************************************************/
#ifdef __cplutplus
}
#endif				/*__cplusplus*/
/****************************************************************/

#endif
