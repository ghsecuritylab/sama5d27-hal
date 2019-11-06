#ifndef _IMU_H_
#define _IMU_H_

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "imu_hal.h"

/****************************************************************/
#ifdef __cplusplus
extern "C" {
#endif           	/*__cplusplus*/
/****************************************************************/

	
#define IMU381_DEV_ID 					 0x3810
//------------IMU381 READ CMD----------------------------
#define IMU381_REG_X_GYRO_OUT    0x0400
#define IMU381_REG_Y_GYRO_OUT    0x0600
#define IMU381_REG_Z_GYRO_OUT    0x0800
#define IMU381_REG_X_ACCL_OUT    0x0A00
#define IMU381_REG_Y_ACCL_OUT    0x0C00
#define IMU381_REG_Z_ACCL_OUT    0x0E00

#define IMU381_REG_X_MAG		     0x1000
#define IMU381_REG_Y_MAG		 		 0x1200
#define IMU381_REG_Z_MAG		     0x1400

#define IMU381_REG_TEMP_OUT      0x1800
#define IMU381_REG_DIAG_STAT     0x3C00

#define IMU381_REG_SMPL_PRD      0x3600
#define IMU381_REG_SENS_AVG      0x3800

#define IMU381_REG_GLOB_CMD		   0x3E00

#define IMU381_REG_PROD_ID		   0x5600
#define IMU381_REG_OUT_SCAL      0x7000
#define IMU381_ZERO_CMD					 0x0000


//------------Function----------------------------

uint16_t imu381_reg_read_16bit(uint16_t addr, uint16_t *rx_data);

extern imu_cfg_t imu_cfg_imu381;

int imu_cfg_init_imu381(int imu_dev, imu_cfg_t *pstImuCfg,int bw);
int imu_data_read_imu381(imu_cfg_t *stImuCfg,imu_data_t *stImuData,int opt);


/****************************************************************/
#ifdef __cplutplus
}
#endif				/*__cplusplus*/
/****************************************************************/

#endif
