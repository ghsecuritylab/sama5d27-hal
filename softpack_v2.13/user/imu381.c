#include "hal.h"
#include "imu381.h"
#include "global.h"

imu_cfg_t imu_cfg_imu381 = 
{
	.type = IMU_TYPE_IMU381ZA,
	.dScaleAccel_32bit = 3.814697265625e-9,
	.dScaleAccel_16bit = 2.5e-4,
	.dScaleGyro_32bit  = 1.52587890625e-7,
	.dScaleGyro_16bit  = 1e-2,
	.dScaleMag_32bit   = 0.0,
	.dScaleMag_16bit   = 0.0,
	.dScaleTemp 	   =  0.07311,
};


uint16_t imu381_reg_read_16bit(uint16_t addr, uint16_t *rx_data)
{
	uint16_t Response;
	uint8_t Request_buff[2] = {0};
	uint8_t Response_buff[2] = {0};
	
	Request_buff[0] = addr>>8;	// 高8位
	Request_buff[1] = addr;

	hal.pspi->send_receive_message(IMU_TYPE_AUTO, Request_buff, Response_buff, 2);

	Response  = (uint16_t)Response_buff[0]<<8;
	Response |= (uint16_t)Response_buff[1];
	
	*rx_data = Response;
	return 0;
}

/* Burst mode 下发送16bit地址GLOB_CMD,读一系列数据*/
int imu381_reg_burst_read_16_bit(uint16_t addr, uint16_t *rx_data)
{
	if(!rx_data)
		return -1;
	int i = 0;
	uint8_t Request_buff[2] = {0};
	uint8_t Response_buff[9*2] = {0};
	
	Request_buff[0] = addr>>8;	// 高8位
	Request_buff[1] = addr;
	
	hal.pspi->send_receive_message(IMU_TYPE_AUTO, Request_buff, Response_buff,2);
	hal.ptimer->wait_us(40);
	hal.pspi->receive_message(Response_buff,8*2);
	for(i=1;i<16/2;i++)
	{
		*(rx_data+i) = ((uint16_t)Response_buff[2*i]<<8) | ((uint16_t)Response_buff[2*i+1]);
		
	}
	return 0;
}

int imu_data_read_imu381(imu_cfg_t *stImuCfg,imu_data_t *stImuData,int opt)
{
	uint16_t rx[8] = {0};
	int i = 0;

#if IMU_DEV_SPI_BURST_MODE	
	{
		uint16_t rx_long[9] = {0};
		uint16_t tx_long[9] = { IMU381_REG_GLOB_CMD };
		
		imu381_reg_read_16bit(tx_long[0], rx_long);	// Read:  N/A, Status, 3 gyro, 3 accel, temp
		for(i=1;i<9;i++)
		{
			imu381_reg_read_16bit(0x0000, rx_long+i);
		  hal.ptimer->wait_us(20);
		}
		memcpy(rx, rx_long+1, sizeof(rx_long)-1);

	}
#else
	{
		uint16_t tx_long[] = {IMU381_REG_DIAG_STAT,
								IMU381_REG_X_GYRO_OUT,
								IMU381_REG_Y_GYRO_OUT,
								IMU381_REG_Z_GYRO_OUT,
								IMU381_REG_X_ACCL_OUT,
								IMU381_REG_Y_ACCL_OUT,
								IMU381_REG_Z_ACCL_OUT,
								IMU381_REG_TEMP_OUT,
								0x0000 };	// read data: N/A, status, 3 gyro, 3 accel, temp
		uint16_t rx_long[ARRAY_SIZE(tx_long)] = {0x0000};
		for(i=0;i<ARRAY_SIZE(tx_long);i++)
		{
			imu381_reg_read_16bit(tx_long[i], rx_long+i);
			//hal.ptimer->wait_us(20);
		}

		memcpy(rx,rx_long+1,sizeof(rx_long)-1);
	}
#endif

	stImuData->iGyro[0] = (int16_t)rx[1];
	stImuData->iGyro[1] = (int16_t)rx[2];
	stImuData->iGyro[2] = (int16_t)rx[3];
	stImuData->iAccel[0] = (int16_t)rx[4];
	stImuData->iAccel[1] = (int16_t)rx[5];
	stImuData->iAccel[2] = (int16_t)rx[6];
	stImuData->dTemp = (int16_t)rx[7] * stImuCfg->dScaleTemp + 31;

	for(i=0;i<3;i++)
	{
		stImuData->dGyro[i]  = stImuData->iGyro[i] * stImuCfg->dScaleGyro_16bit;
		stImuData->dAccel[i] = stImuData->iAccel[i] * stImuCfg->dScaleAccel_16bit;
	}

#ifdef IMU_DEBUG
//	printf("imu_read: gyro, %04X %04X %04X; accel, %04X %04X %04X \r\n",
//			stImuData->iGyro[0],stImuData->iGyro[1],stImuData->iGyro[2],
//			stImuData->iAccel[0],stImuData->iAccel[1],stImuData->iAccel[2]);

	printf("imu_read: gyro, %3.6f %3.6f %3.6f; accel, %3.6f %3.6f %3.6f; temperature %3.2f\r\n",
		stImuData->dGyro[0],stImuData->dGyro[1],stImuData->dGyro[2],
		stImuData->dAccel[0],stImuData->dAccel[1],stImuData->dAccel[2],
		stImuData->dTemp );
#endif

	return 0;
}


int imu_cfg_init_imu381(int imu_dev, imu_cfg_t *pstImuCfg,int bw)
{
	uint16_t imu_reg_SMPL_PRD  = 0;
	uint16_t imu_reg_SENS_AVG = 0;
	uint16_t reg_data = 0;
	double filt_bw = 0;
	double fs_use = 200;

	imu_reg_SMPL_PRD = 0x0101;/*dsf90:内部采样时钟200Hz*/

	if(bw<=7)
	{
		filt_bw = 5;
		imu_reg_SENS_AVG = 0x0006;  /*5Hz*/
	}
	else if(bw<=15)
	{
		filt_bw = 10;
		imu_reg_SENS_AVG = 0x0005;  /*10Hz*/
	}
	else if(bw<=30)
	{
		filt_bw = 20;
		imu_reg_SENS_AVG = 0x0004;  /*20Hz*/
	}
	else if(bw<=60)
	{
		filt_bw = 40;
		imu_reg_SENS_AVG = 0x0003;  /*40Hz*/
	}
	else
	{
		filt_bw = fs_use/2;
		imu_reg_SENS_AVG = 0x0000;  /*--------*/
	}

	switch(imu_dev)
	{
		case IMU_DEV_P2:
			imu_reg_SENS_AVG |= (0x04<<8); /*250deg/s*/
			break;
		case IMU_DEV_I80S:
			imu_reg_SENS_AVG |= (0x08<<8); /*500deg/s*/
			break;
		default:
			printf("----------ERR: unknow dev type------------\r\n");
			return -1;
	}

	printf("reg REG_SENS_AVG set: %04X \r\n",imu_reg_SENS_AVG);

	imu_reg_write(IMU381_REG_SENS_AVG,imu_reg_SENS_AVG); // set SENS_AVG
	imu_reg_read(IMU381_REG_SENS_AVG,&reg_data);
	printf("reg REG_SENS_AVG get: %04X \r\n",reg_data);
	if(reg_data!=imu_reg_SENS_AVG)
	{
		printf("imu_cfg_init_imu381 SENS_AVG fail...........!!!\r\n");
		return -1;
	}
	
	printf("reg REG_SMPL_PRD set: %04X \r\n",imu_reg_SMPL_PRD);
	imu_reg_write(IMU381_REG_SMPL_PRD,imu_reg_SMPL_PRD);
	imu_reg_read(IMU381_REG_SMPL_PRD,&reg_data);
	printf("reg REG_SMPL_PRD get: %04X \r\n",reg_data);
	if(reg_data!=imu_reg_SMPL_PRD)
	{
		printf("imu_cfg_init_imu381 SMPL_PRD fail...........!!!\r\n");
		return -1;
	}
	/*dsf90:内部采样时钟200Hz*/
	pstImuCfg->bw = filt_bw;
	printf("f  set %.3fHz\r\n",fs_use);
	printf("bw set %.3fHz\r\n",filt_bw);

	imu_reg_read(IMU381_REG_OUT_SCAL,&reg_data);
	printf("reg REG_OUT_SCAL get: %04X \r\n",reg_data);
	switch(imu_dev)
	{
		case IMU_DEV_P2:
			reg_data = (0x02<<12)|(0x02<<4); /*250deg/s , 4g*/
			imu_cfg_imu381.dScaleGyro_16bit = 1/100.0;
			imu_cfg_imu381.dScaleAccel_16bit = 1/8192.0;
			break;
		case IMU_DEV_I80S:
			reg_data = (0x03<<12)|(0x04<<4); /*500deg/s , 8g*/
			imu_cfg_imu381.dScaleGyro_16bit = 1/50.0;
			imu_cfg_imu381.dScaleAccel_16bit = 1/4096.0;
			break;
		default:
			printf("----------ERR: unknow dev type------------\r\n");
			return -1;
	}

	printf("reg REG_OUT_SCAL set: %04X \r\n",reg_data);
	imu_reg_write(IMU381_REG_OUT_SCAL,reg_data);
	imu_reg_read(IMU381_REG_OUT_SCAL,&reg_data);
	printf("reg REG_OUT_SCAL get: %04X \r\n",reg_data);

	switch((reg_data>>12))
	{
		case 0x00:
			imu_cfg_imu381.dScaleGyro_16bit = 1/400.0;
			printf("gyro scal used 80deg/s\r\n");
			break;
		case 0x01:
			imu_cfg_imu381.dScaleGyro_16bit = 1/200.0;
			printf("gyro scal used 160deg/s\r\n");
			break;
		case 0x02:
			imu_cfg_imu381.dScaleGyro_16bit = 1/100.0;
			printf("gyro scal used 200deg/s\r\n");
			break;
		case 0x03:
			imu_cfg_imu381.dScaleGyro_16bit = 1/50.0;
			printf("gyro scal used 400deg/s\r\n");
			break;
		case 0x04:
			imu_cfg_imu381.dScaleGyro_16bit = 1/25.0;
			printf("gyro scal used 600deg/s\r\n");
			break;
		default:
			printf("----------ERR: unknow gyro scal------------\r\n");
			break;
	}
	imu_cfg_imu381.dScaleGyro_32bit = imu_cfg_imu381.dScaleGyro_16bit/65536.0;

	switch(((reg_data&0x00FF)>>4))
	{
		case 0x00:
			imu_cfg_imu381.dScaleAccel_16bit = 1/32768.0;
			printf("acc scal used 1g\r\n");
			break;
		case 0x01:
			imu_cfg_imu381.dScaleAccel_16bit = 1/16384.0;
			printf("acc scal used 2g\r\n");
			break;
		case 0x02:
			imu_cfg_imu381.dScaleAccel_16bit = 1/8192.0;
			printf("acc scal used 4g\r\n");
			break;
		case 0x03:
			imu_cfg_imu381.dScaleAccel_16bit = 1/4000.0;
			printf("acc scal used 5g\r\n");
			break;
		case 0x04:
			imu_cfg_imu381.dScaleAccel_16bit = 1/4096.0;
			printf("acc scal used 8g\r\n");
			break;
		default:
			printf("----------ERR: unknow acc scal------------\r\n");
			break;
	}
	imu_cfg_imu381.dScaleAccel_32bit = imu_cfg_imu381.dScaleAccel_16bit/65536.0;

	return 0;
	
}

