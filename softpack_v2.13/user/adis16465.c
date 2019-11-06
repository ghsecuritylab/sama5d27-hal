#include "hal.h"
#include "adis16465.h"
#include "global.h" 

uint16_t adis16465_reg_read_16bit(uint16_t addr, uint16_t *rx_data);

/*adis_filt_cfg: B、N、(f/fs*1000)、f*1000 (fs=2Mhz))*/
static int filt_cfg_adis16465[7][4] = 
{	{0,1 ,320,640000},
	{1,2 ,160,320000},
	{2,4 ,80, 160000},
	{3,8 ,40, 80000},
	{4,16,20, 40000},
	{5,32,10, 20000},
	{6,64,5,  10000},
};

const imu_cfg_t imu_cfg_adis16465_1 = 
{
	.type = IMU_TYPE_ADIS16465_2,
	.dScaleAccel_32bit = 3.814697265625e-9,
	.dScaleAccel_16bit = 2.5e-4,
	.dScaleGyro_32bit  = 9.5367431640625e-8,
	.dScaleGyro_16bit  = 6.25e-3,
	.dScaleMag_32bit   = 0.0,
	.dScaleMag_16bit   = 0.0,
	.dScaleTemp        = 0.1,
};
	
const imu_cfg_t imu_cfg_adis16465_2 = 
{
	.type = IMU_TYPE_ADIS16465_2,
	.dScaleAccel_32bit = 3.814697265625e-9,
	.dScaleAccel_16bit = 2.5e-4,
	.dScaleGyro_32bit  = 3.814697265625e-7,  //0.025/65536
	.dScaleGyro_16bit  = 2.5e-2,             //0.025
	.dScaleMag_32bit   = 0.0,
	.dScaleMag_16bit   = 0.0,
	.dScaleTemp        = 0.1,
};

const imu_cfg_t imu_cfg_adis16465_3 = 
{
	.type = IMU_TYPE_ADIS16465_2,
	.dScaleAccel_32bit = 3.814697265625e-9,
	.dScaleAccel_16bit = 2.5e-4,
	.dScaleGyro_32bit  = 1.5258790625e-6,
	.dScaleGyro_16bit  = 1e-1,
	.dScaleMag_32bit   = 0.0,
	.dScaleMag_16bit   = 0.0,
	.dScaleTemp        = 0.1,
};


uint16_t adis16465_reg_read_16bit(uint16_t addr, uint16_t *rx_data)
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

int adis16465_reg_burst_read_16bit(uint16_t addr, uint16_t *rx_data)
{
	if(!rx_data)
		return -1;
	int i = 0;
	uint8_t Request_buff[2] = {0};
	uint8_t Response_buff[11*2] = {0};
	
	Request_buff[0] = addr>>8;	// 高8位
	Request_buff[1] = addr;
	
	hal.pspi->send_receive_message(IMU_TYPE_AUTO, Request_buff, Response_buff,2);
	hal.ptimer->wait_us(20);
	hal.pspi->receive_message(Response_buff,10*2);
	for(i=1;i<20/2;i++)
	{
		*(rx_data+i) = ((uint16_t)Response_buff[2*i]<<8) | ((uint16_t)Response_buff[2*i+1]);
		
	}
	return 0;

}

uint16_t check_sum_16bit(void *dataBuf,uint32_t len)
{
	uint16_t sum = 0;
	uint8_t data_low = 0, data_high = 0;
	uint32_t i;
	uint16_t *data = (uint16_t *)dataBuf;
	if(!data||!len)
		return 0;
	for(i=0;i<len;i++)
	{
		data_low = (*(data+i))& 0x00FF;		// low 8 bit
		data_high = (*(data+i)) >> 8;			// high 8 bit
		sum += (data_high + data_low);
	}
	return sum;
}

int imu_data_read_adis16465(imu_cfg_t *stImuCfg,imu_data_t *stImuData,int opt)
{
	uint16_t rx[10] = {0};
	uint16_t rx_low[6] = {0};
	int i = 0;
#if IMU_DEV_SPI_BURST_MODE
	{
		uint16_t sum = 0;
		uint16_t rx_long[11] = {0};
		uint16_t tx_long[11] = { ADIS16465_REG_GLOB_CMD };
		
		adis16465_reg_read_16bit(tx_long[0], rx_long);	// read, 16bit data: N/A, DIAG_STAT, 3 gyro, 3 accel, temp, DATA_CNTR, checksum value
		for(i=1;i<11;i++)
		{
			adis16465_reg_read_16bit(0x0000, rx_long+i);
			hal.ptimer->wait_us(20);
		}
		memcpy(rx, rx_long+1, sizeof(rx_long)-1);
		
		sum = check_sum_16bit(rx,9);	// sum = DIAG_STAT+3 gyro+3 accel+temp+DATA_CNTR
		if( sum != (rx[9]))
		{
			printf("imu check sum err, sum %02X, read %02X \r\n", sum, rx[9]);
			return -1;
		}
	}
#else
	{
		uint16_t tx_long[] = {
				ADIS16465_REG_DIAG_STAT,
				ADIS16465_REG_X_GYRO_OUT,
				ADIS16465_REG_Y_GYRO_OUT,
				ADIS16465_REG_Z_GYRO_OUT,
				ADIS16465_REG_X_ACCL_OUT,
				ADIS16465_REG_Y_ACCL_OUT,
				ADIS16465_REG_Z_ACCL_OUT,
				ADIS16465_REG_TEMP_OUT,
				0x0000 };
		uint16_t rx_long[ARRAY_SIZE(tx_long)] = {0x0000};
		
		for(i=0;i<ARRAY_SIZE(tx_long);i++)
		{
			adis16465_reg_read_16bit(tx_long[i], rx_long+i);
			hal.ptimer->wait_us(20);
		}
		memcpy(rx, rx_long+1, sizeof(rx_long)-1);
	}
#endif
	if(opt == 1)
	{
		uint16_t tx_long[] = {
					ADIS16465_REG_X_GYRO_LOW,
					ADIS16465_REG_Y_GYRO_LOW,
					ADIS16465_REG_Z_GYRO_LOW,
					ADIS16465_REG_X_ACCL_LOW,
					ADIS16465_REG_Y_ACCL_LOW,
					ADIS16465_REG_Z_ACCL_LOW,
					0x0000 };
		uint16_t rx_long[ARRAY_SIZE(tx_long)] = {0x0000};
		
		for(i=0;i<ARRAY_SIZE(tx_long);i++)
		{
			adis16465_reg_read_16bit(tx_long[i], rx_long+i);
			hal.ptimer->wait_us(20);
		}
		memcpy(rx_low, rx_long+1, sizeof(rx_long)-1);
	}

	stImuData->iGyro[0] = (int)(rx[1]<<16 | rx_low[0]);
	stImuData->iGyro[1] = (int)(rx[2]<<16 | rx_low[1]);
	stImuData->iGyro[2] = (int)(rx[3]<<16 | rx_low[2]);
	stImuData->iAccel[0] = (int)(rx[4]<<16 | rx_low[3]);
	stImuData->iAccel[1] = (int)(rx[5]<<16 | rx_low[4]);
	stImuData->iAccel[2] = (int)(rx[6]<<16 | rx_low[5]);
	stImuData->dTemp = (int)(rx[7]) * stImuCfg->dScaleTemp;

	for(i=0;i<3;i++)
	{
		stImuData->dGyro[i]  = stImuData->iGyro[i] * stImuCfg->dScaleGyro_32bit;
		stImuData->dAccel[i] = stImuData->iAccel[i] * stImuCfg->dScaleAccel_32bit;
	}

#ifdef IMU_DEBUG
//	printf("imu_read: gyro, %04X %04X %04X; accel, %04X %04X %04X \r\n",
//		stImuData->iGyro[0],stImuData->iGyro[1],stImuData->iGyro[2],
//		stImuData->iAccel[0],stImuData->iAccel[1],stImuData->iAccel[2]);


	printf("imu_read: gyro, %3.6f %3.6f %3.6f; accel, %3.6f %3.6f %3.6f; temperature %3.2f\r\n",
		stImuData->dGyro[0],stImuData->dGyro[1],stImuData->dGyro[2],
		stImuData->dAccel[0],stImuData->dAccel[1],stImuData->dAccel[2],
		stImuData->dTemp);
#endif
	return 0;
}


int imu_cfg_init_adis16465(imu_cfg_t *pstImuCfg,int bw)
{
	uint16_t imu_reg_DEC_RATE  = 0;
	uint16_t imu_reg_FILT_CTRL = 0;
	uint16_t imu_set_FILT_CTRL = 0;
	int fs_use = 2000;

	if(bw<=7)
	{
		fs_use /= 2;
		imu_set_FILT_CTRL = 0x0006;  /*5Hz*/
	}
	else if(bw<=15)
		imu_set_FILT_CTRL = 0x0006;  /*10Hz*/
	else if(bw<=30)
		imu_set_FILT_CTRL = 0x0005;  /*20Hz*/
	else if(bw<=60)
		imu_set_FILT_CTRL = 0x0004;  /*40Hz*/
	else if(bw<=120)
		imu_set_FILT_CTRL = 0x0003;  /*80Hz*/
	else if(bw<=240)
		imu_set_FILT_CTRL = 0x0002;  /*160Hz*/
	else if(bw<=480)
		imu_set_FILT_CTRL = 0x0001;  /*320Hz*/
	else
		imu_set_FILT_CTRL = 0x0000;  /*-----*/
	
	imu_reg_DEC_RATE  = 2000/fs_use-1;/*dsf90:内部采样时钟2000Hz*/
	imu_reg_FILT_CTRL = imu_set_FILT_CTRL;

	printf("reg FILT_CTRL set: %04X \r\n",imu_reg_FILT_CTRL);
	imu_reg_write(ADIS16465_REG_FILT_CTRL,imu_reg_FILT_CTRL);
	hal.ptimer->wait_us(40);
	imu_reg_read(ADIS16465_REG_FILT_CTRL,&imu_reg_FILT_CTRL);
	printf("reg FILT_CTRL get: %04X \r\n",imu_reg_FILT_CTRL);

	if(imu_reg_FILT_CTRL !=imu_set_FILT_CTRL)
	{
		printf("imu_cfg_init_adis16465 fail...........!!!\r\n");
		return -1;
	}

	printf("reg DEC_RATE set: %04X \r\n",imu_reg_DEC_RATE);
	imu_reg_write(ADIS16465_REG_DEC_RATE,imu_reg_DEC_RATE);
	hal.ptimer->wait_us(40);
	imu_reg_read(ADIS16465_REG_DEC_RATE,&imu_reg_DEC_RATE);
	printf("reg DEC_RATE get: %04X \r\n",imu_reg_DEC_RATE);

	pstImuCfg->bw = (double)filt_cfg_adis16465[imu_set_FILT_CTRL][3]/1000.0/(imu_reg_DEC_RATE+1);
	printf("filt scale %.3f\r\n",filt_cfg_adis16465[imu_set_FILT_CTRL][3]/1000.0);
	printf("f  set %.3fHz\r\n",(double)2000/(imu_reg_DEC_RATE+1));
	printf("bw set %.3fHz\r\n",pstImuCfg->bw);

	return 0;
}

