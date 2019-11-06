/*
*	imu_hal.c
*	Created on:2019.3.8
*	Author:liuzhao
*	
*/
#include "imu_hal.h"
#include "imu381.h"
#include "adis16465.h"
#include "board.h"
#include "timer.h"
#include "trace.h"
#include "mm/cache.h"
#include "spi/spid.h"
#include "peripherals/bus.h"
#include "config_sama5d27-som1-ek.h"

static imu_cfg_t s_imu_cfg = {0};


/** descriptor for SPI imu */
static const struct _bus_dev_cfg spi_imu_dev = {
	.bus = SPI_IMU_BUS,
	.spi_dev = {
		//.chip_select = SPI_IMU_CS,
		.bitrate = SPI_IMU_BITRATE,
		.delay = {
			.bs = 0,
			.bct = 0,
		},
		.spi_mode = SPID_MODE_3,
	},
};

static struct _spi_desc imu_slave_dev = {
	.addr = IMU_SLAVE_ADDR,
	.chip_select = 0,
	.transfer_mode = BUS_TRANSFER_MODE_DMA,
};

CACHE_ALIGNED static uint8_t spi_buffer_imu_tx[2];
CACHE_ALIGNED static uint8_t spi_buffer_imu_rx[2];

struct _buffer imu_buf_tx = {
		.data = spi_buffer_imu_tx,
		.size = 2,
                .attr = BUS_BUF_ATTR_TX
		//.attr = BUS_BUF_ATTR_TX | BUS_SPI_BUF_ATTR_RELEASE_CS,
	};

struct _buffer imu_buf_rx = {
                .data = spi_buffer_imu_rx,
		.size = 2,
		.attr = BUS_BUF_ATTR_RX,
        };

/**********************************************************************
*	通过spi发送16bit数据，写IMU寄存器
*	addr: 16bit寄存器地址， data: 写数据
**********************************************************************/
static int imu_reg_write_16bit_data(uint16_t addr,uint16_t data)
{
	/*uint8_t data_low,data_high;
	uint8_t tx[2];

	data_low  = (uint8_t)(data&0x00ff);
	data_high = (uint8_t)(data>>8);

	tx[0] = (uint8_t)(addr>>8) | 0x80; 
	tx[1] = data_low;
	//hal.pspi->send_message(tx,2);
		
	tx[0] = ((uint8_t)(addr>>8) + 1)|0x80;
	tx[1] = data_high;
	//hal.pspi->send_message(tx,2);*/
	return 0;
}

static int _spi_slave_transfer_callback(void* arg, void* arg2)
{
        printf("###imu receive complete\r\n");
	return 0;
}


/**********************************************************************
*	通过spi发送16bit地址，读IMU寄存器
*	addr: 16bit寄存器地址， data: 返回16bit数据指针
**********************************************************************/
static int imu_reg_read_16bit_data(uint16_t addr,uint16_t *data)
{
	/*uint8_t tx[2] = {0};
	uint8_t rx[2] = {0};
	tx[0] = addr>>8;	// 高8位
	tx[1] = addr;		// 低8位，0x00*/
        int ret = 0;
        int i;
        int err;
        
        struct _callback _cb = {
		.method = _spi_slave_transfer_callback,
		.arg = 0,
	};
        
        memset(spi_buffer_imu_tx, 0, 2);
        memset(spi_buffer_imu_rx, 0, 2);
        /*for (i = 0; i < 2; i++)
		spi_buffer_imu_tx[i] = i;*/
        spi_buffer_imu_tx[0] = addr>>8;
        spi_buffer_imu_tx[1] = addr;
        
        printf("...spi_buffer_imu_tx[0] = 0x%02x\r\n",spi_buffer_imu_tx[0]);
        printf("...spi_buffer_imu_tx[1] = 0x%02x\r\n",spi_buffer_imu_tx[1]);
        
        led_set(IMU_CS);
        
        bus_start_transaction(spi_imu_dev.bus);
        
        /*printf("Master receiving...\r\n");
	err = spid_transfer(&imu_slave_dev, &imu_buf_rx, 1, &_cb);
	if (err < 0) {
		trace_error("...SPI: SLAVE: transfer failed.\r\n");
		return 0;
	}*/
        
        printf("Master sending...\r\n");
        bus_transfer(spi_imu_dev.bus, NULL, &imu_buf_tx, 1, NULL);
        //printf("$$$ ret = %d\r\n");
        //bus_transfer(spi_imu_dev.bus, NULL, &imu_buf_rx, 1, NULL);
        
        //printf("spi_buffer_imu_tx[0] = 0x%02x\r\n",spi_buffer_imu_tx[0]);
        //printf("spi_buffer_imu_tx[1] = 0x%02x\r\n",spi_buffer_imu_tx[1]);
        
        
        //spid_wait_transfer(&imu_slave_dev);
        
        //*data = (uint16_t)(spi_buffer_imu_rx[0]<<8|spi_buffer_imu_rx[1]);
        
        //printf("###spi_buffer_imu_rx[0] = 0x%02x\r\n",spi_buffer_imu_rx[0]);
        //printf("###spi_buffer_imu_rx[1] = 0x%02x\r\n",spi_buffer_imu_rx[1]);
        
        //bus_stop_transaction(spi_imu_dev.bus);
        
	//spi_send_receive_message(IMU_TYPE_AUTO,tx, rx, 2);
	
	//*data = (uint16_t)(rx[0]<<8|rx[1]);
        led_clear(IMU_CS);
	return 0;
}

/**********************************************************************
*	imu数据读取正确值是第二个周期读到的值
*	addr: 16bit寄存器地址， data: 返回16bit数据指针
**********************************************************************/
static int imu_reg_read_true_data(uint16_t addr,uint16_t *data)
{
	if(!data)
		return -1;
	imu_reg_read_16bit_data(addr, data);
	usleep(20);
	imu_reg_read_16bit_data(addr, data);
	return 0;
}

/********************************
*	imu_type_judge函数，判断芯片类型
*
*	返回值芯片类型，错误返回-1
*********************************/
static int imu_type_check(void)
{
	uint16_t imu_reg_data = 0;
	
	msleep(50);	// Wait 50 ms until the imu is accessible via SPI
	
	/*imu_reg_read_true_data(IMU381_REG_PROD_ID, &imu_reg_data);
	
	// 第二次读到的才是ID
	if( imu_reg_data ==  IMU381_DEV_ID)
	{
                printf("******* IMU is IMU381\r\n");
		return IMU_TYPE_IMU381ZA;
	}*/

	imu_reg_read_true_data(ADIS16465_REG_PROD_ID, &imu_reg_data);

	/*if( imu_reg_data ==  ADIS16465_DEV_ID  || imu_reg_data == ADIS16505_DEV_ID)
	{
		uint16_t rang_flag = 0;

		imu_reg_read_true_data(ADIS16465_REG_RANG_MDL, &imu_reg_data);
		rang_flag = (imu_reg_data>>2)&0x0003;
		switch(rang_flag)   
		{
			case 0:
                        {
                                printf("******* IMU is ADIS16465_1\r\n");
                                return IMU_TYPE_ADIS16465_1;
                        }
			case 1:
                        {
                                printf("******* IMU is ADIS16465_2\r\n");
                                return IMU_TYPE_ADIS16465_2;
                        }
			case 3:
                        {
                                printf("******* IMU is ADIS16465_3\r\n");
                                return IMU_TYPE_ADIS16465_3;
                        }	
		}
	}
	return -1;*/
}

/********************************
*	imu_cfg_init初始化imu参数
*********************************/
int imu_cfg_init(int imu_dev, imu_cfg_t *pstImuCfg, int bw)
{
	printf("imu cfg bw: %dHz\r\n",bw);
	switch(pstImuCfg->type)
	{
		case IMU_TYPE_IMU381ZA:
			return imu_cfg_init_imu381(imu_dev,pstImuCfg,bw);
		case IMU_TYPE_ADIS16465_1:
		case IMU_TYPE_ADIS16465_2:
		case IMU_TYPE_ADIS16465_3:
			return imu_cfg_init_adis16465(pstImuCfg,bw);
		default:
			printf("imu type unknow\r\n");
	}
	return -1;
}


void imu_init_reset(void)
{
	led_set(IMU_RST);
        msleep(50);
        led_clear(IMU_RST);
        msleep(1000);
}


int imu_init(int imu_type, int imu_dev, int bw)
{
        bus_configure_slave(spi_imu_dev.bus, &spi_imu_dev);
	imu_init_reset();
        

	/*if(IMU_TYPE_AUTO == imu_type)
	{
		imu_type = imu_type_check();
	}*/
        while(1)
        {
            imu_type = imu_type_check();
	}
	/*switch(imu_type)
	{
		case IMU_TYPE_IMU381ZA:
			//s_imu_cfg = imu_cfg_imu381;
			printf("IMU TYPE IMU381 USED\r\n");
			break;
		case IMU_TYPE_ADIS16465_1:
			//s_imu_cfg = imu_cfg_adis16465_1;
			printf("IMU TYPE ADIS16465-2 USED\r\n");
			break;
		case IMU_TYPE_ADIS16465_2:
			//s_imu_cfg = imu_cfg_adis16465_2;
			printf("IMU TYPE ADIS16465-2 USED\r\n");
			break;
		case IMU_TYPE_ADIS16465_3:
			//s_imu_cfg = imu_cfg_adis16465_3;
			printf("IMU TYPE ADIS16465-2 USED\r\n");
			break;
		default:
			printf("imu type unknow\r\n");
			return -1;
	}*/

	/*if(!bw)
	{
		printf("ERR: BW set 0, init fail...\r\n");
		return -1;
	}

	if(imu_cfg_init(imu_dev, &s_imu_cfg,bw)<0)
	{
		printf("ERR: imu cfg init fail...\r\n");
		return -1;
	}*/
	
	return 0;
}

int imu_reg_write(uint16_t addr,uint16_t data)
{
	switch(s_imu_cfg.type)
	{
		case IMU_TYPE_IMU381ZA:
		case IMU_TYPE_ADIS16465_1:
		case IMU_TYPE_ADIS16465_2:
		case IMU_TYPE_ADIS16465_3:
			return imu_reg_write_16bit_data(addr,data);
		default:
			printf("imu type unknow");
			return -1;
	}
}

int imu_reg_read(uint16_t addr,uint16_t *data)
{
	switch(s_imu_cfg.type)
	{
		case IMU_TYPE_IMU381ZA:
		case IMU_TYPE_ADIS16465_1:
		case IMU_TYPE_ADIS16465_2:
		case IMU_TYPE_ADIS16465_3:
			return imu_reg_read_true_data(addr, data);
		default:
			printf("imu type unknow");
			return -1;
	}
}

int imu_data_read(imu_data_t *stImuData)
{
	switch(s_imu_cfg.type)
	{
		case IMU_TYPE_IMU381ZA:
			return imu_data_read_imu381((imu_cfg_t *)&imu_cfg_imu381,stImuData,0);
		case IMU_TYPE_ADIS16465_1:
			return imu_data_read_adis16465((imu_cfg_t *)&imu_cfg_adis16465_1,stImuData,0);
		case IMU_TYPE_ADIS16465_2:
			return imu_data_read_adis16465((imu_cfg_t *)&imu_cfg_adis16465_2,stImuData,0);
		case IMU_TYPE_ADIS16465_3:
			return imu_data_read_adis16465((imu_cfg_t *)&imu_cfg_adis16465_3,stImuData,0);
		default:
			printf("imu type unknow\r\n");
			return -1;
	}
}

