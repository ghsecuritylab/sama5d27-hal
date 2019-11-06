//****************************************************************************************
// SCC2xxx Demo - SPI Interface to SCC2000 series Combo Sensor (Gyroscope + Accelerometer)
// The code supports SCC2x30-D08 gyro accelerometer combo part numbers as well as
// SCR2100 gyro only versions. If SCC2230-E02 is used, the accelerometer sensitivity
// should be updated from 1962LSB/g to 5886LSB/g.
//
// This software is released under the BSD license as follows.
// Copyright (c) 2014, Murata Electronics Oy.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following
// conditions are met:
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following
// disclaimer in the documentation and/or other materials
// provided with the distribution.
// 3. Neither the name of Murata Electronics Oy nor the names of its
// contributors may be used to endorse or promote products derived
// from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
// IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//****************************************************************************************
/* Includes ------------------------------------------------------------------*/
#include "muRuta.h" 
#include <stdbool.h>
#include "hal.h"
#include "imu_hal.h"

/**
* @brief  Sends a word through the SPI interface and return the word received
*         from the SPI bus.
* @param  Request: word to send.
* @retval The value of the received word.
*/
typedef enum {
	FALSE = 0, 
	TRUE = !FALSE,
}TestStatus;

// Send request to sensor and read back the response for previous request.
static uint32_t SendRequest(uint8_t Imu_type, uint32_t Request)
{
	uint32_t Response;
	uint8_t Request_buff[4] = {0};
	uint8_t Response_buff[4] = {0};
	
	Request_buff[0] = (uint8_t)(Request>>24);  //pay attention here, is first uint8_t, then left shift
	Request_buff[1] = (uint8_t)(Request>>16);
	Request_buff[2] = (uint8_t)(Request>>8);
	Request_buff[3] = (uint8_t)Request;
	
	hal.pspi->send_receive_message(Imu_type, Request_buff, Response_buff, 4); 
	
	Response  = (uint32_t)Response_buff[0]<<24;
	Response |= (uint32_t)Response_buff[1]<<16;
	Response |= (uint32_t)Response_buff[2]<<8;
	Response |= (uint32_t)Response_buff[3];
	return Response;
}

static void muRuta_ResetPinInit(void)
{
	GPIO_InitTypeDef GPIO_Initure;
	
	__HAL_RCC_GPIOB_CLK_ENABLE(); 
	__HAL_RCC_GPIOD_CLK_ENABLE(); 	

	GPIO_Initure.Pin = GPIO_PIN_0|GPIO_PIN_1;   
	GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;  	
	GPIO_Initure.Pull = GPIO_PULLUP;              
	GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;    
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);
	
	GPIO_Initure.Pin = GPIO_PIN_10;   
	GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;  	
	GPIO_Initure.Pull = GPIO_PULLUP;              
	GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;    
	HAL_GPIO_Init(GPIOD,&GPIO_Initure);
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,  GPIO_PIN_SET); 
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,  GPIO_PIN_SET); 
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
}

static void SCR2100_X_Init(void)
{
	// Reset sensor
	SCR2100_X_RESET_LOW();
	hal.ptimer->wait_ms(2);
	SCR2100_X_RESET_HIGH();
	
	// Sensor power-up
	hal.ptimer->wait_ms(25); // Wait 25 ms until the SCC2000 is accessible via SPI
	SendRequest(IMU_TYPE_SCR2100_X, REQ_WRITE_FLT_60);
  hal.ptimer->wait_ms(595); // NOTE: wait 595 ms in case the output filter is set to 60 Hz, 725 ms if the filter is set to 10 Hz	
	
	// Clear status registers
	SendRequest(IMU_TYPE_SCR2100_X, REQ_READ_RATE_STAT1);
	SendRequest(IMU_TYPE_SCR2100_X, REQ_READ_RATE_STAT2);
	SendRequest(IMU_TYPE_SCR2100_X, REQ_READ_ACC_STAT);
	SendRequest(IMU_TYPE_SCR2100_X, REQ_READ_COM_STAT);
	SendRequest(IMU_TYPE_SCR2100_X, REQ_READ_STAT_SUM);
	
	// First read temp once to get into desired measurement cycle in off-frame protocol
	SendRequest(IMU_TYPE_SCR2100_X, REQ_READ_TEMP);
}

static void SCR2100_Y_Init(void)
{
	// Reset sensor
	SCR2100_Y_RESET_LOW();
	hal.ptimer->wait_ms(2);
	SCR2100_Y_RESET_HIGH();
	
	// Sensor power-up
	hal.ptimer->wait_ms(25); // Wait 25 ms until the SCC2000 is accessible via SPI
	SendRequest(IMU_TYPE_SCR2100_Y, REQ_WRITE_FLT_60); 
	hal.ptimer->wait_ms(595); // NOTE: wait 595 ms in case the output filter is set to 60 Hz, 725 ms if the filter is set to 10 Hz
	
	// Clear status registers
	SendRequest(IMU_TYPE_SCR2100_Y, REQ_READ_RATE_STAT1);
	SendRequest(IMU_TYPE_SCR2100_Y, REQ_READ_RATE_STAT2);
	SendRequest(IMU_TYPE_SCR2100_Y, REQ_READ_ACC_STAT);
	SendRequest(IMU_TYPE_SCR2100_Y, REQ_READ_COM_STAT);
	SendRequest(IMU_TYPE_SCR2100_Y, REQ_READ_STAT_SUM);
	
	// First read temp once to get into desired measurement cycle in off-frame protocol
	SendRequest(IMU_TYPE_SCR2100_Y, REQ_READ_TEMP);
}

static void SCC2230_Z_Init(void)
{
	// Reset sensor
	SCC2230_Z_RESET_LOW();
	hal.ptimer->wait_ms(2);
	SCC2230_Z_RESET_HIGH();
	
	// Sensor power-up
	hal.ptimer->wait_ms(25); // Wait 25 ms until the SCC2000 is accessible via SPI
	SendRequest(IMU_TYPE_SCC2230, REQ_WRITE_FLT_60); 
  hal.ptimer->wait_ms(595); // NOTE: wait 595 ms in case the output filter is set to 60 Hz, 725 ms if the filter is set to 10 Hz
	
	// Clear status registers
	SendRequest(IMU_TYPE_SCC2230, REQ_READ_RATE_STAT1);
	SendRequest(IMU_TYPE_SCC2230, REQ_READ_RATE_STAT2);
	SendRequest(IMU_TYPE_SCC2230, REQ_READ_ACC_STAT);
	SendRequest(IMU_TYPE_SCC2230, REQ_READ_COM_STAT);
	SendRequest(IMU_TYPE_SCC2230, REQ_READ_STAT_SUM);
	
	// First read temp once to get into desired measurement cycle in off-frame protocol
	SendRequest(IMU_TYPE_SCC2230, REQ_READ_TEMP);
}

static void SCR2100_X_ReadAndProcessData(imu_data_t *stImuData)
{
		uint32_t Response_Rate = 0;
		uint32_t Response_Temp = 0;

		uint32_t Response_RateStat1 = 0;
		uint32_t Response_RateStat2 = 0;
		uint32_t Response_AccStat = 0;
		uint32_t Response_ComStat = 0;
		uint32_t Response_StatSum = 0;
	
		uint8_t RSdata = 0;
	  int16_t Rate = 0;
		int16_t Temp = 0;
		uint8_t DataError = 0;
	
		Response_Temp = SendRequest(IMU_TYPE_SCR2100_X, REQ_READ_RATE);
	  Response_Rate = SendRequest(IMU_TYPE_SCR2100_X, REQ_READ_TEMP);

		// Handle rate data
		RSdata = (Response_Rate & RS_FIELD_MASK) >> 24;
		if (RSdata != 1) 
		{
				DataError |= 0x01;
		}
		else 
		{
				Rate = (Response_Rate & DATA_FIELD_MASK) >> 8;
				DataError &= ~0x01; 
		}
		
		// Handle temperature data
		RSdata = (Response_Temp & RS_FIELD_MASK) >> 24;
		if (RSdata != 1)
		{
				DataError |= 0x10;
		}		
		else
		{
				Temp = (Response_Temp & DATA_FIELD_MASK) >> 8;
				DataError &= ~0x10;
		}
		
	 if (DataError)
	 {
		 	// Clear status registers
		  SendRequest(IMU_TYPE_SCR2100_X, REQ_READ_STAT_SUM);    // Clear the data that is coming from previous frame, request status summary
			Response_StatSum = SendRequest(IMU_TYPE_SCR2100_X, REQ_READ_RATE_STAT1);
			Response_RateStat1 = SendRequest(IMU_TYPE_SCR2100_X, REQ_READ_RATE_STAT2);
			Response_RateStat2 = SendRequest(IMU_TYPE_SCR2100_X, REQ_READ_ACC_STAT);
			Response_AccStat = SendRequest(IMU_TYPE_SCR2100_X, REQ_READ_COM_STAT);
			Response_ComStat = SendRequest(IMU_TYPE_SCR2100_X, REQ_READ_TEMP);  // Request temperature data to get the measurement loop to continue correctly after reading the status registers
		 
			printf("SCR2100-X read error,error code: 0x%02X\r\n", DataError);
		  printf("SCR2100-X --> StatSum: %08X, RateStat1: %08X, RateStat2: %08X, AccStat: %08X, ComStat:  %08X\r\n", 
		          Response_StatSum, Response_RateStat1, Response_RateStat2, Response_AccStat, Response_ComStat);
	 }
	 else
	 {
//		printf("SCR2100-X --> RATE: %08X\r\n", Response_Rate);
//	  printf("SCR2100-X --> RATE: %f, TEMP: %f\r\n", Rate/GYRO_SENSITIVITY, (Temp/14.7 + 60.0)); 
		 Temp = Temp;
		 stImuData->iGyro[0] = Rate;
		 stImuData->dGyro[0] = Rate/GYRO_SENSITIVITY;
	 }
}

static void SCR2100_Y_ReadAndProcessData(imu_data_t *stImuData)
{
		uint32_t Response_Rate = 0;
		uint32_t Response_Temp = 0;
		
		uint32_t Response_RateStat1 = 0;
		uint32_t Response_RateStat2 = 0;
		uint32_t Response_AccStat = 0;
		uint32_t Response_ComStat = 0;
		uint32_t Response_StatSum = 0;
	
		uint8_t RSdata = 0;
	  int16_t Rate = 0;
		int16_t Temp = 0;
		uint8_t DataError = 0;
	
	  Response_Temp = SendRequest(IMU_TYPE_SCR2100_Y, REQ_READ_RATE);
	  Response_Rate = SendRequest(IMU_TYPE_SCR2100_Y, REQ_READ_TEMP);
		 
		// Handle rate data
		RSdata = (Response_Rate & RS_FIELD_MASK) >> 24;
		if (RSdata != 1) 
		{
				DataError |= 0x01;
		}
		else 
		{
				Rate = (Response_Rate & DATA_FIELD_MASK) >> 8;
				DataError &= ~0x01; 
		}
		
		// Handle temperature data
		RSdata = (Response_Temp & RS_FIELD_MASK) >> 24;
		if (RSdata != 1)
		{
				DataError |= 0x10;
		}		
		else
		{
				Temp = (Response_Temp & DATA_FIELD_MASK) >> 8;
				DataError &= ~0x10;
		}
		
	 	if (DataError)
	  {	
			// Clear status registers
		  SendRequest(IMU_TYPE_SCR2100_Y, REQ_READ_STAT_SUM);    // Clear the data that is coming from previous frame, request status summary
			Response_StatSum = SendRequest(IMU_TYPE_SCR2100_Y, REQ_READ_RATE_STAT1);
			Response_RateStat1 = SendRequest(IMU_TYPE_SCR2100_Y, REQ_READ_RATE_STAT2);
			Response_RateStat2 = SendRequest(IMU_TYPE_SCR2100_Y, REQ_READ_ACC_STAT);
			Response_AccStat = SendRequest(IMU_TYPE_SCR2100_Y, REQ_READ_COM_STAT);
			Response_ComStat = SendRequest(IMU_TYPE_SCR2100_Y, REQ_READ_TEMP);  // Request temperature data to get the measurement loop to continue correctly after reading the status registers
			
			printf("SCR2100-Y read error,error code: 0x%02X\r\n", DataError);
			printf("SCR2100-Y --> StatSum: %08X, RateStat1: %08X, RateStat2: %08X, AccStat: %08X, ComStat:  %08X\r\n", 
				     Response_StatSum, Response_RateStat1, Response_RateStat2, Response_AccStat, Response_ComStat);
	  }
		else
		{
//			 printf("SCR2100-Y --> RATE: %08X\r\n", Response_Rate);
	    //printf("SCR2100-Y --> RATE: %f, TEMP: %f\r\n", Rate/GYRO_SENSITIVITY, (Temp/14.7 + 60.0)); 
			 Temp = Temp;
			 stImuData->iGyro[1] = Rate;
			 stImuData->dGyro[1] = Rate/GYRO_SENSITIVITY;
		}
}

static void SCC2230_ReadAndProcessData(imu_data_t *stImuData)
{
		uint32_t Response_Rate = 0;
		uint32_t Response_Acc_X = 0;
		uint32_t Response_Acc_Y = 0;
		uint32_t Response_Acc_Z = 0;
		uint32_t Response_Temp = 0;
		
		uint32_t Response_RateStat1 = 0;
		uint32_t Response_RateStat2 = 0;
		uint32_t Response_AccStat = 0;
		uint32_t Response_ComStat = 0;
		uint32_t Response_StatSum = 0;

		uint8_t RSdata = 0;
	  int16_t Rate = 0;
	  int16_t Acc_X = 0;
		int16_t Acc_Y = 0;
		int16_t Acc_Z = 0;
		int16_t Temp = 0;
	  uint8_t DataError = 0;

// Read temperature, rate & accelerations. Note: interleaved reading due to off-frame protocol
		Response_Temp = SendRequest(IMU_TYPE_SCC2230, REQ_READ_RATE);
		Response_Rate = SendRequest(IMU_TYPE_SCC2230, REQ_READ_ACC_X);
		Response_Acc_X = SendRequest(IMU_TYPE_SCC2230, REQ_READ_ACC_Y);
		Response_Acc_Y = SendRequest(IMU_TYPE_SCC2230, REQ_READ_ACC_Z);
		Response_Acc_Z = SendRequest(IMU_TYPE_SCC2230, REQ_READ_TEMP);
		 
//------------------------------------------------------------------------------------------------------
		// Handle rate data
		RSdata = (Response_Rate & RS_FIELD_MASK) >> 24;
		if (RSdata != 1) 
		{
				DataError |= 0x01;
		}
		else 
		{
				Rate = (Response_Rate & DATA_FIELD_MASK) >> 8;
				DataError &= ~0x01; 
		}
		// Check CRC if necessary
		// if (CalculateCRC(Response_Rate) != (Response_Rate & CRC_FIELD_MASK)) DataError |= 0x01;

		// Handle accelerometer data
		RSdata = (Response_Acc_X & RS_FIELD_MASK) >> 24;
		if (RSdata != 1) 
		{
				DataError |= 0x02;
		}
		else
		{
				Acc_X = (Response_Acc_X & DATA_FIELD_MASK) >> 8;
				DataError &= ~0x02; 
		}
		RSdata = (Response_Acc_Y & RS_FIELD_MASK) >> 24;
		if (RSdata != 1) 
		{
				DataError |= 0x04;
		}
		else
		{
				Acc_Y = (Response_Acc_Y & DATA_FIELD_MASK) >> 8;
				DataError &= ~0x04; 
		}
		RSdata = (Response_Acc_Z & RS_FIELD_MASK) >> 24;
		if (RSdata != 1) 
		{
				DataError |= 0x08;
		}
		else
		{
				Acc_Z = (Response_Acc_Z & DATA_FIELD_MASK) >> 8;
				DataError &= ~0x08;
		}

		// Handle temperature data
		RSdata = (Response_Temp & RS_FIELD_MASK) >> 24;
		if (RSdata != 1)
		{
				DataError |= 0x10;
		}		
		else
		{
				Temp = (Response_Temp & DATA_FIELD_MASK) >> 8;
				DataError &= ~0x10;
		}

		if (DataError)
		{
				// Clear status registers
				SendRequest(IMU_TYPE_SCC2230, REQ_READ_STAT_SUM);    // Clear the data that is coming from previous frame, request status summary
				Response_StatSum = SendRequest(IMU_TYPE_SCC2230, REQ_READ_RATE_STAT1);
				Response_RateStat1 = SendRequest(IMU_TYPE_SCC2230, REQ_READ_RATE_STAT2);
				Response_RateStat2 = SendRequest(IMU_TYPE_SCC2230, REQ_READ_ACC_STAT);
				Response_AccStat = SendRequest(IMU_TYPE_SCC2230, REQ_READ_COM_STAT);
				Response_ComStat = SendRequest(IMU_TYPE_SCC2230, REQ_READ_TEMP);  // Request temperature data to get the measurement loop to continue correctly after reading the status registers
				
				printf("SCC2230-B15 read error,error code: 0x%02X\r\n", DataError);
				printf("SCC2230-B15 --> StatSum: %08X, RateStat1: %08X, RateStat2: %08X, AccStat: %08X, ComStat:  %08X\r\n", 
							 Response_StatSum, Response_RateStat1, Response_RateStat2, Response_AccStat, Response_ComStat);
		}
		else
		{
//				printf("SCC2230-B15 --> RATE: %08X, ACC_X: %08X, ACC_Y: %08X, ACC_Z: %08X\r\n", Response_Rate, Response_Acc_X, Response_Acc_Y, Response_Acc_Z);
		   //printf("SCC2230-B15 --> RATE: %f, ACC_X: %f, ACC_Y: %f, ACC_Z: %f, TEMP: %f\r\n", Rate/GYRO_SENSITIVITY, Acc_X/ACC_SENSITIVITY, Acc_Y/ACC_SENSITIVITY, Acc_Z/ACC_SENSITIVITY, (Temp/14.5 + 60.0)); 
				
				stImuData->iGyro[2] = Rate;
				stImuData->iAccel[0] = Acc_X;
				stImuData->iAccel[1] = Acc_Y;
				stImuData->iAccel[2] = Acc_Z;
				stImuData->iTemp = Temp;
				
				stImuData->dGyro[2] = Rate/GYRO_SENSITIVITY;
				stImuData->dAccel[0] = Acc_X/ACC_SENSITIVITY;
				stImuData->dAccel[1] = Acc_Y/ACC_SENSITIVITY;
				stImuData->dAccel[2] = Acc_Z/ACC_SENSITIVITY;
				stImuData->dTemp = Temp/14.5 + 60.0;
		}
}

void muRuta_Init(void)
{
	muRuta_ResetPinInit();
	
  SCR2100_X_Init();
	SCR2100_Y_Init();
	SCC2230_Z_Init();
}

void muRuta_data_read(imu_data_t *stImuData)
{
	uint8_t i = 0;
	
	for (i=0; i<5; i++)
	{
			SCR2100_X_ReadAndProcessData(stImuData);
			SCR2100_Y_ReadAndProcessData(stImuData);
			SCC2230_ReadAndProcessData(stImuData);
	}
}

//// Calculate CRC for 24 MSB's of the 32 bit dword
//// (8 LSB's are the CRC field and are not included in CRC calculation)
//static uint8_t CRC8(uint32_t BitValue, uint8_t CRC)
//{
//	uint8_t Temp = (uint8_t)(CRC & 0x80);
//	if (BitValue != 0)
//	{
//		Temp ^= 0x80;
//	}
//	CRC <<= 1;
//	if (Temp > 0)
//	{
//		CRC ^= 0x1D;
//	}
//	return CRC;
//}

//uint8_t CalculateCRC(uint32_t Data)
//{
//	uint32_t BitMask;
//	uint32_t BitValue;
//	uint8_t CRC = 0xFF;
//	for (BitMask = 0x80000000; BitMask != 0x80; BitMask >>= 1)
//	{
//		BitValue = Data & BitMask;
//		CRC = CRC8(BitValue, CRC);
//	}
//	CRC = (uint8_t)~CRC;
//	return CRC;
//}

/**
* @brief  Initializes the peripherals used by the SPI SCC2230 driver.
* @param  None
* @retval None
*/



/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/  

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
