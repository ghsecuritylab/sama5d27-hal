/**
  ******************************************************************************
  * @file    stm32_spi_sc2230.h
  * @author  lichunyu
  * @version V1.0
  * @date    02-March-2018
  * @brief   This file contains all the functions prototypes for the stm32_spi_sc2230
  *          firmware driver.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************  
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32_SPI_SCC2230_H
#define __STM32_SPI_SCC2230_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32h7xx_hal.h"

#include "imu_hal.h"
//#include <stdio.h>


/** @addtogroup Common
  * @{
  */	
// Product types
#define SCR 0 // Gyro-only
#define SCC 1 // Gyro & Accelerometer
	 
// Select the correct product type here
#define PRODUCT_TYPE SCC
//#define PRODUCT_TYPE SCR
	 
// SCC2xxx definitions
// Gyroscope and accelerometer sensitivities
#define GYRO_SENSITIVITY 50.0 // LSB/dps
#define ACC_SENSITIVITY 1962.0 // LSB/g

// SCC2xxx status register bits
#define BIT_LOOPF_OK 0x0040

// Standard requests
#define REQ_READ_RATE          0x040000f7
#define REQ_READ_ACC_X         0x100000e9
#define REQ_READ_ACC_Y         0x140000ef
#define REQ_READ_ACC_Z         0x180000e5
#define REQ_READ_TEMP          0x1c0000e3
#define REQ_READ_RATE_STAT1    0x240000c7
#define REQ_READ_RATE_STAT2    0x280000cd
#define REQ_READ_ACC_STAT      0x3c0000d3
#define REQ_READ_SERI_ID0      0x600000a1
#define REQ_READ_SERI_ID1      0x640000a7
#define REQ_READ_COM_STAT      0x6c0000ab
#define REQ_READ_STAT_SUM      0x7c0000b3
#define REQ_WRITE_FLT_60       0xfc200006
#define REQ_WRITE_FLT_10       0xfc1000c7

// Special requests
#define REQ_HARD_RESET         0xD8000431
#define REQ_MONITOR_ST_RESET   0xD80008AD

// Frame field masks
#define OPCODE_FIELD_MASK 0xFC000000
#define RS_FIELD_MASK 0x03000000
#define DATA_FIELD_MASK 0x00FFFF00
#define CRC_FIELD_MASK 0x000000FF

/**
  * @}
  */ 
#define SCR2100_X_CS_PIN                    GPIO_PIN_9                   /* PB.9 */
#define SCR2100_X_CS_GPIO_PORT              GPIOB                        /* GPIOB */
#define SCR2100_Y_CS_PIN                    GPIO_PIN_12                  /* PC.12 */
#define SCR2100_Y_CS_GPIO_PORT              GPIOC                        /* GPIOC */
#define SCC2230_Z_CS_PIN                    GPIO_PIN_4                   /* PA.04 */
#define SCC2230_Z_CS_GPIO_PORT              GPIOA                        /* GPIOA */

#define SCR2100_X_RESET_PIN                 GPIO_PIN_0                   /* PB.00 */
#define SCR2100_X_RESET_GPIO_PORT           GPIOB                        /* GPIOB */
#define SCR2100_X_RESET_GPIO_CLK            __HAL_RCC_GPIOB_CLK_ENABLE();  
#define SCR2100_Y_RESET_PIN                 GPIO_PIN_1                   /* PB.01 */
#define SCR2100_Y_RESET_GPIO_PORT           GPIOB                        /* GPIOB */
#define SCR2100_Y_RESET_GPIO_CLK            __HAL_RCC_GPIOB_CLK_ENABLE();  
#define SCC2230_Z_RESET_PIN                 GPIO_PIN_10                  /* PD.10 */
#define SCC2230_Z_RESET_GPIO_PORT           GPIOD                        /* GPIOD */
#define SCC2230_Z_RESET_GPIO_CLK            __HAL_RCC_GPIOD_CLK_ENABLE();    

#define SCR2100_X_CS_LOW()       HAL_GPIO_WritePin(SCR2100_X_CS_GPIO_PORT, SCR2100_X_CS_PIN, GPIO_PIN_RESET)    //PB12
#define SCR2100_X_CS_HIGH()      HAL_GPIO_WritePin(SCR2100_X_CS_GPIO_PORT, SCR2100_X_CS_PIN, GPIO_PIN_SET)  
#define SCR2100_Y_CS_LOW()       HAL_GPIO_WritePin(SCR2100_Y_CS_GPIO_PORT, SCR2100_Y_CS_PIN, GPIO_PIN_RESET)    //PA15
#define SCR2100_Y__CS_HIGH()     HAL_GPIO_WritePin(SCR2100_Y_CS_GPIO_PORT, SCR2100_Y_CS_PIN, GPIO_PIN_SET)  
#define SCC2230_Z_CS_LOW()       HAL_GPIO_WritePin(SCC2230_Z_CS_GPIO_PORT, SCC2230_Z_CS_PIN, GPIO_PIN_RESET)    //PA4
#define SCC2230_Z_CS_HIGH()      HAL_GPIO_WritePin(SCC2230_Z_CS_GPIO_PORT, SCC2230_Z_CS_PIN, GPIO_PIN_SET)  

#define SCR2100_X_RESET_LOW()    HAL_GPIO_WritePin(SCR2100_X_RESET_GPIO_PORT, SCR2100_X_RESET_PIN, GPIO_PIN_RESET)   //PE09
#define SCR2100_X_RESET_HIGH()   HAL_GPIO_WritePin(SCR2100_X_RESET_GPIO_PORT, SCR2100_X_RESET_PIN, GPIO_PIN_SET)  
#define SCR2100_Y_RESET_LOW()    HAL_GPIO_WritePin(SCR2100_Y_RESET_GPIO_PORT, SCR2100_Y_RESET_PIN, GPIO_PIN_RESET)   //PE07
#define SCR2100_Y_RESET_HIGH()   HAL_GPIO_WritePin(SCR2100_Y_RESET_GPIO_PORT, SCR2100_Y_RESET_PIN, GPIO_PIN_SET)  
#define SCC2230_Z_RESET_LOW()    HAL_GPIO_WritePin(SCC2230_Z_RESET_GPIO_PORT, SCC2230_Z_RESET_PIN, GPIO_PIN_RESET)   //PE10
#define SCC2230_Z_RESET_HIGH()   HAL_GPIO_WritePin(SCC2230_Z_RESET_GPIO_PORT, SCC2230_Z_RESET_PIN, GPIO_PIN_SET)  
/**
  * @}
  */ 
	
// Function prototypes
void muRuta_Init(void);
void muRuta_data_read(imu_data_t *stImuData);
//static uint8_t CRC8(uint32_t BitValue, uint8_t CRC);
//uint8_t CalculateCRC(uint32_t Data);



#ifdef __cplusplus
}
#endif

#endif /* __STM32_SPI_SCC2230_H */
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
