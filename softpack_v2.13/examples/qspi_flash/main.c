/* ----------------------------------------------------------------------------
 *         SAM Software Package License
 * ----------------------------------------------------------------------------
 * Copyright (c) 2015, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

/**
 * \page qspi_flash QSPI Example
 *
 * \section Purpose
 *
 * This example indicates how to use the spi_nor driver in order to
 * access data on a QSPI device.
 *
 * \section Requirements
 *
 * This package can be used with SAMA5D2-PTC-EK, SAMA5D2-XPLAINED, SAMA5D27-SOM1-EK.
 *
 * \section Descriptions
 *
 * This example shows how to use current spi driver package which composed of 
 * low-level functions for spi-nor devices to access QSPI device. All external API 
 * functions that support the unified interface of functions and operations for 
 * all supported QSPI Devices. 
 *
 * \section Usage
 *
 * -# Compile the application and connect the DBGU port of the evaluation board
 *    to the computer.
 * -# Open and configure a terminal application on PC
 *    (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *   - 115200 bauds
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# Download the program inside the evaluation board and run it. Please refer to
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/6421B.pdf">
 *    SAM-BA User Guide</a>, the
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">
 *    GNU-Based Software Development</a> application note or to the
 *    <a href="ftp://ftp.iar.se/WWWfiles/arm/Guides/EWARM_UserGuide.ENU.pdf">
 *    IAR EWARM User Guide</a>, depending on your chosen solution.
 * -# Upon startup, the application will output the following line on the DBGU:
 *    \code
 *     -- QSPI Example xxx --
 *     -- SAMxxxxx-xx
 *     -- Compiled: xxx xx xxxx xx:xx:xx --
 *    \endcode
 * -# Choose an item in the menu to test.
 *
 * \section References
 * - qspi_flash/main.c
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "board_spi.h"
#include "chip.h"
#include "compiler.h"
#include "crypto/trng.h"
#include "gpio/pio.h"
#include "mm/cache.h"
#include "can/can-bus.h"
#include "nvm/spi-nor/spi-nor.h"
#include "peripherals/pmc.h"
#include "serial/console.h"
#include "spi/qspi.h"
#include "trace.h"


int main(void)
{
	/***********************************
         * 1. Test Led
         **********************************/
        board_cfg_led();
        led_set(LED_BLUE);

        /***********************************
         * 3. Test ADC
         **********************************/
        adc_test();
        
        /***********************************
         * 4. Test Usart
         **********************************/
	/* Output example information */
	console_example_info("Test Usart Example");
        
        /***********************************
         * 5. Test QSPI
         **********************************/
	qspi_test();
        
        /***********************************
         * 6. Test CAN
         **********************************/
        can_test();
}
