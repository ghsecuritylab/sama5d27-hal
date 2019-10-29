/* ----------------------------------------------------------------------------
 *         SAM Software Package License
 * ----------------------------------------------------------------------------
 * Copyright (c) 2016, Atmel Corporation
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

/*------------------------------------------------------------------------------
 *         Headers
 *------------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "timer.h"
#include "analog/adc.h"
#include "analog/adcd.h"
#include "mm/cache.h"
#include "board_adc.h"

/*---------------------------------------------------------------------------
 *         Local Define
 *---------------------------------------------------------------------------*/
/** ADC clock */
#define ADC_FREQ (300000)

/** ADC VREF */
#define BOARD_ADC_VREF (3300)

/** MAXIMUM DIGITAL VALUE */
#define DIGITAL_MAX    ((1 << adc_get_resolution()) - 1)

/** Total number of ADC channels in use */
#define NUM_CHANNELS    ARRAY_SIZE(adc_channels)

#define ADC_CHANNEL_NUM_IN_LCDR(d) (((d) & ADC_LCDR_CHNB_Msk) >> ADC_LCDR_CHNB_Pos)
#define ADC_LAST_DATA_IN_LCDR(d)  (((d) & ADC_LCDR_LDATA_Msk) >> ADC_LCDR_LDATA_Pos)


/*---------------------------------------------------------------------------
 *         Local variables
 *---------------------------------------------------------------------------*/
/** ADC sample data */
static struct _adc_sample _data;

/** ADCD configuration for next capture */
static struct _adcd_cfg adcd_cfg;

static struct _timeout adc_timeout;

/** ADC channels to acquire */
static uint8_t adc_channels[] = { 0, 6 };

/** ADCD instance */
static struct _adcd_desc adcd;

CACHE_ALIGNED static uint16_t adc_buffer[NUM_CHANNELS];

static unsigned count = 0;

/** ADC sample data */
struct _adc_sample
{
	uint8_t channel[NUM_CHANNELS];
	int16_t value[NUM_CHANNELS];
};

enum _adc_state {
	STATE_WAITING = 0,
	STATE_CONFIGURED = 1,
	STATE_STARTED = 2,
	STATE_CAPTURING = 3,
	STATE_CAPTURED = 4,
};

static volatile enum _adc_state state;

/** Used to store ADC conversion */
static struct _buffer buf_adc = {
	.data = (uint8_t*)adc_buffer,
	.size = NUM_CHANNELS * sizeof(uint16_t),
};

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/
static int _adc_callback(void* args, void* arg2)
{
	int i;
        printf("num_channels = %d\n",NUM_CHANNELS);
	for (i = 0; i < NUM_CHANNELS; ++i) {
		_data.channel[i] = ADC_CHANNEL_NUM_IN_LCDR(adc_buffer[i]);
		_data.value[i] = ADC_LAST_DATA_IN_LCDR(adc_buffer[i]);
	}

	state = STATE_CAPTURED;
	return 0;
}

/**
 * \brief (Re)init config ADC.
 *
 */
static void _adc_configure(void)
{
	struct _callback _cb;
	int i;
        
        memcpy(&adcd.cfg, &adcd_cfg, sizeof(adcd_cfg));

	/* Init channel number and reset values */
	for (i = 0; i < NUM_CHANNELS; i++) {
		adcd.cfg.channel_mask |= (1u << adc_channels[i]);
		_data.channel[i] = 0;
		_data.value[i] = 0;
	}

	callback_set(&_cb, _adc_callback, &adcd);
	adcd_initialize(&adcd);
	adcd_transfer(&adcd, &buf_adc, &_cb);
        
        state = STATE_CONFIGURED;
}

static void _adc_start(void)
{
        timer_start_timeout(&adc_timeout, 250);
	state = STATE_STARTED;
}

static void _adc_trigger_capture(void)
{
	/* ADC software trigger every 500ms */
	if (timer_timeout_reached(&adc_timeout)) {
		state = STATE_CAPTURING;
		adc_start_conversion();
		//led_toggle(LED_RED);
	}
}

static void _adc_print_results(void)
{
	int i;

	printf("Count: %08d ", count++);
	for (i = 0; i < NUM_CHANNELS; ++i) {
		printf(" CH%02d: %04d mV ", _data.channel[i],
			(int)(_data.value[i] * BOARD_ADC_VREF / DIGITAL_MAX));
	}
	printf("\r");
	state = STATE_WAITING;
}

/*---------------------------------------------------------------------------
 *         Global functions
 *---------------------------------------------------------------------------*/
int adc_test(void)
{
        board_cfg_adc();
 
        /* Set defaut ADC configuration */
	memset(&adcd_cfg, 0, sizeof(adcd_cfg));
	adcd_cfg.trigger_mode = TRIGGER_MODE_SOFTWARE;
	adcd_cfg.trigger_edge = TRIGGER_NO;
	adcd_cfg.freq = ADC_FREQ;
        adcd_cfg.dma_enabled = 1;
	memcpy(&adcd.cfg, &adcd_cfg, sizeof(adcd_cfg));
        
        state = STATE_WAITING;
        
        while (1)
	{
		switch (state) {
			case STATE_WAITING:
				_adc_configure();
				break;
			case STATE_CONFIGURED:
				_adc_start();
			case STATE_STARTED:
				_adc_trigger_capture();
				break;
			case STATE_CAPTURING:
				break;
			case STATE_CAPTURED:
				_adc_print_results();
				break;
		}
                if(count > 5)
                  break;
	}
        
        return 1;
}