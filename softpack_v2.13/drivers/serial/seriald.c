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

/*----------------------------------------------------------------------------
*        Headers
*----------------------------------------------------------------------------*/

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "chip.h"
#include "gpio/pio.h"
#include "irq/irq.h"
#ifdef CONFIG_HAVE_L1CACHE
#include "mm/l1cache.h"
#endif
#ifdef CONFIG_HAVE_L2CACHE
#include "mm/l2cache.h"
#endif
#ifdef CONFIG_HAVE_MMU
#include "mm/mmu.h"
#endif
#ifdef CONFIG_HAVE_FLEXCOM
#include "peripherals/flexcom.h"
#endif
#include "peripherals/pmc.h"
#ifdef CONFIG_HAVE_DBGU
#include "serial/dbgu.h"
#endif
#include "serial/uart.h"
#include "serial/usart.h"
#include "seriald.h"
#include "trace.h"

/*----------------------------------------------------------------------------
 *        Local Types
 *----------------------------------------------------------------------------*/

typedef void (*init_handler_t)(void*, uint32_t, uint32_t);
typedef void (*put_char_handler_t)(void*, uint8_t);
typedef bool (*tx_empty_handler_t)(void*);
typedef uint8_t (*get_char_handler_t)(void*);
typedef bool (*rx_ready_handler_t)(void*);
typedef void (*enable_it_handler_t)(void*, uint32_t);
typedef void (*disable_it_handler_t)(void*, uint32_t);

struct _seriald_ops {
	uint32_t             mode;
	uint32_t             rx_int_mask;
	init_handler_t       init;
	put_char_handler_t   put_char;
	tx_empty_handler_t   tx_empty;
	get_char_handler_t   get_char;
	rx_ready_handler_t   rx_ready;
	enable_it_handler_t  enable_it;
	disable_it_handler_t disable_it;
};

static struct _uart_desc *_serial[UART_IFACE_COUNT];
#define UARTD_POLLING_THRESHOLD  16

/*----------------------------------------------------------------------------
 *        Variables
 *----------------------------------------------------------------------------*/

#ifdef CONFIG_HAVE_SERIALD_USART
static const struct _seriald_ops seriald_ops_usart = {
	.mode = US_MR_CHMODE_NORMAL | US_MR_PAR_NO | US_MR_CHRL_8_BIT,
	.rx_int_mask = US_IER_RXRDY,
	.init = (init_handler_t)usart_configure,
	.put_char = (put_char_handler_t)usart_put_char,
	.tx_empty = (tx_empty_handler_t)usart_is_tx_empty,
	.get_char = (get_char_handler_t)usart_get_char,
	.rx_ready = (rx_ready_handler_t)usart_is_rx_ready,
	.enable_it = (enable_it_handler_t)usart_enable_it,
	.disable_it = (disable_it_handler_t)usart_disable_it,
};
#endif

#ifdef CONFIG_HAVE_SERIALD_UART
static const struct _seriald_ops seriald_ops_uart = {
	.mode = UART_MR_CHMODE_NORMAL | UART_MR_PAR_NO,
	.rx_int_mask = UART_IER_RXRDY,
	.init = (init_handler_t)uart_configure,
	.put_char = (put_char_handler_t)uart_put_char,
	.tx_empty = (tx_empty_handler_t)uart_is_tx_empty,
	.get_char = (get_char_handler_t)uart_get_char,
	.rx_ready = (rx_ready_handler_t)uart_is_rx_ready,
	.enable_it = (enable_it_handler_t)uart_enable_it,
	.disable_it = (disable_it_handler_t)uart_disable_it,
};
#endif

#ifdef CONFIG_HAVE_SERIALD_DBGU
static const struct _seriald_ops seriald_ops_dbgu = {
	.mode = DBGU_MR_CHMODE_NORM | DBGU_MR_PAR_NONE,
	.rx_int_mask = DBGU_IER_RXRDY,
	.init = (init_handler_t)dbgu_configure,
	.put_char = (put_char_handler_t)dbgu_put_char,
	.tx_empty = (tx_empty_handler_t)dbgu_is_tx_empty,
	.get_char = (get_char_handler_t)dbgu_get_char,
	.rx_ready = (rx_ready_handler_t)dbgu_is_rx_ready,
	.enable_it = (enable_it_handler_t)dbgu_enable_it,
	.disable_it = (disable_it_handler_t)dbgu_disable_it,
};
#endif

/*------------------------------------------------------------------------------
 *         Local functions
 *------------------------------------------------------------------------------*/

static void seriald_handler(uint32_t source, void* user_arg)
{
	const struct _seriald* serial = (struct _seriald*)user_arg;
	uint8_t c;

	if (!seriald_is_rx_ready(serial))
		return;

	c = seriald_get_char(serial);
	if (serial->rx_handler)
		serial->rx_handler(c);
}

/*------------------------------------------------------------------------------
 *         Exported functions
 *------------------------------------------------------------------------------*/

int seriald_configure(struct _seriald* serial, void* addr, uint32_t baudrate)
{
	uint32_t id = ID_PERIPH_COUNT;
	const struct _seriald_ops* ops = NULL;

	if (!serial)
		return -EINVAL;

#ifdef CONFIG_HAVE_SERIALD_USART
	id = get_usart_id_from_addr((Usart*)addr);
	if (id != ID_PERIPH_COUNT) {
		ops = &seriald_ops_usart;
#ifdef CONFIG_HAVE_FLEXCOM
		Flexcom* flexcom = get_flexcom_addr_from_id(id);
		if (flexcom)
			flexcom_select(flexcom, FLEX_MR_OPMODE_USART);
#endif
	}
#endif
#ifdef CONFIG_HAVE_SERIALD_UART
	if (!ops) {
		id = get_uart_id_from_addr((Uart*)addr);
		if (id != ID_PERIPH_COUNT)
			ops = &seriald_ops_uart;
	}
#endif
#ifdef CONFIG_HAVE_SERIALD_DBGU
	if (!ops) {
		if (addr == DBGU) {
			id = ID_DBGU;
			ops = &seriald_ops_dbgu;
		}
	}
#endif
	if (!ops)
		return -ENODEV;

	/* Save serial peripheral address and ID */
	memset(serial, 0, sizeof(*serial));
	serial->id = id;
	serial->addr = addr;
	serial->ops = ops;

	/* Initialize driver to use */
	pmc_configure_peripheral(id, NULL, true);
	ops->init(addr, ops->mode, baudrate);

	return 0;
}

void seriald_put_char(const struct _seriald* serial, uint8_t c)
{
	if (!serial || !serial->id)
		return;

	serial->ops->put_char(serial->addr, c);
}

void seriald_put_string(const struct _seriald* serial, const uint8_t* str)
{
	if (!serial || !serial->id)
		return;

	while (*str)
		serial->ops->put_char(serial->addr, *str++);
}

bool seriald_is_tx_empty(const struct _seriald* serial)
{
	if (!serial || !serial->id)
		return true;

	return serial->ops->tx_empty(serial->addr);
}

uint8_t seriald_get_char(const struct _seriald* serial)
{
	if (!serial || !serial->id) {
		assert(0);
		while(1);
	}

	return serial->ops->get_char(serial->addr);
}

bool seriald_is_rx_ready(const struct _seriald* serial)
{
	if (!serial || !serial->id)
		return false;

	return serial->ops->rx_ready(serial->addr);
}

void seriald_set_rx_handler(struct _seriald* serial, seriald_rx_handler_t handler)
{
	if (!serial || !serial->id)
		return;

	serial->rx_handler = handler;
}

void seriald_enable_rx_interrupt(const struct _seriald* serial)
{
	if (!serial || !serial->id)
		return;

	irq_add_handler(serial->id, seriald_handler, (void*)serial);
	irq_enable(serial->id);
	serial->ops->enable_it(serial->addr, serial->ops->rx_int_mask);
}

void seriald_disable_rx_interrupt(const struct _seriald* serial)
{
	if (!serial || !serial->id)
		return;

	serial->ops->disable_it(serial->addr, serial->ops->rx_int_mask);
	irq_disable(serial->id);
	irq_remove_handler(serial->id, seriald_handler);
}

/*
* wwl add
* uart configure
*/
static int _uartd_dma_read_callback(void* arg, void* arg2)
{
	uint8_t iface = (uint32_t)arg;
	assert(iface < UART_IFACE_COUNT);
	struct _uart_desc *desc = _serial[iface];
	struct _dma_channel* channel = desc->dma.rx.channel;

	if (desc->timeout > 0) {
		desc->addr->UART_CR = UART_CR_STTTO;
		uart_disable_it(desc->addr, US_IDR_TIMEOUT);
	}

	if (!dma_is_transfer_done(channel))
		dma_stop_transfer(channel);
	dma_fifo_flush(channel);

	desc->rx.transferred = dma_get_transferred_data_len(channel, desc->dma.rx.cfg_dma.chunk_size, desc->dma.rx.cfg.len);
	dma_reset_channel(desc->dma.rx.channel);

	if (desc->rx.transferred > 0)
		cache_invalidate_region(desc->dma.rx.cfg.daddr, desc->rx.transferred);

	desc->rx.buffer.size = 0;

	mutex_unlock(&desc->rx.mutex);

	callback_call(&desc->rx.callback, NULL);

	return 0;
}

static void _uartd_handler(uint32_t source, void* user_arg)
{
	int iface;
	uint32_t status = 0;
	Uart* addr = get_uart_addr_from_id(source);
	bool _rx_stop = true;
	bool _tx_stop = true;

	for (iface = 0; iface < UART_IFACE_COUNT; iface++) {
		if (_serial[iface]->addr == addr) {
			status = 1;
			break;
		}
	}

	if (!status) {
		/* async descriptor not found, disable interrupt */
		uart_disable_it(addr, UART_IDR_RXRDY | UART_IDR_TXRDY | UART_IDR_TXEMPTY | UART_IDR_TIMEOUT);
		return;
	}

	struct _uart_desc* desc = _serial[iface];
	status = uart_get_masked_status(addr);
	desc->rx.has_timeout = false;

	if (UART_STATUS_RXRDY(status)) {
		if (desc->rx.buffer.size) {
			desc->rx.buffer.data[desc->rx.transferred] = uart_get_char(addr);
			desc->rx.transferred++;

			if (desc->rx.transferred >= desc->rx.buffer.size)
				uart_disable_it(addr, UART_IDR_RXRDY);
			else
				_rx_stop = false;
		}
	}

	if (UART_STATUS_TXRDY(status)) {
		if (desc->tx.buffer.size) {
			uart_put_char(addr, desc->tx.buffer.data[desc->tx.transferred]);
			desc->tx.transferred++;

			if (desc->tx.transferred > desc->tx.buffer.size) {
				uart_disable_it(addr, UART_IDR_TXRDY);
				uart_enable_it(addr, UART_IER_TXEMPTY);
			}
			_tx_stop = false;
		}
	}

	if (UART_STATUS_TIMEOUT(status)) {
		switch (desc->transfer_mode) {
		case UARTD_MODE_ASYNC:
			desc->addr->UART_CR = UART_CR_STTTO;
			uart_disable_it(addr, UART_IDR_TIMEOUT);
			break;
		case UARTD_MODE_DMA:
			_uartd_dma_read_callback((void *)iface, NULL);
			break;
		}

		if (desc->rx.buffer.size)
			uart_disable_it(addr, UART_IDR_RXRDY);

		if (desc->tx.buffer.size)
			uart_disable_it(addr, UART_IDR_TXRDY | UART_IDR_TXEMPTY);

		desc->rx.has_timeout = true;
	}

	if (UART_STATUS_TXEMPTY(status)) {
		uart_disable_it(addr, UART_IDR_TXEMPTY);
	}

	if (_rx_stop) {
		desc->addr->UART_CR = UART_CR_STTTO;
		desc->rx.buffer.size = 0;
		mutex_unlock(&desc->rx.mutex);
	}
	if (_tx_stop) {
		desc->tx.buffer.size = 0;
		mutex_unlock(&desc->tx.mutex);
	}
}

void uartd_configure(uint8_t iface, struct _uart_desc* config)
{
	uint32_t id = get_uart_id_from_addr(config->addr);
	assert(id < ID_PERIPH_COUNT);
	assert(iface < UART_IFACE_COUNT);

	_serial[iface] = config;

	pmc_configure_peripheral(id, NULL, true);
	uart_configure(config->addr, config->mode, config->baudrate);
	uart_set_rx_timeout(config->addr, config->baudrate, config->timeout);
	irq_add_handler(get_uart_id_from_addr(config->addr), _uartd_handler, NULL);
	/* Enable USART interrupt */
	irq_enable(id);

	config->dma.rx.cfg_dma.incr_saddr = false;
	config->dma.rx.cfg_dma.incr_daddr = true;
	config->dma.rx.cfg_dma.loop = false;
	config->dma.rx.cfg_dma.data_width = DMA_DATA_WIDTH_BYTE;
	config->dma.rx.cfg_dma.chunk_size = DMA_CHUNK_SIZE_1;

	config->dma.tx.cfg_dma.incr_saddr = true;
	config->dma.tx.cfg_dma.incr_daddr = false;
	config->dma.tx.cfg_dma.loop = false;
	config->dma.tx.cfg_dma.data_width = DMA_DATA_WIDTH_BYTE;
	config->dma.tx.cfg_dma.chunk_size = DMA_CHUNK_SIZE_1;

	config->dma.rx.channel = dma_allocate_channel(id, DMA_PERIPH_MEMORY);
	assert(config->dma.rx.channel);

	config->dma.tx.channel = dma_allocate_channel(DMA_PERIPH_MEMORY, id);
	assert(config->dma.tx.channel);
}

void uartd_finish_tx_transfer(uint8_t iface)
{
	assert(iface < UART_IFACE_COUNT);
	mutex_unlock(&_serial[iface]->tx.mutex);
}

void uartd_wait_tx_transfer(const uint8_t iface)
{
	assert(iface < UART_IFACE_COUNT);
	while (mutex_is_locked(&_serial[iface]->tx.mutex));
}

static int _uartd_dma_write_callback(void* arg, void* arg2)
{
	uint8_t iface = (uint32_t)arg;
	assert(iface < UART_IFACE_COUNT);

	dma_reset_channel(_serial[iface]->dma.tx.channel);

	mutex_unlock(&_serial[iface]->tx.mutex);

	callback_call(&_serial[iface]->tx.callback, NULL);

	return 0;
}

static void _uartd_dma_write(uint8_t iface)
{
	struct _callback _cb;
	assert(iface < UART_IFACE_COUNT);
	struct _uart_desc* desc = _serial[iface];
	struct _dma_transfer_cfg cfg;

	cfg.saddr = desc->tx.buffer.data;
	cfg.daddr = (void *)&desc->addr->UART_THR;
	cfg.len = desc->tx.buffer.size;
	dma_configure_transfer(desc->dma.tx.channel, &desc->dma.tx.cfg_dma, &cfg, 1);

	callback_set(&_cb, _uartd_dma_write_callback, (void*)(uint32_t)iface);
	dma_set_callback(desc->dma.tx.channel, &_cb);
	cache_clean_region(cfg.saddr, cfg.len);
	dma_start_transfer(desc->dma.tx.channel);
}

static void _uartd_dma_read(uint8_t iface)
{
	struct _callback _cb;
	assert(iface < UART_IFACE_COUNT);
	struct _uart_desc* desc = _serial[iface];

	memset(&desc->dma.rx.cfg, 0x0, sizeof(desc->dma.rx.cfg));

	desc->dma.rx.cfg.saddr = (void *)&desc->addr->UART_RHR;
	desc->dma.rx.cfg.daddr = desc->rx.buffer.data;
	desc->dma.rx.cfg.len = desc->rx.buffer.size;
	dma_configure_transfer(desc->dma.rx.channel, &desc->dma.rx.cfg_dma, &desc->dma.rx.cfg, 1);

	callback_set(&_cb, _uartd_dma_read_callback, (void*)(uint32_t)iface);
	dma_set_callback(desc->dma.rx.channel, &_cb);
	uart_enable_it(desc->addr, UART_IER_TIMEOUT);
	uart_restart_rx_timeout(desc->addr);
	dma_start_transfer(desc->dma.rx.channel);
}

uint32_t uartd_transfer(uint8_t iface, struct _buffer* buf, struct _callback* cb)
{
	assert(iface < UART_IFACE_COUNT);
	struct _uart_desc *desc = _serial[iface];
	uint8_t tmode;
	uint32_t csr;
	uint32_t i;

	if ((buf == NULL) || (buf->size == 0))
		return UARTD_SUCCESS;

	if (buf->attr & UARTD_BUF_ATTR_READ) {
		if (!mutex_try_lock(&desc->rx.mutex))
			return UARTD_ERROR_LOCK;

		desc->rx.transferred = 0;
		desc->rx.buffer.data = buf->data;
		desc->rx.buffer.size = buf->size;
		desc->rx.buffer.attr = buf->attr;
		callback_copy(&desc->rx.callback, cb);

		desc->rx.has_timeout = false;
	}

	if (buf->attr & UARTD_BUF_ATTR_WRITE) {
		if (!mutex_try_lock(&desc->tx.mutex))
			return UARTD_ERROR_LOCK;

		desc->tx.transferred = 0;
		desc->tx.buffer.data = buf->data;
		desc->tx.buffer.size = buf->size;
		desc->tx.buffer.attr = buf->attr;
		callback_copy(&desc->tx.callback, cb);
	}

	tmode = desc->transfer_mode;

	/* If short transfer detected, use POLLING mode */
	if (tmode != UARTD_MODE_POLLING)
		if (buf->size < UARTD_POLLING_THRESHOLD)
			tmode = UARTD_MODE_POLLING;
        
        if (buf->attr & UARTD_BUF_ATTR_WRITE)
			_uartd_dma_write(0);
        
#if 0
	switch (tmode) {
	case UARTD_MODE_POLLING:
		i = 0;

		if (buf->attr & UARTD_BUF_ATTR_READ) {
			desc->addr->UART_SR;
			uart_restart_rx_timeout(desc->addr);
		}

		while (i < buf->size) {
			if (i < desc->tx.buffer.size) {
				{
					/* Wait for the transmitter to be ready */
					while (!UART_STATUS_TXRDY(desc->addr->UART_SR));

					writeb(&desc->addr->UART_THR, desc->tx.buffer.data[i]);
					i++;
				}
				desc->tx.transferred = i;

				if (desc->tx.transferred >= desc->tx.buffer.size) {
					desc->tx.buffer.size = 0;
					mutex_unlock(&desc->tx.mutex);
					callback_call(&desc->tx.callback, NULL);
				}
			}
			if (i < desc->rx.buffer.size) {
				{
					/* Wait for the transmitter to be ready */
					csr = desc->addr->UART_SR;
					while (!UART_STATUS_RXRDY(csr)) {
						if (UART_STATUS_TIMEOUT(csr)) {
							desc->addr->UART_CR = UART_CR_STTTO;
							desc->rx.buffer.size = 0;
							desc->rx.transferred = i;
							mutex_unlock(&desc->rx.mutex);
							return UARTD_ERROR_TIMEOUT;
						}
						csr = desc->addr->UART_SR;
					}

					readb(&desc->addr->UART_RHR, &desc->rx.buffer.data[i]);
					i++;
				}
				desc->rx.transferred = i;

				if (desc->rx.transferred >= desc->rx.buffer.size) {
					desc->rx.buffer.size = 0;
					mutex_unlock(&desc->rx.mutex);
					callback_call(&desc->rx.callback, NULL);
				}
			}
		}
		break;

	case UARTD_MODE_ASYNC:
		if (buf->attr & UARTD_BUF_ATTR_WRITE)
			uart_enable_it(desc->addr, UART_IER_TXRDY);

		if (buf->attr & UARTD_BUF_ATTR_READ) {
			uart_get_status(desc->addr);

			uart_restart_rx_timeout(desc->addr);
			uart_enable_it(desc->addr, UART_IER_RXRDY | UART_IER_TIMEOUT);
		}
		break;

	case UARTD_MODE_DMA:
		if (buf->attr & UARTD_BUF_ATTR_WRITE)
			_uartd_dma_write(0);
		if (buf->attr & UARTD_BUF_ATTR_READ)
			//_uartd_dma_read(0);
		break;

	default:
		trace_fatal("Unknown Uart mode!\r\n");
	}
#endif
	return UARTD_SUCCESS;
}
/* wwl add end */