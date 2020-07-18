/*
 * Copyright (c) 2019 R3 IoT Ltd.
 *
 * License: Apache-2.0
*/


#include <zephyr/types.h>
#include <device.h>
#include <drivers/uart.h>

#include <sdi_12/sdi_12_uart.h>

#define LOG_LEVEL CONFIG_SDI_12_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(sdi_12_uart);

struct uart_config uart_conf;

struct k_timer tx_ongoing;

struct k_sem tx_finished_tmr_sem;

void tx_ongoing_timer_clbk(struct k_timer* timer_id);

K_TIMER_DEFINE(tx_ongoing_timer,
	       tx_ongoing_timer_clbk, NULL);

struct uart_irq_data {
	struct device *uart_dev;
	uint8_t *tx_buffer;
	uint8_t *rx_buffer;
	unsigned int tx_remaining;
	int rx_remaining;
	unsigned int rx_collected;
	struct k_sem tx_complete_sem;
	struct k_sem rx_complete_sem;
	struct k_sem rx_first_received_sem;
	bool term_found;
};

static struct uart_irq_data irq_data;

void tx_ongoing_timer_clbk(struct k_timer* timer_id)
{
	k_sem_give(&tx_finished_tmr_sem);
}

int8_t sdi_12_uart_init(struct device *uart_dev);

int8_t sdi_12_uart_send_break(struct device *uart_dev);

int8_t sdi_12_uart_tx(struct device *uart_dev, uint8_t *buffer, unsigned int len);

int8_t sdi_12_uart_rx(struct device *uart_dev, uint8_t *buffer,
			unsigned int len, unsigned int timeout_start,
			unsigned int timeout_end);

static void sdi_12_uart_isr(void *user_data)
{
	struct uart_irq_data *data = user_data;
	unsigned int ret;
	char c = 0;
	int i;

	uart_irq_update(data->uart_dev);

	if (uart_irq_rx_ready(data->uart_dev)) {
		while ( uart_fifo_read(data->uart_dev, &c, 1) != 0) {
			if ( data->rx_remaining > 0 ) {
				data->rx_buffer[0] = c;
				data->rx_buffer++;
				data->rx_remaining--;
				data->rx_collected++;
				if (data->rx_collected >= 2) {
					data->term_found = \
					data->rx_buffer[-2] == SDI_12_TERM[0] &&
					data->rx_buffer[-1] == SDI_12_TERM[1];
				} else {
					data->term_found = false;
				}

				k_sem_give(&data->rx_first_received_sem);
			}

			if ( data->rx_remaining == 0 || data->term_found) {
				/* Stop processing received data */
				data->rx_remaining = -1;
				uart_irq_rx_disable(data->uart_dev);
				k_sem_give(&data->rx_first_received_sem);
				k_sem_give(&data->rx_complete_sem);
			}
		}

	}

	if (uart_irq_tx_ready(data->uart_dev)) {
		if ( data->tx_remaining > 0 ) {
			ret = uart_fifo_fill(data->uart_dev,
					     data->tx_buffer,
					     data->tx_remaining);
			for (i=0; i<ret; i++) {
				// LOG_DBG("tx:0x%x", data->tx_buffer[i]);
			}
			if (ret < data->tx_remaining) {
				data->tx_buffer += ret;
				data->tx_remaining -= ret;
			} else {
				// LOG_DBG("TX done");
				data->tx_remaining = 0;
				uart_irq_tx_disable(data->uart_dev);
				k_sem_give(&data->tx_complete_sem);
			}
		}
	}
}

int8_t sdi_12_uart_init(struct device *uart_dev)
{
	int ret;
	uart_conf.baudrate = SDI_12_BAUDRATE;
	uart_conf.parity = SDI_12_PARITY;
	uart_conf.stop_bits = SDI_12_STOP_BITS;
	uart_conf.data_bits = SDI_12_DATA_BITS;
	uart_conf.flow_ctrl = SDI_12_FLOW_CONTROL;

	ret = uart_configure(uart_dev, &uart_conf);

	if (ret < 0 ) {
		LOG_ERR("UART config error: %d", ret);
		return SDI_12_STATUS_CONFIG_ERROR;
	}

	irq_data.uart_dev = uart_dev;
	irq_data.tx_buffer = NULL;
	irq_data.rx_buffer = NULL;
	irq_data.tx_remaining = 0;
	irq_data.rx_remaining = -1;
	irq_data.rx_collected = 0;
	k_sem_init(&irq_data.tx_complete_sem, 0, 1);
	k_sem_init(&irq_data.rx_complete_sem, 0, 1);
	k_sem_init(&irq_data.rx_first_received_sem, 0, 1);
	k_sem_init(&tx_finished_tmr_sem, 0, 1);

	uart_irq_tx_disable(uart_dev);
	uart_irq_rx_disable(uart_dev);

	uart_irq_callback_user_data_set(uart_dev, sdi_12_uart_isr, &irq_data);

	return SDI_12_STATUS_OK;
}

/* static int8_t sdi_12_send_break(struct device *uart_dev) */
int8_t sdi_12_uart_send_break(struct device *uart_dev)
{
	int ret;
	char zero = '\0';

	uart_conf.baudrate = SDI_12_BREAK_BAUDRATE;
	ret = uart_configure(uart_dev, &uart_conf);

	if (ret < 0) {
		LOG_ERR("UART config error: %d", ret);
		return SDI_12_STATUS_CONFIG_ERROR;
	}

	LOG_DBG("Sending break");
	sdi_12_uart_tx(uart_dev, &zero, 1);
	k_sleep(K_MSEC(SDI_12_BREAKING_MS - SDI_12_SINGLE_SYMBOL_MS));
	uart_conf.baudrate = SDI_12_BAUDRATE;

	ret = uart_configure(uart_dev, &uart_conf);

	if (ret < 0) {
		LOG_ERR("UART config error: %d", ret);
		return SDI_12_STATUS_CONFIG_ERROR;
	}

	return 0;
}

int8_t sdi_12_uart_tx(struct device *uart_dev, uint8_t *buffer,
			unsigned int len)
{
	LOG_INF("TX first %d chars of %s", len, log_strdup(buffer));

	irq_data.tx_buffer = buffer;

	irq_data.tx_remaining = len;

	k_timer_start(&tx_ongoing_timer, K_MSEC(SDI_12_SINGLE_SYMBOL_MS*len+1),
			K_NO_WAIT);

	uart_irq_tx_enable(uart_dev);
	/* Rx is also enabled because TX bytes will be Rxed as a single line is
	   used - they need to be discarded. */
	uart_irq_rx_enable(uart_dev);

	k_sem_take(&irq_data.tx_complete_sem, K_FOREVER);
	k_sem_take(&tx_finished_tmr_sem, K_FOREVER);
	
	uart_irq_rx_disable(uart_dev);

	LOG_DBG("TX done");

	k_timer_stop(&tx_ongoing_timer);

	return SDI_12_STATUS_OK;
}

int8_t sdi_12_uart_rx(struct device *uart_dev, uint8_t *buffer,
			unsigned int len, unsigned int timeout_start,
			unsigned int timeout_end)
{
	int ret;

	irq_data.rx_buffer = buffer;
	irq_data.rx_collected = 0;
	irq_data.rx_remaining = len - 1;
	irq_data.term_found = false;

	/* Consume possible leftover semaphore */
	ret = k_sem_take(&irq_data.rx_first_received_sem, K_NO_WAIT);
	if (ret == 0) {
		LOG_DBG("Consume leftover semaphore");
	}

	uart_irq_rx_enable(uart_dev);

	ret = k_sem_take(&irq_data.rx_first_received_sem, K_MSEC(timeout_start));
	if (ret == -EAGAIN) {
		irq_data.rx_remaining = -1;
		uart_irq_rx_disable(irq_data.uart_dev);
		LOG_DBG("Timed out waiting for first char (%dms, sem:%d)",
			timeout_start, 	k_sem_count_get(&irq_data.rx_first_received_sem));
		return SDI_12_STATUS_TIMEOUT;
	}

	/* This should technically time out after timeout_end-<whatever time
	   passed since the begginging of this function> but we'd need to create
	   a new timer to measure that so it's likely not worth it */
	LOG_DBG("rx_complete_sem: %d", k_sem_count_get(&irq_data.rx_complete_sem));
	ret = k_sem_take(&irq_data.rx_complete_sem, K_MSEC(timeout_end));
	if (ret == -EAGAIN) {
		irq_data.rx_remaining = -1;
		uart_irq_rx_disable(irq_data.uart_dev);
		LOG_DBG("Timed out waiting for complete RX (%dms)",
			timeout_end);
		return SDI_12_STATUS_TIMEOUT;
	}

	uart_irq_rx_disable(irq_data.uart_dev);
	irq_data.rx_remaining = -1;

	if ( !irq_data.term_found ) {
		/* If no timeout occured and no terminator was found it must
		   mean that RX stopped due to a full buffer. */ 
		LOG_ERR("Buffer Full");
		return SDI_12_STATUS_BUFFER_FULL;
	} else {
		buffer[irq_data.rx_collected] = '\0';
		irq_data.term_found = false;
	}

	LOG_INF("RXed: %s (first %d chars)",
		log_strdup(buffer), irq_data.rx_collected);

	return SDI_12_STATUS_OK;

}