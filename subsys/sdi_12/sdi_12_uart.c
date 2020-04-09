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
struct k_timer rx_timeout_timer;
bool rx_timeout = 0;

int8_t sdi_12_uart_init(struct device *uart_dev);

int8_t sdi_12_uart_send_break(struct device *uart_dev);

int8_t sdi_12_uart_tx(struct device *uart_dev, uint8_t *buffer, unsigned int len);

int8_t sdi_12_uart_rx(struct device *uart_dev, uint8_t *buffer, unsigned int len,
				char *terminator, unsigned int timeout);


static void sdi_12_uart_rx_tmr_handler(struct k_timer *timer)
{
	bool *rx_timeout_p = k_timer_user_data_get(timer);
	*rx_timeout_p = 1;
	k_timer_stop(timer);
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

	k_timer_init(&rx_timeout_timer, sdi_12_uart_rx_tmr_handler, NULL);
	k_timer_user_data_set(&rx_timeout_timer, (void *)(&rx_timeout));

	return SDI_12_STATUS_OK;
}

/* static int8_t sdi_12_send_break(struct device *uart_dev) */
int8_t sdi_12_uart_send_break(struct device *uart_dev)
{
	int ret;


	uart_conf.baudrate = SDI_12_BREAK_BAUDRATE;
	ret = uart_configure(uart_dev, &uart_conf);

	if (ret < 0) {
		LOG_ERR("UART config error: %d", ret);
		return SDI_12_STATUS_CONFIG_ERROR;
	}

	LOG_DBG("Sending break");
	uart_poll_out(uart_dev, '\0');
	uart_conf.baudrate = SDI_12_BAUDRATE;

	ret = uart_configure(uart_dev, &uart_conf);

	if (ret < 0) {
		LOG_ERR("UART config error: %d", ret);
		return SDI_12_STATUS_CONFIG_ERROR;
	}

	return 0;
}

int8_t sdi_12_uart_tx(struct device *uart_dev, uint8_t *buffer, unsigned int len)
{
	int idx;

	for ( idx = 0; idx < len; idx++ ) {
		uart_poll_out(uart_dev, buffer[idx]);
	}

	LOG_DBG("TX: %s", log_strdup(buffer));

	return SDI_12_STATUS_OK;
}

int8_t sdi_12_uart_rx(struct device *uart_dev, uint8_t *buffer, unsigned int len,
				char *terminator, unsigned int timeout)
{
	/* For now this has an ugly, blocking implementation as a quick
	 * placeholder
	 */
	int idx;
	char c;
	int term_size;

	if (terminator != NULL) {
		term_size = strlen(terminator);
	} else {
		term_size = 0;
	}

	rx_timeout = 0;
	k_timer_start(&rx_timeout_timer, timeout,
			timeout);

	for ( idx = 0; idx < len-1; idx++ ) {
		while ( uart_poll_in(uart_dev, &c) != 0 ) {
			if ( rx_timeout != 0) {
				LOG_WRN("Response timeout");
				buffer[++idx] = '\0';
				LOG_DBG("RX: %s", log_strdup(buffer));
				return SDI_12_STATUS_TIMEOUT;
			}

			/* Shell uart needs this sleep to print correctly */
			k_sleep(1);
		}

		buffer[idx] = c;

		if (term_size > 0 && idx >= term_size-1) {
			if (strncmp(&(buffer[idx-term_size+1]),
				terminator, term_size) == 0) {
				LOG_DBG("Found terminator");
				buffer[++idx] = '\0';
				LOG_DBG("RX: %s", log_strdup(buffer));
				return SDI_12_STATUS_OK;
			}
		}
	}

	buffer[idx] = '\0';

	LOG_DBG("RX: %s", log_strdup(buffer));

	if (terminator != NULL) {
		return SDI_12_STATUS_BUFFER_FULL;
	} else {
		return SDI_12_STATUS_OK;
	}
}