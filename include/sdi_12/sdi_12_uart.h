#ifndef SDI_12_UART_H__
#define SDI_12_UART_H__

#include <drivers/uart.h>
#include <zephyr/types.h>
#include <device.h>

#include <sdi_12/sdi_12.h>

#define SDI_12_BAUDRATE 1200
#define SDI_12_PARITY UART_CFG_PARITY_EVEN
#define SDI_12_STOP_BITS UART_CFG_STOP_BITS_1
#define SDI_12_DATA_BITS UART_CFG_DATA_BITS_7
#define SDI_12_FLOW_CONTROL UART_CFG_FLOW_CTRL_NONE
#define SDI_12_BREAK_BAUDRATE 750
#define SDI_12_RESPONSE_TIMEOUT 5000

/**
 * @brief Initializes UART device with correct settings as per the SDI-12 spec
 *
 * @param uart_dev Device handle for the UART interface used as as the data line
 *
 * @return SDI_12_STATUS_OK if succeeded or SDI_12_STATUS_CONFIG_ERROR
 * status otheriwse. 
 */
int8_t sdi_12_uart_init(struct device *uart_dev);

/**
 * @brief Sends a break signal to sensor.
 *
 * A break being defined as a low level UART signal for at least 12m
 *
 * @param uart_dev Device handle for the UART interface used as as the data line
 *
 * @return SDI_12_STATUS_OK if succeeded or SDI_12_STATUS_CONFIG_ERROR
 * status otheriwse. 
 */
int8_t sdi_12_uart_send_break(struct device *uart_dev);

/**
 * @brief Transmitts buffer over uart
 *
 * @param uart_dev Device handle for the UART interface used as as the data line
 * @param buffer Pointer to the beginning of the buffer to send
 * @param len Number of bytes to send
 *
 * @return SDI_12_STATUS_OK if succeeded or a negative SDI_12_STATUS_e
 * status otheriwse. 
 */
int8_t sdi_12_uart_tx(struct device *uart_dev, uint8_t *buffer, unsigned int len);

/**
 * @brief Receives a number of characters over uart
 *
 * This function will read a number of characters over the provided UART device
 * until it completely fills the provided buffer, or (optionally) encounters
 * a terminating string, or (optionally) times out. In all of the above cases
 * all received characters are resident in the provided output buffer and '\0'
 * terminated.
 *
 * @param uart_dev Device handle for the UART interface used as as the data line
 * @param buffer Pointer to the beginning of the string where RXed characters
 * will be placed
 * @param len The size of available buffer. At most \p len - 1
 * @param terminator (optional) A pointer to a cstring which if detected at the
 * end of received transmission will finish reception
 * @param timeout Receive timeout
 *
 * @return SDI_12_STATUS_OK if succeeded, SDI_12_STATUS_BUFFER_FULL if
 * filled the buffer before fulfilling other conditions, or
 * SDI_12_STATUS_TIMEOUT if timed out.
 */
int8_t sdi_12_uart_rx(struct device *uart_dev, uint8_t *buffer, unsigned int len,
				char *terminator, unsigned int timeout);

#endif /* SDI_12_UART_H__ */