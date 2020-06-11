/**
 * @file sdi_12/sdi_12.h
 *
 * @brief Public APIs for the SDI-12 subsystem. 
 */

 /*
 * Copyright (c) 2019 R3 IoT Ltd.
 *
 * License: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_SDI_12_H_
#define ZEPHYR_INCLUDE_SDI_12_H_

#include <zephyr/types.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>

#include <sdi_12/sdi_12_uart.h>


typedef enum {
	SDI_12_STATUS_ERROR = INT8_MIN,
	SDI_12_STATUS_ADDR_MISMATCH,
	SDI_12_STATUS_ADDR_INVALID,
	SDI_12_STATUS_BAD_CRC,
	SDI_12_STATUS_CONFIG_ERROR,
	SDI_12_STATUS_BUFFER_FULL,
	SDI_12_STATUS_TIMEOUT,
	SDI_12_STATUS_OK = 0
} SDI_12_STATUS_e;

#define SDI_12_ERR_TO_STR(x) \
({ \
		char *tmp;				\
		switch(x){				\
		case SDI_12_STATUS_ERROR:		\
		tmp = "generic error";			\
		break;					\
		case SDI_12_STATUS_ADDR_MISMATCH:	\
		tmp = "addr mismatch";			\
		break;					\
		case SDI_12_STATUS_ADDR_INVALID:	\
		tmp = "addr invalid";			\
		break;					\
		case SDI_12_STATUS_BAD_CRC:	\
		tmp = "bad crc";			\
		break;					\
		case SDI_12_STATUS_CONFIG_ERROR:	\
		tmp = "config err";			\
		break;					\
		case SDI_12_STATUS_BUFFER_FULL:		\
		tmp = "buffer full";			\
		break;					\
		case SDI_12_STATUS_TIMEOUT:		\
		tmp = "timeout";			\
		break;					\
		case SDI_12_STATUS_OK:			\
		tmp = "ok";				\
		break;					\
		default:				\
		tmp = "";				\
		break;					\
		}					\
		tmp;					\
})

#define SDI_12_CMD_RESP_BUF_MAX_LEN 100
#define SDI_12_RESP_VALUES_MAX_CHARS 75
#define SDI_12_MAX_VALS_IN_RESP (SDI_12_RESP_VALUES_MAX_CHARS / 4)
#define SDI_12_RESP_MAX_READS_IN_GROUP (SDI_12_RESP_VALUES_MAX_CHARS/2)
#define SDI_12_TERM "\x0d\x0a"
#define SDI_12_TERM_LEN 2
#define SDI_12_NULL_PARAM '\0'

#define SDI_12_MARKING_MS 9
#define SDI_12_BREAK_NEEDED_TIME_MS 87
#define SDI_12_RESP_START_TIMEOUT_MS (17+9)
#define SDI_12_RESP_RETRY_DELAY_MS (17+10)
#define SDI_12_RESP_END_TIMEOUT_MS 780
#define SDI_12_RETRY_TIMEOUT_MS 100
#define SDI_12_INNER_TRY_MIN 3
#define SDI_12_OUTER_TRY_MIN 3

#define SDI_12_TX_ENABLE_INV

#ifdef SDI_12_TX_ENABLE_INV
#define TX_ENABLE_ON 0
#define TX_ENABLE_OFF 1
#define TX_ENABLE_SETTING GPIO_OUTPUT_HIGH
#else
#define TX_ENABLE_ON 1
#define TX_ENABLE_OFF 0
#define TX_ENABLE_SETTING GPIO_OUTPUT_LOW
#endif


int8_t sdi_12_rx(struct device *uart_dev, uint8_t *buffer, unsigned int len,
				char *terminator, unsigned int timeout);
typedef enum  {
	SDI_12_CMD_ACK_ACTIVE = 0,
	SDI_12_CMD_SERVICE_REQ = 0,
	SDI_12_CMD_SEND_ID,
	SDI_12_CMD_CHNG_ADDR,
	SDI_12_CMD_ADDR_QUERY,
	SDI_12_CMD_START_MEAS,
	SDI_12_CMD_START_MEAS_CRC,
	SDI_12_CMD_SEND_DATA,
	SDI_12_CMD_ADDIT_MEAS,
	SDI_12_CMD_ADDIT_MEAS_CRC,
	SDI_12_CMD_START_VERIF,
	SDI_12_CMD_START_CONCUR_MEAS,
	SDI_12_CMD_START_CONCUR_MEAS_CRC,
	SDI_12_CMD_ADDIT_CONCUR_MEAS,
	SDI_12_CMD_ADDIT_CONCUR_MEAS_CRC,
	SDI_12_CMD_CONT_MEAS,
	SDI_12_CMD_CONT_MEAS_CRC,
	SDI_12_CMD_TYPE_MAX
} SDI_12_CMD_TYPE_e;

typedef enum  {
	SDI_12_NO_PAYLD = 0,
	SDI_12_ID_PAYLD,
	SDI_12_ADDR_PAYLD,
	SDI_12_MEAS_PAYLD,
	SDI_12_VAL_PAYLD,
	SDI_12_FREEFORM_PAYLD,
	SDI_12_PAYLOAD_TYPE_MAX
} SDI_12_PAYLOAD_TYPE_e;

struct sdi_12_sensr_id {
	char ver_no[3];
	char vend_id[9];
	char sens_mod_no[7];
	char sens_ver[4];
	char id_other[14];
};

struct sdi_12_meas_resp {
	uint16_t ready_in_sec;
	uint8_t meas_no;
};

struct sdi_12_value_resp {
	uint8_t len;
	double values[SDI_12_MAX_VALS_IN_RESP];
};

/**
 * @brief Initializes UART device with correct settings as per the SDI-12 spec.
 *
 * @param uart_dev Device handle for the UART interface used as as the data line
 *
 * @return SDI_12_STATUS_OK if succeeded or SDI_12_STATUS_CONFIG_ERROR
 * status otheriwse. 
 */
int8_t sdi_12_init(struct device *uart_dev, struct device *gpio_dev,
			gpio_pin_t tx_enable_pin);

/**
 * @brief Function used to estabilish whether an SDI-12 device is active. 
 *
 * @param uart_dev Device handle for the UART interface used as as the data line
 * @param address address of the device to ping.
 *
 * @return 
 */
int8_t sdi_12_ack_active(struct device *uart_dev, char address);

/**
 * @brief Get SDI-12 devices information
 *
 * @param uart_dev Device handle for the UART interface used as as the data line
 * @param address address of the device to get information from.
 * @param info Pointer to the output struct sdi_12_sensr_id
 *
 * @return 
 */
int8_t sdi_12_get_info(struct device *uart_dev, char address,
				struct sdi_12_sensr_id *info);

/**
 * @brief Get the address of a device on the SDI-12 line. Works correctly only
 * when a single device sensor is used.
 *
 * @param uart_dev Device handle for the UART interface used as as the data line
 * @param address Pointer to a char variable where read address will be returned
 *
 * @return SDI_12_STATUS_OK if succeeded or a negative SDI_12_STATUS_e otheriwse 
 */
int8_t sdi_12_get_address(struct device *uart_dev, char* address);

/**
 * @brief Set the address of a device on the SDI-12 line.
 *
 * @param uart_dev Device handle for the UART interface used as as the data line
 * @param address_old Address of the sensor to be affected
 * @param address_new The new address to be ued for this device
 *
 * @return SDI_12_STATUS_OK if succeeded or a negative SDI_12_STATUS_e otheriwse 
 */
int8_t sdi_12_change_address(struct device *uart_dev, char address_old,
				char address_new);

/**
 * @brief Starts measurement process and delivers its results blocking until
 * they are available.
 *
 * @param uart_dev Device handle for the UART interface used as as the data line
 * @param address Address of the sensor used
 * @param data_out Pointer to an array of length \p len of doubles where
 * measured values will be placed
 * @param len Lentgh of \p data_out
 *
 * @return number of measurements placed in the array or a negative
 * SDI_12_STATUS_e on failure 
 */
int8_t sdi_12_get_measurements(struct device *uart_dev, char address,
				double* data_out, unsigned int len, bool crc);

/**
 * @brief Creates and sends a command, waiting and parsing the responses when
 * appropriate.
 *
 * @param uart_dev Device handle for the UART interface used as as the data line
 * @param cmd_type SDI_12_CMD_TYPE_e of command to be serviced
 * @param address Address of the sensor to be serviced
 * @param param_cmd (optional) Parameters to include in command.
 * @param param_resp (optional) Pointer to a variable where response's outputs
 * should be placed
 *
 * @return SDI_12_STATUS_OK if succeeded or a negative SDI_12_STATUS_e otheriwse 
 */
int8_t sdi_12_cmd_n_resp(struct device *uart_dev, SDI_12_CMD_TYPE_e cmd_type,
			char address, char param_cmd, void* param_resp);

int8_t sdi_12_parse_response(char *resp, char resp_len,
			SDI_12_CMD_TYPE_e cmd_type, char *address, void* data);

int8_t sdi_12_prep_command(char *cmd, char address,
				SDI_12_CMD_TYPE_e cmd_type, char param);

void sdi_12_calc_crc_ascii(char *cmd, uint8_t cmd_len, char* crc);

#endif