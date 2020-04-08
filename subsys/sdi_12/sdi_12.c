/*
 * Copyright (c) 2019 R3 IoT Ltd.
 *
 * License: Apache-2.0
*/

#include <sdi_12/sdi_12.h>
#include <sdi_12/sdi_12_uart.h>

#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <device.h>
#include <errno.h>
#include <init.h>
#include <sys/__assert.h>
#include <soc.h>
#include <drivers/uart.h>
#include <drivers/dma.h>

#define BUFFER_LEN 100

#define LOG_LEVEL CONFIG_SDI_12_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(sdi_12);

static char sdi_12_cmd_char[SDI_12_CMD_TYPE_MAX] = {
		SDI_12_NULL_PARAM,	// SDI_12_CMD_ACK_ACTIVE
		'I',			// SDI_12_CMD_SEND_ID
		'A',			// SDI_12_CMD_CHNG_ADDR
		SDI_12_NULL_PARAM,	// SDI_12_CMD_ADDR_QUERY
		'M',			// SDI_12_CMD_START_MEAS
		'M',			// SDI_12_CMD_START_MEAS_CRC
		'D',			// SDI_12_CMD_SEND_DATA
		'M',			// SDI_12_CMD_ADDIT_MEAS
		'M',			// SDI_12_CMD_ADDIT_MEAS_CRC
		'V',			// SDI_12_CMD_START_VERIF
		'C',			// SDI_12_CMD_START_CONCUR_MEAS
		'C',			// SDI_12_CMD_START_CONCUR_MEAS_CRC
		'C',			// SDI_12_CMD_ADDIT_CONCUR_MEAS
		'C',			// SDI_12_CMD_ADDIT_CONCUR_MEAS_CRC
		'R',			// SDI_12_CMD_CONT_MEAS
		'R'			// SDI_12_CMD_CONT_MEAS_CRC
};		

static bool sdi_12_cmd_crc[SDI_12_CMD_TYPE_MAX] = \
		{false,		// SDI_12_CMD_ACK_ACTIVE
		false,		// SDI_12_CMD_SEND_ID
		false,		// SDI_12_CMD_CHNG_ADDR
		false,		// SDI_12_CMD_ADDR_QUERY
		false,		// SDI_12_CMD_START_MEAS
		true,		// SDI_12_CMD_START_MEAS_CRC
		false,		// SDI_12_CMD_SEND_DATA
		false,		// SDI_12_CMD_ADDIT_MEAS
		true,		// SDI_12_CMD_ADDIT_MEAS_CRC
		false,		// SDI_12_CMD_START_VERIF
		false,		// SDI_12_CMD_START_CONCUR_MEAS
		true,		// SDI_12_CMD_START_CONCUR_MEAS_CRC
		false,		// SDI_12_CMD_ADDIT_CONCUR_MEAS
		true,		// SDI_12_CMD_ADDIT_CONCUR_MEAS_CRC
		false,		// SDI_12_CMD_CONT_MEAS
		true		// SDI_12_CMD_CONT_MEAS_CRC
};		
static bool sdi_12_cmd_param[SDI_12_CMD_TYPE_MAX] = \
		{false,		// SDI_12_CMD_ACK_ACTIVE
		false,		// SDI_12_CMD_SEND_ID
		true,		// SDI_12_CMD_CHNG_ADDR
		false,		// SDI_12_CMD_ADDR_QUERY
		false,		// SDI_12_CMD_START_MEAS
		false,		// SDI_12_CMD_START_MEAS_CRC
		true,		// SDI_12_CMD_SEND_DATA
		true,		// SDI_12_CMD_ADDIT_MEAS
		true,		// SDI_12_CMD_ADDIT_MEAS_CRC
		false,		// SDI_12_CMD_START_VERIF
		false,		// SDI_12_CMD_START_CONCUR_MEAS
		false,		// SDI_12_CMD_START_CONCUR_MEAS_CRC
		true,		// SDI_12_CMD_ADDIT_CONCUR_MEAS
		true,		// SDI_12_CMD_ADDIT_CONCUR_MEAS_CRC
		true,		// SDI_12_CMD_CONT_MEAS
		true		// SDI_12_CMD_CONT_MEAS_CRC
};	

static SDI_12_PAYLOAD_TYPE_e sdi_12_resp_param[SDI_12_CMD_TYPE_MAX] = \
		{SDI_12_NO_PAYLD,	// SDI_12_CMD_ACK_ACTIVE and SDI_12_CMD_SERVICE_REQ
		SDI_12_ID_PAYLD,	// SDI_12_CMD_SEND_ID
		SDI_12_NO_PAYLD,	// SDI_12_CMD_CHNG_ADDR
		SDI_12_NO_PAYLD,	// SDI_12_CMD_ADDR_QUERY
		SDI_12_MEAS_PAYLD,	// SDI_12_CMD_START_MEAS
		SDI_12_MEAS_PAYLD,	// SDI_12_CMD_START_MEAS_CRC
		SDI_12_VAL_PAYLD,	// SDI_12_CMD_SEND_DATA
		SDI_12_MEAS_PAYLD,	// SDI_12_CMD_ADDIT_MEAS
		SDI_12_MEAS_PAYLD,	// SDI_12_CMD_ADDIT_MEAS_CRC
		SDI_12_FREEFORM_PAYLD,	// SDI_12_CMD_START_VERIF
		SDI_12_MEAS_PAYLD,	// SDI_12_CMD_START_CONCUR_MEAS
		SDI_12_MEAS_PAYLD,	// SDI_12_CMD_START_CONCUR_MEAS_CRC
		SDI_12_MEAS_PAYLD,	// SDI_12_CMD_ADDIT_CONCUR_MEAS
		SDI_12_MEAS_PAYLD,	// SDI_12_CMD_ADDIT_CONCUR_MEAS_CRC
		SDI_12_VAL_PAYLD,	// SDI_12_CMD_CONT_MEAS
		SDI_12_VAL_PAYLD	// SDI_12_CMD_CONT_MEAS_CRC
	};

int8_t sdi_12_cmd_n_resp(struct device *uart_dev, SDI_12_CMD_TYPE_e cmd_type,
            char address, char param_cmd, void* param_resp);

int8_t sdi_12_parse_response(char *resp, char resp_len,
			SDI_12_CMD_TYPE_e cmd_type, char *address, void* data);

int8_t sdi_12_prep_command(char *cmd, char address,
				SDI_12_CMD_TYPE_e cmd_type, char param);

void sdi_12_calc_crc_ascii(char *cmd, uint8_t cmd_len, char *crc);

int8_t sdi_12_init(struct device *uart_dev)
{
	return sdi_12_uart_init(uart_dev);

}

int8_t sdi_12_ack_active(struct device *uart_dev, char address)
{
	int ret;
	ret = sdi_12_cmd_n_resp(uart_dev, SDI_12_CMD_ACK_ACTIVE, address,
			SDI_12_NULL_PARAM, NULL);
	if ( ret != SDI_12_STATUS_OK ) {
		LOG_ERR("Ping sensor error");
		return ret;
	}

	LOG_DBG("Sensor %c active", address);
	return SDI_12_STATUS_OK;
}

int8_t sdi_12_get_info(struct device *uart_dev, char address,
				struct sdi_12_sensr_id *info)
{
	int ret;
	ret = sdi_12_cmd_n_resp(uart_dev, SDI_12_CMD_SEND_ID, address,
			SDI_12_NULL_PARAM, (void*)info);
	if ( ret != SDI_12_STATUS_OK ) {
		LOG_ERR("Information retrieve error");
		return ret;
	}

	LOG_DBG("Sensor %c info:", address);
	LOG_DBG("ver_no: %s", log_strdup(info->ver_no));
	LOG_DBG("vend_id: %s", log_strdup(info->vend_id));
	LOG_DBG("sens_mod_no: %s", log_strdup(info->sens_mod_no));
	LOG_DBG("sens_ver: %s", log_strdup(info->sens_ver));
	LOG_DBG("id_other: %s", log_strdup(info->id_other));
	return SDI_12_STATUS_OK;
}

int8_t sdi_12_get_address(struct device *uart_dev, char* address)
{
	int ret;
	ret = sdi_12_cmd_n_resp(uart_dev, SDI_12_CMD_ADDR_QUERY,
				SDI_12_NULL_PARAM, SDI_12_NULL_PARAM, address);
	if ( ret != SDI_12_STATUS_OK ) {
		LOG_ERR("Error querying address");
		return ret;
	}

	LOG_DBG("Queried address: %c", *address);
	return SDI_12_STATUS_OK;
}

int8_t sdi_12_change_address(struct device *uart_dev, char address_old,
			  char address_new)
{
	int ret;
	char ret_address;
	ret = sdi_12_cmd_n_resp(uart_dev, SDI_12_CMD_CHNG_ADDR, address_old,
			address_new, &ret_address);
	if ( ret != SDI_12_STATUS_OK ) {
		LOG_ERR("Error changing address");
		return ret;
	} else if (ret_address != address_new) {
		LOG_ERR("Address changing error");
		return ret;
	}

	LOG_DBG("Changed sensor address %c to %c", address_old, address_new);
	return SDI_12_STATUS_OK;
}

int8_t sdi_12_get_measurements(struct device *uart_dev, char address,
				double* data_out, unsigned int len, bool crc)
{	
	int ret;
	int idx;
	int data_read;
	int data_portion;
	struct sdi_12_meas_resp measurement_info;
	struct sdi_12_value_resp resp_values;;
	SDI_12_CMD_TYPE_e meas_cmd;

#if LOG_LEVEL >= LOG_LEVEL_DBG
	char log_buffer[50] = {'\0'};
#else
	char *log_buffer;
#endif

	if (!crc) {
		meas_cmd = SDI_12_CMD_START_MEAS;
	} else {
		meas_cmd = SDI_12_CMD_START_MEAS_CRC;
	}

	ret = sdi_12_cmd_n_resp(uart_dev, meas_cmd, address,
				SDI_12_NULL_PARAM, &measurement_info);
	if ( ret != SDI_12_STATUS_OK ) {
		LOG_ERR("Requesting measurement failed");
		return ret;
	}

	if ( measurement_info.meas_no > len ) {
		LOG_ERR("More measurements than able to return");
		return SDI_12_STATUS_BUFFER_FULL;
	}

	LOG_DBG("Waiting %ds for %d measurements",
			measurement_info.ready_in_sec,
			measurement_info.meas_no);
	k_sleep(measurement_info.ready_in_sec * 1000);


	data_read = 0;
	data_portion = 0;
	while (data_read < measurement_info.meas_no) {
		ret = sdi_12_cmd_n_resp(uart_dev, SDI_12_CMD_SEND_DATA, address,
					data_portion+'0', &resp_values);
		if ( ret != SDI_12_STATUS_OK ) {
			LOG_ERR("Retrieving measurements failed");
			return ret;
		}
#if LOG_LEVEL >= LOG_LEVEL_DBG
		log_buffer[0] = '\0';
#endif		
		for (idx = 0; idx < resp_values.len; idx++) {
			if ( data_read+1 > len ) {
				LOG_ERR("Too many measurements returned");
				return SDI_12_STATUS_BUFFER_FULL;
			}
			data_out[data_read++] = resp_values.values[idx];
#if LOG_LEVEL >= LOG_LEVEL_DBG
				sprintf(log_buffer+strlen(log_buffer),
					"%d.%-2d ",
					(int)resp_values.values[idx],
					(int)fabs((resp_values.values[idx] - \
					(int)(resp_values.values[idx]))*100));
#endif
		}
		data_portion++;
		LOG_DBG("Got measurements: %s", log_strdup(log_buffer));
	}

	return SDI_12_STATUS_OK;
}

int8_t sdi_12_cmd_n_resp(struct device *uart_dev, SDI_12_CMD_TYPE_e cmd_type,
            char address, char param_cmd, void* param_resp)
{
	int ret;
	char resp_address;
	char buffer[BUFFER_LEN] = {'\0'};

	ret = sdi_12_prep_command(buffer, address, cmd_type, param_cmd);
	if ( ret != SDI_12_STATUS_OK ) {
		LOG_DBG("Error preparing command");
		return ret;
	}

	ret = sdi_12_uart_tx(uart_dev, buffer, strlen(buffer));
	if ( ret != SDI_12_STATUS_OK ) {
		LOG_DBG("TX error");
		return ret;
	}

	memset(buffer, '\0', BUFFER_LEN);

	ret = sdi_12_uart_rx(uart_dev, buffer, BUFFER_LEN, SDI_12_TERM_STR,
				SDI_12_RESP_TIMEOUT);
	if ( ret != SDI_12_STATUS_OK ) {
		LOG_DBG("Response RX error: %s", SDI_12_ERR_TO_STR(ret));
		return ret;
	}

	ret = sdi_12_parse_response(buffer, strlen(buffer),
				cmd_type, &resp_address, param_resp);

	if (cmd_type == SDI_12_CMD_CHNG_ADDR ||
		cmd_type == SDI_12_CMD_ADDR_QUERY) {
		*(char*)(param_resp) = resp_address;
	}

	if ( ret != SDI_12_STATUS_ADDR_INVALID &&
		cmd_type != SDI_12_CMD_ADDR_QUERY &&
		((cmd_type != SDI_12_CMD_CHNG_ADDR &&
			resp_address != address) ||
		(cmd_type == SDI_12_CMD_CHNG_ADDR &&
			resp_address != param_cmd))) {
		LOG_DBG("Cmd-resp address mismatch");
		return SDI_12_STATUS_ADDR_MISMATCH;
	}
	else if ( ret != SDI_12_STATUS_OK ) {
		LOG_DBG("Error parsing response");
		return ret;
	}


	return SDI_12_STATUS_OK;
}

int8_t sdi_12_parse_response(char *resp, char resp_len, 
				SDI_12_CMD_TYPE_e cmd_type,
				char *address, void* data)
{
	int idx;
	int val_strt_idx;
	int resp_idx = 0;
	int pld_end_idx = 0;
	char buff[10];
	char crc_str[4];
	char *tmp_char_ptr;

	struct sdi_12_sensr_id *data_sens_id_s = data;
	struct sdi_12_meas_resp *data_meas_s = data;
	struct sdi_12_value_resp *data_elems_s = data;

	if (isalnum((int)resp[resp_idx]) == 0 &&
		!(cmd_type == SDI_12_CMD_ADDR_QUERY &&
		resp[resp_idx] == '?')) {
		LOG_WRN("Invalid response address");
		return SDI_12_STATUS_ADDR_INVALID;
	}
	*address = resp[resp_idx++];

	for ( idx = resp_idx; idx < resp_len-1; idx++) {
		if (resp[idx] == '\x0d' && 
			resp[idx+1] == '\x0a') {
			pld_end_idx = idx;
		}
	}
	if (pld_end_idx == 0) {
		LOG_WRN("Malformed response (no termination)");
		return SDI_12_STATUS_ERROR;
	}

	if( sdi_12_cmd_crc[cmd_type] ) {
		pld_end_idx -= 3;
		if ( pld_end_idx < 1 ) {
			LOG_WRN("Malformed response (missing CRC)");
			return SDI_12_STATUS_ERROR;
		}
	}

	switch (sdi_12_resp_param[cmd_type]) {
	case SDI_12_NO_PAYLD:
		if (resp_idx != pld_end_idx) {
			LOG_WRN("Unexpected payload %d %d",
					resp_idx, pld_end_idx);
			return SDI_12_STATUS_ERROR;
		}
		break;
	case SDI_12_ID_PAYLD:
		if ( (pld_end_idx-resp_idx) < 19 ||
		     (pld_end_idx-resp_idx) > 32 ) {
			LOG_WRN("Unexpected payload length");
			return SDI_12_STATUS_ERROR;
		}
		memcpy(data_sens_id_s->ver_no, resp+resp_idx,
			sizeof(data_sens_id_s->ver_no)-1);
		resp_idx += sizeof(data_sens_id_s->ver_no) / 
			    sizeof(data_sens_id_s->ver_no[0]);
		resp_idx--;
		data_sens_id_s->ver_no[ \
			sizeof(data_sens_id_s->ver_no)-1] = '\0';

		memcpy(data_sens_id_s->vend_id, resp+resp_idx,
			sizeof(data_sens_id_s->vend_id)-1);
		resp_idx += sizeof(data_sens_id_s->vend_id) / 
			    sizeof(data_sens_id_s->vend_id[0]);
		resp_idx--;
		data_sens_id_s->vend_id[ \
			sizeof(data_sens_id_s->vend_id)-1] = '\0';

		memcpy(data_sens_id_s->sens_mod_no, resp+resp_idx,
			sizeof(data_sens_id_s->sens_mod_no)-1);
		resp_idx += sizeof(data_sens_id_s->sens_mod_no) / 
			    sizeof(data_sens_id_s->sens_mod_no[0]);
		resp_idx--;
		data_sens_id_s->sens_mod_no[ \
			sizeof(data_sens_id_s->sens_mod_no)-1] = '\0';

		memcpy(data_sens_id_s->sens_ver, resp+resp_idx,
			sizeof(data_sens_id_s->sens_ver)-1);
		resp_idx += sizeof(data_sens_id_s->sens_ver) / 
			    sizeof(data_sens_id_s->sens_ver[0]);
		resp_idx--;
		data_sens_id_s->sens_ver[ \
			sizeof(data_sens_id_s->sens_ver)-1] = '\0';
		memset(data_sens_id_s->id_other, '\0', 
			sizeof(data_sens_id_s->id_other));
		memcpy(data_sens_id_s->id_other, resp+resp_idx,
			sizeof(resp[resp_idx]) * (pld_end_idx-resp_idx));
		data_sens_id_s->id_other[(pld_end_idx-resp_idx)] = '\0';
		resp_idx += (pld_end_idx-resp_idx)/sizeof(resp[resp_idx]);

		break;
	case SDI_12_ADDR_PAYLD:
		if ( pld_end_idx-resp_idx != 1 ||
			isalnum((int)*address) != 0 ) {
			LOG_WRN("Unexpected payload");
			return SDI_12_STATUS_ERROR;
		}
		*( (char*)data ) = resp[resp_idx];
		break;
	case SDI_12_MEAS_PAYLD:
		if ( (pld_end_idx-resp_idx) < 4 ||
		     (pld_end_idx-resp_idx) > 5 ) {
			LOG_WRN("Unexpected payload lenght");
			return SDI_12_STATUS_ERROR;
		}

		for (idx=resp_idx; idx<pld_end_idx; idx++) {
			if ( isdigit(resp[idx]) == 0) {
				LOG_WRN("Unexpected payload format");
				return SDI_12_STATUS_ERROR;
			}
		}

		buff[3] = '\0';
		memcpy( buff, resp+resp_idx, 3 );
		data_meas_s->ready_in_sec = atoi(buff);

		buff[1] = '\0';
		buff[2] = '\0';
		memcpy( buff, resp+resp_idx+3, (pld_end_idx-resp_idx-3) );
		data_meas_s->meas_no = atoi(buff);

		break;
	case SDI_12_VAL_PAYLD:
		if ( (pld_end_idx-resp_idx) < 2 ||
		     (pld_end_idx-resp_idx) > 75 ) {
			LOG_WRN("Unexpected payload lenght");
			return SDI_12_STATUS_ERROR;
		}

		val_strt_idx = -1;

		if ( resp[resp_idx] != '+' && resp[resp_idx] != '-' ) {
			LOG_WRN("Unexpected payload format (missing polarity)");
			return SDI_12_STATUS_ERROR;
		}

		data_elems_s->len = 0;

		tmp_char_ptr = &(resp[resp_idx]);


		while ( *tmp_char_ptr == '+' || *tmp_char_ptr == '-' ) {
			data_elems_s->values[data_elems_s->len] = \
					strtod( tmp_char_ptr, &tmp_char_ptr );
			data_elems_s->len++;
		}


		if ( tmp_char_ptr == &(resp[pld_end_idx])) {

			break;

		} else if ( pld_end_idx -
			    (tmp_char_ptr-resp)/sizeof(resp[0]) == 3) {

			/* Assume a CRC at the end of a Send Data response but
			we had no way of deducing that just from a Send Data 
			command. */
			sdi_12_calc_crc_ascii(resp, pld_end_idx-3, crc_str);
			if ( memcmp( &(resp[pld_end_idx-3]), crc_str, 3) != 0 ){
				LOG_WRN("Incorrect CRC");
				return SDI_12_STATUS_BAD_CRC;
			}

		} else {

			LOG_WRN("Unexpected payload format");
			return SDI_12_STATUS_ERROR;

		}

		break;
	case SDI_12_FREEFORM_PAYLD:
		/* This payload type is used for when no particular response
		format is specified. Just return succesfully. */
		return 0;
	default:
		LOG_ERR("Unexpected command type");
		return SDI_12_STATUS_ERROR;
		break;
	}
	if( sdi_12_cmd_crc[cmd_type] ) {
		if ( pld_end_idx < 1 ) {
			LOG_ERR("Malformed response (missing CRC)");
			return SDI_12_STATUS_ERROR;
		}
		sdi_12_calc_crc_ascii(resp, pld_end_idx, crc_str);
		if ( memcmp( &(resp[pld_end_idx]), crc_str, 3) != 0 ){
			LOG_ERR("Incorrect CRC");
			return SDI_12_STATUS_BAD_CRC;
		}
	}

	return SDI_12_STATUS_OK;
}

int8_t sdi_12_prep_command(char *cmd, char address,
				SDI_12_CMD_TYPE_e cmd_type, char param)
{
	int cmd_idx = 0;

	if (cmd_type == SDI_12_CMD_ADDR_QUERY) {
		cmd[cmd_idx++] = '?';
	} else if (isalnum((int)address) != 0) {
		cmd[cmd_idx++] = address;
	} else {
		LOG_ERR("Invalid address '%c'", address);
		return SDI_12_STATUS_ERROR;
	}

	if (sdi_12_cmd_char[cmd_type] != SDI_12_NULL_PARAM) {
		cmd[cmd_idx++] = sdi_12_cmd_char[cmd_type];
	}

	if (sdi_12_cmd_crc[cmd_type]) {
		cmd[cmd_idx++] = 'C';		
	}

	if (cmd_type == SDI_12_CMD_CHNG_ADDR) {
		if (isalnum((int)param) == 0) {
			LOG_ERR("Invalid address param '%c'", (char)param);
			return SDI_12_STATUS_ERROR;
		}
		cmd[cmd_idx++] = param;
	} else if (sdi_12_cmd_param[cmd_type]) {
		if (isdigit(param) == 0) {
			LOG_ERR("Invalid parameter (not a digit)");
			return SDI_12_STATUS_ERROR;
		}
		cmd[cmd_idx++] = param;
	}

	cmd[cmd_idx++] = '!';
	cmd[cmd_idx++] = '\x0d';
	cmd[cmd_idx++] = '\x0a';
	cmd[cmd_idx] = '\0';

	return SDI_12_STATUS_OK;
}

void sdi_12_calc_crc_ascii(char *cmd, uint8_t cmd_len, char* crc_str)
{
	uint16_t crc_num = 0;
	int idx, jdx;

	for ( idx=0; idx<cmd_len; idx++ ){
		crc_num ^= cmd[idx];
		for ( jdx=0; jdx<8; jdx++ ){
			if ( (crc_num & 0x0001) == 0x0001){
				crc_num >>= 1;
				crc_num ^= 0xA001;
			} else {
				crc_num >>= 1;
			}
		}
	}

	crc_str[0] = 0x40 | (crc_num >> 12);
	crc_str[1] = 0x40 | ((crc_num >> 6) & 0x3F);
	crc_str[2] = 0x40 | (crc_num & 0x3F);
	crc_str[3] = '\0';
}
