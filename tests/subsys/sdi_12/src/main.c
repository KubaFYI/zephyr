/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ztest.h>
#include <math.h>
#include "sdi_12/sdi_12.h"

#define EPSILON 1e-7
#define TEST_ADDRESS_1 7
#define TEST_ADDRESS_2 4


/**
 * @brief Test CRC calculation
 *
 * This test verifies the CRC calculation using an example from SDI-12 Spec v1.4
 * section 4.4.12.3a.
 *
 */
static void test_crc(void)
{
	char crc_str[4];
	char test_cmd[16] = "0+3.14OqZ\x0d\x0a";
	uint8_t crc_in_cmd_idx = 6;
 	sdi_12_calc_crc_ascii(test_cmd, crc_in_cmd_idx, crc_str);
 	zassert_mem_equal(crc_str, test_cmd+crc_in_cmd_idx, 3,
 			"CRC incorrect (got: %s)", crc_str);
}

/**
 * @brief Test command preparation - SDI_12_CMD_ACK_ACTIVE
 *
 * This test verifies the SDI_12_CMD_ACK_ACTIVE command is prepared correctly.
 *
 */
static void test_cmd_ack_active(void)
{
	int8_t retval;
	char cmd_buffer[50] = {'\0'};
	char *cmd_expected = "7!\x0d\x0a";
	retval = sdi_12_prep_command(cmd_buffer, '7',
                	SDI_12_CMD_ACK_ACTIVE, '\0');
	zassert_equal(retval, 0,
		"Prep command function returned an error %d", retval);
 	zassert_mem_equal(cmd_buffer, cmd_expected, strlen(cmd_expected),
 		"Output cmd: %s", cmd_buffer);
}

/**
 * @brief Test command preparation - SDI_12_CMD_SEND_ID
 *
 * This test verifies the SDI_12_CMD_SEND_ID command is prepared correctly.
 *
 */
static void test_cmd_send_id(void)
{
	int8_t retval;
	char cmd_buffer[50] = {'\0'};
	char *cmd_expected = "7I!\x0d\x0a";
	retval = sdi_12_prep_command(cmd_buffer, '7',
                	SDI_12_CMD_SEND_ID, '\0');
	zassert_equal(retval, 0,
		"Prep command function returned an error %d", retval);
 	zassert_mem_equal(cmd_buffer, cmd_expected, strlen(cmd_expected),
 		"Output cmd: %s", cmd_buffer);
}

/**
 * @brief Test command preparation - SDI_12_CMD_CHNG_ADDR
 *
 * This test verifies the SDI_12_CMD_CHNG_ADDR command is prepared correctly.
 *
 */
static void test_cmd_change_address(void)
{
	int8_t retval;
	char cmd_buffer[50] = {'\0'};
	char *cmd_expected = "7A4!\x0d\x0a";
	retval = sdi_12_prep_command(cmd_buffer, '7',
                	SDI_12_CMD_CHNG_ADDR, '4');
	zassert_equal(retval, 0,
		"Prep command function returned an error %d", retval);
 	zassert_mem_equal(cmd_buffer, cmd_expected, strlen(cmd_expected),
 		"Output cmd: %s", cmd_buffer);
}

/**
 * @brief Test command preparation - SDI_12_CMD_ADDR_QUERY
 *
 * This test verifies the SDI_12_CMD_ADDR_QUERY command is prepared correctly.
 *
 */
static void test_cmd_address_query(void)
{
	int8_t retval;
	char cmd_buffer[50] = {'\0'};
	char *cmd_expected = "?!\x0d\x0a";
	retval = sdi_12_prep_command(cmd_buffer, '\0',
                	SDI_12_CMD_ADDR_QUERY, '\0');
	zassert_equal(retval, 0,
		"Prep command function returned an error %d", retval);
 	zassert_mem_equal(cmd_buffer, cmd_expected, strlen(cmd_expected),
 		"Output cmd: %s", cmd_buffer);
}

/**
 * @brief Test command preparation - SDI_12_CMD_START_MEAS
 *
 * This test verifies the SDI_12_CMD_START_MEAS command is prepared correctly.
 *
 */
static void test_cmd_start_measurement(void)
{
	int8_t retval;
	char cmd_buffer[50] = {'\0'};
	char *cmd_expected = "7M!\x0d\x0a";
	retval = sdi_12_prep_command(cmd_buffer, '7',
                	SDI_12_CMD_START_MEAS, '\0');
	zassert_equal(retval, 0,
		"Prep command function returned an error %d", retval);
 	zassert_mem_equal(cmd_buffer, cmd_expected, strlen(cmd_expected),
 		"Output cmd: %s", cmd_buffer);
}

/**
 * @brief Test command preparation - SDI_12_CMD_START_MEAS_CRC
 *
 * This test verifies the SDI_12_CMD_START_MEAS_CRC command is prepared correctly.
 *
 */
static void test_cmd_start_measurement_crc(void)
{
	int8_t retval;
	char cmd_buffer[50] = {'\0'};
	char *cmd_expected = "7MC!\x0d\x0a";
	retval = sdi_12_prep_command(cmd_buffer, '7',
                	SDI_12_CMD_START_MEAS_CRC, '\0');
	zassert_equal(retval, 0,
		"Prep command function returned an error %d", retval);
 	zassert_mem_equal(cmd_buffer, cmd_expected, strlen(cmd_expected),
 		"Output cmd: %s", cmd_buffer);
}

/**
 * @brief Test command preparation - SDI_12_CMD_SEND_DATA
 *
 * This test verifies the SDI_12_CMD_SEND_DATA command is prepared correctly.
 *
 */
static void test_cmd_send_data(void)
{
	int8_t retval;
	char cmd_buffer[50] = {'\0'};
	char *cmd_expected = "7D4!\x0d\x0a";
	retval = sdi_12_prep_command(cmd_buffer, '7',
                	SDI_12_CMD_SEND_DATA, '4');
	zassert_equal(retval, 0,
		"Prep command function returned an error %d", retval);
 	zassert_mem_equal(cmd_buffer, cmd_expected, strlen(cmd_expected),
 		"Output cmd: %s", cmd_buffer);
}

/**
 * @brief Test command preparation - SDI_12_CMD_ADDIT_MEAS
 *
 * This test verifies the SDI_12_CMD_ADDIT_MEAS command is prepared correctly.
 *
 */
static void test_cmd_additional_measurement(void)
{
	int8_t retval;
	char cmd_buffer[50] = {'\0'};
	char *cmd_expected = "7M4!\x0d\x0a";
	retval = sdi_12_prep_command(cmd_buffer, '7',
                	SDI_12_CMD_ADDIT_MEAS, '4');
	zassert_equal(retval, 0,
		"Prep command function returned an error %d", retval);
 	zassert_mem_equal(cmd_buffer, cmd_expected, strlen(cmd_expected),
 		"Output cmd: %s", cmd_buffer);
}

/**
 * @brief Test command preparation - SDI_12_CMD_ADDIT_MEAS_CRC
 *
 * This test verifies the SDI_12_CMD_ADDIT_MEAS_CRC command is prepared correctly.
 *
 */
static void test_cmd_additional_measurement_crc(void)
{
	int8_t retval;
	char cmd_buffer[50] = {'\0'};
	char *cmd_expected = "7MC4!\x0d\x0a";
	retval = sdi_12_prep_command(cmd_buffer, '7',
                	SDI_12_CMD_ADDIT_MEAS_CRC, '4');
	zassert_equal(retval, 0,
		"Prep command function returned an error %d", retval);
 	zassert_mem_equal(cmd_buffer, cmd_expected, strlen(cmd_expected),
 		"Output cmd: %s", cmd_buffer);
}

/**
 * @brief Test command preparation - SDI_12_CMD_START_VERIF
 *
 * This test verifies the SDI_12_CMD_START_VERIF command is prepared correctly.
 *
 */
static void test_cmd_start_verification(void)
{
	int8_t retval;
	char cmd_buffer[50] = {'\0'};
	char *cmd_expected = "7V!\x0d\x0a";
	retval = sdi_12_prep_command(cmd_buffer, '7',
                	SDI_12_CMD_START_VERIF, '\0');
	zassert_equal(retval, 0,
		"Prep command function returned an error %d", retval);
 	zassert_mem_equal(cmd_buffer, cmd_expected, strlen(cmd_expected),
 		"Output cmd: %s", cmd_buffer);
}

/**
 * @brief Test command preparation - SDI_12_CMD_START_CONCUR_MEAS
 *
 * This test verifies the SDI_12_CMD_START_CONCUR_MEAS command is prepared correctly.
 *
 */
static void test_cmd_start_concur_meas(void)
{
	int8_t retval;
	char cmd_buffer[50] = {'\0'};
	char *cmd_expected = "7C!\x0d\x0a";
	retval = sdi_12_prep_command(cmd_buffer, '7',
                	SDI_12_CMD_START_CONCUR_MEAS, '\0');
	zassert_equal(retval, 0,
		"Prep command function returned an error %d", retval);
 	zassert_mem_equal(cmd_buffer, cmd_expected, strlen(cmd_expected),
 		"Output cmd: %s", cmd_buffer);
}

/**
 * @brief Test command preparation - SDI_12_CMD_START_CONCUR_MEAS_CRC
 *
 * This test verifies the SDI_12_CMD_START_CONCUR_MEAS_CRC command is prepared correctly.
 *
 */
static void test_cmd_start_concur_meas_crc(void)
{
	int8_t retval;
	char cmd_buffer[50] = {'\0'};
	char *cmd_expected = "7CC!\x0d\x0a";
	retval = sdi_12_prep_command(cmd_buffer, '7',
                	SDI_12_CMD_START_CONCUR_MEAS_CRC, '\0');
	zassert_equal(retval, 0,
		"Prep command function returned an error %d", retval);
 	zassert_mem_equal(cmd_buffer, cmd_expected, strlen(cmd_expected),
 		"Output cmd: %s", cmd_buffer);
}

/**
 * @brief Test command preparation - SDI_12_CMD_ADDIT_CONCUR_MEAS
 *
 * This test verifies the SDI_12_CMD_ADDIT_CONCUR_MEAS command is prepared correctly.
 *
 */
static void test_cmd_addit_concur_meas(void)
{
	int8_t retval;
	char cmd_buffer[50] = {'\0'};
	char *cmd_expected = "7C4!\x0d\x0a";
	retval = sdi_12_prep_command(cmd_buffer, '7',
                	SDI_12_CMD_ADDIT_CONCUR_MEAS, '4');
	zassert_equal(retval, 0,
		"Prep command function returned an error %d", retval);
 	zassert_mem_equal(cmd_buffer, cmd_expected, strlen(cmd_expected),
 		"Output cmd: %s", cmd_buffer);
}

/**
 * @brief Test command preparation - SDI_12_CMD_ADDIT_CONCUR_MEAS_CRC
 *
 * This test verifies the SDI_12_CMD_ADDIT_CONCUR_MEAS_CRC command is prepared correctly.
 *
 */
static void test_cmd_addit_concur_meas_crc(void)
{
	int8_t retval;
	char cmd_buffer[50] = {'\0'};
	char *cmd_expected = "7CC4!\x0d\x0a";
	retval = sdi_12_prep_command(cmd_buffer, '7',
                	SDI_12_CMD_ADDIT_CONCUR_MEAS_CRC, '4');
	zassert_equal(retval, 0,
		"Prep command function returned an error %d", retval);
 	zassert_mem_equal(cmd_buffer, cmd_expected, strlen(cmd_expected),
 		"Output cmd: %s", cmd_buffer);
}

/**
 * @brief Test command preparation - SDI_12_CMD_CONT_MEAS
 *
 * This test verifies the SDI_12_CMD_CONT_MEAS command is prepared correctly.
 *
 */
static void test_cmd_cont_meas(void)
{
	int8_t retval;
	char cmd_buffer[50] = {'\0'};
	char *cmd_expected = "7R4!\x0d\x0a";
	retval = sdi_12_prep_command(cmd_buffer, '7',
                	SDI_12_CMD_CONT_MEAS, '4');
	zassert_equal(retval, 0,
		"Prep command function returned an error %d", retval);
 	zassert_mem_equal(cmd_buffer, cmd_expected, strlen(cmd_expected),
 		"Output cmd: %s", cmd_buffer);
}

/**
 * @brief Test command preparation - SDI_12_CMD_CONT_MEAS_CRC
 *
 * This test verifies the SDI_12_CMD_CONT_MEAS_CRC command is prepared correctly.
 *
 */
static void test_cmd_cont_meas_crc(void)
{
	int8_t retval;
	char cmd_buffer[50] = {'\0'};
	char *cmd_expected = "7RC4!\x0d\x0a";
	retval = sdi_12_prep_command(cmd_buffer, '7',
                	SDI_12_CMD_CONT_MEAS_CRC, '4');
	zassert_equal(retval, 0,
		"Prep command function returned an error %d", retval);
 	zassert_mem_equal(cmd_buffer, cmd_expected, strlen(cmd_expected),
 		"Output cmd: %s", cmd_buffer);
}

/**
 * @brief Test response parsing - SDI_12_CMD_ACK_ACTIVE
 *
 * This test verifies the response to SDI_12_CMD_ACK_ACTIVE command
 * is parsed correctly.
 *
 */
static void test_resp_ack_active(void)
{
	int retval;
	char *resp_to_parse = "7\x0d\x0a";
	char address = '\xff';
	void* data = NULL;
	retval = sdi_12_parse_response(resp_to_parse, strlen(resp_to_parse), 
				SDI_12_CMD_ACK_ACTIVE,
				&address, data);
	zassert_equal(retval, 0,
		"Parse response function returned an error %d", retval);
	zassert_equal(address, '7', "Parsed address incorrect.");
}

/**
 * @brief Test response parsing - SDI_12_CMD_SEND_ID
 *
 * This test verifies the response to SDI_12_CMD_SEND_ID command
 * is parsed correctly.
 *
 */
static void test_resp_send_id(void)
{
	int retval;
	char *resp_to_parse = "714companyxsensor123optional\x0d\x0a";
	char address = '\xff';
	struct sdi_12_sensr_id data;
	struct sdi_12_sensr_id expctd_data;

	strcpy(expctd_data.ver_no, "14");
	strcpy(expctd_data.vend_id, "companyx");
	strcpy(expctd_data.sens_mod_no, "sensor");
	strcpy(expctd_data.sens_ver, "123");
	strcpy(expctd_data.id_other, "optional");

	retval = sdi_12_parse_response(resp_to_parse, strlen(resp_to_parse), 
				SDI_12_CMD_SEND_ID,
				&address, &data);
	zassert_equal(retval, 0,
		"Parse response function returned an error %d", retval);
	zassert_equal(address, '7', "Parsed address incorrect.");
	zassert_mem_equal(data.ver_no, expctd_data.ver_no,
		sizeof(expctd_data.ver_no), "Parsed data wrong (%s!=%s)",
		data.ver_no, expctd_data.ver_no);
	zassert_mem_equal(data.vend_id, expctd_data.vend_id,
		sizeof(expctd_data.vend_id), "Parsed data wrong (%s!=%s)",
		data.vend_id, expctd_data.vend_id);
	zassert_mem_equal(data.sens_mod_no, expctd_data.sens_mod_no,
		sizeof(expctd_data.sens_mod_no), "Parsed data wrong (%s!=%s)",
		data.sens_mod_no, expctd_data.sens_mod_no);
	zassert_mem_equal(data.sens_ver, expctd_data.sens_ver,
		sizeof(expctd_data.sens_ver), "Parsed data wrong (%s!=%s)",
		data.sens_ver, expctd_data.sens_ver);
	zassert_mem_equal(data.id_other, expctd_data.id_other,
		strlen(expctd_data.id_other)*sizeof(expctd_data.id_other[0]),
		"Parsed data wrong (%s!=%s)",
		data.sens_ver, expctd_data.sens_ver);
}

/**
 * @brief Test response parsing - SDI_12_CMD_CHNG_ADDR
 *
 * This test verifies the response to SDI_12_CMD_CHNG_ADDR command
 * is parsed correctly.
 *
 */
static void test_resp_change_address(void)
{
	int retval;
	char *resp_to_parse = "4\x0d\x0a";
	char address = '\xff';
	retval = sdi_12_parse_response(resp_to_parse, strlen(resp_to_parse), 
				SDI_12_CMD_CHNG_ADDR,
				&address, NULL);
	zassert_equal(retval, 0,
		"Parse response function returned an error %d", retval);
	zassert_equal(address, '4', "Parsed address incorrect.");
}

/**
 * @brief Test response parsing - SDI_12_CMD_ADDR_QUERY
 *
 * This test verifies the response to SDI_12_CMD_ADDR_QUERY command
 * is parsed correctly.
 *
 */
static void test_resp_address_query(void)
{
	int retval;
	char *resp_to_parse = "7\x0d\x0a";
	char address = '\xff';
	void* data = NULL;
	retval = sdi_12_parse_response(resp_to_parse, strlen(resp_to_parse), 
				SDI_12_CMD_ADDR_QUERY,
				&address, data);
	zassert_equal(retval, 0,
		"Parse response function returned an error %d", retval);
	zassert_equal(address, '7', "Parsed address incorrect.");
}

/**
 * @brief Test response parsing - SDI_12_CMD_START_MEAS
 *
 * This test verifies the response to SDI_12_CMD_START_MEAS command
 * is parsed correctly.
 *
 */
static void test_resp_start_measurement(void)
{
	int retval;
	char *resp_to_parse = "73335\x0d\x0a";
	char address = '\xff';
	struct sdi_12_meas_resp data;
	struct sdi_12_meas_resp expctd_data;
	expctd_data.ready_in_sec = 333;
	expctd_data.meas_no = 5;
	retval = sdi_12_parse_response(resp_to_parse, strlen(resp_to_parse), 
				SDI_12_CMD_START_MEAS,
				&address, &data);
	zassert_equal(retval, 0,
		"Parse response function returned an error %d", retval);
	zassert_equal(address, '7', "Parsed address incorrect.");
	zassert_equal(data.ready_in_sec, expctd_data.ready_in_sec,
		"Parsed data incorrect.");
	zassert_equal(data.meas_no, expctd_data.meas_no,
		"Parsed data incorrect.");
}

/*
 * @brief Test response parsing - SDI_12_CMD_START_MEAS_CRC
 *
 * This test verifies the response to SDI_12_CMD_START_MEAS_CRC command
 * is parsed correctly.
 */
 
static void test_resp_start_measurement_crc(void)
{
	int retval;
	char *resp_to_parse = "73335Fc^\x0d\x0a";
	char address = '\xff';
	struct sdi_12_meas_resp data;
	struct sdi_12_meas_resp expctd_data;
	expctd_data.ready_in_sec = 333;
	expctd_data.meas_no = 5;
	retval = sdi_12_parse_response(resp_to_parse, strlen(resp_to_parse), 
				SDI_12_CMD_START_MEAS_CRC,
				&address, &data);
	zassert_equal(retval, 0,
		"Parse response function returned an error %d", retval);
	zassert_equal(address, '7', "Parsed address incorrect.");
	zassert_equal(data.ready_in_sec, expctd_data.ready_in_sec,
		"Parsed data incorrect.");
	zassert_equal(data.meas_no, expctd_data.meas_no,
		"Parsed data incorrect.");
}

/*
 * @brief Test response parsing with incorrect CRC - SDI_12_CMD_START_MEAS_CRC
 *
 * This test verifies the response to SDI_12_CMD_START_MEAS_CRC command fails
 * correctly when attached CRC is not correct.
 */
 
static void test_resp_start_measurement_bad_crc(void)
{
	int retval;
	char *resp_to_parse = "73335foo\x0d\x0a";
	char address = '\xff';
	struct sdi_12_meas_resp data;
	struct sdi_12_meas_resp expctd_data;
	expctd_data.ready_in_sec = 333;
	expctd_data.meas_no = 5;
	retval = sdi_12_parse_response(resp_to_parse, strlen(resp_to_parse), 
				SDI_12_CMD_START_MEAS_CRC,
				&address, &data);
	zassert_equal(retval, SDI_12_STATUS_BAD_CRC,
		"Parse response function returned an error %d", retval);
}

/**
 * @brief Test response parsing - SDI_12_CMD_SEND_DATA
 *
 * This test verifies the response to SDI_12_CMD_SEND_DATA command
 * is parsed correctly.
 *
 */
static void test_resp_send_data(void)
{
	int retval;
	char *resp_to_parse = "7-7.77\x0d\x0a";
	char address = '\xff';
	struct sdi_12_value_resp data;
	struct sdi_12_value_resp expctd_data;
	expctd_data.len = 1;
	expctd_data.values[0] = -7.77;
	retval = sdi_12_parse_response(resp_to_parse, strlen(resp_to_parse), 
				SDI_12_CMD_SEND_DATA,
				&address, &data);
	zassert_equal(retval, 0,
		"Parse response function returned an error %d", retval);
	zassert_equal(address, '7', "Parsed address incorrect.");
	zassert_equal(data.len, expctd_data.len, "Parsed data lenght.");
	zassert_true(fabs(data.values[0]-expctd_data.values[0]) < EPSILON, 
		"Parsed data value incorrect.");
}

/**
 * @brief Test response parsing - SDI_12_CMD_SEND_DATA
 *
 * This test verifies the response to SDI_12_CMD_SEND_DATA command
 * is parsed correctly.
 *
 */
static void test_resp_send_data_multiple(void)
{
	int retval;
	char *resp_to_parse = "7-1.11+2.22-3.33+4.44\x0d\x0a";
	char address = '\xff';
	struct sdi_12_value_resp data;
	struct sdi_12_value_resp expctd_data;
	expctd_data.len = 4;
	expctd_data.values[0] = -1.11;
	expctd_data.values[1] = 2.22;
	expctd_data.values[2] = -3.33;
	expctd_data.values[3] = 4.44;
	retval = sdi_12_parse_response(resp_to_parse, strlen(resp_to_parse), 
				SDI_12_CMD_SEND_DATA,
				&address, &data);
	zassert_equal(retval, 0,
		"Parse response function returned an error %d", retval);
	zassert_equal(address, '7', "Parsed address incorrect.");
	zassert_equal(data.len, expctd_data.len, "Bad parsed values length.");
	for (int i=0; i<4; i++) {
		zassert_true(fabs(data.values[i]-expctd_data.values[i]) < \
									EPSILON,
			"Parsed data value incorrect.");
	}
}

/**
 * @brief Test response parsing - SDI_12_CMD_SEND_DATA
 *
 * This test verifies the response to SDI_12_CMD_SEND_DATA command
 * is parsed correctly, in the case that it follows a CRCd request.
 *
 */

static void test_resp_send_data_multiple_crc(void)
{
	int retval;
	char *resp_to_parse = "7-1.11+2.22-3.33+4.44MpV\x0d\x0a";
	char address = '\xff';
	struct sdi_12_value_resp data;
	struct sdi_12_value_resp expctd_data;
	expctd_data.len = 4;
	expctd_data.values[0] = -1.11;
	expctd_data.values[1] = 2.22;
	expctd_data.values[2] = -3.33;
	expctd_data.values[3] = 4.44;
	retval = sdi_12_parse_response(resp_to_parse, strlen(resp_to_parse), 
				SDI_12_CMD_SEND_DATA,
				&address, &data);
	zassert_equal(retval, 0,
		"Parse response function returned an error %d", retval);
	zassert_equal(address, '7', "Parsed address incorrect.");
	zassert_equal(data.len, expctd_data.len, "Bad parsed values length.");
	for (int i=0; i<4; i++) {
		zassert_true(fabs(data.values[i]-expctd_data.values[i]) < \
									EPSILON,
			"Parsed data incorrect (exp: %f, got %f).",
			expctd_data, data);
	}
}

/*
 * @brief Test response parsing - SDI_12_CMD_ADDIT_MEAS
 *
 * This test verifies the response to SDI_12_CMD_ADDIT_MEAS command
 * is parsed correctly.
 *
 */
static void test_resp_additional_measurement(void)
{
	int retval;
	char *resp_to_parse = "73335\x0d\x0a";
	char address = '\xff';
	struct sdi_12_meas_resp data;
	struct sdi_12_meas_resp expctd_data;
	expctd_data.ready_in_sec = 333;
	expctd_data.meas_no = 5;
	retval = sdi_12_parse_response(resp_to_parse, strlen(resp_to_parse), 
				SDI_12_CMD_START_MEAS,
				&address, &data);
	zassert_equal(retval, 0,
		"Parse response function returned an error %d", retval);
	zassert_equal(address, '7', "Parsed address incorrect.");
	zassert_equal(data.ready_in_sec, expctd_data.ready_in_sec,
		"Parsed data incorrect.");
	zassert_equal(data.meas_no, expctd_data.meas_no,
		"Parsed data incorrect.");
}

/**
 * @brief Test response parsing - SDI_12_CMD_ADDIT_MEAS_CRC
 *
 * This test verifies the response to SDI_12_CMD_ADDIT_MEAS_CRC command
 * is parsed correctly.
 *
 */
static void test_resp_additional_measurement_crc(void)
{
	int retval;
	char *resp_to_parse = "73335Fc^\x0d\x0a";
	char address = '\xff';
	struct sdi_12_meas_resp data;
	struct sdi_12_meas_resp expctd_data;
	expctd_data.ready_in_sec = 333;
	expctd_data.meas_no = 5;
	retval = sdi_12_parse_response(resp_to_parse, strlen(resp_to_parse), 
				SDI_12_CMD_ADDIT_MEAS_CRC,
				&address, &data);
	zassert_equal(retval, 0,
		"Parse response function returned an error %d", retval);
	zassert_equal(address, '7', "Parsed address incorrect.");
	zassert_equal(data.ready_in_sec, expctd_data.ready_in_sec,
		"Parsed data incorrect.");
	zassert_equal(data.meas_no, expctd_data.meas_no,
		"Parsed data incorrect.");
}

/**
 * @brief Test response parsing - SDI_12_CMD_START_VERIF
 *
 * This test verifies the response to SDI_12_CMD_START_VERIF command
 * is parsed correctly.
 *
 */
static void test_resp_start_verification(void)
{
	int retval;
	char *resp_to_parse = "7\x0d\x0a";
	char address = '\xff';
	void* data = NULL;
	retval = sdi_12_parse_response(resp_to_parse, strlen(resp_to_parse), 
				SDI_12_CMD_START_VERIF,
				&address, data);
	zassert_equal(retval, 0,
		"Parse response function returned an error %d", retval);
	zassert_equal(address, '7', "Parsed address incorrect.");
}

/*
 * @brief Test response parsing - SDI_12_CMD_START_CONCUR_MEAS
 *
 * This test verifies the response to SDI_12_CMD_START_CONCUR_MEAS command
 * is parsed correctly.
 */
 
static void test_resp_start_concur_meas(void)
{
	int retval;
	char *resp_to_parse = "733355\x0d\x0a";
	char address = '\xff';
	struct sdi_12_meas_resp data;
	struct sdi_12_meas_resp expctd_data;
	expctd_data.ready_in_sec = 333;
	expctd_data.meas_no = 55;
	retval = sdi_12_parse_response(resp_to_parse, strlen(resp_to_parse), 
				SDI_12_CMD_START_MEAS,
				&address, &data);
	zassert_equal(retval, 0,
		"Parse response function returned an error %d", retval);
	zassert_equal(address, '7', "Parsed address incorrect.");
	zassert_equal(data.ready_in_sec, expctd_data.ready_in_sec,
		"Parsed data incorrect.");
	zassert_equal(data.meas_no, expctd_data.meas_no,
		"Parsed data incorrect.");
}

/**
 * @brief Test response parsing - SDI_12_CMD_START_CONCUR_MEAS_CRC
 *
 * This test verifies the response to SDI_12_CMD_START_CONCUR_MEAS_CRC command
 * is parsed correctly.
 *
 */
static void test_resp_start_concur_meas_crc(void)
{
	int retval;
	char *resp_to_parse = "733355D|h\x0d\x0a";
	char address = '\xff';
	struct sdi_12_meas_resp data;
	struct sdi_12_meas_resp expctd_data;
	expctd_data.ready_in_sec = 333;
	expctd_data.meas_no = 55;
	retval = sdi_12_parse_response(resp_to_parse, strlen(resp_to_parse), 
				SDI_12_CMD_START_CONCUR_MEAS_CRC,
				&address, &data);
	zassert_equal(retval, 0,
		"Parse response function returned an error %d", retval);
	zassert_equal(address, '7', "Parsed address incorrect.");
	zassert_equal(data.ready_in_sec, expctd_data.ready_in_sec,
		"Parsed data incorrect.");
	zassert_equal(data.meas_no, expctd_data.meas_no,
		"Parsed data incorrect.");
}

/**
 * @brief Test response parsing - SDI_12_CMD_ADDIT_CONCUR_MEAS
 *
 * This test verifies the response to SDI_12_CMD_ADDIT_CONCUR_MEAS command
 * is parsed correctly.
 *
 */
static void test_resp_addit_concur_meas(void)
{
	int retval;
	char *resp_to_parse = "733355\x0d\x0a";
	char address = '\xff';
	struct sdi_12_meas_resp data;
	struct sdi_12_meas_resp expctd_data;
	expctd_data.ready_in_sec = 333;
	expctd_data.meas_no = 55;
	retval = sdi_12_parse_response(resp_to_parse, strlen(resp_to_parse), 
				SDI_12_CMD_ADDIT_CONCUR_MEAS,
				&address, &data);
	zassert_equal(retval, 0,
		"Parse response function returned an error %d", retval);
	zassert_equal(address, '7', "Parsed address incorrect.");
	zassert_equal(data.ready_in_sec, expctd_data.ready_in_sec,
		"Parsed data incorrect.");
	zassert_equal(data.meas_no, expctd_data.meas_no,
		"Parsed data incorrect.");
}

/**
 * @brief Test response parsing - SDI_12_CMD_ADDIT_CONCUR_MEAS_CRC
 *
 * This test verifies the response to SDI_12_CMD_ADDIT_CONCUR_MEAS_CRC command
 * is parsed correctly.
 *
 */
static void test_resp_addit_concur_meas_crc(void)
{
	int retval;
	char *resp_to_parse = "733355D|h\x0d\x0a";
	char address = '\xff';
	struct sdi_12_meas_resp data;
	struct sdi_12_meas_resp expctd_data;
	expctd_data.ready_in_sec = 333;
	expctd_data.meas_no = 55;
	retval = sdi_12_parse_response(resp_to_parse, strlen(resp_to_parse), 
				SDI_12_CMD_ADDIT_CONCUR_MEAS_CRC,
				&address, &data);
	zassert_equal(retval, 0,
		"Parse response function returned an error %d", retval);
	zassert_equal(address, '7', "Parsed address incorrect.");
	zassert_equal(data.ready_in_sec, expctd_data.ready_in_sec,
		"Parsed data incorrect.");
	zassert_equal(data.meas_no, expctd_data.meas_no,
		"Parsed data incorrect.");
}

/**
 * @brief Test response parsing - SDI_12_CMD_CONT_MEAS
 *
 * This test verifies the response to SDI_12_CMD_CONT_MEAS command
 * is parsed correctly.
 *
 */
static void test_resp_cont_meas(void)
{
	int retval;
	char *resp_to_parse = "7-1.11+2.22-3.33+4.44\x0d\x0a";
	char address = '\xff';
	struct sdi_12_value_resp data;
	struct sdi_12_value_resp expctd_data;
	expctd_data.len = 4;
	expctd_data.values[0] = -1.11;
	expctd_data.values[1] = 2.22;
	expctd_data.values[2] = -3.33;
	expctd_data.values[3] = 4.44;
	retval = sdi_12_parse_response(resp_to_parse, strlen(resp_to_parse), 
				SDI_12_CMD_CONT_MEAS,
				&address, &data);
	zassert_equal(retval, 0,
		"Parse response function returned an error %d", retval);
	zassert_equal(address, '7', "Parsed address incorrect.");
	zassert_equal(data.len, expctd_data.len, "Bad parsed values length.");
	for (int i=0; i<4; i++) {
		zassert_true(fabs(data.values[i]-expctd_data.values[i]) < \
									EPSILON,
			"Parsed data incorrect (exp: %f, got %f).",
			expctd_data, data);
	}
}

/**
 * @brief Test response parsing - SDI_12_CMD_CONT_MEAS_CRC
 *
 * This test verifies the response to SDI_12_CMD_CONT_MEAS_CRC command
 * is parsed correctly.
 *
 */
static void test_resp_cont_meas_crc(void)
{
	int retval;
	char *resp_to_parse = "7-1.11+2.22-3.33+4.44MpV\x0d\x0a";
	char address = '\xff';
	struct sdi_12_value_resp data;
	struct sdi_12_value_resp expctd_data;
	expctd_data.len = 4;
	expctd_data.values[0] = -1.11;
	expctd_data.values[1] = 2.22;
	expctd_data.values[2] = -3.33;
	expctd_data.values[3] = 4.44;
	retval = sdi_12_parse_response(resp_to_parse, strlen(resp_to_parse), 
				SDI_12_CMD_CONT_MEAS_CRC,
				&address, &data);
	zassert_equal(retval, 0,
		"Parse response function returned an error %d", retval);
	zassert_equal(address, '7', "Parsed address incorrect.");
	zassert_equal(data.len, expctd_data.len, "Bad parsed values length.");
	for (int i=0; i<4; i++) {
		zassert_true(fabs(data.values[i]-expctd_data.values[i]) < \
									EPSILON,
			"Parsed data incorrect (exp: %f, got %f).",
			expctd_data, data);
	}
}

void test_main(void)
{
	ztest_test_suite(framework_tests,
		ztest_unit_test(test_crc),
		ztest_unit_test(test_cmd_ack_active),
		ztest_unit_test(test_cmd_send_id),
		ztest_unit_test(test_cmd_change_address),
		ztest_unit_test(test_cmd_address_query),
		ztest_unit_test(test_cmd_start_measurement),
		ztest_unit_test(test_cmd_start_measurement_crc),
		ztest_unit_test(test_cmd_send_data),
		ztest_unit_test(test_cmd_additional_measurement),
		ztest_unit_test(test_cmd_additional_measurement_crc),
		ztest_unit_test(test_cmd_start_verification),
		ztest_unit_test(test_cmd_start_concur_meas),
		ztest_unit_test(test_cmd_start_concur_meas_crc),
		ztest_unit_test(test_cmd_addit_concur_meas),
		ztest_unit_test(test_cmd_addit_concur_meas_crc),
		ztest_unit_test(test_cmd_cont_meas),
		ztest_unit_test(test_cmd_cont_meas_crc),
		ztest_unit_test(test_resp_ack_active),
		ztest_unit_test(test_resp_send_id),
		ztest_unit_test(test_resp_change_address),
		ztest_unit_test(test_resp_address_query),
		ztest_unit_test(test_resp_start_measurement),
		ztest_unit_test(test_resp_start_measurement_crc),
		ztest_unit_test(test_resp_start_measurement_bad_crc),
		ztest_unit_test(test_resp_send_data),
		ztest_unit_test(test_resp_send_data_multiple),
		ztest_unit_test(test_resp_send_data_multiple_crc),
		ztest_unit_test(test_resp_additional_measurement),
		ztest_unit_test(test_resp_additional_measurement_crc),
		ztest_unit_test(test_resp_start_verification),
		ztest_unit_test(test_resp_start_concur_meas),
		ztest_unit_test(test_resp_start_concur_meas_crc),
		ztest_unit_test(test_resp_addit_concur_meas),
		ztest_unit_test(test_resp_addit_concur_meas_crc),
		ztest_unit_test(test_resp_cont_meas),
		ztest_unit_test(test_resp_cont_meas_crc)
	);

	ztest_run_test_suite(framework_tests);
}
