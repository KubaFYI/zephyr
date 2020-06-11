/*
 * Copyright (c) 2019 R3 IoT Ltd.
 *
 * License: Apache-2.0
 *
 * An app used to test the SDI-12 module functionality with a connected mock
 * slave device.
 *
 * The mock slave device is designed to change its behaviour based on the
 * address assigned to it. With that several different behaviours of the device
 * can be tested:
 *
 *	- ADDR	-	BEHAVIOUR
 *	- '0'	- Zero measurement period, no service request, delivering 3
 *		  values (1.11, -2.22, 3.33)
 *	- '1'	- As '0' except 10s measurement period
 *	- '2'	- As '0' except delivering 9 values (1.11, -2.22, ..., 9.99)
 *		  split over three send data requests
 *	- '3'	- As '1' except, the sensor issues a service request after 5
 *		  seconds
 *	- '4'	- As '0' except the returned CRCs are incorrect
 *	- '5' 	- As '0' except every second response is absent (for testing
 * 		  retries)
 *	- '6'	- No response to measurements.
 *
 * The mock devices reported stats are:
 * protocol version number: 11
 * vendor id: ZEPHYRIO
 * sensor model number: 000001
 * sensor version: 1.0
 * other information: OtherInfo
 *
*/
#include <zephyr.h>
#include <math.h>
#include <ctype.h>
#include <drivers/gpio.h>
#include <drivers/pinmux.h>
#include <power/reboot.h>
#include <settings/settings.h>
#include <sdi_12/sdi_12.h>

#define LOG_LEVEL CONFIG_SDI_12_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(app);

#define EPSILON 1e-7

#define UART_INTERFACE DT_INST_1_ATMEL_SAM0_UART_LABEL

static int8_t test_address_query(struct device *dev);
static int8_t test_address_change(struct device *dev);
static int8_t test_id(struct device *dev);
static int8_t test_reading_simple(struct device *dev);
static int8_t test_reading_simple_retries(struct device *dev);
static int8_t test_reading_simple_no_resp(struct device *dev);
static int8_t test_reading_simple_crc(struct device *dev);
static int8_t test_reading_simple_10s(struct device *dev);
static int8_t test_reading_more_values(struct device *dev);
static int8_t test_reading_with_service_req(struct device *dev);
static int8_t test_incorrect_crc(struct device *dev);

static int8_t (*test_functions[]) (struct device *dev) = {
test_address_query,
test_address_change,
test_id,
test_reading_simple,
test_reading_simple_retries,
test_reading_simple_no_resp,
test_reading_simple_crc,
test_reading_simple_10s,
test_reading_more_values,
test_reading_with_service_req,
test_incorrect_crc
};

static char *test_functions_str[] = {
"test_address_query",
"test_address_change",
"test_id",
"test_reading_simple",
"test_reading_simple_retries",
"test_reading_simple_no_resp",
"test_reading_simple_crc",
"test_reading_simple_10s",
"test_reading_more_values",
"test_reading_with_service_req",
"test_incorrect_crc"
};

int test_function_no = sizeof(test_functions) / sizeof(test_functions[0]);

static int8_t prep_addr(struct device *dev, char addr)
{
	char old_address;
	int ret;
	LOG_DBG("Prep_addr...");
	ret = sdi_12_get_address(dev, &old_address);

	if ( ret != 0 ) {
		return ret;
	}

	if (old_address != addr) {
		ret = sdi_12_change_address(dev, old_address, addr);

		if ( ret != 0 ) {
			return ret;
		}
	}

	LOG_DBG("Prep_addr success");
	return 0;
}

static int8_t test_address_query(struct device *dev)
{
	int ret;
	char address;

	ret = sdi_12_get_address(dev, &address);

	if ( ret != 0 ) {
		return ret;
	}

	if (isalnum(address) == 0) {
		return -1;
	}

	return 0;
}

static int8_t test_address_change(struct device *dev)
{
	int ret;
	char address;
	char address_orig = '0';
	char address_test = 'x';

	ret = prep_addr(dev, address_orig);
	if ( ret != 0 ) {
		return ret;
	}

	ret = sdi_12_change_address(dev, address_orig, address_test);

	if ( ret != 0 ) {
		return ret;
	}

	ret = sdi_12_get_address(dev, &address);

	if ( ret != 0 ) {
		return ret;
	}

	if (address != address_test) {
		return -1;
	}

	ret = sdi_12_change_address(dev, address_test, address_orig);

	if ( ret != 0 ) {
		return ret;
	}

	return 0;
}

static int8_t test_id(struct device *dev)
{
	int ret;
	struct sdi_12_sensr_id data;
	struct sdi_12_sensr_id expected_data;

	strcpy(expected_data.ver_no, "11");
	strcpy(expected_data.vend_id, "ZEPHYRIO");
	strcpy(expected_data.sens_mod_no, "000001");
	strcpy(expected_data.sens_ver, "1.0");
	strcpy(expected_data.id_other, "OtherInfo");

	ret = sdi_12_get_info(dev, '0', &data);

	if ( ret != 0 ) {
		return ret;
	}

	if (strcmp(data.ver_no, expected_data.ver_no) != 0 ||
	    strcmp(data.vend_id, expected_data.vend_id) != 0 ||
	    strcmp(data.sens_mod_no, expected_data.sens_mod_no) != 0 ||
	    strcmp(data.sens_ver, expected_data.sens_ver) != 0 ||
	    strcmp(data.id_other, expected_data.id_other) != 0) {
	    	return -1;
	}

	return 0;
}

static int8_t test_reading_simple(struct device *dev)
{
	int ret;
	int i;
	double expected_data[] = {1.11, -2.22, 3.33};
	double data[3];
	char addr_to_use = '0';

	ret = prep_addr(dev, addr_to_use);
	if ( ret != 0 ) {
		return ret;
	}

	ret = sdi_12_get_measurements(dev, addr_to_use, data, 3, false);
	if ( ret != 0 ) {
		return ret;
	}

	for (i=0; i<3; i++) {
		if (fabs(expected_data[i] - data[i]) > EPSILON) {
			LOG_ERR("Wrong data received");
			return -1;
		}
	}
	return 0;
}

static int8_t test_reading_simple_retries(struct device *dev)
{
	int ret;
	int i;
	double expected_data[] = {1.11, -2.22, 3.33};
	double data[3];
	char addr_to_use = '5';

	ret = prep_addr(dev, addr_to_use);
	if ( ret != 0 ) {
		return ret;
	}

	ret = sdi_12_get_measurements(dev, addr_to_use, data, 3, false);
	if ( ret != 0 ) {
		return ret;
	}

	for (i=0; i<3; i++) {
		if (fabs(expected_data[i] - data[i]) > EPSILON) {
			LOG_ERR("Wrong data received");
			return -1;
		}
	}

	ret = prep_addr(dev, '0');
	if ( ret != 0 ) {
		return ret;
	}

	return 0;
}

static int8_t test_reading_simple_no_resp(struct device *dev)
{
	int ret;
	double data[3];
	char addr_to_use = '6';

	ret = prep_addr(dev, addr_to_use);
	if ( ret != 0 ) {
		return ret;
	}

	ret = sdi_12_get_measurements(dev, addr_to_use, data, 3, false);
	if ( ret == 0 ) {
		return -1;
	}

	ret = prep_addr(dev, '0');
	if ( ret != 0 ) {
		return ret;
	}

	return 0;
}

static int8_t test_reading_simple_crc(struct device *dev)
{
	int ret;
	int i;
	double expected_data[] = {1.11, -2.22, 3.33};
	double data[3];
	char addr_to_use = '0';

	ret = prep_addr(dev, addr_to_use);
	if ( ret != 0 ) {
		return ret;
	}

	ret = sdi_12_get_measurements(dev, addr_to_use, data, 3, true);

	if ( ret != 0 ) {
		return ret;
	}

	for (i=0; i<3; i++) {
		if (fabs(expected_data[i] - data[i]) > EPSILON) {
			LOG_ERR("Wrong data received");
			return -1;
		}
	}
	return 0;
}

static int8_t test_reading_simple_10s(struct device *dev)
{
	int ret;
	int i;
	double expected_data[] = {1.11, -2.22, 3.33};
	double data[3];
	char addr_to_use = '1';

	ret = prep_addr(dev, addr_to_use);
	if ( ret != 0 ) {
		return ret;
	}

	ret = sdi_12_get_measurements(dev, addr_to_use, data, 3, false);

	if ( ret != 0 ) {
		return ret;
	}

	for (i=0; i<3; i++) {
		if (fabs(expected_data[i] - data[i]) > EPSILON) {
			LOG_ERR("Wrong data received");
			return -1;
		}
	}
	return 0;
}

static int8_t test_reading_more_values(struct device *dev)
{
	int ret;
	int i;
	double expected_data[] = {1.11, -2.22, 3.33, -4.44,
				  5.55, -6.66, 7.77, -8.88,
				  9.99};
	double data[9];
	char addr_to_use = '2';

	ret = prep_addr(dev, addr_to_use);
	if ( ret != 0 ) {
		return ret;
	}
	ret = sdi_12_get_measurements(dev, addr_to_use, data, 9, false);

	if ( ret != 0 ) {
		return ret;
	}

	for (i=0; i<9; i++) {
		if (fabs(expected_data[i] - data[i]) > EPSILON) {
			LOG_ERR("Wrong data received");
			return -1;
		}
	}
	return 0;
}

static int8_t test_reading_with_service_req(struct device *dev)
{
	int ret;
	int i;
	s64_t time_at_start;
	double expected_data[] = {1.11, -2.22, 3.33};
	double data[3];
	char addr_to_use = '3';

	ret = prep_addr(dev, addr_to_use);
	if ( ret != 0 ) {
		return ret;
	}

	time_at_start = k_uptime_get();

	ret = sdi_12_get_measurements(dev, addr_to_use, data, 3, false);

	if ( ret != 0 ) {
		return ret;
	}

	if (k_uptime_get() - time_at_start > 6000) {
		LOG_ERR("Didn't see the service request.");
		return -1;
	}

	for (i=0; i<3; i++) {
		if (fabs(expected_data[i] - data[i]) > EPSILON) {
			LOG_ERR("Wrong data received");
			return -1;
		}
	}

	return 0;
}

static int8_t test_incorrect_crc(struct device *dev)
{
	int ret;
	double data[3];
	char addr_to_use = '4';

	ret = prep_addr(dev, addr_to_use);
	if ( ret != 0 ) {
		return ret;
	}

	ret = sdi_12_get_measurements(dev, addr_to_use, data, 3, false);

	if ( ret != SDI_12_STATUS_BAD_CRC ) {
		return ret;
	}

	return 0;

}

/**
 * @brief Main application entry point.
 *
 * The main application thread is responsible for initializing the
 * CANopen stack and doing the non real-time processing.
 */
void main(void)
{
	int ret;
	int i =0;


	static struct device *uart_dev;
	static struct device *gpio_dev;
	gpio_pin_t tx_enable_pin;

#ifdef CONFIG_BOARD_ADAFRUIT_FEATHER_M0_BASIC_PROTO
	struct device *muxa = device_get_binding(
					DT_LABEL(DT_NODELABEL(pinmux_a)));
	/* PA16 - Feather pin 11 - pad 0 - TX */
	pinmux_pin_set(muxa, 16, PINMUX_FUNC_C);
	/* PA18 - Feather pin 10 - pad 2 - RX */
	pinmux_pin_set(muxa, 18, PINMUX_FUNC_C);
	uart_dev = device_get_binding(DT_LABEL(DT_NODELABEL(sercom0)));
	gpio_dev = device_get_binding(DT_LABEL(DT_NODELABEL(porta)));
	tx_enable_pin = 19;
#else
	uart_dev = device_get_binding("YOUR_SOC_UART_LABEL");
	gpio_dev = device_get_binding("YOUR_SOC_GPIO_PORT_LABEL");
#endif

	if (uart_dev == NULL) {
		LOG_ERR("Getting UART device failed.");
		return;
	}

	if (gpio_dev == NULL) {
		LOG_ERR("Getting UART device failed.");
		return;
	}

	ret = sdi_12_init(uart_dev, gpio_dev, tx_enable_pin);
	if (ret != 0) {
		LOG_ERR("Initialization failed!");
		return;
	}

	k_sleep(K_SECONDS(1));

	int successes = 0, failures = 0;
	for (i=0; i<test_function_no; i++) {
		LOG_INF("Executing %s", test_functions_str[i]);
		ret = (test_functions[i])(uart_dev);
		if (ret == SDI_12_STATUS_OK) {
			LOG_INF("Test %s suceeded", test_functions_str[i]);
			successes++;
		} else {
			LOG_ERR("Test %s failed", test_functions_str[i]);
			failures++;
		}
		k_sleep(K_SECONDS(2));
	}
	LOG_INF("successes = %d failures =%d", successes, failures);
}
