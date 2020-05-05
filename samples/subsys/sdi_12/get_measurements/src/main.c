/*
 * Copyright (c) 2019 R3 IoT Ltd.
 *
 * License: Apache-2.0
 *
 * An example app using the SDI-12 subsystem for sensor interaction.
*/
#include <zephyr.h>
#include <drivers/gpio.h>
#include <drivers/pinmux.h>
#include <power/reboot.h>
#include <settings/settings.h>
#include <sdi_12/sdi_12.h>

#define LOG_LEVEL CONFIG_SDI_12_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(app);


#define UART_INTERFACE DT_INST_1_ATMEL_SAM0_UART_LABEL

struct device *uart_dev;

/**
 * @brief Main application entry point.
 *
 * The main application thread is responsible for initializing the
 * CANopen stack and doing the non real-time processing.
 */
void main(void)
{
    int ret;
    char addr = '5';
    double data[1];
    int datalen = 1;

#ifdef CONFIG_BOARD_ADAFRUIT_FEATHER_M0_BASIC_PROTO
    struct device *muxa = device_get_binding(DT_ATMEL_SAM0_PINMUX_PINMUX_A_LABEL);

    pinmux_pin_set(muxa, 16, PINMUX_FUNC_C);
    pinmux_pin_set(muxa, 18, PINMUX_FUNC_C);
    uart_dev = device_get_binding("SERCOM1");
#else
    uart_dev = device_get_binding("YOUR_SOC_UART_LABEL");
#endif


    LOG_INF("uart dev: 0x%x", uart_dev);
    ret = sdi_12_init(uart_dev);
    if (ret != 0) {
        LOG_ERR("Initialization failed!");
        return;
    }


    while (1) {
        ret = sdi_12_get_measurements(uart_dev, addr,
                data, datalen, false);
        if (ret == 0 ) {
            LOG_INF("okay");
        } else {
            LOG_INF("%s!", log_strdup(SDI_12_ERR_TO_STR(ret)));
        }
        k_sleep(5000);
    }
}
