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
    char addr = '0';
    double data[10];
    int datalen = 10;
    int i;

    static struct device *uart_dev;
    static struct device *gpio_dev;
    gpio_pin_t tx_enable_pin;

#ifdef CONFIG_BOARD_ADAFRUIT_FEATHER_M0_BASIC_PROTO
    struct device *muxa = device_get_binding(DT_LABEL(DT_NODELABEL(pinmux_a)));
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

    while (1) {
        ret = sdi_12_get_measurements(uart_dev, addr,
                data, datalen, false);
        if (ret == 0 ) {
            LOG_INF("Data read ok");
            for(i=0; i<datalen; i++) {
                LOG_INF("Meas %d: %d.%d", i, (int)data[i],
                        (int)((data[i]-(int)data[i])*10));
            }
        } else {
            LOG_INF("err: %s!", log_strdup(SDI_12_ERR_TO_STR(ret)));
        }
        k_sleep(K_SECONDS(5));
    }
}
