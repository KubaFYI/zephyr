# SPDX-License-Identifier: Apache-2.0

zephyr_sources_ifdef(CONFIG_SDI_12    sdi_12.c)

target_sources(app PRIVATE
  sdi_12_uart.c)