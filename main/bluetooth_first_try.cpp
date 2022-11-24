/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"
#include "esp_bt.h"
#include "soc/uhci_periph.h"
#include "driver/uart.h"
#include "esp_private/periph_ctrl.h" // for enabling UHCI module, remove it after UHCI driver is released
#include "esp_log.h"
#include "bluetooth/bluetooth/include/Bluetooth.hpp"

extern "C"
{ 
    void app_main(); 
}

void app_main(void)
{
    Bluetooth bluetooth("ESP_32");
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        printf("Bluetooth conected\n");
    }
}