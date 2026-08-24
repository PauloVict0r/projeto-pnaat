/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <inttypes.h>
#include <stdio.h>

#include "i2c_config.h"
#include "imu_config.h"
#include "oled_printf.h"
#include "oled_setup.h"

static const char TAG[] = "main";

extern lv_disp_t *local_disp;

void app_main(void)
{
    enable_vext_rail(); // rail de energia da placa, uma vez, antes de qualquer I2C

    i2c_port_t i2c_port_num = I2C_NUM_0;

    initialize_i2c(&i2c_port_num, PIN_NUM_SDA, PIN_NUM_SCL); // barramento do OLED

    configure_oled_screen(&i2c_port_num);

    oled_printf_init(local_disp);

    imu_config_init(&i2c_port_num); // sobe seu proprio barramento (I2C_NUM_1), ver imu_config.c

    ESP_LOGI(TAG, "Enter in the main loop...");

    while (1) { 
        imu_data_t imu_data; 
        if (xQueueReceive(imu_queue, &imu_data, portMAX_DELAY) == pdPASS) { 
            printf("%02.4f, %02.4f, %02.4f\n", 
                /*imu_data.accel_x, imu_data.accel_y, imu_data.accel_z, */
                imu_data.rotation_x, imu_data.rotation_y, imu_data.rotation_z); 
        printf_oled("X:%.1f Y:%.1f Z:%.1f\n", 
                        imu_data.rotation_x, imu_data.rotation_y, imu_data.rotation_z);
        } 
    } 
}
