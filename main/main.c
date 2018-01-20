#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <driver/i2c.h>
#include <esp_log.h>

#include "esp_bme280.h"

static void bme280_task(void* arg) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 18,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = 19,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 1 * 1000 * 1000,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_1, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_1, conf.mode, 0, 0, 0));

    esp_bme280_t bme280;
    ESP_ERROR_CHECK(esp_bme280_init(&bme280));

#ifdef CONFIG_BME280_FLOAT_ENABLE
    double temperature, pressure, humidity;
#else
    int32_t temperature;
    uint32_t pressure, humidity;
#endif 

    while(true) {
        ESP_ERROR_CHECK(esp_bme280_get_readings(&bme280, &temperature, &pressure, &humidity));

#ifdef CONFIG_BME280_FLOAT_ENABLE
        ESP_LOGD("app", "T: %f P: %f H: %f", temperature, pressure, humidity)
#else
        ESP_LOGD("app", "T: %d P: %d H: %d", temperature, pressure, humidity)
#endif

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void app_main() {
    xTaskCreate(bme280_task, "bme280_task", 1024 * 2, NULL, 10, NULL);
}