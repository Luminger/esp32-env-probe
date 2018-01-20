#include <math.h>
#include <driver/i2c.h>
#include <esp_log.h>

#include "esp_bme280.h"

static const char* TAG = "esp_bme280";

static void user_delay_ms(uint32_t period)
{
    // For the curious: The Bosch BME280 driver does use this function with
    // rather low values (2 for example). If the devision below is done
    // without the promotion of period to float, it will just result in 0, 
    // which breaks the protocol between us and the BME280. Therefor we do
    // the math in floating point and, for good measure, make sure to sleep
    // at least the requested amount of time by ceiling the value as well.
    vTaskDelay(ceil((float)period / portTICK_PERIOD_MS));
}

static int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if(cmd == NULL) {
        // TODO: terminate
    }

    esp_err_t err = ESP_OK;
    err |= i2c_master_start(cmd);
    err |= i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    err |= i2c_master_write_byte(cmd, reg_addr, I2C_MASTER_ACK);

    err |= i2c_master_start(cmd);
    err |= i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_READ, I2C_MASTER_ACK);
    err |= i2c_master_read(cmd, reg_data, len, I2C_MASTER_LAST_NACK);
    err |= i2c_master_stop(cmd);
    ESP_ERROR_CHECK(err);

    // As this might happen without being fatal, we don't use ESP_ERROR_CHECK here.
    err = i2c_master_cmd_begin(ESP_BME280_I2C_MASTER_NUM, cmd, 10 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return err == ESP_OK ? BME280_OK : BME280_E_COMM_FAIL;
}

static int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if(cmd == NULL) {
        // TODO: terminate
    }

    esp_err_t err = ESP_OK;
    err |= i2c_master_start(cmd);
    err |= i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    err |= i2c_master_write_byte(cmd, reg_addr, I2C_MASTER_ACK);
    err |= i2c_master_write(cmd, reg_data, len, I2C_MASTER_ACK);
    err |= i2c_master_stop(cmd);
    ESP_ERROR_CHECK(err);

    // As this might happen without being fatal, we don't use ESP_ERROR_CHECK here.
    err = i2c_master_cmd_begin(ESP_BME280_I2C_MASTER_NUM, cmd, 10 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return err == ESP_OK ? BME280_OK : BME280_E_COMM_FAIL;
}

esp_err_t esp_bme280_init(esp_bme280_t* handle) {
    if(handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    handle->dev = malloc(sizeof(struct bme280_dev));
    if(handle->dev == NULL) {
        return ESP_ERR_NO_MEM;
    }

    handle->dev->dev_id = ESP_BME280_I2C_ADDR;
    handle->dev->intf = BME280_I2C_INTF;
    handle->dev->read = user_i2c_read;
    handle->dev->write = user_i2c_write;
    handle->dev->delay_ms = user_delay_ms;

    int8_t err = bme280_init(handle->dev);
    if(err != BME280_OK) {
        ESP_LOGE(TAG, "Failed to init device: %d returned", err);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

void esp_bme280_free(esp_bme280_t* handle) {
    if(handle == NULL) {
        return;
    }

    free(handle->dev);
}

#ifdef CONFIG_BME280_FLOAT_ENABLE
esp_err_t esp_bme280_get_readings(esp_bme280_t* handle, double* temperature, double* pressure, double* humidity) {
#else 
esp_err_t esp_bme280_get_readings(esp_bme280_t* handle, int32_t* temperature, uint32_t* pressure, uint32_t* humidity) {
#endif

    if(handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Mode of operation: humidity sensor */
    handle->dev->settings.osr_h = BME280_OVERSAMPLING_1X;
    handle->dev->settings.osr_p = BME280_OVERSAMPLING_1X;
    handle->dev->settings.osr_t = BME280_OVERSAMPLING_1X;
    handle->dev->settings.filter = BME280_FILTER_COEFF_OFF;

    /* Recommended mode of operation: Indoor navigation */
    //dev->settings.osr_h = BME280_OVERSAMPLING_1X;
    //dev->settings.osr_p = BME280_OVERSAMPLING_16X;
    //dev->settings.osr_t = BME280_OVERSAMPLING_2X;
    //dev->settings.filter = BME280_FILTER_COEFF_16;

    uint8_t settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL;
    int8_t err = bme280_set_sensor_settings(settings_sel, handle->dev);
    if(err != BME280_OK) {
        ESP_LOGE(TAG, "bme280_set_sensor_settings() failed: %d\n", err);
        return ESP_ERR_INVALID_RESPONSE;
    }

    err = bme280_set_sensor_mode(BME280_FORCED_MODE, handle->dev);
    if(err != BME280_OK) {
        ESP_LOGE(TAG, "bme280_set_sensor_mode() failed: %d\n", err);
        return ESP_ERR_INVALID_RESPONSE;
    }

    vTaskDelay(20 / portTICK_PERIOD_MS);

    struct bme280_data comp_data;
    err = bme280_get_sensor_data(BME280_ALL, &comp_data, handle->dev);
    if(err != BME280_OK) {
        printf("bme280_get_sensor_data() failed: %d\n", err);
        return ESP_ERR_INVALID_RESPONSE;
    }

    if(temperature != NULL) {
        *temperature = comp_data.temperature;
    }

    if(pressure != NULL) {
        *pressure = comp_data.pressure;
    }

    if(humidity != NULL) {
        *humidity = comp_data.humidity;
    }

    return ESP_OK;
}