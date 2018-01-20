#include "bme280.h"
#include "esp_bme280_def.h"

typedef struct {
    struct bme280_dev* dev;
} esp_bme280_t;

esp_err_t esp_bme280_init(esp_bme280_t* handle);
void esp_bme280_free(esp_bme280_t* handle);

#ifdef CONFIG_BME280_FLOAT_ENABLE
esp_err_t esp_bme280_get_readings(esp_bme280_t* handle, double* temperature, double* pressure, double* humidity);
#else
esp_err_t esp_bme280_get_readings(esp_bme280_t* handle, int32_t* temperature, uint32_t* pressure, uint32_t* humidity);
#endif