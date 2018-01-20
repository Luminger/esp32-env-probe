#include <driver/i2c.h>
#include "bme280_defs.h"

#define ESP_BME280_I2C_SCL 19
#define ESP_BME280_I2C_SDA 18
#define ESP_BME280_I2C_MASTER_NUM I2C_NUM_1
#define ESP_BME280_I2C_ADDR BME280_I2C_ADDR_PRIM