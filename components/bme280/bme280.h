#ifndef BME280_H
#define BME280_H

#include "driver/i2c.h"
#include "esp_err.h"

typedef struct {
    i2c_port_t i2c_port;
    uint8_t addr;
} bme280_t;

esp_err_t bme280_init_desc(bme280_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);
esp_err_t bme280_init(bme280_t *dev);
esp_err_t bme280_read_float_data(bme280_t *dev, float *temperature, float *pressure, float *humidity);

#endif
