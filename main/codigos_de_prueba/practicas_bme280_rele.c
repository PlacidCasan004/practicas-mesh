#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#include "bme280.h"

#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

#define BME280_I2C_ADDRESS 0x76

#define RELAY_PIN 23
#define RELAY_ACTIVE_LOW 1
#define TEMP_UMBRAL 28.0f

static const char *TAG = "BME280_RELAY";

static void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

static void relay_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RELAY_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    if (RELAY_ACTIVE_LOW) {
        gpio_set_level(RELAY_PIN, 1);
    } else {
        gpio_set_level(RELAY_PIN, 0);
    }
}

static void relay_set(int on)
{
    if (RELAY_ACTIVE_LOW) {
        gpio_set_level(RELAY_PIN, on ? 0 : 1);
    } else {
        gpio_set_level(RELAY_PIN, on ? 1 : 0);
    }
}

void app_main(void)
{
    i2c_master_init();
    relay_init();

    bme280_t dev;
    ESP_ERROR_CHECK(bme280_init_desc(&dev, BME280_I2C_ADDRESS, I2C_MASTER_NUM,
                                     I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));
    ESP_ERROR_CHECK(bme280_init(&dev));

    while (1) {
        float temperature = 0.0f;
        float pressure = 0.0f;
        float humidity = 0.0f;

        esp_err_t res = bme280_read_float_data(&dev, &temperature, &pressure, &humidity);

        if (res == ESP_OK) {
            printf("Temperatura: %.2f C | Presion: %.2f hPa | Humedad: %.2f %%\n",
                   temperature, pressure / 100.0f, humidity);

            if (temperature >= TEMP_UMBRAL) {
                printf("Temperatura alta -> RELE ACTIVADO\n");
                relay_set(1);
            } else {
                printf("Temperatura normal -> RELE DESACTIVADO\n");
                relay_set(0);
            }
        } else {
            ESP_LOGE(TAG, "Error leyendo BME280");
            relay_set(0);
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
