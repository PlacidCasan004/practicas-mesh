#include "bme280.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define BME280_REG_ID         0xD0
#define BME280_REG_RESET      0xE0
#define BME280_REG_CTRL_HUM   0xF2
#define BME280_REG_STATUS     0xF3
#define BME280_REG_CTRL_MEAS  0xF4
#define BME280_REG_CONFIG     0xF5
#define BME280_REG_PRESS_MSB  0xF7
#define BME280_REG_CALIB00    0x88
#define BME280_REG_CALIB26    0xE1

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;

    int32_t  t_fine;
} bme280_calib_data_t;

static bme280_calib_data_t calib;

static esp_err_t bme280_write_reg(bme280_t *dev, uint8_t reg, uint8_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t bme280_read_regs(bme280_t *dev, uint8_t reg, uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->addr << 1) | I2C_MASTER_READ, true);

    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK);

    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static uint16_t read_u16_le(const uint8_t *buf)
{
    return (uint16_t)(buf[0] | (buf[1] << 8));
}

static int16_t read_s16_le(const uint8_t *buf)
{
    return (int16_t)(buf[0] | (buf[1] << 8));
}

static esp_err_t read_calibration_data(bme280_t *dev)
{
    uint8_t buf1[26];
    uint8_t buf2[7];

    esp_err_t ret = bme280_read_regs(dev, BME280_REG_CALIB00, buf1, sizeof(buf1));
    if (ret != ESP_OK) return ret;

    ret = bme280_read_regs(dev, BME280_REG_CALIB26, buf2, sizeof(buf2));
    if (ret != ESP_OK) return ret;

    calib.dig_T1 = read_u16_le(&buf1[0]);
    calib.dig_T2 = read_s16_le(&buf1[2]);
    calib.dig_T3 = read_s16_le(&buf1[4]);

    calib.dig_P1 = read_u16_le(&buf1[6]);
    calib.dig_P2 = read_s16_le(&buf1[8]);
    calib.dig_P3 = read_s16_le(&buf1[10]);
    calib.dig_P4 = read_s16_le(&buf1[12]);
    calib.dig_P5 = read_s16_le(&buf1[14]);
    calib.dig_P6 = read_s16_le(&buf1[16]);
    calib.dig_P7 = read_s16_le(&buf1[18]);
    calib.dig_P8 = read_s16_le(&buf1[20]);
    calib.dig_P9 = read_s16_le(&buf1[22]);

    calib.dig_H1 = buf1[25];
    calib.dig_H2 = read_s16_le(&buf2[0]);
    calib.dig_H3 = buf2[2];
    calib.dig_H4 = (int16_t)((buf2[3] << 4) | (buf2[4] & 0x0F));
    calib.dig_H5 = (int16_t)((buf2[5] << 4) | (buf2[4] >> 4));
    calib.dig_H6 = (int8_t)buf2[6];

    return ESP_OK;
}

esp_err_t bme280_init_desc(bme280_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    (void)sda_gpio;
    (void)scl_gpio;

    if (!dev) return ESP_ERR_INVALID_ARG;
    dev->addr = addr;
    dev->i2c_port = port;
    return ESP_OK;
}

esp_err_t bme280_init(bme280_t *dev)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    uint8_t id = 0;
    esp_err_t ret = bme280_read_regs(dev, BME280_REG_ID, &id, 1);
    if (ret != ESP_OK) return ret;
    if (id != 0x60) return ESP_ERR_INVALID_RESPONSE;

    ret = bme280_write_reg(dev, BME280_REG_RESET, 0xB6);
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(10));

    ret = read_calibration_data(dev);
    if (ret != ESP_OK) return ret;

    ret = bme280_write_reg(dev, BME280_REG_CTRL_HUM, 0x01);
    if (ret != ESP_OK) return ret;

    ret = bme280_write_reg(dev, BME280_REG_CTRL_MEAS, 0x27);
    if (ret != ESP_OK) return ret;

    ret = bme280_write_reg(dev, BME280_REG_CONFIG, 0x00);
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

static int32_t compensate_temperature(int32_t adc_T)
{
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) *
            ((int32_t)calib.dig_T3)) >> 14;
    calib.t_fine = var1 + var2;
    T = (calib.t_fine * 5 + 128) >> 8;
    return T;
}

static uint32_t compensate_pressure(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)calib.t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) + ((var1 * (int64_t)calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib.dig_P1) >> 33;

    if (var1 == 0) return 0;

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4);

    return (uint32_t)p;
}

static uint32_t compensate_humidity(int32_t adc_H)
{
    int32_t v_x1_u32r;
    v_x1_u32r = calib.t_fine - ((int32_t)76800);
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)calib.dig_H4) << 20) -
                    (((int32_t)calib.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                 (((((((v_x1_u32r * ((int32_t)calib.dig_H6)) >> 10) *
                      (((v_x1_u32r * ((int32_t)calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                    ((int32_t)2097152)) * ((int32_t)calib.dig_H2) + 8192) >> 14));
    v_x1_u32r = v_x1_u32r -
                (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)calib.dig_H1)) >> 4);
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r >> 12);
}

esp_err_t bme280_read_float_data(bme280_t *dev, float *temperature, float *pressure, float *humidity)
{
    if (!dev || !temperature || !pressure || !humidity) return ESP_ERR_INVALID_ARG;

    uint8_t data[8];
    esp_err_t ret = bme280_read_regs(dev, BME280_REG_PRESS_MSB, data, 8);
    if (ret != ESP_OK) return ret;

    int32_t adc_P = (int32_t)((data[0] << 12) | (data[1] << 4) | (data[2] >> 4));
    int32_t adc_T = (int32_t)((data[3] << 12) | (data[4] << 4) | (data[5] >> 4));
    int32_t adc_H = (int32_t)((data[6] << 8) | data[7]);

    int32_t temp = compensate_temperature(adc_T);
    uint32_t press = compensate_pressure(adc_P);
    uint32_t hum = compensate_humidity(adc_H);

    *temperature = temp / 100.0f;
    *pressure = press / 256.0f;
    *humidity = hum / 1024.0f;

    return ESP_OK;
}
