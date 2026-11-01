#include <stdbool.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "bmp280.h"

/* registers, datasheet section 4.2 */
#define REG_CALIB           0x88    /* 24 bytes, 0x88..0x9F */
#define REG_ID              0xD0
#define REG_RESET           0xE0
#define REG_STATUS          0xF3
#define REG_CTRL_MEAS       0xF4
#define REG_CONFIG          0xF5
#define REG_DATA            0xF7    /* 6 bytes: press[3], temp[3] */

#define CHIP_ID             0x58
#define RESET_CMD           0xB6
#define STATUS_MEASURING    (1 << 3)
#define STATUS_IM_UPDATE    (1 << 0)

/* mode bits in ctrl_meas[1:0] */
#define MODE_SLEEP          0x00
#define MODE_FORCED         0x01
#define MODE_NORMAL         0x03

/* data registers hold this value when no measurement has been made */
#define ADC_NO_DATA         0x80000

#define I2C_SPEED_HZ        100000
#define I2C_TIMEOUT_MS      100
#define POLL_MS             5
#define POLL_TRIES          20

static const char *TAG = "bmp280";

struct bmp280 {
	i2c_master_dev_handle_t i2c;
	bmp280_config_t config;
	uint8_t ctrl_meas;          /* osrs_t, osrs_p and mode, ready to write */
	uint32_t meas_ms;           /* worst-case measurement time */
	/* factory calibration, datasheet section 3.11.2 */
	uint16_t T1;
	int16_t T2, T3;
	uint16_t P1;
	int16_t P2, P3, P4, P5, P6, P7, P8, P9;
};

static esp_err_t read_regs(bmp280_handle_t dev, uint8_t reg, uint8_t *buf, size_t len)
{
	return i2c_master_transmit_receive(dev->i2c, &reg, 1, buf, len, I2C_TIMEOUT_MS);
}

static esp_err_t write_reg(bmp280_handle_t dev, uint8_t reg, uint8_t val)
{
	uint8_t buf[2] = { reg, val };

	return i2c_master_transmit(dev->i2c, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

/* poll STATUS until all bits in @mask are clear */
static esp_err_t wait_status_clear(bmp280_handle_t dev, uint8_t mask)
{
	uint8_t status;

	for (int i = 0; i < POLL_TRIES; i++) {
		ESP_RETURN_ON_ERROR(read_regs(dev, REG_STATUS, &status, 1), TAG, "read status");
		if (!(status & mask))
			return ESP_OK;
		vTaskDelay(pdMS_TO_TICKS(POLL_MS));
	}
	return ESP_ERR_TIMEOUT;
}

static uint16_t u16le(const uint8_t *p)
{
	return (uint16_t)(p[0] | (p[1] << 8));
}

static bool config_valid(const bmp280_config_t *c)
{
	return c->mode <= BMP280_MODE_NORMAL &&
	       c->temp_osr >= BMP280_OSR_X1 && c->temp_osr <= BMP280_OSR_X16 &&
	       c->press_osr >= BMP280_OSR_X1 && c->press_osr <= BMP280_OSR_X16 &&
	       c->filter <= BMP280_FILTER_16 &&
	       c->standby <= BMP280_STANDBY_4000_MS;
}

/* datasheet section 3.8.1: max time = 1.25 + 2.3 * osr_t + (2.3 * osr_p + 0.575) ms */
static uint32_t measurement_ms(const bmp280_config_t *c)
{
	uint32_t osr_t = 1u << (c->temp_osr - 1);
	uint32_t osr_p = 1u << (c->press_osr - 1);
	uint32_t us = 1250 + 2300 * osr_t + 2300 * osr_p + 575;

	return (us + 999) / 1000;
}

static esp_err_t load_calibration(bmp280_handle_t dev)
{
	uint8_t c[24];

	ESP_RETURN_ON_ERROR(read_regs(dev, REG_CALIB, c, sizeof(c)), TAG, "read calibration");
	dev->T1 = u16le(&c[0]);
	dev->T2 = (int16_t)u16le(&c[2]);
	dev->T3 = (int16_t)u16le(&c[4]);
	dev->P1 = u16le(&c[6]);
	dev->P2 = (int16_t)u16le(&c[8]);
	dev->P3 = (int16_t)u16le(&c[10]);
	dev->P4 = (int16_t)u16le(&c[12]);
	dev->P5 = (int16_t)u16le(&c[14]);
	dev->P6 = (int16_t)u16le(&c[16]);
	dev->P7 = (int16_t)u16le(&c[18]);
	dev->P8 = (int16_t)u16le(&c[20]);
	dev->P9 = (int16_t)u16le(&c[22]);
	return ESP_OK;
}

/* write config, then ctrl_meas; config writes can be ignored outside sleep mode */
static esp_err_t apply_config(bmp280_handle_t dev)
{
	const bmp280_config_t *c = &dev->config;
	uint8_t mode = c->mode == BMP280_MODE_NORMAL ? MODE_NORMAL : MODE_SLEEP;
	uint8_t config = (c->standby << 5) | (c->filter << 2);

	dev->ctrl_meas = (c->temp_osr << 5) | (c->press_osr << 2);
	dev->meas_ms = measurement_ms(c);

	ESP_RETURN_ON_ERROR(write_reg(dev, REG_CTRL_MEAS, dev->ctrl_meas | MODE_SLEEP), TAG, "sleep");
	ESP_RETURN_ON_ERROR(write_reg(dev, REG_CONFIG, config), TAG, "config");
	ESP_RETURN_ON_ERROR(write_reg(dev, REG_CTRL_MEAS, dev->ctrl_meas | mode), TAG, "ctrl_meas");
	return ESP_OK;
}

/* datasheet section 3.11.3: returns 0.01 degC, sets t_fine for pressure */
static int32_t compensate_t(bmp280_handle_t dev, int32_t adc_t, int32_t *t_fine)
{
	int32_t var1 = ((((adc_t >> 3) - ((int32_t)dev->T1 << 1))) * dev->T2) >> 11;
	int32_t var2 = (((((adc_t >> 4) - dev->T1) * ((adc_t >> 4) - dev->T1)) >> 12) * dev->T3) >> 14;

	*t_fine = var1 + var2;
	return (*t_fine * 5 + 128) >> 8;
}

/* datasheet section 3.11.3: returns Pa in Q24.8 (divide by 256), 0 on error */
static uint32_t compensate_p(bmp280_handle_t dev, int32_t adc_p, int32_t t_fine)
{
	int64_t var1 = (int64_t)t_fine - 128000;
	int64_t var2 = var1 * var1 * dev->P6;

	var2 += (var1 * dev->P5) << 17;
	var2 += (int64_t)dev->P4 << 35;
	var1 = ((var1 * var1 * dev->P3) >> 8) + ((var1 * dev->P2) << 12);
	var1 = ((((int64_t)1 << 47) + var1) * dev->P1) >> 33;
	if (var1 == 0)
		return 0;

	int64_t p = 1048576 - adc_p;

	p = (((p << 31) - var2) * 3125) / var1;
	var1 = ((int64_t)dev->P9 * (p >> 13) * (p >> 13)) >> 25;
	var2 = ((int64_t)dev->P8 * p) >> 19;
	p = ((p + var1 + var2) >> 8) + ((int64_t)dev->P7 << 4);
	return (uint32_t)p;
}

esp_err_t bmp280_init(i2c_master_bus_handle_t bus, const bmp280_config_t *config,
		      bmp280_handle_t *ret_handle)
{
	ESP_RETURN_ON_FALSE(bus && config && ret_handle, ESP_ERR_INVALID_ARG, TAG, "NULL argument");
	*ret_handle = NULL;
	ESP_RETURN_ON_FALSE(config_valid(config), ESP_ERR_INVALID_ARG, TAG, "invalid config");

	esp_err_t ret = ESP_OK;
	struct bmp280 *dev = calloc(1, sizeof(*dev));

	ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "no memory");
	dev->config = *config;

	uint8_t addr = config->addr;
	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = addr,
		.scl_speed_hz = I2C_SPEED_HZ,
	};
	ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus, &dev_cfg, &dev->i2c), fail, TAG,
			  "add device 0x%02x", addr);

	uint8_t id;
	ESP_GOTO_ON_FALSE(read_regs(dev, REG_ID, &id, 1) == ESP_OK, ESP_ERR_NOT_FOUND, fail, TAG,
			  "no response at 0x%02x", addr);
	ESP_GOTO_ON_FALSE(id == CHIP_ID, ESP_ERR_NOT_SUPPORTED, fail, TAG,
			  "0x%02x is not a BMP280 (id 0x%02x)", addr, id);

	/* reset to a known state, wait until it has copied its calibration from NVM */
	ESP_GOTO_ON_ERROR(write_reg(dev, REG_RESET, RESET_CMD), fail, TAG, "reset");
	vTaskDelay(pdMS_TO_TICKS(2));
	ESP_GOTO_ON_ERROR(wait_status_clear(dev, STATUS_IM_UPDATE), fail, TAG, "reset timeout");

	ESP_GOTO_ON_ERROR(load_calibration(dev), fail, TAG, "calibration");
	ESP_GOTO_ON_ERROR(apply_config(dev), fail, TAG, "apply config");

	ESP_LOGD(TAG, "ready at 0x%02x, %s mode, measurement %lu ms", addr,
		 config->mode == BMP280_MODE_NORMAL ? "normal" : "forced",
		 (unsigned long)dev->meas_ms);
	*ret_handle = dev;
	return ESP_OK;

fail:
	bmp280_delete(dev);
	return ret;
}

esp_err_t bmp280_read(bmp280_handle_t handle, float *temp_c, float *press_pa)
{
	ESP_RETURN_ON_FALSE(handle && temp_c && press_pa, ESP_ERR_INVALID_ARG, TAG, "NULL argument");

	if (handle->config.mode == BMP280_MODE_FORCED) {
		/* start one measurement; the sensor returns to sleep when done */
		ESP_RETURN_ON_ERROR(write_reg(handle, REG_CTRL_MEAS, handle->ctrl_meas | MODE_FORCED),
				    TAG, "start");
		vTaskDelay(pdMS_TO_TICKS(handle->meas_ms));
		ESP_RETURN_ON_ERROR(wait_status_clear(handle, STATUS_MEASURING), TAG, "measurement timeout");
	}

	/* one burst read, so pressure and temperature come from the same measurement */
	uint8_t d[6];

	ESP_RETURN_ON_ERROR(read_regs(handle, REG_DATA, d, sizeof(d)), TAG, "read data");

	int32_t adc_p = (d[0] << 12) | (d[1] << 4) | (d[2] >> 4);
	int32_t adc_t = (d[3] << 12) | (d[4] << 4) | (d[5] >> 4);

	if (adc_t == ADC_NO_DATA)
		return ESP_ERR_INVALID_STATE;    /* normal mode, first measurement not ready */

	int32_t t_fine;
	int32_t t = compensate_t(handle, adc_t, &t_fine);
	uint32_t p = compensate_p(handle, adc_p, t_fine);

	ESP_RETURN_ON_FALSE(p != 0, ESP_ERR_INVALID_RESPONSE, TAG, "invalid calibration");
	*temp_c = t / 100.0f;
	*press_pa = p / 256.0f;
	return ESP_OK;
}

void bmp280_delete(bmp280_handle_t handle)
{
	if (!handle)
		return;
	if (handle->i2c) {
		write_reg(handle, REG_CTRL_MEAS, MODE_SLEEP);    /* stop normal mode, best effort */
		i2c_master_bus_rm_device(handle->i2c);
	}
	free(handle);
}

