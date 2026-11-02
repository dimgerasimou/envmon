#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "ens160.h"

/* registers, ENS160 datasheet section 16 */
#define REG_PART_ID       0x00 /* 2 bytes, little endian */
#define REG_OPMODE        0x10
#define REG_TEMP_IN       0x13 /* 2 bytes temperature + 2 bytes humidity */
#define REG_DEVICE_STATUS 0x20
#define REG_DATA_AQI      0x21 /* 5 bytes: AQI, TVOC[2], ECO2[2] */

#define PART_ID 0x0160

#define OPMODE_DEEP_SLEEP 0x00
#define OPMODE_IDLE       0x01
#define OPMODE_STANDARD   0x02
#define OPMODE_RESET      0xF0

#define STATUS_ERROR       (1 << 6)
#define STATUS_VALIDITY(s) (((s) >> 2) & 0x03)

#define RESET_MS       10
#define MODE_CHANGE_MS 10

#define I2C_SPEED_HZ   100000
#define I2C_TIMEOUT_MS 100

static const char *TAG = "ens160";

struct ens160 {
	i2c_master_dev_handle_t i2c;
};

static esp_err_t read_regs(ens160_handle_t dev, uint8_t reg, uint8_t *buf, size_t len)
{
	return i2c_master_transmit_receive(dev->i2c, &reg, 1, buf, len, I2C_TIMEOUT_MS);
}

static esp_err_t set_opmode(ens160_handle_t dev, uint8_t mode)
{
	uint8_t buf[2] = { REG_OPMODE, mode };

	return i2c_master_transmit(dev->i2c, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

esp_err_t ens160_init(i2c_master_bus_handle_t bus, const ens160_config_t *config,
                      ens160_handle_t *ret_handle)
{
	ESP_RETURN_ON_FALSE(bus && config && ret_handle, ESP_ERR_INVALID_ARG, TAG, "NULL argument");
	*ret_handle = NULL;
	ESP_RETURN_ON_FALSE(config->addr == ENS160_ADDR_LOW || config->addr == ENS160_ADDR_HIGH,
	                    ESP_ERR_INVALID_ARG, TAG, "invalid address 0x%02x", config->addr);

	esp_err_t ret = ESP_OK;
	struct ens160 *dev = calloc(1, sizeof(*dev));

	ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "no memory");

	uint8_t addr = config->addr;
	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = addr,
		.scl_speed_hz = I2C_SPEED_HZ,
	};
	ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus, &dev_cfg, &dev->i2c), fail, TAG,
	                  "add device 0x%02x", addr);

	uint8_t id[2];

	ESP_GOTO_ON_FALSE(read_regs(dev, REG_PART_ID, id, sizeof(id)) == ESP_OK, ESP_ERR_NOT_FOUND,
	                  fail, TAG, "no response at 0x%02x", addr);

	uint16_t part_id = id[0] | (id[1] << 8);

	ESP_GOTO_ON_FALSE(part_id == PART_ID, ESP_ERR_NOT_SUPPORTED, fail, TAG,
	                  "0x%02x is not an ENS160 (part id 0x%04x)", addr, part_id);

	/* reset, then idle, then standard: the sequence the datasheet requires */
	ESP_GOTO_ON_ERROR(set_opmode(dev, OPMODE_RESET), fail, TAG, "reset");
	vTaskDelay(pdMS_TO_TICKS(RESET_MS));
	ESP_GOTO_ON_ERROR(set_opmode(dev, OPMODE_IDLE), fail, TAG, "idle");
	vTaskDelay(pdMS_TO_TICKS(MODE_CHANGE_MS));
	ESP_GOTO_ON_ERROR(set_opmode(dev, OPMODE_STANDARD), fail, TAG, "standard mode");

	ESP_LOGD(TAG, "ready at 0x%02x", addr);
	*ret_handle = dev;
	return ESP_OK;

fail:
	ens160_delete(dev);
	return ret;
}

esp_err_t ens160_set_compensation(ens160_handle_t handle, float temp_c, float humidity_pct)
{
	ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "NULL handle");
	ESP_RETURN_ON_FALSE(temp_c > -40.0f && temp_c < 85.0f && humidity_pct >= 0.0f &&
	                            humidity_pct <= 100.0f,
	                    ESP_ERR_INVALID_ARG, TAG, "out of range");

	/* TEMP_IN is kelvin * 64, RH_IN is %RH * 512, both little endian */
	uint16_t t = (uint16_t)((temp_c + 273.15f) * 64.0f + 0.5f);
	uint16_t h = (uint16_t)(humidity_pct * 512.0f + 0.5f);
	uint8_t buf[5] = { REG_TEMP_IN, t & 0xFF, t >> 8, h & 0xFF, h >> 8 };

	return i2c_master_transmit(handle->i2c, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

esp_err_t ens160_read(ens160_handle_t handle, ens160_data_t *out)
{
	ESP_RETURN_ON_FALSE(handle && out, ESP_ERR_INVALID_ARG, TAG, "NULL argument");

	uint8_t status;

	ESP_RETURN_ON_ERROR(read_regs(handle, REG_DEVICE_STATUS, &status, 1), TAG, "read status");
	ESP_RETURN_ON_FALSE(!(status & STATUS_ERROR), ESP_ERR_INVALID_RESPONSE, TAG,
	                    "sensor error (status 0x%02x)", status);

	/* the data registers always hold the latest measurement */
	uint8_t d[5];

	ESP_RETURN_ON_ERROR(read_regs(handle, REG_DATA_AQI, d, sizeof(d)), TAG, "read data");
	out->aqi = d[0] & 0x07;
	out->tvoc_ppb = d[1] | (d[2] << 8);
	out->eco2_ppm = d[3] | (d[4] << 8);
	out->validity = STATUS_VALIDITY(status);
	return ESP_OK;
}

void ens160_delete(ens160_handle_t handle)
{
	if (!handle)
		return;
	if (handle->i2c) {
		set_opmode(handle, OPMODE_DEEP_SLEEP); /* stop the heater, best effort */
		i2c_master_bus_rm_device(handle->i2c);
	}
	free(handle);
}
