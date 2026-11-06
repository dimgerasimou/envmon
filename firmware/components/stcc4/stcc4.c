#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "sensirion.h"
#include "stcc4.h"

/* commands, from Sensirion's embedded-i2c-stcc4 driver */
#define CMD_START_CONTINUOUS 0x218B
#define CMD_STOP_CONTINUOUS  0x3F86
#define CMD_READ_MEASUREMENT 0xEC05
#define CMD_GET_PRODUCT_ID   0x365B
#define CMD_SET_PRESSURE     0xE016

#define STOP_MS      1200 /* one interval + 200 ms tolerance */
#define FIRST_MS     1200 /* first result after start, with margin */
#define CMD_MS       1
#define I2C_SPEED_HZ 100000

static const char *TAG = "stcc4";

struct stcc4 {
	i2c_master_dev_handle_t i2c;
	TickType_t started; /* tick of start_continuous */
};

esp_err_t stcc4_init(i2c_master_bus_handle_t bus, const stcc4_config_t *config,
                     stcc4_handle_t *ret_handle)
{
	ESP_RETURN_ON_FALSE(bus && config && ret_handle, ESP_ERR_INVALID_ARG, TAG, "NULL argument");
	*ret_handle = NULL;
	ESP_RETURN_ON_FALSE(config->addr == STCC4_ADDR_LOW || config->addr == STCC4_ADDR_HIGH,
	                    ESP_ERR_INVALID_ARG, TAG, "invalid address 0x%02x", config->addr);

	esp_err_t ret = ESP_OK;
	struct stcc4 *dev = calloc(1, sizeof(*dev));

	ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "no memory");

	uint8_t addr = config->addr;
	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = addr,
		.scl_speed_hz = I2C_SPEED_HZ,
	};
	ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus, &dev_cfg, &dev->i2c), fail, TAG,
	                  "add device 0x%02x", addr);

	/*
	 * The sensor keeps power across an ESP reset and may still be measuring,
	 * and it ignores most commands while it does. Stop it; no ACK means
	 * nothing is at this address.
	 */
	ESP_GOTO_ON_FALSE(sensirion_write(dev->i2c, CMD_STOP_CONTINUOUS, NULL, 0) == ESP_OK,
	                  ESP_ERR_NOT_FOUND, fail, TAG, "no response at 0x%02x", addr);
	sensirion_delay_ms(STOP_MS);

	/* product ID and serial number with valid CRCs is the identity check */
	uint16_t id[6];

	ESP_GOTO_ON_FALSE(sensirion_cmd_read(dev->i2c, CMD_GET_PRODUCT_ID, CMD_MS, id, 6) == ESP_OK,
	                  ESP_ERR_INVALID_RESPONSE, fail, TAG,
	                  "0x%02x: bad product ID, not an STCC4?", addr);

	ESP_GOTO_ON_ERROR(sensirion_write(dev->i2c, CMD_START_CONTINUOUS, NULL, 0), fail, TAG,
	                  "start");
	dev->started = xTaskGetTickCount();

	ESP_LOGD(TAG, "ready at 0x%02x, product %04x%04x, serial %04x%04x%04x%04x", addr, id[0],
	         id[1], id[2], id[3], id[4], id[5]);
	*ret_handle = dev;
	return ESP_OK;

fail:
	if (dev->i2c)
		i2c_master_bus_rm_device(dev->i2c);
	free(dev);
	return ret;
}

esp_err_t stcc4_read(stcc4_handle_t handle, stcc4_data_t *out)
{
	ESP_RETURN_ON_FALSE(handle && out, ESP_ERR_INVALID_ARG, TAG, "NULL argument");

	uint16_t raw[4]; /* CO2, temperature, humidity, status */

	/* the sensor NACKs until its first result; don't count that as an error */
	if (xTaskGetTickCount() - handle->started < pdMS_TO_TICKS(FIRST_MS))
		return ESP_ERR_NOT_FINISHED;

	ESP_RETURN_ON_ERROR(sensirion_cmd_read(handle->i2c, CMD_READ_MEASUREMENT, CMD_MS, raw, 4),
	                    TAG, "read");

	float rh = -6.0f + 125.0f * raw[2] / 65535.0f;

	out->co2_ppm = (int16_t)raw[0];
	out->temp_c = -45.0f + 175.0f * raw[1] / 65535.0f;
	out->humidity_pct = rh < 0.0f ? 0.0f : rh > 100.0f ? 100.0f : rh;
	return ESP_OK;
}

esp_err_t stcc4_set_pressure(stcc4_handle_t handle, float pressure_pa)
{
	ESP_RETURN_ON_FALSE(handle && pressure_pa >= 40000.0f && pressure_pa <= 110000.0f,
	                    ESP_ERR_INVALID_ARG, TAG, "invalid args");

	uint16_t raw = (uint16_t)(pressure_pa / 2.0f); /* sent in units of 2 Pa */

	ESP_RETURN_ON_ERROR(sensirion_write(handle->i2c, CMD_SET_PRESSURE, &raw, 1), TAG,
	                    "set pressure");
	sensirion_delay_ms(CMD_MS);
	return ESP_OK;
}

void stcc4_delete(stcc4_handle_t handle)
{
	if (!handle)
		return;
	sensirion_write(handle->i2c, CMD_STOP_CONTINUOUS, NULL, 0); /* best effort */
	i2c_master_bus_rm_device(handle->i2c);
	free(handle);
}
