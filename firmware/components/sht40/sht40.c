#include <stdlib.h>
#include "esp_check.h"
#include "esp_log.h"
#include "sensirion.h"
#include "sht40.h"

/* commands, SHT4x datasheet section 4.5: 8-bit, unlike most Sensirion parts */
#define CMD_SOFT_RESET  0x94
#define CMD_READ_SERIAL 0x89

#define RESET_MS       1
#define SERIAL_MS      1
#define I2C_SPEED_HZ   100000
#define I2C_TIMEOUT_MS 100

static const char *TAG = "sht40";

/* command and worst-case duration for each precision */
static const struct {
	uint8_t cmd;
	uint8_t ms;
} s_measure[] = {
	[SHT40_PRECISION_HIGH] = { 0xFD, 9 },
	[SHT40_PRECISION_MEDIUM] = { 0xF6, 5 },
	[SHT40_PRECISION_LOW] = { 0xE0, 2 },
};

struct sht40 {
	i2c_master_dev_handle_t i2c;
	sht40_precision_t precision;
};

static esp_err_t send_cmd(i2c_master_dev_handle_t dev, uint8_t cmd)
{
	return i2c_master_transmit(dev, &cmd, 1, I2C_TIMEOUT_MS);
}

esp_err_t sht40_init(i2c_master_bus_handle_t bus, const sht40_config_t *config,
                     sht40_handle_t *ret_handle)
{
	ESP_RETURN_ON_FALSE(bus && config && ret_handle, ESP_ERR_INVALID_ARG, TAG, "NULL argument");
	*ret_handle = NULL;
	ESP_RETURN_ON_FALSE(config->addr == SHT40_ADDR_A || config->addr == SHT40_ADDR_B,
	                    ESP_ERR_INVALID_ARG, TAG, "invalid address 0x%02x", config->addr);
	ESP_RETURN_ON_FALSE(config->precision <= SHT40_PRECISION_LOW, ESP_ERR_INVALID_ARG, TAG,
	                    "invalid precision");

	esp_err_t ret = ESP_OK;
	struct sht40 *dev = calloc(1, sizeof(*dev));

	ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "no memory");
	dev->precision = config->precision;

	uint8_t addr = config->addr;
	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = addr,
		.scl_speed_hz = I2C_SPEED_HZ,
	};
	ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus, &dev_cfg, &dev->i2c), fail, TAG,
	                  "add device 0x%02x", addr);

	/* soft reset; no ACK here means nothing is at this address */
	ESP_GOTO_ON_FALSE(send_cmd(dev->i2c, CMD_SOFT_RESET) == ESP_OK, ESP_ERR_NOT_FOUND, fail,
	                  TAG, "no response at 0x%02x", addr);
	sensirion_delay_ms(RESET_MS);

	/* the serial number with valid CRCs is the identity check */
	uint16_t serial[2];

	ESP_GOTO_ON_ERROR(send_cmd(dev->i2c, CMD_READ_SERIAL), fail, TAG, "serial cmd");
	sensirion_delay_ms(SERIAL_MS);
	ESP_GOTO_ON_FALSE(sensirion_read(dev->i2c, serial, 2) == ESP_OK, ESP_ERR_INVALID_RESPONSE,
	                  fail, TAG, "0x%02x: bad serial reply, not an SHT40?", addr);

	ESP_LOGD(TAG, "ready at 0x%02x, serial %04x%04x", addr, serial[0], serial[1]);
	*ret_handle = dev;
	return ESP_OK;

fail:
	if (dev->i2c)
		i2c_master_bus_rm_device(dev->i2c);
	free(dev);
	return ret;
}

esp_err_t sht40_read(sht40_handle_t handle, float *temp_c, float *humidity_pct)
{
	ESP_RETURN_ON_FALSE(handle && temp_c && humidity_pct, ESP_ERR_INVALID_ARG, TAG,
	                    "NULL argument");

	uint16_t raw[2];

	ESP_RETURN_ON_ERROR(send_cmd(handle->i2c, s_measure[handle->precision].cmd), TAG, "start");
	sensirion_delay_ms(s_measure[handle->precision].ms);
	ESP_RETURN_ON_ERROR(sensirion_read(handle->i2c, raw, 2), TAG, "read data");

	/* datasheet section 4.6 */
	float rh = -6.0f + 125.0f * raw[1] / 65535.0f;

	*temp_c = -45.0f + 175.0f * raw[0] / 65535.0f;
	*humidity_pct = rh < 0.0f ? 0.0f : rh > 100.0f ? 100.0f : rh;
	return ESP_OK;
}

void sht40_delete(sht40_handle_t handle)
{
	if (!handle)
		return;
	i2c_master_bus_rm_device(handle->i2c);
	free(handle);
}
