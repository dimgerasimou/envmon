#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "esp_check.h"
#include "esp_log.h"
#include "sen66.h"
#include "sensirion.h"

/* commands, from Sensirion's embedded-i2c-sen66 driver */
#define CMD_START            0x0021
#define CMD_STOP             0x0104
#define CMD_READ_VALUES      0x0300
#define CMD_READ_STATUS      0xD206
#define CMD_GET_PRODUCT_NAME 0xD014
#define CMD_DEVICE_RESET     0xD304
#define CMD_AMBIENT_PRESSURE 0x6720

#define RESET_MS     1200
#define START_MS     50
#define CMD_MS       20     /* minimum gap after every command */
#define I2C_SPEED_HZ 100000 /* SEN6x supports standard mode only */

#define UNKNOWN_U16 0xFFFF
#define UNKNOWN_I16 0x7FFF

static const char *TAG = "sen66";

struct sen66 {
	i2c_master_dev_handle_t i2c;
};

/* Scaled unsigned value, or NAN if the sensor marks it unknown. */
static float scale_u(uint16_t raw, float factor)
{
	return raw == UNKNOWN_U16 ? NAN : raw / factor;
}

/* VOC/NOx index: valid range 1..500; 0 is reported while the algorithm starts. */
static float scale_index(uint16_t raw)
{
	return raw == UNKNOWN_I16 || raw == 0 ? NAN : (int16_t)raw / 10.0f;
}

/* Scaled signed value, or NAN if the sensor marks it unknown. */
static float scale_s(uint16_t raw, float factor)
{
	return raw == UNKNOWN_I16 ? NAN : (int16_t)raw / factor;
}

esp_err_t sen66_init(i2c_master_bus_handle_t bus, const sen66_config_t *config,
                     sen66_handle_t *ret_handle)
{
	ESP_RETURN_ON_FALSE(bus && config && ret_handle, ESP_ERR_INVALID_ARG, TAG, "NULL argument");
	*ret_handle = NULL;
	ESP_RETURN_ON_FALSE(config->addr == SEN66_ADDR, ESP_ERR_INVALID_ARG, TAG,
	                    "invalid address 0x%02x", config->addr);

	esp_err_t ret = ESP_OK;
	struct sen66 *dev = calloc(1, sizeof(*dev));

	ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "no memory");

	uint8_t addr = config->addr;
	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = addr,
		.scl_speed_hz = I2C_SPEED_HZ,
	};
	ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus, &dev_cfg, &dev->i2c), fail, TAG,
	                  "add device 0x%02x", addr);

	/* reset works in any state, also if still measuring from before an ESP reset */
	ESP_GOTO_ON_FALSE(sensirion_write(dev->i2c, CMD_DEVICE_RESET, NULL, 0) == ESP_OK,
	                  ESP_ERR_NOT_FOUND, fail, TAG, "no response at 0x%02x", addr);
	sensirion_delay_ms(RESET_MS);

	/* product name: 32 bytes, NUL-terminated ASCII */
	uint16_t words[16];
	char name[33] = { 0 };

	ESP_GOTO_ON_FALSE(
	        sensirion_cmd_read(dev->i2c, CMD_GET_PRODUCT_NAME, CMD_MS, words, 16) == ESP_OK,
	        ESP_ERR_INVALID_RESPONSE, fail, TAG, "0x%02x: bad product name reply", addr);
	for (int i = 0; i < 16; i++) {
		name[2 * i] = (char)(words[i] >> 8);
		name[2 * i + 1] = (char)(words[i] & 0xFF);
	}
	ESP_GOTO_ON_FALSE(strncmp(name, "SEN66", 5) == 0, ESP_ERR_INVALID_RESPONSE, fail, TAG,
	                  "0x%02x is a \"%s\", not a SEN66", addr, name);

	sensirion_delay_ms(CMD_MS);
	ESP_GOTO_ON_ERROR(sensirion_write(dev->i2c, CMD_START, NULL, 0), fail, TAG, "start");
	sensirion_delay_ms(START_MS);

	ESP_LOGD(TAG, "ready at 0x%02x (%s)", addr, name);
	*ret_handle = dev;
	return ESP_OK;

fail:
	if (dev->i2c)
		i2c_master_bus_rm_device(dev->i2c);
	free(dev);
	return ret;
}

esp_err_t sen66_read(sen66_handle_t handle, sen66_data_t *out)
{
	ESP_RETURN_ON_FALSE(handle && out, ESP_ERR_INVALID_ARG, TAG, "NULL argument");

	uint16_t raw[9];

	ESP_RETURN_ON_ERROR(sensirion_cmd_read(handle->i2c, CMD_READ_VALUES, CMD_MS, raw, 9), TAG,
	                    "read");

	out->pm1_0 = scale_u(raw[0], 10.0f);
	out->pm2_5 = scale_u(raw[1], 10.0f);
	out->pm4_0 = scale_u(raw[2], 10.0f);
	out->pm10 = scale_u(raw[3], 10.0f);
	out->humidity_pct = scale_s(raw[4], 100.0f);
	out->temp_c = scale_s(raw[5], 200.0f);
	out->voc_index = scale_index(raw[6]);
	out->nox_index = scale_index(raw[7]);
	out->co2_ppm = scale_u(raw[8], 1.0f);
	return ESP_OK;
}

esp_err_t sen66_read_status(sen66_handle_t handle, uint32_t *status)
{
	ESP_RETURN_ON_FALSE(handle && status, ESP_ERR_INVALID_ARG, TAG, "NULL argument");

	uint16_t raw[2];

	ESP_RETURN_ON_ERROR(sensirion_cmd_read(handle->i2c, CMD_READ_STATUS, CMD_MS, raw, 2), TAG,
	                    "read status");
	*status = (uint32_t)raw[0] << 16 | raw[1];
	return ESP_OK;
}

esp_err_t sen66_set_pressure(sen66_handle_t handle, float pressure_pa)
{
	ESP_RETURN_ON_FALSE(handle && pressure_pa >= 70000.0f && pressure_pa <= 120000.0f,
	                    ESP_ERR_INVALID_ARG, TAG, "invalid args");

	uint16_t hpa = (uint16_t)lroundf(pressure_pa / 100.0f);

	ESP_RETURN_ON_ERROR(sensirion_write(handle->i2c, CMD_AMBIENT_PRESSURE, &hpa, 1), TAG,
	                    "set pressure");
	sensirion_delay_ms(CMD_MS);
	return ESP_OK;
}

void sen66_delete(sen66_handle_t handle)
{
	if (!handle)
		return;
	sensirion_write(handle->i2c, CMD_STOP, NULL, 0); /* best effort: fan off */
	i2c_master_bus_rm_device(handle->i2c);
	free(handle);
}
