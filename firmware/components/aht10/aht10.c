#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "aht10.h"

/* commands, AHT10 datasheet section 5 */
#define CMD_RESET   0xBA
#define CMD_INIT    0xE1 /* + 0x08 0x00: load calibration */
#define CMD_MEASURE 0xAC /* + 0x33 0x00: start one measurement */

#define STATUS_BUSY       (1 << 7)
#define STATUS_CALIBRATED (1 << 3)

#define RESET_MS   20
#define INIT_MS    10
#define MEASURE_MS 80
#define POLL_MS    10
#define POLL_TRIES 5

#define I2C_SPEED_HZ   100000
#define I2C_TIMEOUT_MS 100

static const char *TAG = "aht10";

struct aht10 {
	i2c_master_dev_handle_t i2c;
};

static esp_err_t send_cmd(aht10_handle_t dev, uint8_t cmd, uint8_t arg0, uint8_t arg1)
{
	uint8_t buf[3] = { cmd, arg0, arg1 };

	return i2c_master_transmit(dev->i2c, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

static esp_err_t read_bytes(aht10_handle_t dev, uint8_t *buf, size_t len)
{
	return i2c_master_receive(dev->i2c, buf, len, I2C_TIMEOUT_MS);
}

esp_err_t aht10_init(i2c_master_bus_handle_t bus, const aht10_config_t *config,
                     aht10_handle_t *ret_handle)
{
	ESP_RETURN_ON_FALSE(bus && config && ret_handle, ESP_ERR_INVALID_ARG, TAG, "NULL argument");
	*ret_handle = NULL;
	ESP_RETURN_ON_FALSE(config->addr == AHT10_ADDR_LOW || config->addr == AHT10_ADDR_HIGH,
	                    ESP_ERR_INVALID_ARG, TAG, "invalid address 0x%02x", config->addr);

	esp_err_t ret = ESP_OK;
	struct aht10 *dev = calloc(1, sizeof(*dev));

	ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "no memory");

	uint8_t addr = config->addr;
	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = addr,
		.scl_speed_hz = I2C_SPEED_HZ,
	};
	ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(bus, &dev_cfg, &dev->i2c), fail, TAG,
	                  "add device 0x%02x", addr);

	/* soft reset; no ACK here means nothing is at this address */
	uint8_t reset = CMD_RESET;

	ESP_GOTO_ON_FALSE(i2c_master_transmit(dev->i2c, &reset, 1, I2C_TIMEOUT_MS) == ESP_OK,
	                  ESP_ERR_NOT_FOUND, fail, TAG, "no response at 0x%02x", addr);
	vTaskDelay(pdMS_TO_TICKS(RESET_MS));

	/* load calibration, then check the calibrated bit: the only identity check this chip has */
	uint8_t status;

	ESP_GOTO_ON_ERROR(send_cmd(dev, CMD_INIT, 0x08, 0x00), fail, TAG, "init command");
	vTaskDelay(pdMS_TO_TICKS(INIT_MS));
	ESP_GOTO_ON_ERROR(read_bytes(dev, &status, 1), fail, TAG, "read status");
	ESP_GOTO_ON_FALSE(status & STATUS_CALIBRATED, ESP_ERR_NOT_SUPPORTED, fail, TAG,
	                  "0x%02x not calibrated (status 0x%02x), not an AHT10?", addr, status);

	ESP_LOGD(TAG, "ready at 0x%02x", addr);
	*ret_handle = dev;
	return ESP_OK;

fail:
	aht10_delete(dev);
	return ret;
}

esp_err_t aht10_read(aht10_handle_t handle, float *temp_c, float *humidity_pct)
{
	ESP_RETURN_ON_FALSE(handle && temp_c && humidity_pct, ESP_ERR_INVALID_ARG, TAG,
	                    "NULL argument");

	ESP_RETURN_ON_ERROR(send_cmd(handle, CMD_MEASURE, 0x33, 0x00), TAG, "start");
	vTaskDelay(pdMS_TO_TICKS(MEASURE_MS));

	/* 6 bytes: status, 20-bit humidity, 20-bit temperature (sharing byte 3) */
	uint8_t d[6];
	int tries = 0;

	for (;;) {
		ESP_RETURN_ON_ERROR(read_bytes(handle, d, sizeof(d)), TAG, "read data");
		if (!(d[0] & STATUS_BUSY))
			break;
		ESP_RETURN_ON_FALSE(++tries < POLL_TRIES, ESP_ERR_TIMEOUT, TAG,
		                    "measurement timeout");
		vTaskDelay(pdMS_TO_TICKS(POLL_MS));
	}
	ESP_RETURN_ON_FALSE(d[0] & STATUS_CALIBRATED, ESP_ERR_INVALID_STATE, TAG,
	                    "lost calibration");

	uint32_t raw_h = ((uint32_t)d[1] << 12) | ((uint32_t)d[2] << 4) | (d[3] >> 4);
	uint32_t raw_t = (((uint32_t)d[3] & 0x0F) << 16) | ((uint32_t)d[4] << 8) | d[5];

	*humidity_pct = raw_h * 100.0f / (1 << 20);
	*temp_c = raw_t * 200.0f / (1 << 20) - 50.0f;
	return ESP_OK;
}

void aht10_delete(aht10_handle_t handle)
{
	if (!handle)
		return;
	if (handle->i2c)
		i2c_master_bus_rm_device(handle->i2c);
	free(handle);
}
