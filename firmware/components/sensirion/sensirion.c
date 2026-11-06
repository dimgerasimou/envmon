#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "sensirion.h"

#define I2C_TIMEOUT_MS 100

static const char *TAG = "sensirion";

uint8_t sensirion_crc8(const uint8_t *data, size_t len)
{
	uint8_t crc = 0xFF;

	for (size_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (int bit = 0; bit < 8; bit++)
			crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
	}
	return crc;
}

void sensirion_delay_ms(uint32_t ms)
{
	/* round up, plus one tick: the current tick is already partly over */
	vTaskDelay((ms + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS + 1);
}

esp_err_t sensirion_write(i2c_master_dev_handle_t dev, uint16_t cmd, const uint16_t *args,
                          size_t n_args)
{
	ESP_RETURN_ON_FALSE(dev && n_args <= SENSIRION_MAX_ARGS && (args || n_args == 0),
	                    ESP_ERR_INVALID_ARG, TAG, "invalid args");

	uint8_t buf[2 + 3 * SENSIRION_MAX_ARGS];
	size_t len = 0;

	buf[len++] = cmd >> 8;
	buf[len++] = cmd & 0xFF;
	for (size_t i = 0; i < n_args; i++) {
		buf[len++] = args[i] >> 8;
		buf[len++] = args[i] & 0xFF;
		buf[len] = sensirion_crc8(&buf[len - 2], 2);
		len++;
	}
	return i2c_master_transmit(dev, buf, len, I2C_TIMEOUT_MS);
}

esp_err_t sensirion_read(i2c_master_dev_handle_t dev, uint16_t *words, size_t n_words)
{
	ESP_RETURN_ON_FALSE(dev && words && n_words > 0 && n_words <= SENSIRION_MAX_WORDS,
	                    ESP_ERR_INVALID_ARG, TAG, "invalid args");

	uint8_t buf[3 * SENSIRION_MAX_WORDS];
	esp_err_t err = i2c_master_receive(dev, buf, 3 * n_words, I2C_TIMEOUT_MS);

	if (err != ESP_OK)
		return err;

	for (size_t i = 0; i < n_words; i++) {
		const uint8_t *p = &buf[3 * i];

		if (sensirion_crc8(p, 2) != p[2])
			return ESP_ERR_INVALID_CRC;
		words[i] = (uint16_t)(p[0] << 8 | p[1]);
	}
	return ESP_OK;
}

esp_err_t sensirion_cmd_read(i2c_master_dev_handle_t dev, uint16_t cmd, uint32_t delay_ms,
                             uint16_t *words, size_t n_words)
{
	esp_err_t err = sensirion_write(dev, cmd, NULL, 0);

	if (err != ESP_OK)
		return err;
	sensirion_delay_ms(delay_ms);
	return sensirion_read(dev, words, n_words);
}
