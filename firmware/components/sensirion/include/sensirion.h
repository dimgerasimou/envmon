#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Helpers for the I2C protocol shared by Sensirion sensors (SHT4x, STCC4, SEN6x):
 * big-endian 16-bit commands and words, every word followed by a CRC-8 byte.
 */

#define SENSIRION_MAX_ARGS  4  /*!< Most words sensirion_write() can send */
#define SENSIRION_MAX_WORDS 16 /*!< Most words sensirion_read() can receive */

/**
 * @brief CRC-8 used by Sensirion: polynomial 0x31, init 0xFF.
 */
uint8_t sensirion_crc8(const uint8_t *data, size_t len);

/**
 * @brief Sleep for at least @p ms milliseconds.
 *
 * Unlike vTaskDelay(pdMS_TO_TICKS(ms)), never rounds down to zero ticks.
 */
void sensirion_delay_ms(uint32_t ms);

/**
 * @brief Send a 16-bit command, optionally followed by argument words.
 *
 * @param dev     Device handle
 * @param cmd     Command code
 * @param args    Argument words, or NULL if n_args is 0
 * @param n_args  Number of argument words, at most SENSIRION_MAX_ARGS
 * @return ESP_OK, ESP_ERR_INVALID_ARG, or an I2C error
 */
esp_err_t sensirion_write(i2c_master_dev_handle_t dev, uint16_t cmd, const uint16_t *args,
                          size_t n_args);

/**
 * @brief Read words and check the CRC of each.
 *
 * @param dev      Device handle
 * @param words    Output, n_words values
 * @param n_words  Number of words, at most SENSIRION_MAX_WORDS
 * @return ESP_OK, ESP_ERR_INVALID_ARG, ESP_ERR_INVALID_CRC, or an I2C error
 */
esp_err_t sensirion_read(i2c_master_dev_handle_t dev, uint16_t *words, size_t n_words);

/**
 * @brief Send a command, wait, then read the reply.
 *
 * Write and read are separate transfers with a STOP between them; some
 * Sensirion sensors (SEN6x) answer 0xFF to a repeated start.
 */
esp_err_t sensirion_cmd_read(i2c_master_dev_handle_t dev, uint16_t cmd, uint32_t delay_ms,
                             uint16_t *words, size_t n_words);

#ifdef __cplusplus
}
#endif
