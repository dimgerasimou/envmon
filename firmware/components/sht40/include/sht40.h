#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SHT40_ADDR_A 0x44 /*!< SHT40-AD1B, most modules */
#define SHT40_ADDR_B 0x45 /*!< SHT40-BD1B */

/**
 * @brief Measurement precision: higher is less noisy but takes longer
 */
typedef enum {
	SHT40_PRECISION_HIGH,   /*!< 8.3 ms, repeatability 0.04 °C / 0.08 %RH */
	SHT40_PRECISION_MEDIUM, /*!< 4.5 ms */
	SHT40_PRECISION_LOW,    /*!< 1.6 ms */
} sht40_precision_t;

/**
 * @brief Sensor configuration
 */
typedef struct {
	uint8_t addr;                /*!< SHT40_ADDR_A or SHT40_ADDR_B */
	sht40_precision_t precision; /*!< Precision of every read */
} sht40_config_t;

/**
 * @brief Default configuration: address 0x44, high precision
 */
// clang-format off
#define SHT40_DEFAULT_CONFIG() {                \
	.addr = SHT40_ADDR_A,                   \
	.precision = SHT40_PRECISION_HIGH,      \
}
// clang-format on

/**
 * @brief Handle of an SHT40 sensor
 */
typedef struct sht40 *sht40_handle_t;

/**
 * @brief Add an SHT40 to an I2C bus, reset it and read its serial number.
 *
 * @note Not thread-safe.
 *
 * @param bus         I2C bus handle
 * @param config      Sensor configuration
 * @param ret_handle  Returned sensor handle
 * @return
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if an argument or the config is invalid
 *     - ESP_ERR_NOT_FOUND if nothing answers at the address
 *     - ESP_ERR_INVALID_RESPONSE if the device answers but not like an SHT40
 *     - ESP_ERR_NO_MEM if out of memory
 */
esp_err_t sht40_init(i2c_master_bus_handle_t bus, const sht40_config_t *config,
                     sht40_handle_t *ret_handle);

/**
 * @brief Take one measurement (blocks for about 10 ms at high precision).
 *
 * @note Not thread-safe.
 *
 * @param handle        Sensor handle
 * @param temp_c        Temperature, °C
 * @param humidity_pct  Relative humidity, %, clamped to 0..100
 * @return ESP_OK, ESP_ERR_INVALID_ARG, ESP_ERR_INVALID_CRC, or an I2C error
 */
esp_err_t sht40_read(sht40_handle_t handle, float *temp_c, float *humidity_pct);

/**
 * @brief Remove the sensor from the bus and free the handle.
 *
 * The SHT40 idles between measurements, so nothing is sent.
 */
void sht40_delete(sht40_handle_t handle);

#ifdef __cplusplus
}
#endif
