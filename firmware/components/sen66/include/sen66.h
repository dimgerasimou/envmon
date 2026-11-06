#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SEN66_ADDR 0x6B /*!< Fixed */

/* sen66_read_status() bits, SEN6x datasheet "Read Device Status" */
#define SEN66_STATUS_FAN_SPEED_WARNING (1UL << 21) /*!< Fan speed off by >10% */
#define SEN66_STATUS_CO2_ERROR         (1UL << 12) /*!< CO2 sensor error */
#define SEN66_STATUS_PM_ERROR          (1UL << 11) /*!< PM sensor error */
#define SEN66_STATUS_CO2_2_ERROR       (1UL << 9)  /*!< CO2 sensor error (second flag) */
#define SEN66_STATUS_GAS_ERROR         (1UL << 7)  /*!< VOC/NOx sensor error */
#define SEN66_STATUS_RHT_ERROR         (1UL << 6)  /*!< Temperature/humidity sensor error */
#define SEN66_STATUS_FAN_ERROR         (1UL << 4)  /*!< Fan stopped or blocked */

/**
 * @brief Sensor configuration
 */
typedef struct {
	uint8_t addr; /*!< SEN66_ADDR */
} sen66_config_t;

/**
 * @brief Default configuration
 */
#define SEN66_DEFAULT_CONFIG()                                                                     \
	{                                                                                          \
		.addr = SEN66_ADDR,                                                                \
	}

/**
 * @brief One measurement. A value the sensor does not know yet is NAN.
 *
 * After start: CO2 is unknown for about 6 s; the VOC and NOx indexes until
 * their algorithms start (about 11 s for NOx).
 */
typedef struct {
	float pm1_0;        /*!< PM1.0, µg/m³ */
	float pm2_5;        /*!< PM2.5, µg/m³ */
	float pm4_0;        /*!< PM4.0, µg/m³ */
	float pm10;         /*!< PM10, µg/m³ */
	float humidity_pct; /*!< Relative humidity, %, compensated for module self-heating */
	float temp_c;       /*!< Temperature, °C, compensated for module self-heating */
	float voc_index;    /*!< Sensirion VOC index, 1..500, 100 = typical */
	float nox_index;    /*!< Sensirion NOx index, 1..500, 1 = typical */
	float co2_ppm;      /*!< CO2 concentration, ppm */
} sen66_data_t;

/**
 * @brief Handle of a SEN66 sensor
 */
typedef struct sen66 *sen66_handle_t;

/**
 * @brief Add a SEN66 to an I2C bus, reset it and start measuring (fan on).
 *
 * Blocks for about 1.3 s (reset). New values are ready every second.
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
 *     - ESP_ERR_INVALID_RESPONSE if the device does not report itself as a SEN66
 *     - ESP_ERR_NO_MEM if out of memory
 */
esp_err_t sen66_init(i2c_master_bus_handle_t bus, const sen66_config_t *config,
                     sen66_handle_t *ret_handle);

/**
 * @brief Read the latest measurement (blocks for about 20 ms).
 *
 * @note Not thread-safe.
 *
 * @return ESP_OK, ESP_ERR_INVALID_ARG, ESP_ERR_INVALID_CRC, or an I2C error
 */
esp_err_t sen66_read(sen66_handle_t handle, sen66_data_t *out);

/**
 * @brief Read the device status flags (SEN66_STATUS_*). 0 means healthy.
 *
 * Error flags stay set until the next reset, so they need not be polled often.
 */
esp_err_t sen66_read_status(sen66_handle_t handle, uint32_t *status);

/**
 * @brief Set the ambient pressure used to compensate the CO2 value.
 *
 * @param handle       Sensor handle
 * @param pressure_pa  Ambient pressure, 70000 to 120000 Pa
 * @return ESP_OK, ESP_ERR_INVALID_ARG, or an I2C error
 */
esp_err_t sen66_set_pressure(sen66_handle_t handle, float pressure_pa);

/**
 * @brief Stop measuring (fan off), remove the sensor from the bus and free the handle.
 */
void sen66_delete(sen66_handle_t handle);

#ifdef __cplusplus
}
#endif
