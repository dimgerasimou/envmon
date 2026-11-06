#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STCC4_ADDR_LOW  0x64 /*!< Default; DFRobot jumper on 0x64 */
#define STCC4_ADDR_HIGH 0x65

/**
 * @brief Sensor configuration
 */
typedef struct {
	uint8_t addr; /*!< STCC4_ADDR_LOW or STCC4_ADDR_HIGH */
} stcc4_config_t;

/**
 * @brief Default configuration: address 0x64
 */
#define STCC4_DEFAULT_CONFIG()                                                                     \
	{                                                                                          \
		.addr = STCC4_ADDR_LOW,                                                            \
	}

/**
 * @brief One measurement
 *
 * Temperature and humidity come from the SHT4x wired to the STCC4's own
 * bus (on the DFRobot board, with the compensation jumper on Enable).
 */
typedef struct {
	int16_t co2_ppm;    /*!< CO2 concentration, ppm */
	float temp_c;       /*!< Temperature, °C */
	float humidity_pct; /*!< Relative humidity, %, clamped to 0..100 */
} stcc4_data_t;

/**
 * @brief Handle of an STCC4 sensor
 */
typedef struct stcc4 *stcc4_handle_t;

/**
 * @brief Add an STCC4 to an I2C bus and start continuous measurement (1 s interval).
 *
 * Blocks for about 1.2 s: a measurement left running from before an ESP reset
 * is stopped first. The first result is ready about 1 s after this returns.
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
 *     - ESP_ERR_INVALID_RESPONSE if the device answers but not like an STCC4
 *     - ESP_ERR_NO_MEM if out of memory
 */
esp_err_t stcc4_init(i2c_master_bus_handle_t bus, const stcc4_config_t *config,
                     stcc4_handle_t *ret_handle);

/**
 * @brief Read the latest measurement.
 *
 * Read at most once per second; the sensor does not answer while no new
 * result is ready.
 *
 * @note Not thread-safe.
 *
 * @return
 *     - ESP_OK on success
 *     - ESP_ERR_NOT_FINISHED if called within 1.2 s of init (no result yet)
 *     - ESP_ERR_INVALID_ARG, ESP_ERR_INVALID_CRC, or an I2C error
 */
esp_err_t stcc4_read(stcc4_handle_t handle, stcc4_data_t *out);

/**
 * @brief Set the ambient pressure used to compensate the CO2 value.
 *
 * Without this call the sensor uses its built-in default. The value is
 * kept until the sensor loses power.
 *
 * @param handle       Sensor handle
 * @param pressure_pa  Ambient pressure, 40000 to 110000 Pa
 * @return ESP_OK, ESP_ERR_INVALID_ARG, or an I2C error
 */
esp_err_t stcc4_set_pressure(stcc4_handle_t handle, float pressure_pa);

/**
 * @brief Stop measuring, remove the sensor from the bus and free the handle.
 */
void stcc4_delete(stcc4_handle_t handle);

#ifdef __cplusplus
}
#endif
