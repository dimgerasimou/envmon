#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ENS160_ADDR_LOW     0x52    /*!< ADDR pin to GND */
#define ENS160_ADDR_HIGH    0x53    /*!< ADDR pin to VCC (most breakout boards) */

/**
 * @brief How far the sensor trusts its own output
 */
typedef enum {
	ENS160_VALIDITY_NORMAL = 0,     /*!< Output is valid */
	ENS160_VALIDITY_WARMUP = 1,     /*!< About 3 minutes after every power-on */
	ENS160_VALIDITY_STARTUP = 2,    /*!< About 1 hour, once in a new sensor's life */
	ENS160_VALIDITY_INVALID = 3,    /*!< No valid output */
} ens160_validity_t;

/**
 * @brief One set of air quality values
 */
typedef struct {
	uint8_t aqi;                    /*!< UBA air quality index, 1 (excellent) to 5 (unhealthy) */
	uint16_t tvoc_ppb;              /*!< Total volatile organic compounds, ppb */
	uint16_t eco2_ppm;              /*!< Equivalent CO2, ppm: estimated from VOCs, not measured CO2 */
	ens160_validity_t validity;     /*!< Whether the values above can be trusted yet */
} ens160_data_t;

/**
 * @brief Sensor configuration
 */
typedef struct {
	uint8_t addr;   /*!< ENS160_ADDR_LOW or ENS160_ADDR_HIGH */
} ens160_config_t;

/**
 * @brief Default configuration: address 0x53
 */
#define ENS160_DEFAULT_CONFIG() { \
	.addr = ENS160_ADDR_HIGH, \
}

/**
 * @brief Handle of an ENS160 sensor
 */
typedef struct ens160 *ens160_handle_t;

/**
 * @brief Attach an ENS160 to an I2C bus, verify it and start continuous measurement
 *
 * The sensor then measures every second on its own; ens160_read() returns the latest values.
 *
 * @param[in]  bus         I2C master bus the sensor is connected to
 * @param[in]  config      Sensor configuration, start from ENS160_DEFAULT_CONFIG()
 * @param[out] ret_handle  Returned sensor handle, NULL on failure
 *
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: NULL argument or an address out of range
 *      - ESP_ERR_NO_MEM: Out of memory
 *      - ESP_ERR_NOT_FOUND: No device answered at config->addr
 *      - ESP_ERR_NOT_SUPPORTED: A device answered, but it is not an ENS160
 *      - Other: I2C error from the bus
 */
esp_err_t ens160_init(i2c_master_bus_handle_t bus, const ens160_config_t *config,
		      ens160_handle_t *ret_handle);

/**
 * @brief Give the sensor the ambient temperature and humidity it uses for compensation
 *
 * Call it with values from a real T/RH sensor before each ens160_read(). Without it
 * the sensor assumes 25 degC and 50 %RH.
 *
 * @param[in] handle        Sensor handle
 * @param[in] temp_c        Ambient temperature in degrees Celsius
 * @param[in] humidity_pct  Ambient relative humidity in percent
 *
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: NULL handle or values out of range
 *      - Other: I2C error from the bus
 */
esp_err_t ens160_set_compensation(ens160_handle_t handle, float temp_c, float humidity_pct);

/**
 * @brief Read the latest air quality values
 *
 * Does not block. Check out->validity: during warm-up the values are not reliable yet.
 *
 * @note Not thread-safe: call from one task only.
 *
 * @param[in]  handle  Sensor handle
 * @param[out] out     Latest values
 *
 * @return
 *      - ESP_OK: Success (see out->validity)
 *      - ESP_ERR_INVALID_ARG: An argument is NULL
 *      - ESP_ERR_INVALID_RESPONSE: The sensor reports an error (needs re-init)
 *      - Other: I2C error from the bus
 */
esp_err_t ens160_read(ens160_handle_t handle, ens160_data_t *out);

/**
 * @brief Put the sensor in deep sleep, detach it from the bus and free the handle
 *
 * @param[in] handle  Sensor handle, NULL is allowed
 */
void ens160_delete(ens160_handle_t handle);

#ifdef __cplusplus
}
#endif
