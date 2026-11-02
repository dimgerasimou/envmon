#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AHT10_ADDR_LOW  0x38 /*!< ADR pin to GND (most modules) */
#define AHT10_ADDR_HIGH 0x39 /*!< ADR pin to VCC */

/**
 * @brief Sensor configuration
 */
typedef struct {
	uint8_t addr; /*!< AHT10_ADDR_LOW or AHT10_ADDR_HIGH */
} aht10_config_t;

/**
 * @brief Default configuration: address 0x38
 */
#define AHT10_DEFAULT_CONFIG()                                                                     \
	{                                                                                          \
		.addr = AHT10_ADDR_LOW,                                                            \
	}

/**
 * @brief Handle of an AHT10 sensor
 */
typedef struct aht10 *aht10_handle_t;

/**
 * @brief Attach an AHT10 to an I2C bus, reset it and load its calibration
 *
 * @param[in]  bus         I2C master bus the sensor is connected to
 * @param[in]  config      Sensor configuration, start from AHT10_DEFAULT_CONFIG()
 * @param[out] ret_handle  Returned sensor handle, NULL on failure
 *
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: NULL argument or an address out of range
 *      - ESP_ERR_NO_MEM: Out of memory
 *      - ESP_ERR_NOT_FOUND: No device answered at config->addr
 *      - ESP_ERR_NOT_SUPPORTED: A device answered, but did not report itself calibrated
 */
esp_err_t aht10_init(i2c_master_bus_handle_t bus, const aht10_config_t *config,
                     aht10_handle_t *ret_handle);

/**
 * @brief Measure temperature and relative humidity
 *
 * Starts a measurement and blocks about 80 ms until it is done. Keep calls at
 * least 2 s apart to avoid self-heating.
 *
 * @note Not thread-safe: call from one task only.
 *
 * @param[in]  handle        Sensor handle
 * @param[out] temp_c        Temperature in degrees Celsius
 * @param[out] humidity_pct  Relative humidity in percent
 *
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: An argument is NULL
 *      - ESP_ERR_TIMEOUT: The measurement did not finish in time
 *      - ESP_ERR_INVALID_STATE: The sensor lost its calibration (needs re-init)
 *      - Other: I2C error from the bus
 */
esp_err_t aht10_read(aht10_handle_t handle, float *temp_c, float *humidity_pct);

/**
 * @brief Detach the sensor from the bus and free the handle
 *
 * @param[in] handle  Sensor handle, NULL is allowed
 */
void aht10_delete(aht10_handle_t handle);

#ifdef __cplusplus
}
#endif
