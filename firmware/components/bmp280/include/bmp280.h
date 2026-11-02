#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BMP280_ADDR_LOW  0x76 /*!< SDO pin to GND */
#define BMP280_ADDR_HIGH 0x77 /*!< SDO pin to VCC */

/**
 * @brief Oversampling: more samples per measurement, less noise, slower
 */
typedef enum {
	BMP280_OSR_X1 = 1,
	BMP280_OSR_X2,
	BMP280_OSR_X4,
	BMP280_OSR_X8,
	BMP280_OSR_X16,
} bmp280_osr_t;

/**
 * @brief IIR filter coefficient: smooths short disturbances such as a door slamming
 */
typedef enum {
	BMP280_FILTER_OFF = 0,
	BMP280_FILTER_2,
	BMP280_FILTER_4,
	BMP280_FILTER_8,
	BMP280_FILTER_16,
} bmp280_filter_t;

/**
 * @brief Power mode
 */
typedef enum {
	BMP280_MODE_FORCED, /*!< One measurement per bmp280_read(), sleeps in between */
	BMP280_MODE_NORMAL, /*!< Measures continuously; bmp280_read() returns the latest */
} bmp280_mode_t;

/**
 * @brief Pause between measurements in normal mode
 */
typedef enum {
	BMP280_STANDBY_0_5_MS = 0,
	BMP280_STANDBY_62_5_MS,
	BMP280_STANDBY_125_MS,
	BMP280_STANDBY_250_MS,
	BMP280_STANDBY_500_MS,
	BMP280_STANDBY_1000_MS,
	BMP280_STANDBY_2000_MS,
	BMP280_STANDBY_4000_MS,
} bmp280_standby_t;

/**
 * @brief Sensor configuration
 */
typedef struct {
	uint8_t addr;             /*!< BMP280_ADDR_LOW or BMP280_ADDR_HIGH */
	bmp280_mode_t mode;       /*!< Forced or normal mode */
	bmp280_osr_t temp_osr;    /*!< Temperature oversampling */
	bmp280_osr_t press_osr;   /*!< Pressure oversampling */
	bmp280_filter_t filter;   /*!< IIR filter coefficient */
	bmp280_standby_t standby; /*!< Standby time, normal mode only */
} bmp280_config_t;

/**
 * @brief Default configuration: forced mode, low noise, no filter
 */
#define BMP280_DEFAULT_CONFIG()                                                                    \
	{                                                                                          \
		.addr = BMP280_ADDR_LOW,                                                           \
		.mode = BMP280_MODE_FORCED,                                                        \
		.temp_osr = BMP280_OSR_X2,                                                         \
		.press_osr = BMP280_OSR_X16,                                                       \
		.filter = BMP280_FILTER_OFF,                                                       \
		.standby = BMP280_STANDBY_1000_MS,                                                 \
	}

/**
 * @brief Handle of a BMP280 sensor
 */
typedef struct bmp280 *bmp280_handle_t;

/**
 * @brief Attach a BMP280 to an I2C bus, verify it, load its calibration and apply the configuration
 *
 * @param[in]  bus         I2C master bus the sensor is connected to
 * @param[in]  config      Sensor configuration, start from BMP280_DEFAULT_CONFIG()
 * @param[out] ret_handle  Returned sensor handle, NULL on failure
 *
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: NULL argument or a configuration value out of range
 *      - ESP_ERR_NO_MEM: Out of memory
 *      - ESP_ERR_NOT_FOUND: No device answered at config->addr
 *      - ESP_ERR_NOT_SUPPORTED: A device answered, but it is not a BMP280
 *      - ESP_ERR_TIMEOUT: The sensor did not finish its reset
 */
esp_err_t bmp280_init(i2c_master_bus_handle_t bus, const bmp280_config_t *config,
                      bmp280_handle_t *ret_handle);

/**
 * @brief Read compensated temperature and pressure
 *
 * In forced mode this starts a measurement and blocks until it is done
 * (about 6 to 45 ms depending on oversampling). In normal mode it returns the
 * latest measurement immediately.
 *
 * @note Not thread-safe: call from one task only.
 *
 * @param[in]  handle    Sensor handle
 * @param[out] temp_c    Temperature in degrees Celsius
 * @param[out] press_pa  Absolute pressure in pascal
 *
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: An argument is NULL
 *      - ESP_ERR_INVALID_STATE: Normal mode has not finished its first measurement yet
 *      - ESP_ERR_TIMEOUT: The measurement did not finish in time
 *      - ESP_ERR_INVALID_RESPONSE: The data could not be compensated
 *      - Other: I2C error from the bus
 */
esp_err_t bmp280_read(bmp280_handle_t handle, float *temp_c, float *press_pa);

/**
 * @brief Put the sensor to sleep, detach it from the bus and free the handle
 *
 * @param[in] handle  Sensor handle, NULL is allowed
 */
void bmp280_delete(bmp280_handle_t handle);

#ifdef __cplusplus
}
#endif
