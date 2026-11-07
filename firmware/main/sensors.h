
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SENSORS_PERIOD_MIN_MS 2000    /*!< Shortest allowed read period */
#define SENSORS_PERIOD_MAX_MS 3600000 /*!< Longest allowed read period (1 h) */

/**
 * @brief One measured quantity and the sensor that supplied it
 */
typedef struct {
	bool valid;      /*!< false when no sensor could supply it this cycle */
	float value;     /*!< Measured value, unit given by the field name */
	const char *src; /*!< Sensor that supplied the value, e.g. "aht10" */
} sensors_param_t;

/**
 * @brief Every quantity the firmware knows about
 */
typedef enum {
	SENSORS_TEMPERATURE, /*!< °C */
	SENSORS_HUMIDITY,    /*!< %RH */
	SENSORS_PRESSURE,    /*!< hPa */
	SENSORS_CO2,         /*!< Measured CO2, ppm (SEN66, STCC4) */
	SENSORS_PM1_0,       /*!< µg/m³ */
	SENSORS_PM2_5,       /*!< µg/m³ */
	SENSORS_PM4_0,       /*!< µg/m³ */
	SENSORS_PM10,        /*!< µg/m³ */
	SENSORS_VOC_INDEX,   /*!< Sensirion VOC index, 1..500, 100 = typical */
	SENSORS_NOX_INDEX,   /*!< Sensirion NOx index, 1..500, 1 = typical */
	SENSORS_AQI,         /*!< ENS160 UBA index, 1..5 */
	SENSORS_TVOC,        /*!< ENS160 total VOC, ppb */
	SENSORS_ECO2,        /*!< ENS160 CO2 estimated from VOCs, ppm */
	SENSORS_QTY_COUNT,
} sensors_qty_t;

/**
 * @brief Name and unit of a quantity, e.g. "temperature", "C"
 */
typedef struct {
	const char *name; /*!< snake_case, also usable as a JSON/MQTT key */
	const char *unit; /*!< ASCII unit, empty for indexes */
} sensors_qty_info_t;

/**
 * @brief One sample: the chosen value of every quantity
 *
 * Index with sensors_qty_t, e.g. data.qty[SENSORS_CO2]. A quantity stays
 * invalid when no sensor on this board measures it, or none could this cycle.
 */
typedef struct {
	sensors_param_t qty[SENSORS_QTY_COUNT];
} sensors_data_t;

#define SENSORS_MAX_READINGS 24 /*!< Enough for every sensor/quantity pair */

/**
 * @brief One value from one sensor, chosen or not
 */
typedef struct {
	sensors_qty_t qty;
	float value;
	const char *src;
} sensors_reading_t;

/**
 * @brief Every reading of one cycle, in priority order per quantity
 */
typedef struct {
	size_t count;
	sensors_reading_t r[SENSORS_MAX_READINGS];
} sensors_readings_t;

/**
 * @brief Start the periodic sensor task.
 *
 * Initializes the sensors enabled in the board config, then reads them every
 * period and publishes the latest sample to a single-slot queue. A sensor
 * that is missing or stops answering does not stop the task: its values are
 * marked invalid and it is retried in the background with a growing delay.
 *
 * The board I2C bus must be initialized first.
 *
 * @param period_ms  Time between reads, SENSORS_PERIOD_MIN_MS to SENSORS_PERIOD_MAX_MS.
 * @param ret_queue  Returned queue of sensors_data_t, read with xQueueReceive().
 * @return
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if ret_queue is NULL or period_ms is out of range
 *     - ESP_ERR_INVALID_STATE if already started
 *     - ESP_ERR_NOT_SUPPORTED if the board config enables no sensor
 *     - ESP_ERR_NO_MEM if the queue or task could not be created
 */
esp_err_t sensors_start(uint32_t period_ms, QueueHandle_t *ret_queue);

/**
 * @brief Change the read period.
 *
 * Safe to call from any task. If the sensor task is running, it reads
 * immediately and then continues with the new period.
 *
 * @param period_ms  SENSORS_PERIOD_MIN_MS to SENSORS_PERIOD_MAX_MS.
 * @return ESP_OK, or ESP_ERR_INVALID_ARG if out of range.
 */
esp_err_t sensors_set_period(uint32_t period_ms);

/**
 * @brief Copy the latest sample. Safe to call from any task.
 *
 * Unlike the queue, reading this does not consume the sample.
 *
 * @param best  Chosen value per quantity, or NULL
 * @param all   Every reading of the cycle including lower-priority ones, or NULL
 * @return
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if both are NULL
 *     - ESP_ERR_INVALID_STATE if sensors_start() has not run
 *     - ESP_ERR_NOT_FOUND if no sample has been taken yet
 */
esp_err_t sensors_get_latest(sensors_data_t *best, sensors_readings_t *all);

/**
 * @brief Name and unit of a quantity. Never NULL.
 */
const sensors_qty_info_t *sensors_qty_info(sensors_qty_t q);

/**
 * @brief Whether a sensor enabled in the board config can measure @p q.
 *
 * Fixed at build time. A supported quantity can still be invalid in a
 * sample if its sensors are offline.
 */
bool sensors_qty_supported(sensors_qty_t q);

/**
 * @brief Current read period in milliseconds.
 */
uint32_t sensors_get_period(void);

#ifdef __cplusplus
}
#endif
