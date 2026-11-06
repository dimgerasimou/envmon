#pragma once

#include <stdbool.h>
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
 * @brief One sample of every quantity the node can measure
 *
 * A field stays invalid when no sensor on this board measures it, or none
 * could this cycle.
 */
typedef struct {
	sensors_param_t temperature_c; /*!< °C */
	sensors_param_t humidity_pct;  /*!< %RH */
	sensors_param_t pressure_hpa;  /*!< hPa */
	sensors_param_t co2_ppm;       /*!< Measured CO2, ppm (SEN66, STCC4) */
	sensors_param_t pm1_0;         /*!< µg/m³ */
	sensors_param_t pm2_5;         /*!< µg/m³ */
	sensors_param_t pm4_0;         /*!< µg/m³ */
	sensors_param_t pm10;          /*!< µg/m³ */
	sensors_param_t voc_index;     /*!< Sensirion VOC index, 1..500, 100 = typical */
	sensors_param_t nox_index;     /*!< Sensirion NOx index, 1..500, 1 = typical */
	sensors_param_t aqi;           /*!< ENS160 UBA index, 1..5 */
	sensors_param_t tvoc_ppb;      /*!< ENS160 total VOC, ppb */
	sensors_param_t eco2_ppm;      /*!< ENS160 CO2 estimated from VOCs, ppm */
} sensors_data_t;

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
 * @brief Current read period in milliseconds.
 */
uint32_t sensors_get_period(void);

#ifdef __cplusplus
}
#endif
