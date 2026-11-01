#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef struct {
	bool valid;
	float   value;
	const char *src;
} sensors_param_t;

typedef struct {
	sensors_param_t temperature_c;
	sensors_param_t humidity_pct;
	sensors_param_t pressure_hpa;
	sensors_param_t aqi;
	sensors_param_t tvoc_ppb;
	sensors_param_t eco2_ppm;
} sensors_data_t;

esp_err_t sensors_init(void);

esp_err_t sensors_read(sensors_data_t *data);

/**
 * @brief Start the periodic sensor task.
 *
 * Initializes the sensors, then reads them every @p period_ms and
 * publishes the latest sample to a single-slot queue.
 *
 * @param period_ms  Time between reads, in milliseconds.
 * @param ret_queue  Returned queue of sensors_data_t.
 * @return ESP_OK, ESP_ERR_INVALID_ARG, ESP_ERR_INVALID_STATE if already
 *         started, ESP_ERR_NO_MEM, or an error from sensors_init().
 */
esp_err_t sensors_start(uint32_t period_ms, QueueHandle_t *ret_queue);
