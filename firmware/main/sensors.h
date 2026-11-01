#pragma once

#include <stdbool.h>
#include "esp_err.h"

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
