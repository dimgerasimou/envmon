#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the serial console on the USB port (runs in its own task).
 *
 * Commands: help, show, set, sensors. Call after settings_init() and
 * sensors_start().
 *
 * @return ESP_OK, or an error from esp_console
 */
esp_err_t console_start(void);

#ifdef __cplusplus
}
#endif
