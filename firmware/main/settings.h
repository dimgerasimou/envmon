#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Every runtime setting of the node. Empty string = not set.
 */
typedef struct {
	char wifi_ssid[33]; /*!< 802.11 limit: 32 bytes + NUL */
	char wifi_pass[65]; /*!< WPA2: 8..63 characters, or 64 hex digits */
	uint32_t period_ms; /*!< Sensor read period */
} settings_t;

/**
 * @brief Mount NVS and load the settings into RAM. Call once at boot.
 *
 * Missing values get their defaults. Erases NVS if it is full or was
 * written by an incompatible IDF version.
 *
 * @return ESP_OK, ESP_ERR_NO_MEM, or an NVS error
 */
esp_err_t settings_init(void);

/**
 * @brief Copy the current settings into @p out. Safe from any task.
 */
void settings_get(settings_t *out);

/**
 * @brief Save the WiFi name. An empty string removes it.
 *
 * @return ESP_OK, ESP_ERR_INVALID_ARG if NULL, ESP_ERR_INVALID_SIZE if too
 * long, or an NVS error
 */
esp_err_t settings_set_wifi_ssid(const char *ssid);

/**
 * @brief Save the WiFi password. An empty string removes it.
 *
 * @return ESP_OK, ESP_ERR_INVALID_ARG if NULL or not 8-64 characters,
 *         or an NVS error
 */
esp_err_t settings_set_wifi_pass(const char *pass);

/**
 * @brief Save the sensor read period. Does not change the running sensor task.
 *
 * @return ESP_OK, ESP_ERR_INVALID_ARG if outside SENSORS_PERIOD_MIN_MS..MAX_MS,
 *         or an NVS error
 */
esp_err_t settings_set_period(uint32_t period_ms);

/**
 * @brief Erase every saved setting. The RAM copy is not changed: reboot
 *        afterwards so settings_init() loads the defaults.
 *
 * @return ESP_OK or an NVS error
 */
esp_err_t settings_factory_reset(void);

#ifdef __cplusplus
}
#endif
