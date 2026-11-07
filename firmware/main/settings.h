#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SETTINGS_NODE_NAME_MAX 24 /*!< Buffer size: 23 characters + NUL */

/**
 * @brief Every runtime setting of the node. Empty string = not set.
 */
typedef struct {
	char wifi_ssid[33];                     /*!< 802.11 limit: 32 bytes + NUL */
	char wifi_pass[65];                     /*!< WPA2: 8..63 characters, or 64 hex digits */
	uint32_t period_ms;                     /*!< Sensor read period */
	char node_name[SETTINGS_NODE_NAME_MAX]; /*!< a-z 0-9 '-', never empty */
	char mqtt_uri[128];                     /*!< e.g. mqtt://192.168.1.10:1883 */
	char mqtt_user[33];                     /*!< Empty = anonymous */
	char mqtt_pass[65];
} settings_t;

/**
 * @brief Mount NVS and load the settings into RAM. Call once at boot.
 *
 * Missing values get their defaults. node_name defaults to "node-" plus the
 * last three bytes of the WiFi MAC, e.g. "node-a1b2c3". Erases NVS if it is
 * full or was written by an incompatible IDF version.
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
 * @brief Save the node name, used in the hostname, MQTT topics and HA ids.
 *
 * Lowercase letters, digits and '-', 1 to 23 characters, not starting or
 * ending with '-'. An empty string goes back to the MAC-based default.
 *
 * @return ESP_OK, ESP_ERR_INVALID_ARG if NULL or not a valid name, or an NVS error
 */
esp_err_t settings_set_node_name(const char *name);

/**
 * @brief Save the MQTT broker address. An empty string removes it.
 *
 * @return ESP_OK, ESP_ERR_INVALID_ARG if NULL or not starting with mqtt://
 *         or mqtts://, ESP_ERR_INVALID_SIZE if too long, or an NVS error
 */
esp_err_t settings_set_mqtt_uri(const char *uri);

/**
 * @brief Save the MQTT user name. An empty string removes it.
 *
 * @return ESP_OK, ESP_ERR_INVALID_ARG if NULL, ESP_ERR_INVALID_SIZE if too
 *         long, or an NVS error
 */
esp_err_t settings_set_mqtt_user(const char *user);

/**
 * @brief Save the MQTT password. An empty string removes it.
 *
 * @return ESP_OK, ESP_ERR_INVALID_ARG if NULL, ESP_ERR_INVALID_SIZE if too
 *         long, or an NVS error
 */
esp_err_t settings_set_mqtt_pass(const char *pass);

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
