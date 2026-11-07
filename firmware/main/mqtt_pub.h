#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "sensors.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the MQTT client and keep it connected.
 *
 * Connects to the broker saved in settings (mqtt_uri, mqtt_user, mqtt_pass)
 * as client "envmon-<node_name>". On every connect it publishes "online",
 * retained, to envmon/<node_name>/availability; the broker publishes
 * "offline" there for us (last will) if the node disappears. Also
 * (re)publishes the Home Assistant discovery configs on every connect. The client
 * reconnects on its own, so this can be called before WiFi is up.
 *
 * settings_init() and esp_event_loop_create_default() must be called first.
 *
 * @return
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_STATE if already started
 *     - ESP_ERR_NOT_FOUND if no mqtt_uri is saved (MQTT stays off)
 *     - ESP_ERR_NO_MEM, or an error from the MQTT client
 */
esp_err_t mqtt_pub_start(void);

/**
 * @brief Publish one sample as JSON to envmon/<node_name>/state.
 *
 * Every quantity this board supports is included, keyed by
 * sensors_qty_info()->name; one with no reading this cycle is null, e.g.
 * {"temperature":23.41,"humidity":45.20,"co2":null}.
 *
 * @return
 *     - ESP_OK when handed to the network stack
 *     - ESP_ERR_INVALID_STATE if not connected (the sample is dropped)
 *     - ESP_ERR_INVALID_SIZE if the JSON did not fit
 *     - ESP_FAIL if the client could not send it
 */
esp_err_t mqtt_pub_state(const sensors_data_t *d);

/**
 * @brief Connection snapshot, for the console
 */
typedef struct {
	bool started;   /*!< false: no mqtt_uri saved */
	bool connected; /*!< Connected to the broker right now */
	uint32_t sent;  /*!< State messages published since boot */
	uint32_t drops; /*!< Disconnects since boot */
} mqtt_pub_status_t;

/**
 * @brief Fill @p out with the current state. Safe from any task.
 */
void mqtt_pub_get_status(mqtt_pub_status_t *out);

#ifdef __cplusplus
}
#endif
