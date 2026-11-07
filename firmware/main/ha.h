#pragma once

#include "mqtt_client.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief The node's own MQTT topics, which the HA configs point to
 */
typedef struct {
	const char *node;       /*!< Node name, e.g. "livingroom" */
	const char *state;      /*!< State JSON */
	const char *avail;      /*!< "online" / "offline" */
	const char *period;     /*!< Current read period in seconds, retained */
	const char *period_set; /*!< HA publishes the new period here */
} ha_topics_t;

/**
 * @brief Announce this node to Home Assistant (MQTT discovery).
 *
 * For every quantity this board supports, publishes a retained config to
 * homeassistant/sensor/envmon_<node>/<quantity>/config, so HA creates the
 * entity, reads its value from the state JSON and marks it unavailable
 * with the availability topic. For every quantity it does not support, an
 * empty retained message removes any old entity.
 *
 * Also announces a "Read period" number entity, under the device's
 * configuration section, that HA can change.
 *
 * Call on every MQTT connect: HA may have been reset meanwhile.
 *
 * @param client  Connected MQTT client
 * @param t       Topics of this node
 * @return ESP_OK, ESP_ERR_INVALID_SIZE if a config did not fit, or ESP_FAIL
 *         if the client could not send one
 */
esp_err_t ha_publish_discovery(esp_mqtt_client_handle_t client, const ha_topics_t *t);

#ifdef __cplusplus
}
#endif
