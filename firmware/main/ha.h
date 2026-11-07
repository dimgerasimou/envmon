#pragma once

#include "mqtt_client.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Announce this node's sensors to Home Assistant (MQTT discovery).
 *
 * For every quantity this board supports, publishes a retained config to
 * homeassistant/sensor/envmon_<node>/<quantity>/config, so HA creates the
 * entity, reads its value from the state JSON and marks it unavailable
 * with the availability topic. For every quantity it does not support, an
 * empty retained message removes any old entity.
 *
 * Call on every MQTT connect: HA may have been reset meanwhile.
 *
 * @param client       Connected MQTT client
 * @param node         Node name, e.g. "livingroom"
 * @param state_topic  Where the state JSON is published
 * @param avail_topic  Where "online"/"offline" is published
 * @return ESP_OK, ESP_ERR_INVALID_SIZE if a config did not fit, or ESP_FAIL
 *         if the client could not send one
 */
esp_err_t ha_publish_discovery(esp_mqtt_client_handle_t client, const char *node,
                               const char *state_topic, const char *avail_topic);

#ifdef __cplusplus
}
#endif
