#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Where the WiFi connection is
 */
typedef enum {
	NET_OFF,        /*!< net_start() not called, or no SSID saved */
	NET_CONNECTING, /*!< Joining the network or waiting to retry */
	NET_CONNECTED,  /*!< Has an IP address */
} net_state_t;

/**
 * @brief Snapshot of the connection, for the console
 */
typedef struct {
	net_state_t state;
	char ip[16];          /*!< "192.168.1.42", empty unless connected */
	int8_t rssi;          /*!< dBm, 0 unless connected */
	uint8_t channel;      /*!< 0 unless connected */
	uint32_t disconnects; /*!< Since boot */
	uint8_t last_reason;  /*!< wifi_err_reason_t of the last disconnect, 0 = none */
} net_status_t;

/**
 * @brief Start WiFi in station mode and keep it connected.
 *
 * Connects to the network saved in settings (wifi_ssid, wifi_pass) and
 * reconnects on its own after any disconnect, waiting 1 s, 2 s, 4 s ... up
 * to 60 s between attempts. Returns without waiting for the connection.
 *
 * settings_init() and esp_event_loop_create_default() must be called first.
 *
 * @return
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_STATE if already started
 *     - ESP_ERR_NOT_FOUND if no SSID is saved (WiFi stays off)
 *     - ESP_ERR_NO_MEM, or an error from the WiFi or netif driver
 */
esp_err_t net_start(void);

/**
 * @brief Wait until the node has an IP address.
 *
 * @param timeout  Maximum time to wait, in ticks; portMAX_DELAY waits forever.
 * @return true if connected, false on timeout or if WiFi is off.
 */
bool net_wait_connected(TickType_t timeout);

/**
 * @brief Fill @p out with the current connection state. Safe from any task.
 */
void net_get_status(net_status_t *out);

/**
 * @brief Short text for a wifi_err_reason_t, e.g. "wrong password"
 */
const char *net_reason_str(uint8_t reason);

#ifdef __cplusplus
}
#endif
