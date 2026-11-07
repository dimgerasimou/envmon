#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "net.h"
#include "settings.h"

#define CONNECTED_BIT BIT0
#define RETRY_MIN_MS  1000
#define RETRY_MAX_MS  60000
#define HOSTNAME      "envmon" /* becomes envmon-<node_name> in the next step */

static const char *TAG = "net";

static EventGroupHandle_t s_events; /* NULL until net_start() succeeds */
static TimerHandle_t s_retry_timer;
static esp_netif_t *s_netif;
static uint32_t s_retry_ms = RETRY_MIN_MS; /* only touched by the event task */

/* written by the event task, read by the console: guard with a spinlock */
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_disconnects;
static uint8_t s_last_reason;

const char *net_reason_str(uint8_t reason)
{
	switch (reason) {
	case 0:
		return "none";
	case WIFI_REASON_NO_AP_FOUND:
	case WIFI_REASON_NO_AP_FOUND_W_COMPATIBLE_SECURITY:
	case WIFI_REASON_NO_AP_FOUND_IN_AUTHMODE_THRESHOLD:
	case WIFI_REASON_NO_AP_FOUND_IN_RSSI_THRESHOLD:
		return "network not found";
	case WIFI_REASON_AUTH_FAIL:
	case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
	case WIFI_REASON_HANDSHAKE_TIMEOUT:
		return "wrong password?";
	case WIFI_REASON_BEACON_TIMEOUT:
		return "signal lost";
	case WIFI_REASON_ASSOC_LEAVE:
		return "left";
	default:
		return "other";
	}
}

/* Runs in the FreeRTOS timer task when the retry delay is over. */
static void retry_cb(TimerHandle_t timer)
{
	ESP_LOGI(TAG, "reconnecting");
	esp_wifi_connect();
}

/* Runs in the default event loop task. Must not block. */
static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
	if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
		esp_wifi_connect();

	} else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
		const wifi_event_sta_disconnected_t *e = data;

		xEventGroupClearBits(s_events, CONNECTED_BIT);
		taskENTER_CRITICAL(&s_mux);
		s_disconnects++;
		s_last_reason = e->reason;
		taskEXIT_CRITICAL(&s_mux);

		ESP_LOGW(TAG, "disconnected: %s (%d), retry in %lu s", net_reason_str(e->reason),
		         e->reason, (unsigned long)(s_retry_ms / 1000));
		/* on a stopped timer this also starts it */
		xTimerChangePeriod(s_retry_timer, pdMS_TO_TICKS(s_retry_ms), 0);
		s_retry_ms = s_retry_ms >= RETRY_MAX_MS / 2 ? RETRY_MAX_MS : s_retry_ms * 2;

	} else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
		const ip_event_got_ip_t *e = data;

		ESP_LOGI(TAG, "connected, IP " IPSTR, IP2STR(&e->ip_info.ip));
		s_retry_ms = RETRY_MIN_MS;
		xEventGroupSetBits(s_events, CONNECTED_BIT);
	}
}

esp_err_t net_start(void)
{
	static settings_t cfg; /* static: ~100 bytes we only need once */

	ESP_RETURN_ON_FALSE(!s_events, ESP_ERR_INVALID_STATE, TAG, "already started");
	settings_get(&cfg);
	if (!cfg.wifi_ssid[0])
		return ESP_ERR_NOT_FOUND; /* main tells the user what to do */

	EventGroupHandle_t events = xEventGroupCreate();

	s_retry_timer = xTimerCreate("wifi_retry", 1, pdFALSE, NULL, retry_cb);
	ESP_RETURN_ON_FALSE(events && s_retry_timer, ESP_ERR_NO_MEM, TAG, "alloc failed");

	/* TCP/IP stack and the station network interface (DHCP client) */
	ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif init");
	s_netif = esp_netif_create_default_wifi_sta();
	ESP_RETURN_ON_FALSE(s_netif, ESP_FAIL, TAG, "create netif");
	ESP_RETURN_ON_ERROR(esp_netif_set_hostname(s_netif, HOSTNAME), TAG, "hostname");

	/* WiFi driver */
	wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();

	ESP_RETURN_ON_ERROR(esp_wifi_init(&init_cfg), TAG, "wifi init");
	/* credentials live in our settings; don't let the driver keep its own copy in NVS */
	ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "storage");

	s_events = events; /* set before any event can arrive */
	ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
	                                                        on_event, NULL, NULL),
	                    TAG, "register wifi events");
	ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
	                                                        on_event, NULL, NULL),
	                    TAG, "register ip events");

	wifi_config_t wcfg = { 0 };

	/* sta.ssid is 32 bytes with no room for NUL; a 32-character SSID fills it exactly */
	memcpy(wcfg.sta.ssid, cfg.wifi_ssid, sizeof(wcfg.sta.ssid));
	memcpy(wcfg.sta.password, cfg.wifi_pass, sizeof(wcfg.sta.password));
	/* with a password, refuse to join an open or WEP network of the same name */
	wcfg.sta.threshold.authmode = cfg.wifi_pass[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
	wcfg.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH; /* also join WPA3 networks */

	ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "set mode");
	ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wcfg), TAG, "set config");
	ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "start"); /* fires WIFI_EVENT_STA_START */

	ESP_LOGI(TAG, "connecting to \"%s\" as %s", cfg.wifi_ssid, HOSTNAME);
	return ESP_OK;
}

bool net_wait_connected(TickType_t timeout)
{
	if (!s_events)
		return false;
	return xEventGroupWaitBits(s_events, CONNECTED_BIT, pdFALSE, pdTRUE, timeout) &
	       CONNECTED_BIT;
}

void net_get_status(net_status_t *out)
{
	memset(out, 0, sizeof(*out));
	if (!s_events)
		return; /* NET_OFF */

	taskENTER_CRITICAL(&s_mux);
	out->disconnects = s_disconnects;
	out->last_reason = s_last_reason;
	taskEXIT_CRITICAL(&s_mux);

	if (!(xEventGroupGetBits(s_events) & CONNECTED_BIT)) {
		out->state = NET_CONNECTING;
		return;
	}
	out->state = NET_CONNECTED;

	esp_netif_ip_info_t ip;
	wifi_ap_record_t ap;

	if (esp_netif_get_ip_info(s_netif, &ip) == ESP_OK)
		snprintf(out->ip, sizeof(out->ip), IPSTR, IP2STR(&ip.ip));
	if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
		out->rssi = ap.rssi;
		out->channel = ap.primary;
	}
}
