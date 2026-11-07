#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include "esp_check.h"
#include "esp_log.h"
#include "ha.h"
#include "mqtt_client.h"
#include "mqtt_pub.h"
#include "settings.h"

#define TOPIC_MAX   64  /* "envmon/" + node_name + "/availability" */
#define PAYLOAD_MAX 512 /* every quantity at ~25 bytes is ~330 */
#define KEEPALIVE_S 30  /* broker sends our last will ~45 s after we vanish */

static const char *TAG = "mqtt";

static esp_mqtt_client_handle_t s_client; /* NULL until started */
static char s_topic_state[TOPIC_MAX];
static char s_topic_avail[TOPIC_MAX];
static char s_node[SETTINGS_NODE_NAME_MAX];

/* written by the MQTT task, read by main and the console */
static atomic_bool s_connected;
static atomic_uint s_sent;
static atomic_uint s_drops;

/* Runs in the MQTT client's own task. */
static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
	esp_mqtt_event_handle_t e = data;

	switch ((esp_mqtt_event_id_t)id) {
	case MQTT_EVENT_CONNECTED:
		ESP_LOGI(TAG, "connected");
		atomic_store(&s_connected, true);
		ha_publish_discovery(s_client, s_node, s_topic_state, s_topic_avail);
		/* retained, so it replaces the "offline" the broker may hold from our last will */
		esp_mqtt_client_publish(s_client, s_topic_avail, "online", 0, 1, 1);
		break;

	case MQTT_EVENT_DISCONNECTED:
		if (atomic_exchange(&s_connected, false)) { /* log once, not on every retry */
			atomic_fetch_add(&s_drops, 1);
			ESP_LOGW(TAG, "disconnected, retrying");
		}
		break;

	case MQTT_EVENT_ERROR:
		if (e->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED)
			ESP_LOGW(TAG, "broker refused us (code %d): check mqtt_user/mqtt_pass",
			         e->error_handle->connect_return_code);
		else if (e->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
			ESP_LOGW(TAG, "cannot reach broker (errno %d)",
			         e->error_handle->esp_transport_sock_errno);
		break;

	default:
		break;
	}
}

esp_err_t mqtt_pub_start(void)
{
	static settings_t cfg; /* static: ~350 bytes we only need once */
	char client_id[8 + SETTINGS_NODE_NAME_MAX];

	ESP_RETURN_ON_FALSE(!s_client, ESP_ERR_INVALID_STATE, TAG, "already started");
	settings_get(&cfg);
	if (!cfg.mqtt_uri[0])
		return ESP_ERR_NOT_FOUND; /* main tells the user what to do */

	strcpy(s_node, cfg.node_name); /* same buffer size */
	snprintf(s_topic_state, sizeof(s_topic_state), "envmon/%s/state", cfg.node_name);
	snprintf(s_topic_avail, sizeof(s_topic_avail), "envmon/%s/availability", cfg.node_name);
	snprintf(client_id, sizeof(client_id), "envmon-%s", cfg.node_name);

	const esp_mqtt_client_config_t mcfg = {
		.broker.address.uri = cfg.mqtt_uri,
		.credentials = {
			.client_id = client_id,
			.username = cfg.mqtt_user[0] ? cfg.mqtt_user : NULL, /* NULL = anonymous */
			.authentication.password = cfg.mqtt_pass[0] ? cfg.mqtt_pass : NULL,
		},
		.session = {
			.keepalive = KEEPALIVE_S,
			.last_will = {
				.topic = s_topic_avail,
				.msg = "offline",
				.qos = 1,
				.retain = 1,
			},
		},
	};

	/* init copies every string, so the locals above may go out of scope */
	s_client = esp_mqtt_client_init(&mcfg);
	ESP_RETURN_ON_FALSE(s_client, ESP_ERR_NO_MEM, TAG, "client init");
	ESP_RETURN_ON_ERROR(
	        esp_mqtt_client_register_event(s_client, MQTT_EVENT_ANY, on_event, NULL), TAG,
	        "register events");
	ESP_RETURN_ON_ERROR(esp_mqtt_client_start(s_client), TAG, "start");

	ESP_LOGI(TAG, "broker %s, topic %s", cfg.mqtt_uri, s_topic_state);
	return ESP_OK;
}

esp_err_t mqtt_pub_state(const sensors_data_t *d)
{
	char buf[PAYLOAD_MAX];
	size_t n = 0;
	bool first = true;

	if (!s_client || !atomic_load(&s_connected))
		return ESP_ERR_INVALID_STATE;

	buf[n++] = '{';
	for (sensors_qty_t q = 0; q < SENSORS_QTY_COUNT; q++) {
		if (!sensors_qty_supported(q))
			continue;

		const char *sep = first ? "" : ",";
		const char *key = sensors_qty_info(q)->name;
		int len;

		/* null: HA shows "unknown" instead of keeping a stale value */
		if (d->qty[q].valid)
			len = snprintf(buf + n, sizeof(buf) - n, "%s\"%s\":%.2f", sep, key,
			               d->qty[q].value);
		else
			len = snprintf(buf + n, sizeof(buf) - n, "%s\"%s\":null", sep, key);

		if (len < 0 || (size_t)len >= sizeof(buf) - n)
			return ESP_ERR_INVALID_SIZE;
		n += len;
		first = false;
	}
	if (n + 2 > sizeof(buf)) /* room for '}' and NUL */
		return ESP_ERR_INVALID_SIZE;
	buf[n++] = '}';
	buf[n] = '\0';

	/* QoS 0, not retained: a missed sample is replaced by the next one anyway */
	if (esp_mqtt_client_publish(s_client, s_topic_state, buf, n, 0, 0) < 0)
		return ESP_FAIL;

	atomic_fetch_add(&s_sent, 1);
	ESP_LOGD(TAG, "%s %s", s_topic_state, buf);
	return ESP_OK;
}

void mqtt_pub_get_status(mqtt_pub_status_t *out)
{
	out->started = s_client != NULL;
	out->connected = atomic_load(&s_connected);
	out->sent = atomic_load(&s_sent);
	out->drops = atomic_load(&s_drops);
}
