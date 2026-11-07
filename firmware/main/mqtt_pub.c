#include <math.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
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
static char s_topic_period[TOPIC_MAX];
static char s_topic_period_set[TOPIC_MAX];
static char s_node[SETTINGS_NODE_NAME_MAX];
static ha_topics_t s_ha_topics;

/* written by the MQTT task, read by main and the console */
static atomic_bool s_connected;
static atomic_uint s_sent;
static atomic_uint s_drops;
static atomic_uint s_period_sent; /* last period published, 0 = must publish */

/* Retained, so HA shows the real period even if it starts after us. */
static void publish_period(void)
{
	char txt[12];
	uint32_t s = sensors_get_period() / 1000;

	snprintf(txt, sizeof(txt), "%lu", (unsigned long)s);
	if (esp_mqtt_client_publish(s_client, s_topic_period, txt, 0, 1, 1) >= 0)
		atomic_store(&s_period_sent, s);
}

/* HA sent a new period, as text in seconds, e.g. "60" (or "60.0") */
static void on_period_set(const char *data, int len)
{
	char txt[16];
	char *end;

	if (len <= 0 || len >= (int)sizeof(txt)) {
		ESP_LOGW(TAG, "period: bad payload");
		publish_period();
		return;
	}
	memcpy(txt, data, len); /* MQTT payloads are not NUL-terminated */
	txt[len] = '\0';

	float s = strtof(txt, &end);
	esp_err_t err = ESP_ERR_INVALID_ARG;

	/* range-check the float first, so a huge value can't overflow the cast */
	if (end != txt && *end == '\0' && s >= SENSORS_PERIOD_MIN_MS / 1000.0f &&
	    s <= SENSORS_PERIOD_MAX_MS / 1000.0f) {
		uint32_t ms = (uint32_t)lroundf(s) * 1000;

		err = settings_set_period(ms); /* saved: survives a reboot */
		if (err == ESP_OK)
			err = sensors_set_period(ms); /* applied now */
	}

	if (err == ESP_OK)
		ESP_LOGI(TAG, "period set to %s s from Home Assistant", txt);
	else
		ESP_LOGW(TAG, "period '%s' rejected: %s", txt, esp_err_to_name(err));
	publish_period(); /* echo the real value: a rejected one snaps back in HA */
}

static bool topic_is(const esp_mqtt_event_t *e, const char *topic)
{
	return e->topic_len == (int)strlen(topic) && memcmp(e->topic, topic, e->topic_len) == 0;
}

/* Runs in the MQTT client's own task. */
static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
	esp_mqtt_event_handle_t e = data;

	switch ((esp_mqtt_event_id_t)id) {
	case MQTT_EVENT_CONNECTED:
		ESP_LOGI(TAG, "connected");
		atomic_store(&s_connected, true);
		ha_publish_discovery(s_client, &s_ha_topics);
		/* clean session: the broker forgets subscriptions, so subscribe on every connect */
		esp_mqtt_client_subscribe(s_client, s_topic_period_set, 1);
		publish_period();
		/* retained, so it replaces the "offline" the broker may hold from our last will */
		esp_mqtt_client_publish(s_client, s_topic_avail, "online", 0, 1, 1);
		break;

	case MQTT_EVENT_DISCONNECTED:
		if (atomic_exchange(&s_connected, false)) { /* log once, not on every retry */
			atomic_fetch_add(&s_drops, 1);
			ESP_LOGW(TAG, "disconnected, retrying");
		}
		break;

	case MQTT_EVENT_DATA:
		/* small payloads arrive in one piece; current_data_offset > 0 = a fragment */
		if (e->current_data_offset == 0 && topic_is(e, s_topic_period_set))
			on_period_set(e->data, e->data_len);
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
	snprintf(s_topic_period, sizeof(s_topic_period), "envmon/%s/period", cfg.node_name);
	snprintf(s_topic_period_set, sizeof(s_topic_period_set), "envmon/%s/period/set",
	         cfg.node_name);
	snprintf(client_id, sizeof(client_id), "envmon-%s", cfg.node_name);

	s_ha_topics = (ha_topics_t){
		.node = s_node,
		.state = s_topic_state,
		.avail = s_topic_avail,
		.period = s_topic_period,
		.period_set = s_topic_period_set,
	};

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
	/* period changed from the console (or a publish failed): tell HA */
	if (atomic_load(&s_period_sent) != sensors_get_period() / 1000)
		publish_period();
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
