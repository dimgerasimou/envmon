#include <stdarg.h>
#include <stdio.h>
#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_log.h"
#include "ha.h"
#include "sensors.h"
#include "sdkconfig.h"

#define TOPIC_MAX   96
#define PAYLOAD_MAX 768 /* a config is ~470 bytes; MQTT out buffer is 1024 */

static const char *TAG = "ha";

/*
 * How HA should show each quantity. device_class picks the icon, the unit
 * conversions and the graph type; NULL = generic sensor. Units must be the
 * exact strings HA expects for that device class.
 */
typedef struct {
	const char *name;         /* entity name, shown after the device name */
	const char *device_class; /* NULL = none */
	const char *unit;         /* NULL = no unit (indexes) */
	int precision;            /* decimals HA shows by default */
} ha_qty_t;

static const ha_qty_t s_ha[SENSORS_QTY_COUNT] = {
	[SENSORS_TEMPERATURE] = { "Temperature", "temperature", "°C", 1 },
	[SENSORS_HUMIDITY] = { "Humidity", "humidity", "%", 1 },
	[SENSORS_PRESSURE] = { "Pressure", "atmospheric_pressure", "hPa", 1 },
	[SENSORS_CO2] = { "CO2", "carbon_dioxide", "ppm", 0 },
	[SENSORS_PM1_0] = { "PM1", "pm1", "µg/m³", 1 },
	[SENSORS_PM2_5] = { "PM2.5", "pm25", "µg/m³", 1 },
	[SENSORS_PM4_0] = { "PM4", NULL, "µg/m³", 1 },
	[SENSORS_PM10] = { "PM10", "pm10", "µg/m³", 1 },
	[SENSORS_VOC_INDEX] = { "VOC index", NULL, NULL, 0 },
	[SENSORS_NOX_INDEX] = { "NOx index", NULL, NULL, 0 },
	[SENSORS_AQI] = { "Air quality", "aqi", NULL, 0 },
	[SENSORS_TVOC] = { "TVOC", "volatile_organic_compounds_parts", "ppb", 0 },
	[SENSORS_ECO2] = { "eCO2", "carbon_dioxide", "ppm", 0 },
};

/* snprintf that appends at *n and remembers if anything did not fit */
static void append(char *buf, size_t size, size_t *n, bool *overflow, const char *fmt, ...)
{
	va_list ap;

	if (*overflow)
		return;
	va_start(ap, fmt);
	int len = vsnprintf(buf + *n, size - *n, fmt, ap);
	va_end(ap);

	if (len < 0 || (size_t)len >= size - *n)
		*overflow = true;
	else
		*n += len;
}

/* Same device block in every config: HA groups the entities under one device. */
static void append_device(char *buf, size_t size, size_t *n, bool *overflow, const char *node)
{
	const esp_app_desc_t *app = esp_app_get_description();

	append(buf, size, n, overflow,
	       ",\"device\":{\"identifiers\":[\"envmon_%s\"],\"name\":\"envmon %s\","
	       "\"manufacturer\":\"envmon\",\"model\":\"%s\",\"sw_version\":\"%s\"}}",
	       node, node, CONFIG_IDF_TARGET, app->version);
}

/* QoS 1 + retained: HA finds it even if it starts after us */
static esp_err_t send(esp_mqtt_client_handle_t client, const char *topic, const char *buf, size_t n,
                      bool overflow)
{
	if (overflow) {
		ESP_LOGE(TAG, "%s: config does not fit in %d bytes", topic, PAYLOAD_MAX);
		return ESP_ERR_INVALID_SIZE;
	}
	if (esp_mqtt_client_publish(client, topic, buf, n, 1, 1) < 0)
		return ESP_FAIL;
	return ESP_OK;
}

static esp_err_t publish_sensor(esp_mqtt_client_handle_t client, const ha_topics_t *t,
                                sensors_qty_t q, char *buf, size_t size)
{
	const char *key = sensors_qty_info(q)->name;
	const ha_qty_t *ha = &s_ha[q];
	char topic[TOPIC_MAX];
	size_t n = 0;
	bool overflow = false;

	snprintf(topic, sizeof(topic), "homeassistant/sensor/envmon_%s/%s/config", t->node, key);

	if (!sensors_qty_supported(q)) {
		/* empty retained message: HA deletes the entity if it existed */
		esp_mqtt_client_publish(client, topic, "", 0, 1, 1);
		return ESP_ERR_NOT_SUPPORTED;
	}

	append(buf, size, &n, &overflow,
	       "{\"name\":\"%s\",\"unique_id\":\"envmon_%s_%s\","
	       "\"state_topic\":\"%s\",\"value_template\":\"{{ value_json.%s }}\","
	       "\"availability_topic\":\"%s\",\"state_class\":\"measurement\","
	       "\"suggested_display_precision\":%d",
	       ha->name, t->node, key, t->state, key, t->avail, ha->precision);
	if (ha->device_class)
		append(buf, size, &n, &overflow, ",\"device_class\":\"%s\"", ha->device_class);
	if (ha->unit)
		append(buf, size, &n, &overflow, ",\"unit_of_measurement\":\"%s\"", ha->unit);
	append_device(buf, size, &n, &overflow, t->node);
	return send(client, topic, buf, n, overflow);
}

/*
 * A number entity: HA shows a box, publishes what the user types to
 * command_topic, and shows whatever we publish back to state_topic. So a
 * value we reject snaps back to the real one.
 */
static esp_err_t publish_period(esp_mqtt_client_handle_t client, const ha_topics_t *t, char *buf,
                                size_t size)
{
	char topic[TOPIC_MAX];
	size_t n = 0;
	bool overflow = false;

	snprintf(topic, sizeof(topic), "homeassistant/number/envmon_%s/period/config", t->node);
	append(buf, size, &n, &overflow,
	       "{\"name\":\"Read period\",\"unique_id\":\"envmon_%s_period\","
	       "\"command_topic\":\"%s\",\"state_topic\":\"%s\","
	       "\"availability_topic\":\"%s\",\"min\":%d,\"max\":%d,\"step\":1,"
	       "\"mode\":\"box\",\"unit_of_measurement\":\"s\",\"device_class\":\"duration\","
	       "\"entity_category\":\"config\"",
	       t->node, t->period_set, t->period, t->avail, SENSORS_PERIOD_MIN_MS / 1000,
	       SENSORS_PERIOD_MAX_MS / 1000);
	append_device(buf, size, &n, &overflow, t->node);
	return send(client, topic, buf, n, overflow);
}

esp_err_t ha_publish_discovery(esp_mqtt_client_handle_t client, const ha_topics_t *t)
{
	static char buf[PAYLOAD_MAX]; /* static: only the MQTT task calls this */
	esp_err_t ret = ESP_OK;
	int published = 0;

	for (sensors_qty_t q = 0; q < SENSORS_QTY_COUNT; q++) {
		esp_err_t err = publish_sensor(client, t, q, buf, sizeof(buf));

		if (err == ESP_OK)
			published++;
		else if (err != ESP_ERR_NOT_SUPPORTED)
			ret = err;
	}

	esp_err_t err = publish_period(client, t, buf, sizeof(buf));

	if (err != ESP_OK)
		ret = err;

	ESP_LOGI(TAG, "discovery: %d sensors + read period announced", published);
	return ret;
}
