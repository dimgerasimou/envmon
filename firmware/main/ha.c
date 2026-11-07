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

esp_err_t ha_publish_discovery(esp_mqtt_client_handle_t client, const char *node,
                               const char *state_topic, const char *avail_topic)
{
	static char buf[PAYLOAD_MAX]; /* static: only the MQTT task calls this */
	char topic[TOPIC_MAX];
	const esp_app_desc_t *app = esp_app_get_description();
	esp_err_t ret = ESP_OK;
	int published = 0;

	for (sensors_qty_t q = 0; q < SENSORS_QTY_COUNT; q++) {
		const char *key = sensors_qty_info(q)->name;
		const ha_qty_t *ha = &s_ha[q];

		snprintf(topic, sizeof(topic), "homeassistant/sensor/envmon_%s/%s/config", node,
		         key);

		if (!sensors_qty_supported(q)) {
			/* empty retained message: HA deletes the entity if it existed */
			esp_mqtt_client_publish(client, topic, "", 0, 1, 1);
			continue;
		}

		size_t n = 0;
		bool overflow = false;

		append(buf, sizeof(buf), &n, &overflow,
		       "{\"name\":\"%s\",\"unique_id\":\"envmon_%s_%s\","
		       "\"state_topic\":\"%s\",\"value_template\":\"{{ value_json.%s }}\","
		       "\"availability_topic\":\"%s\",\"state_class\":\"measurement\","
		       "\"suggested_display_precision\":%d",
		       ha->name, node, key, state_topic, key, avail_topic, ha->precision);
		if (ha->device_class)
			append(buf, sizeof(buf), &n, &overflow, ",\"device_class\":\"%s\"",
			       ha->device_class);
		if (ha->unit)
			append(buf, sizeof(buf), &n, &overflow, ",\"unit_of_measurement\":\"%s\"",
			       ha->unit);
		/* same device block in every config: HA groups the entities under one device */
		append(buf, sizeof(buf), &n, &overflow,
		       ",\"device\":{\"identifiers\":[\"envmon_%s\"],\"name\":\"envmon %s\","
		       "\"manufacturer\":\"envmon\",\"model\":\"%s\",\"sw_version\":\"%s\"}}",
		       node, node, CONFIG_IDF_TARGET, app->version);

		if (overflow) {
			ESP_LOGE(TAG, "%s: config does not fit in %d bytes", key, PAYLOAD_MAX);
			ret = ESP_ERR_INVALID_SIZE;
			continue;
		}
		/* QoS 1 + retained: HA finds it even if it starts after us */
		if (esp_mqtt_client_publish(client, topic, buf, n, 1, 1) < 0) {
			ret = ESP_FAIL;
			continue;
		}
		published++;
	}

	ESP_LOGI(TAG, "discovery: %d sensors announced", published);
	return ret;
}
