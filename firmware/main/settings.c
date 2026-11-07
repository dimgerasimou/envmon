#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sensors.h"
#include "settings.h"

#define NVS_NAMESPACE "envmon"
#define KEY_WIFI_SSID "wifi_ssid" /* NVS keys: max 15 characters */
#define KEY_WIFI_PASS "wifi_pass"
#define KEY_PERIOD    "period_ms"

static const char *TAG = "settings";

static settings_t s_cfg;         /* RAM copy: reads never touch flash */
static SemaphoreHandle_t s_lock; /* guards s_cfg */

static bool period_valid(uint32_t period_ms)
{
	return period_ms >= SENSORS_PERIOD_MIN_MS && period_ms <= SENSORS_PERIOD_MAX_MS;
}

/* Read one string; missing or broken -> empty. */
static void load_str(nvs_handle_t h, const char *key, char *dst, size_t size)
{
	size_t len = size; /* in: buffer size, out: bytes read */
	esp_err_t err = nvs_get_str(h, key, dst, &len);

	if (err != ESP_OK) {
		if (err != ESP_ERR_NVS_NOT_FOUND)
			ESP_LOGW(TAG, "%s: %s", key, esp_err_to_name(err));
		dst[0] = '\0';
	}
}

/* Save one string to NVS, then to RAM. Empty value removes the key. */
static esp_err_t save_str(const char *key, char *dst, size_t size, const char *value)
{
	ESP_RETURN_ON_FALSE(value, ESP_ERR_INVALID_ARG, TAG, "NULL value");
	ESP_RETURN_ON_FALSE(strlen(value) < size, ESP_ERR_INVALID_SIZE, TAG, "%s too long", key);

	xSemaphoreTake(s_lock, portMAX_DELAY);
	bool same = strcmp(dst, value) == 0;
	xSemaphoreGive(s_lock);
	if (same)
		return ESP_OK; /* unchanged: skip the flash write */

	nvs_handle_t h;

	ESP_RETURN_ON_ERROR(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h), TAG, "open");

	esp_err_t err = value[0] ? nvs_set_str(h, key, value) : nvs_erase_key(h, key);

	if (err == ESP_ERR_NVS_NOT_FOUND) /* erasing a key that was never saved */
		err = ESP_OK;
	if (err == ESP_OK)
		err = nvs_commit(h);
	nvs_close(h);
	ESP_RETURN_ON_ERROR(err, TAG, "save %s", key);

	/* flash first, then RAM: if the save fails, RAM still matches flash */
	xSemaphoreTake(s_lock, portMAX_DELAY);
	strcpy(dst, value); /* length checked above */
	xSemaphoreGive(s_lock);
	return ESP_OK;
}

esp_err_t settings_init(void)
{
	esp_err_t err = nvs_flash_init();

	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_LOGW(TAG, "erasing NVS (%s)", esp_err_to_name(err));
		ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "erase");
		err = nvs_flash_init();
	}
	ESP_RETURN_ON_ERROR(err, TAG, "nvs init");

	s_lock = xSemaphoreCreateMutex();
	ESP_RETURN_ON_FALSE(s_lock, ESP_ERR_NO_MEM, TAG, "mutex");

	/* defaults first, then overwrite with whatever is saved */
	memset(&s_cfg, 0, sizeof(s_cfg));
	s_cfg.period_ms = CONFIG_ENVMON_SENSOR_PERIOD_S * 1000;

	nvs_handle_t h;

	err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
	if (err == ESP_ERR_NVS_NOT_FOUND) {
		ESP_LOGI(TAG, "nothing saved yet, using defaults");
		return ESP_OK;
	}
	ESP_RETURN_ON_ERROR(err, TAG, "open");

	load_str(h, KEY_WIFI_SSID, s_cfg.wifi_ssid, sizeof(s_cfg.wifi_ssid));
	load_str(h, KEY_WIFI_PASS, s_cfg.wifi_pass, sizeof(s_cfg.wifi_pass));

	uint32_t period;

	if (nvs_get_u32(h, KEY_PERIOD, &period) == ESP_OK && period_valid(period))
		s_cfg.period_ms = period;

	nvs_close(h);
	return ESP_OK;
}

void settings_get(settings_t *out)
{
	xSemaphoreTake(s_lock, portMAX_DELAY);
	*out = s_cfg;
	xSemaphoreGive(s_lock);
}

esp_err_t settings_set_wifi_ssid(const char *ssid)
{
	return save_str(KEY_WIFI_SSID, s_cfg.wifi_ssid, sizeof(s_cfg.wifi_ssid), ssid);
}

esp_err_t settings_set_wifi_pass(const char *pass)
{
	ESP_RETURN_ON_FALSE(pass, ESP_ERR_INVALID_ARG, TAG, "NULL password");

	size_t len = strlen(pass);

	ESP_RETURN_ON_FALSE(len == 0 || (len >= 8 && len <= 64), ESP_ERR_INVALID_ARG, TAG,
	                    "password must be 8-64 characters");
	return save_str(KEY_WIFI_PASS, s_cfg.wifi_pass, sizeof(s_cfg.wifi_pass), pass);
}

esp_err_t settings_set_period(uint32_t period_ms)
{
	ESP_RETURN_ON_FALSE(period_valid(period_ms), ESP_ERR_INVALID_ARG, TAG,
	                    "period out of range");

	xSemaphoreTake(s_lock, portMAX_DELAY);
	bool same = s_cfg.period_ms == period_ms;
	xSemaphoreGive(s_lock);
	if (same)
		return ESP_OK;

	nvs_handle_t h;

	ESP_RETURN_ON_ERROR(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h), TAG, "open");

	esp_err_t err = nvs_set_u32(h, KEY_PERIOD, period_ms);

	if (err == ESP_OK)
		err = nvs_commit(h);
	nvs_close(h);
	ESP_RETURN_ON_ERROR(err, TAG, "save period");

	xSemaphoreTake(s_lock, portMAX_DELAY);
	s_cfg.period_ms = period_ms;
	xSemaphoreGive(s_lock);
	return ESP_OK;
}

esp_err_t settings_factory_reset(void)
{
	nvs_handle_t h;

	ESP_RETURN_ON_ERROR(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h), TAG, "open");

	esp_err_t err = nvs_erase_all(h); /* only keys in "envmon", not WiFi/PHY data */

	if (err == ESP_OK)
		err = nvs_commit(h);
	nvs_close(h);
	ESP_RETURN_ON_ERROR(err, TAG, "erase");
	return ESP_OK;
}
