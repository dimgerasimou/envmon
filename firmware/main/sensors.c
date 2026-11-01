#include "sensors.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "board.h"
#include "bmp280.h"
#include "aht10.h"
#include "ens160.h"

#define MAX_FAILURES        3
#define SENSORS_TASK_STACK  4096
#define SENSORS_TASK_PRIO   5

static const char *TAG = "sensors";

static QueueHandle_t s_queue;
static TaskHandle_t s_task;
static uint32_t s_period_ms;

#if CONFIG_BOARD_HAS_BMP280
static bmp280_handle_t bmp280 = NULL;
#endif

#if CONFIG_BOARD_HAS_AHT10
static aht10_handle_t aht10 = NULL;
#endif

#if CONFIG_BOARD_HAS_ENS160
static ens160_handle_t ens160 = NULL;
#endif

static esp_err_t init_bmp280(void)
{
#if CONFIG_BOARD_HAS_BMP280
	bmp280_config_t bmp280_cfg = BMP280_DEFAULT_CONFIG();

	bmp280_cfg.addr = CONFIG_BOARD_BMP280_ADDR;
	bmp280_cfg.filter = BMP280_FILTER_4;

	esp_err_t err = bmp280_init(board_i2c_bus(), &bmp280_cfg, &bmp280);

	if (err == ESP_OK)
		ESP_LOGI(TAG, "BMP280 ready");
	else
		ESP_LOGE(TAG, "BMP280 unavailable: %s", esp_err_to_name(err));
#endif

	return ESP_OK;
}

static esp_err_t init_aht10(void)
{
#if CONFIG_BOARD_HAS_AHT10
	aht10_config_t aht10_cfg = AHT10_DEFAULT_CONFIG();

	aht10_cfg.addr = CONFIG_BOARD_AHT10_ADDR;

	esp_err_t err = aht10_init(board_i2c_bus(), &aht10_cfg, &aht10);

	if (err == ESP_OK)
		ESP_LOGI(TAG, "AHT10 ready");
	else
		ESP_LOGE(TAG, "AHT10 unavailable: %s", esp_err_to_name(err));
#endif

	return ESP_OK;
}

static esp_err_t init_ens160(void)
{
#if CONFIG_BOARD_HAS_ENS160
	ens160_config_t ens160_cfg = ENS160_DEFAULT_CONFIG();

	ens160_cfg.addr = CONFIG_BOARD_ENS160_ADDR;

	esp_err_t err = ens160_init(board_i2c_bus(), &ens160_cfg, &ens160);

	if (err == ESP_OK)
		ESP_LOGI(TAG, "ENS160 ready");
	else
		ESP_LOGE(TAG, "ENS160 unavailable: %s", esp_err_to_name(err));
#endif

	return ESP_OK;
}

static void set_value(sensors_param_t *v, float value, const char *src)
{
	if (v->valid)
		return;
	v->value = value;
	v->src = src;
	v->valid = true;
}

static void read_bmp280(sensors_data_t *data)
{
#if CONFIG_BOARD_HAS_BMP280
	static uint8_t failures;
	float t, p;

	if (!bmp280 && init_bmp280() != ESP_OK)
		return;

	esp_err_t err = bmp280_read(bmp280, &t, &p);

	if (err != ESP_OK) {
		ESP_LOGW(TAG, "BMP280 read failed: %s", esp_err_to_name(err));
		if (++failures >= MAX_FAILURES) {
			bmp280_delete(bmp280);
			bmp280 = NULL;
			failures = 0;
		}
		return;
	}
	failures = 0;
	set_value(&data->temperature_c, t, "bmp280");
	set_value(&data->pressure_hpa, p / 100.0f, "bmp280");
#endif
}

static void read_aht10(sensors_data_t *data)
{
#if CONFIG_BOARD_HAS_AHT10
	static uint8_t failures;
	float t, rh;

	if (!aht10 && init_aht10() != ESP_OK)
		return;

	esp_err_t err = aht10_read(aht10, &t, &rh);

	if (err != ESP_OK) {
		ESP_LOGW(TAG, "AHT10 read failed: %s", esp_err_to_name(err));
		if (++failures >= MAX_FAILURES) {
			aht10_delete(aht10);
			aht10 = NULL;
			failures = 0;
		}
		return;
	}
	failures = 0;
	set_value(&data->temperature_c, t, "aht10");
	set_value(&data->humidity_pct, rh, "aht10");
#endif
}

static void read_ens160(sensors_data_t *data)
{
#if CONFIG_BOARD_HAS_ENS160
	static uint8_t failures;
	ens160_data_t d;

	if (!ens160 && init_ens160() != ESP_OK)
		return;

	/* best available ambient values from the sensors read before this one */
	if (data->temperature_c.valid && data->humidity_pct.valid)
		ens160_set_compensation(ens160, data->temperature_c.value, data->humidity_pct.value);

	esp_err_t err = ens160_read(ens160, &d);

	if (err != ESP_OK) {
		ESP_LOGW(TAG, "ENS160 read failed: %s", esp_err_to_name(err));
		if (++failures >= MAX_FAILURES) {
			ens160_delete(ens160);
			ens160 = NULL;
			failures = 0;
		}
		return;
	}
	failures = 0;

	if (d.validity != ENS160_VALIDITY_NORMAL) {
		ESP_LOGI(TAG, "ENS160 warming up (validity %d)", d.validity);
		return;
	}
	set_value(&data->aqi, d.aqi, "ens160");
	set_value(&data->tvoc_ppb, d.tvoc_ppb, "ens160");
	set_value(&data->eco2_ppm, d.eco2_ppm, "ens160");
#endif
}

static void sensors_task(void *arg)
{
	TickType_t last_wake = xTaskGetTickCount();
	sensors_data_t data;

	for (;;) {
		if (sensors_read(&data) == ESP_OK)
			xQueueOverwrite(s_queue, &data);

		ESP_LOGD(TAG, "stack free: %u bytes", (unsigned)uxTaskGetStackHighWaterMark(NULL));
		xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(s_period_ms));
	}
}

esp_err_t sensors_init(void)
{
	init_bmp280();
	init_aht10();
	init_ens160();
	
	return ESP_OK;
}

esp_err_t sensors_read(sensors_data_t *data)
{
	*data = (sensors_data_t){ 0 };

	read_aht10(data);
	read_bmp280(data);
	read_ens160(data);

	return ESP_OK;
}

esp_err_t sensors_start(uint32_t period_ms, QueueHandle_t *ret_queue)
{
	ESP_RETURN_ON_FALSE(ret_queue && period_ms > 0, ESP_ERR_INVALID_ARG, TAG, "invalid args");
	ESP_RETURN_ON_FALSE(!s_task, ESP_ERR_INVALID_STATE, TAG, "already started");
	ESP_RETURN_ON_ERROR(sensors_init(), TAG, "init failed");

	s_queue = xQueueCreate(1, sizeof(sensors_data_t));
	ESP_RETURN_ON_FALSE(s_queue, ESP_ERR_NO_MEM, TAG, "queue alloc failed");

	s_period_ms = period_ms;
	if (xTaskCreate(sensors_task, "sensors", SENSORS_TASK_STACK, NULL,
	                SENSORS_TASK_PRIO, &s_task) != pdPASS) {
		vQueueDelete(s_queue);
		s_queue = NULL;
		return ESP_ERR_NO_MEM;
	}

	*ret_queue = s_queue;
	return ESP_OK;
}

