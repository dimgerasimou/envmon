#include <stdatomic.h>
#include "sensors.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "board.h"
#include "bmp280.h"
#include "aht10.h"
#include "ens160.h"

#define MAX_FAILURES        3                       /* consecutive read errors before re-init */
#define RETRY_MIN           pdMS_TO_TICKS(5000)     /* first re-init delay */
#define RETRY_MAX           pdMS_TO_TICKS(300000)   /* re-init delay cap (5 min) */
#define SENSORS_TASK_STACK  4096
#define SENSORS_TASK_PRIO   5

static const char *TAG = "sensors";

static QueueHandle_t s_queue;
static TaskHandle_t s_task;
static _Atomic uint32_t s_period_ms = 30000;

/*
 * Health bookkeeping shared by every sensor
 */

typedef struct {
	const char *name;
	uint8_t failures;       /* consecutive read errors */
	TickType_t backoff;     /* 0 = healthy, else wait this long between init attempts */
	TickType_t last_try;    /* tick of the last init attempt */
} sensor_state_t;

/* Whether a sensor that is not initialized should be tried now. */
static bool retry_due(sensor_state_t *s)
{
	TickType_t now = xTaskGetTickCount();

	if (s->backoff && now - s->last_try < s->backoff)
		return false;
	s->last_try = now;
	return true;
}

/* Record the result of an init attempt. Logs an error only on the first failure. */
static esp_err_t init_done(sensor_state_t *s, esp_err_t err)
{
	if (err == ESP_OK) {
		ESP_LOGI(TAG, "%s ready", s->name);
		s->failures = 0;
		s->backoff = 0;
		return ESP_OK;
	}

	if (!s->backoff) {
		ESP_LOGE(TAG, "%s unavailable: %s, retrying in background",
		         s->name, esp_err_to_name(err));
		s->backoff = RETRY_MIN;
	} else {
		ESP_LOGD(TAG, "%s still unavailable: %s", s->name, esp_err_to_name(err));
		s->backoff = s->backoff >= RETRY_MAX / 2 ? RETRY_MAX : s->backoff * 2;
	}
	return err;
}

/* Record a read error. Returns true when the driver should be deleted and re-initialized. */
static bool read_failed(sensor_state_t *s, esp_err_t err)
{
	ESP_LOGW(TAG, "%s read failed (%u/%u): %s",
	         s->name, s->failures + 1, MAX_FAILURES, esp_err_to_name(err));
	if (++s->failures < MAX_FAILURES)
		return false;

	ESP_LOGE(TAG, "%s lost, re-initializing", s->name);
	s->failures = 0;
	s->backoff = 0;         /* try again on the next cycle */
	return true;
}

/* Fill a value unless a higher-priority sensor already did this cycle. */
static void set_value(sensors_param_t *v, float value, const char *src)
{
	if (v->valid)
		return;
	v->value = value;
	v->src = src;
	v->valid = true;
}

/*
 * Sensors. Each one: open (init with board config), read (with re-init on errors).
 * Priority between sensors that measure the same quantity is the call order
 * in read_all().
 */

#if CONFIG_BOARD_HAS_AHT10
static aht10_handle_t s_aht10;
static sensor_state_t s_aht10_st = { .name = "AHT10" };

static esp_err_t open_aht10(void)
{
	aht10_config_t cfg = AHT10_DEFAULT_CONFIG();

	cfg.addr = CONFIG_BOARD_AHT10_ADDR;
	return init_done(&s_aht10_st, aht10_init(board_i2c_bus(), &cfg, &s_aht10));
}

static void read_aht10(sensors_data_t *data)
{
	float t, rh;

	if (!s_aht10 && (!retry_due(&s_aht10_st) || open_aht10() != ESP_OK))
		return;

	esp_err_t err = aht10_read(s_aht10, &t, &rh);

	if (err != ESP_OK) {
		if (read_failed(&s_aht10_st, err)) {
			aht10_delete(s_aht10);
			s_aht10 = NULL;
		}
		return;
	}
	s_aht10_st.failures = 0;
	set_value(&data->temperature_c, t, "aht10");
	set_value(&data->humidity_pct, rh, "aht10");
}
#endif

#if CONFIG_BOARD_HAS_BMP280
static bmp280_handle_t s_bmp280;
static sensor_state_t s_bmp280_st = { .name = "BMP280" };

static esp_err_t open_bmp280(void)
{
	bmp280_config_t cfg = BMP280_DEFAULT_CONFIG();

	cfg.addr = CONFIG_BOARD_BMP280_ADDR;
	cfg.filter = BMP280_FILTER_4;
	return init_done(&s_bmp280_st, bmp280_init(board_i2c_bus(), &cfg, &s_bmp280));
}

static void read_bmp280(sensors_data_t *data)
{
	float t, p;

	if (!s_bmp280 && (!retry_due(&s_bmp280_st) || open_bmp280() != ESP_OK))
		return;

	esp_err_t err = bmp280_read(s_bmp280, &t, &p);

	if (err != ESP_OK) {
		if (read_failed(&s_bmp280_st, err)) {
			bmp280_delete(s_bmp280);
			s_bmp280 = NULL;
		}
		return;
	}
	s_bmp280_st.failures = 0;
	set_value(&data->temperature_c, t, "bmp280");
	set_value(&data->pressure_hpa, p / 100.0f, "bmp280");
}
#endif

#if CONFIG_BOARD_HAS_ENS160
static ens160_handle_t s_ens160;
static sensor_state_t s_ens160_st = { .name = "ENS160" };

static esp_err_t open_ens160(void)
{
	ens160_config_t cfg = ENS160_DEFAULT_CONFIG();

	cfg.addr = CONFIG_BOARD_ENS160_ADDR;
	return init_done(&s_ens160_st, ens160_init(board_i2c_bus(), &cfg, &s_ens160));
}

static void read_ens160(sensors_data_t *data)
{
	static ens160_validity_t last_validity = ENS160_VALIDITY_NORMAL;
	ens160_data_t d;
	esp_err_t err = ESP_OK;

	if (!s_ens160 && (!retry_due(&s_ens160_st) || open_ens160() != ESP_OK))
		return;

	/* best ambient values from the sensors read before this one */
	if (data->temperature_c.valid && data->humidity_pct.valid)
		err = ens160_set_compensation(s_ens160, data->temperature_c.value,
		                              data->humidity_pct.value);
	if (err == ESP_OK)
		err = ens160_read(s_ens160, &d);

	if (err != ESP_OK) {
		if (read_failed(&s_ens160_st, err)) {
			ens160_delete(s_ens160);
			s_ens160 = NULL;
		}
		return;
	}
	s_ens160_st.failures = 0;

	if (d.validity != last_validity) {
		ESP_LOGI(TAG, "ENS160 validity %d -> %d", last_validity, d.validity);
		last_validity = d.validity;
	}
	if (d.validity != ENS160_VALIDITY_NORMAL)
		return;

	set_value(&data->aqi, d.aqi, "ens160");
	set_value(&data->tvoc_ppb, d.tvoc_ppb, "ens160");
	set_value(&data->eco2_ppm, d.eco2_ppm, "ens160");
}
#endif

static void open_all(void)
{
#if CONFIG_BOARD_HAS_AHT10
	open_aht10();
#endif
#if CONFIG_BOARD_HAS_BMP280
	open_bmp280();
#endif
#if CONFIG_BOARD_HAS_ENS160
	open_ens160();
#endif
}

/* Call order is priority order. ENS160 last: it uses the T/RH read before it. */
static void read_all(sensors_data_t *data)
{
	*data = (sensors_data_t){ 0 };

#if CONFIG_BOARD_HAS_AHT10
	read_aht10(data);
#endif
#if CONFIG_BOARD_HAS_BMP280
	read_bmp280(data);
#endif
#if CONFIG_BOARD_HAS_ENS160
	read_ens160(data);
#endif
}

static void sensors_task(void *arg)
{
	sensors_data_t data;

	for (;;) {
		TickType_t start = xTaskGetTickCount();

		read_all(&data);
		xQueueOverwrite(s_queue, &data);
		ESP_LOGD(TAG, "stack free: %u bytes", (unsigned)uxTaskGetStackHighWaterMark(NULL));

		/* sleep for the rest of the period, or until sensors_set_period() wakes us */
		TickType_t period = pdMS_TO_TICKS(atomic_load(&s_period_ms));
		TickType_t elapsed = xTaskGetTickCount() - start;

		ulTaskNotifyTake(pdTRUE, elapsed < period ? period - elapsed : 0);
	}
}

static bool period_valid(uint32_t period_ms)
{
	return period_ms >= SENSORS_PERIOD_MIN_MS && period_ms <= SENSORS_PERIOD_MAX_MS;
}

esp_err_t sensors_set_period(uint32_t period_ms)
{
	ESP_RETURN_ON_FALSE(period_valid(period_ms), ESP_ERR_INVALID_ARG, TAG,
	                    "period %lu ms out of range", (unsigned long)period_ms);

	atomic_store(&s_period_ms, period_ms);
	ESP_LOGI(TAG, "period set to %lu ms", (unsigned long)period_ms);
	if (s_task)
		xTaskNotifyGive(s_task);
	return ESP_OK;
}

uint32_t sensors_get_period(void)
{
	return atomic_load(&s_period_ms);
}

esp_err_t sensors_start(uint32_t period_ms, QueueHandle_t *ret_queue)
{
	ESP_RETURN_ON_FALSE(ret_queue && period_valid(period_ms), ESP_ERR_INVALID_ARG, TAG,
	                    "invalid args");
	ESP_RETURN_ON_FALSE(!s_task, ESP_ERR_INVALID_STATE, TAG, "already started");

#if !CONFIG_BOARD_HAS_AHT10 && !CONFIG_BOARD_HAS_BMP280 && !CONFIG_BOARD_HAS_ENS160
	ESP_LOGE(TAG, "no sensor enabled in board config");
	return ESP_ERR_NOT_SUPPORTED;
#endif

	s_queue = xQueueCreate(1, sizeof(sensors_data_t));
	ESP_RETURN_ON_FALSE(s_queue, ESP_ERR_NO_MEM, TAG, "queue alloc failed");

	/* missing sensors are not fatal: they get retried from the task */
	open_all();

	atomic_store(&s_period_ms, period_ms);
	if (xTaskCreate(sensors_task, "sensors", SENSORS_TASK_STACK, NULL,
	                SENSORS_TASK_PRIO, &s_task) != pdPASS) {
		vQueueDelete(s_queue);
		s_queue = NULL;
		return ESP_ERR_NO_MEM;
	}

	*ret_queue = s_queue;
	return ESP_OK;
}

