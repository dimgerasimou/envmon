#include <math.h>
#include <stdatomic.h>
#include "sensors.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "board.h"
#include "aht10.h"
#include "bmp280.h"
#include "ens160.h"
#include "sen66.h"
#include "sht40.h"
#include "stcc4.h"

#define MAX_FAILURES       3                     /* consecutive read errors before re-init */
#define RETRY_MIN          pdMS_TO_TICKS(5000)   /* first re-init delay */
#define RETRY_MAX          pdMS_TO_TICKS(300000) /* re-init delay cap (5 min) */
#define PRESSURE_STEP_PA   100.0f                /* resend CO2 compensation after 1 hPa change */
#define SEN66_STATUS_EVERY 30                    /* check SEN66 health every N reads */
#define SENSORS_TASK_STACK 4096
#define SENSORS_TASK_PRIO  5

static const char *TAG = "sensors";

static QueueHandle_t s_queue;
static TaskHandle_t s_task;
static _Atomic uint32_t s_period_ms = 30000;

/*
 * Health bookkeeping shared by every sensor
 */

typedef struct {
	const char *name;
	uint8_t failures;    /* consecutive read errors */
	TickType_t backoff;  /* 0 = healthy, else wait this long between init attempts */
	TickType_t last_try; /* tick of the last init attempt */
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
		ESP_LOGE(TAG, "%s unavailable: %s, retrying in background", s->name,
		         esp_err_to_name(err));
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
	if (err == ESP_ERR_NOT_FINISHED) { /* sensor just started, no result yet */
		ESP_LOGD(TAG, "%s not ready yet", s->name);
		return false;
	}
	ESP_LOGW(TAG, "%s read failed (%u/%u): %s", s->name, s->failures + 1, MAX_FAILURES,
	         esp_err_to_name(err));
	if (++s->failures < MAX_FAILURES)
		return false;

	ESP_LOGE(TAG, "%s lost, re-initializing", s->name);
	s->failures = 0;
	s->backoff = 0; /* try again on the next cycle */
	return true;
}

/* One cycle's results: the winner per quantity, plus every reading from every sensor. */
typedef struct {
	sensors_data_t best;
	sensors_readings_t all;
} sample_t;

/*
 * Record a reading from one sensor. It always goes into the full list; it
 * becomes the quantity's value unless a higher-priority sensor already set it
 * this cycle. Unknown values (NAN) are dropped.
 */
static void set_value(sample_t *smp, sensors_qty_t q, float value, const char *src)
{
	if (isnan(value))
		return;

	if (smp->all.count < SENSORS_MAX_READINGS)
		smp->all.r[smp->all.count++] =
		        (sensors_reading_t){ .qty = q, .value = value, .src = src };

	sensors_param_t *v = &smp->best.qty[q];

	if (!v->valid)
		*v = (sensors_param_t){ .valid = true, .value = value, .src = src };
}

/* copy of the latest sample for sensors_get_latest() */
static SemaphoreHandle_t s_latest_lock;
static sample_t s_latest;
static bool s_have_latest;

/*
 * Sensors. Each one: open (init with board config), read (with re-init on errors).
 * Priority between sensors that measure the same quantity is the call order
 * in read_all().
 */

#if CONFIG_BOARD_HAS_SHT40
static sht40_handle_t s_sht40;
static sensor_state_t s_sht40_st = { .name = "SHT40" };

static esp_err_t open_sht40(void)
{
	sht40_config_t cfg = SHT40_DEFAULT_CONFIG();

	cfg.addr = CONFIG_BOARD_SHT40_ADDR;
	return init_done(&s_sht40_st, sht40_init(board_i2c_bus(), &cfg, &s_sht40));
}

static void read_sht40(sample_t *smp)
{
	float t, rh;

	if (!s_sht40 && (!retry_due(&s_sht40_st) || open_sht40() != ESP_OK))
		return;

	esp_err_t err = sht40_read(s_sht40, &t, &rh);

	if (err != ESP_OK) {
		if (read_failed(&s_sht40_st, err)) {
			sht40_delete(s_sht40);
			s_sht40 = NULL;
		}
		return;
	}
	s_sht40_st.failures = 0;
	set_value(smp, SENSORS_TEMPERATURE, t, "sht40");
	set_value(smp, SENSORS_HUMIDITY, rh, "sht40");
}
#endif

#if CONFIG_BOARD_HAS_SEN66
static sen66_handle_t s_sen66;
static sensor_state_t s_sen66_st = { .name = "SEN66" };
static float s_sen66_pressure; /* compensation last sent, 0 = none since init */

static esp_err_t open_sen66(void)
{
	sen66_config_t cfg = SEN66_DEFAULT_CONFIG();

	s_sen66_pressure = 0;
	return init_done(&s_sen66_st, sen66_init(board_i2c_bus(), &cfg, &s_sen66));
}

/* Error flags stay set until reset; log only when they change. */
static void check_sen66_status(void)
{
	static unsigned reads;
	static uint32_t last_status;
	uint32_t status;

	if (reads++ % SEN66_STATUS_EVERY != 0 || sen66_read_status(s_sen66, &status) != ESP_OK)
		return;
	if (status != last_status) {
		if (status) /* bits: SEN66_STATUS_* in sen66.h */
			ESP_LOGW(TAG, "SEN66 status 0x%08lx", (unsigned long)status);
		else
			ESP_LOGI(TAG, "SEN66 status OK");
		last_status = status;
	}
}

static void read_sen66(sample_t *smp)
{
	sen66_data_t d;

	if (!s_sen66 && (!retry_due(&s_sen66_st) || open_sen66() != ESP_OK))
		return;

	esp_err_t err = sen66_read(s_sen66, &d);

	if (err != ESP_OK) {
		if (read_failed(&s_sen66_st, err)) {
			sen66_delete(s_sen66);
			s_sen66 = NULL;
		}
		return;
	}
	s_sen66_st.failures = 0;
	check_sen66_status();

	/* unknown values (NAN) right after start are skipped by set_value() */
	set_value(smp, SENSORS_TEMPERATURE, d.temp_c, "sen66");
	set_value(smp, SENSORS_HUMIDITY, d.humidity_pct, "sen66");
	set_value(smp, SENSORS_CO2, d.co2_ppm, "sen66");
	set_value(smp, SENSORS_PM1_0, d.pm1_0, "sen66");
	set_value(smp, SENSORS_PM2_5, d.pm2_5, "sen66");
	set_value(smp, SENSORS_PM4_0, d.pm4_0, "sen66");
	set_value(smp, SENSORS_PM10, d.pm10, "sen66");
	set_value(smp, SENSORS_VOC_INDEX, d.voc_index, "sen66");
	set_value(smp, SENSORS_NOX_INDEX, d.nox_index, "sen66");
}
#endif

#if CONFIG_BOARD_HAS_STCC4
static stcc4_handle_t s_stcc4;
static sensor_state_t s_stcc4_st = { .name = "STCC4" };
static float s_stcc4_pressure; /* compensation last sent, 0 = none since init */

static esp_err_t open_stcc4(void)
{
	stcc4_config_t cfg = STCC4_DEFAULT_CONFIG();

	cfg.addr = CONFIG_BOARD_STCC4_ADDR;
	s_stcc4_pressure = 0;
	return init_done(&s_stcc4_st, stcc4_init(board_i2c_bus(), &cfg, &s_stcc4));
}

static void read_stcc4(sample_t *smp)
{
	stcc4_data_t d;

	if (!s_stcc4 && (!retry_due(&s_stcc4_st) || open_stcc4() != ESP_OK))
		return;

	esp_err_t err = stcc4_read(s_stcc4, &d);

	if (err != ESP_OK) {
		if (read_failed(&s_stcc4_st, err)) {
			stcc4_delete(s_stcc4);
			s_stcc4 = NULL;
		}
		return;
	}
	s_stcc4_st.failures = 0;
	set_value(smp, SENSORS_TEMPERATURE, d.temp_c, "stcc4");
	set_value(smp, SENSORS_HUMIDITY, d.humidity_pct, "stcc4");
	set_value(smp, SENSORS_CO2, d.co2_ppm, "stcc4");
}
#endif

#if CONFIG_BOARD_HAS_AHT10
static aht10_handle_t s_aht10;
static sensor_state_t s_aht10_st = { .name = "AHT10" };

static esp_err_t open_aht10(void)
{
	aht10_config_t cfg = AHT10_DEFAULT_CONFIG();

	cfg.addr = CONFIG_BOARD_AHT10_ADDR;
	return init_done(&s_aht10_st, aht10_init(board_i2c_bus(), &cfg, &s_aht10));
}

static void read_aht10(sample_t *smp)
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
	set_value(smp, SENSORS_TEMPERATURE, t, "aht10");
	set_value(smp, SENSORS_HUMIDITY, rh, "aht10");
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

static void read_bmp280(sample_t *smp)
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
	set_value(smp, SENSORS_TEMPERATURE, t, "bmp280");
	set_value(smp, SENSORS_PRESSURE, p / 100.0f, "bmp280");
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

static void read_ens160(sample_t *smp)
{
	static ens160_validity_t last_validity = ENS160_VALIDITY_NORMAL;
	ens160_data_t d;
	esp_err_t err = ESP_OK;

	if (!s_ens160 && (!retry_due(&s_ens160_st) || open_ens160() != ESP_OK))
		return;

	/* best ambient values from the sensors read before this one */
	const sensors_param_t *t = &smp->best.qty[SENSORS_TEMPERATURE];
	const sensors_param_t *rh = &smp->best.qty[SENSORS_HUMIDITY];

	if (t->valid && rh->valid)
		err = ens160_set_compensation(s_ens160, t->value, rh->value);
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

	set_value(smp, SENSORS_AQI, d.aqi, "ens160");
	set_value(smp, SENSORS_TVOC, d.tvoc_ppb, "ens160");
	set_value(smp, SENSORS_ECO2, d.eco2_ppm, "ens160");
}
#endif

/*
 * CO2 sensors are more accurate with the real ambient pressure. Send it when
 * it moved by PRESSURE_STEP_PA since the last send, or after a re-init.
 * Uses this cycle's pressure, so it applies from the next read on.
 */
static void update_co2_pressure(const sensors_data_t *best)
{
	const sensors_param_t *p = &best->qty[SENSORS_PRESSURE];

	if (!p->valid)
		return;

	float pa = p->value * 100.0f;

#if CONFIG_BOARD_HAS_SEN66
	if (s_sen66 && fabsf(pa - s_sen66_pressure) >= PRESSURE_STEP_PA &&
	    sen66_set_pressure(s_sen66, pa) == ESP_OK)
		s_sen66_pressure = pa;
#endif
#if CONFIG_BOARD_HAS_STCC4
	if (s_stcc4 && fabsf(pa - s_stcc4_pressure) >= PRESSURE_STEP_PA &&
	    stcc4_set_pressure(s_stcc4, pa) == ESP_OK)
		s_stcc4_pressure = pa;
#endif
	(void)pa;
}

static void open_all(void)
{
#if CONFIG_BOARD_HAS_SHT40
	open_sht40();
#endif
#if CONFIG_BOARD_HAS_SEN66
	open_sen66();
#endif
#if CONFIG_BOARD_HAS_STCC4
	open_stcc4();
#endif
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

/*
 * Call order is priority order:
 *   temperature, humidity: SHT40 > SEN66 > STCC4 > AHT10 > BMP280
 *   CO2: SEN66 > STCC4
 * ENS160 last: it uses the T/RH read before it.
 */
static void read_all(sample_t *smp)
{
	*smp = (sample_t){ 0 };

#if CONFIG_BOARD_HAS_SHT40
	read_sht40(smp);
#endif
#if CONFIG_BOARD_HAS_SEN66
	read_sen66(smp);
#endif
#if CONFIG_BOARD_HAS_STCC4
	read_stcc4(smp);
#endif
#if CONFIG_BOARD_HAS_AHT10
	read_aht10(smp);
#endif
#if CONFIG_BOARD_HAS_BMP280
	read_bmp280(smp);
#endif
#if CONFIG_BOARD_HAS_ENS160
	read_ens160(smp);
#endif
	update_co2_pressure(&smp->best);
}

static void sensors_task(void *arg)
{
	static sample_t smp; /* static: ~450 bytes, keep it off the task stack */

	for (;;) {
		TickType_t start = xTaskGetTickCount();

		read_all(&smp);
		xQueueOverwrite(s_queue, &smp.best);

		xSemaphoreTake(s_latest_lock, portMAX_DELAY);
		s_latest = smp;
		s_have_latest = true;
		xSemaphoreGive(s_latest_lock);

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

esp_err_t sensors_get_latest(sensors_data_t *best, sensors_readings_t *all)
{
	ESP_RETURN_ON_FALSE(best || all, ESP_ERR_INVALID_ARG, TAG, "NULL arguments");
	if (!s_latest_lock)
		return ESP_ERR_INVALID_STATE;

	xSemaphoreTake(s_latest_lock, portMAX_DELAY);
	bool have = s_have_latest;

	if (have && best)
		*best = s_latest.best;
	if (have && all)
		*all = s_latest.all;
	xSemaphoreGive(s_latest_lock);
	return have ? ESP_OK : ESP_ERR_NOT_FOUND;
}

/* name and unit of each quantity; ASCII so console columns line up */
static const sensors_qty_info_t s_info[SENSORS_QTY_COUNT] = {
	[SENSORS_TEMPERATURE] = { "temperature", "C" },
	[SENSORS_HUMIDITY] = { "humidity", "%" },
	[SENSORS_PRESSURE] = { "pressure", "hPa" },
	[SENSORS_CO2] = { "co2", "ppm" },
	[SENSORS_PM1_0] = { "pm1_0", "ug/m3" },
	[SENSORS_PM2_5] = { "pm2_5", "ug/m3" },
	[SENSORS_PM4_0] = { "pm4_0", "ug/m3" },
	[SENSORS_PM10] = { "pm10", "ug/m3" },
	[SENSORS_VOC_INDEX] = { "voc_index", "" },
	[SENSORS_NOX_INDEX] = { "nox_index", "" },
	[SENSORS_AQI] = { "aqi", "" },
	[SENSORS_TVOC] = { "tvoc", "ppb" },
	[SENSORS_ECO2] = { "eco2", "ppm" },
};

#define Q(q) (1UL << (q))

/* What the sensors enabled in the board config can measure. A constant after optimization. */
static uint32_t supported_mask(void)
{
	uint32_t m = 0;

#if CONFIG_BOARD_HAS_SHT40
	m |= Q(SENSORS_TEMPERATURE) | Q(SENSORS_HUMIDITY);
#endif
#if CONFIG_BOARD_HAS_SEN66
	m |= Q(SENSORS_TEMPERATURE) | Q(SENSORS_HUMIDITY) | Q(SENSORS_CO2);
	m |= Q(SENSORS_PM1_0) | Q(SENSORS_PM2_5) | Q(SENSORS_PM4_0) | Q(SENSORS_PM10);
	m |= Q(SENSORS_VOC_INDEX) | Q(SENSORS_NOX_INDEX);
#endif
#if CONFIG_BOARD_HAS_STCC4
	m |= Q(SENSORS_TEMPERATURE) | Q(SENSORS_HUMIDITY) | Q(SENSORS_CO2);
#endif
#if CONFIG_BOARD_HAS_AHT10
	m |= Q(SENSORS_TEMPERATURE) | Q(SENSORS_HUMIDITY);
#endif
#if CONFIG_BOARD_HAS_BMP280
	m |= Q(SENSORS_TEMPERATURE) | Q(SENSORS_PRESSURE);
#endif
#if CONFIG_BOARD_HAS_ENS160
	m |= Q(SENSORS_AQI) | Q(SENSORS_TVOC) | Q(SENSORS_ECO2);
#endif
	return m;
}

const sensors_qty_info_t *sensors_qty_info(sensors_qty_t q)
{
	static const sensors_qty_info_t unknown = { "?", "" };

	return q < SENSORS_QTY_COUNT ? &s_info[q] : &unknown;
}

bool sensors_qty_supported(sensors_qty_t q)
{
	return q < SENSORS_QTY_COUNT && (supported_mask() & Q(q));
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

#if !CONFIG_BOARD_HAS_SHT40 && !CONFIG_BOARD_HAS_SEN66 && !CONFIG_BOARD_HAS_STCC4 &&               \
        !CONFIG_BOARD_HAS_AHT10 && !CONFIG_BOARD_HAS_BMP280 && !CONFIG_BOARD_HAS_ENS160
	ESP_LOGE(TAG, "no sensor enabled in board config");
	return ESP_ERR_NOT_SUPPORTED;
#endif

	s_queue = xQueueCreate(1, sizeof(sensors_data_t));
	ESP_RETURN_ON_FALSE(s_queue, ESP_ERR_NO_MEM, TAG, "queue alloc failed");
	s_latest_lock = xSemaphoreCreateMutex();
	ESP_RETURN_ON_FALSE(s_latest_lock, ESP_ERR_NO_MEM, TAG, "mutex alloc failed");

	/* missing sensors are not fatal: they get retried from the task. SEN66/STCC4 take ~1.2 s
	 * each */
	open_all();

	atomic_store(&s_period_ms, period_ms);
	if (xTaskCreate(sensors_task, "sensors", SENSORS_TASK_STACK, NULL, SENSORS_TASK_PRIO,
	                &s_task) != pdPASS) {
		vQueueDelete(s_queue);
		s_queue = NULL;
		return ESP_ERR_NO_MEM;
	}

	*ret_queue = s_queue;
	return ESP_OK;
}
