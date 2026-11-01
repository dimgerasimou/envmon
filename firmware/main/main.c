#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "board.h"
#include "sensors.h"

static const char *TAG = "envmon";

static void log_param(const char *name, const sensors_param_t *p, const char *unit)
{
	if (p->valid)
		ESP_LOGI(TAG, "%-12s %8.2f %-4s (%s)", name, p->value, unit, p->src);
}

void app_main(void)
{
	QueueHandle_t queue;
	sensors_data_t data;

	ESP_ERROR_CHECK(board_i2c_init());
	ESP_ERROR_CHECK(sensors_start(2000, &queue));

	for (;;) {
		xQueueReceive(queue, &data, portMAX_DELAY);
		log_param("temperature", &data.temperature_c, "C");
		log_param("humidity",    &data.humidity_pct,  "%");
		log_param("pressure",    &data.pressure_hpa,  "hPa");
		log_param("aqi",         &data.aqi,           "");
		log_param("tvoc",        &data.tvoc_ppb,      "ppb");
		log_param("eco2",        &data.eco2_ppm,      "ppm");
	}
}

