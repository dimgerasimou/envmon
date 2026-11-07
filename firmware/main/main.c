#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_event.h"
#include "esp_log.h"
#include "board.h"
#include "console.h"
#include "net.h"
#include "sensors.h"
#include "settings.h"

static const char *TAG = "envmon";

/* Debug level: the "sensors" console command shows the same data on demand. */
static void log_sample(const sensors_data_t *d)
{
	for (sensors_qty_t q = 0; q < SENSORS_QTY_COUNT; q++) {
		const sensors_param_t *p = &d->qty[q];

		if (p->valid)
			ESP_LOGD(TAG, "%-12s %8.2f %-5s (%s)", sensors_qty_info(q)->name, p->value,
			         sensors_qty_info(q)->unit, p->src);
	}
}

void app_main(void)
{
	QueueHandle_t queue;
	sensors_data_t data;
	settings_t cfg;

	ESP_ERROR_CHECK(settings_init());
	settings_get(&cfg);

	ESP_ERROR_CHECK(board_i2c_init());
	ESP_ERROR_CHECK(sensors_start(cfg.period_ms, &queue));
	ESP_ERROR_CHECK(console_start());

	/* one shared event loop: WiFi and IP events now, MQTT later */
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	/* no WiFi is not fatal: the node keeps measuring and the console still works */
	esp_err_t err = net_start();

	if (err == ESP_ERR_NOT_FOUND)
		ESP_LOGW(TAG, "WiFi not configured: set wifi_ssid <name>, set wifi_pass "
		              "<password>, reboot");
	else if (err != ESP_OK)
		ESP_LOGE(TAG, "WiFi failed to start: %s", esp_err_to_name(err));

	for (;;) {
		xQueueReceive(queue, &data, portMAX_DELAY);
		log_sample(&data);
	}
}
