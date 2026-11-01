#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "board.h"
#include "sensors.h"

static const char *TAG = "envmon";

void app_main(void)
{
	sensors_data_t data = {
		.temperature_c.valid = 0,
		.pressure_hpa.valid = 0,
	};

	ESP_ERROR_CHECK(board_i2c_init());
	ESP_LOGI(TAG, "I2C bus ready");

	sensors_init();

	for (;;) {
		sensors_read(&data);
		if (data.temperature_c.valid)
			ESP_LOGI(TAG, "temperature %.2f C (%s)", data.temperature_c.value, data.temperature_c.src);
		if (data.pressure_hpa.valid)
			ESP_LOGI(TAG, "pressure %.2f hPa (%s)", data.pressure_hpa.value, data.pressure_hpa.src);
		if (data.humidity_pct.valid)
			ESP_LOGI(TAG, "humidity %.2f %% (%s)", data.humidity_pct.value, data.humidity_pct.src);
		if (data.aqi.valid)
			ESP_LOGI(TAG, "AQI: %.0f (%s)", data.aqi.value, data.aqi.src);
		if (data.tvoc_ppb.valid)
			ESP_LOGI(TAG, "tVOC %.0f ppb (%s)", data.tvoc_ppb.value, data.tvoc_ppb.src);
		if (data.eco2_ppm.valid)
			ESP_LOGI(TAG, "eCO2 %.0f ppm (%s)", data.eco2_ppm.value, data.eco2_ppm.src);
		vTaskDelay(pdMS_TO_TICKS(2000));
	}
}

