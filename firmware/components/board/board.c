#include "esp_log.h"
#include "board.h"

static const char *TAG = "board";

static i2c_master_bus_handle_t i2c_bus;

esp_err_t board_i2c_init(void)
{
	if (i2c_bus)
		return ESP_OK;

	i2c_master_bus_config_t cfg = {
		.i2c_port = -1,
		.sda_io_num = CONFIG_BOARD_I2C_SDA,
		.scl_io_num = CONFIG_BOARD_I2C_SCL,
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = true,
	};
	esp_err_t err = i2c_new_master_bus(&cfg, &i2c_bus);

	if (err == ESP_OK)
		ESP_LOGD(TAG, "I2C bus on SDA=%d SCL=%d", CONFIG_BOARD_I2C_SDA,
		         CONFIG_BOARD_I2C_SCL);
	return err;
}

i2c_master_bus_handle_t board_i2c_bus(void)
{
	return i2c_bus;
}
