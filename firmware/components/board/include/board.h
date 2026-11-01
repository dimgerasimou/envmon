#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Create the board's I2C bus on the pins set in menuconfig (Board)
 *
 * Safe to call more than once; later calls do nothing.
 *
 * @return
 *      - ESP_OK: Success, or the bus already exists
 *      - Other: Error from i2c_new_master_bus()
 */
esp_err_t board_i2c_init(void);

/**
 * @brief Get the board's I2C bus
 *
 * @return Bus handle, or NULL if board_i2c_init() has not succeeded yet
 */
i2c_master_bus_handle_t board_i2c_bus(void);

#ifdef __cplusplus
}
#endif
