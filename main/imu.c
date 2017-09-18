
#include <stdio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include "driver/i2c.h"
#include "bno055.h"

#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22

#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1

void i2c_master_init() {
	i2c_config_t i2c_config = { .mode = I2C_MODE_MASTER, .sda_io_num = SDA_PIN,
			.scl_io_num = SCL_PIN, .sda_pullup_en = GPIO_PULLUP_ENABLE,
			.scl_pullup_en = GPIO_PULLUP_ENABLE, .master.clk_speed = 400000 };
	i2c_param_config(I2C_NUM_0, &i2c_config);
	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
	u8 iError = BNO055_INIT_VALUE;
	esp_err_t espRc;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, 1);

	i2c_master_write_byte(cmd, reg_addr, 1);
	i2c_master_write(cmd, reg_data, cnt, 1);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		iError = BNO055_SUCCESS;
	} else {
		iError = BNO055_ERROR;
	}
	i2c_cmd_link_delete(cmd);

	return (s8) iError;
}

s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
	u8 iError = BNO055_INIT_VALUE;
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, 1);
	i2c_master_write_byte(cmd, reg_addr, 1);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, 1);

	if (cnt > 1) {
		i2c_master_read(cmd, reg_data, cnt - 1, I2C_MASTER_ACK);
	}
	i2c_master_read_byte(cmd, reg_data + cnt - 1, I2C_MASTER_NACK);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		iError = BNO055_SUCCESS;
	} else {
		iError = BNO055_ERROR;
	}

	i2c_cmd_link_delete(cmd);

	return (s8) iError;
}

void BNO055_delay_msek(u32 msek) {
	vTaskDelay(msek / portTICK_PERIOD_MS);
}

void imu_init() {
}

