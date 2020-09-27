/* SPI Master example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#include "IMU.h"
#include "MPU9250.h"

static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x68 << 1) | I2C_MASTER_READ, 0x1);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x68 << 1) | I2C_MASTER_WRITE, 0x1);
    i2c_master_write(cmd, data_wr, size, 0x1);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


extern "C" void app_main(void)
{
    esp_err_t ret = ESP_FAIL;

    i2c_port_t I2Cport = I2C_NUM_0;

	uint8_t address;
	address = 0x68;

	MPU9250 *imu = new MPU9250(&I2Cport);
	imu->initializeBus(&I2Cport);

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, 0x1);
	i2c_master_stop(cmd);
	esp_err_t ret1 = i2c_master_cmd_begin(I2Cport, cmd, portMAX_DELAY);
	i2c_cmd_link_delete(cmd);
	if (ret1 == ESP_OK) {
		printf("Detected device at %02x\n", address);
	} else if (ret1 == ESP_ERR_TIMEOUT) {
		printf("UU ");
	} else {
		printf("-- %d", ret1);
	}

	if(!imu->isCalibrated())
		imu->Calibrate();
	else
		printf("calibrated already.\n");

	fflush(stdout);

	while(1){
		//imu->ReadAll();
		vTaskDelay(portTICK_PERIOD_MS);
	}


    ESP_ERROR_CHECK(ret);
}
