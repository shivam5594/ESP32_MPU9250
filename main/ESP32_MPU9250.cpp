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
#include "wifi_logger.h"

extern "C" void app_main(void)
{
	start_wifi_logger(); // Start wifi logger
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
		ESP_LOGI(_TAG, "Detected device at %02x\n", address);
	} else if (ret1 == ESP_ERR_TIMEOUT) {
		ESP_LOGI(_TAG, "UU ");
	} else {
		ESP_LOGI(_TAG, "-- %d", ret1);
	}

	if(ESP_OK == imu->Configure())
	{
		if(!imu->isCalibrated())
			imu->Calibrate();
	}

	while(1){
		imu->update();
		ESP_LOGI(_TAG, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t", 	imu->getAccelX_mss(),
				imu->getAccelY_mss(),
				imu->getAccelZ_mss(),
				imu->getGyroX_rads(),
				imu->getGyroY_rads(),
				imu->getGyroZ_rads(),
				imu->getMagX_uT(),
				imu->getMagY_uT(),
				imu->getMagZ_uT());
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}


	ESP_ERROR_CHECK(ret);
}
