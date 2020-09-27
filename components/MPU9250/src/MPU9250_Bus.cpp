/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */

#include "MPU9250_Bus.h"
#include "driver/i2c.h"  // I2C library
#include "driver/spi_master.h" // SPI Library
#include "driver/gpio.h"
#include "esp_system.h"

esp_err_t MPU9250_I2C::initializeBus(i2c_port_t *bus){
	int i2c_master_port = *bus;
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = 15;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_io_num = 14;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = I2C_FREQUENCY;

	ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));

	return i2c_driver_install(i2c_master_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/* writes a byte to MPU9250 register given a register address and data */
esp_err_t MPU9250_I2C::writeRegister(uint8_t subAddress, uint8_t data){
	uint8_t buff;

	//_bus->Write(subAddress, data);
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (0x68 << 1) | I2C_MASTER_WRITE, 0x1);
	i2c_master_write(cmd, &subAddress, 1, 0x1);
	i2c_master_write(cmd, &data, 1, 0x1);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(_bus, cmd, portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	if (!ret == ESP_OK) {
		return ESP_FAIL;
	}

	vTaskDelay(portTICK_RATE_MS); // need to slow down how fast I write to MPU9250

	/* read back the register */
	//buff = _bus->Read(subAddress);
	ret = readRegisters(subAddress,1,&buff);

	/* check the read back register against the written register */
	if(buff == data) {
		return ESP_OK;
	}
	else{
		return ESP_FAIL;
	}
}

/* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */
esp_err_t MPU9250_I2C::readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest)
{
	//_bus->Read(subAddress, dest, count);
    if (count == 0) {
        return ESP_OK;
    }

    // Initiate read by addressing Device ID and then register number
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (0x68 << 1) | I2C_MASTER_WRITE, 0x1);
	i2c_master_write(cmd, &subAddress, 1, 0x1);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(_bus, cmd, portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	if (!ret == ESP_OK) {
		return ESP_FAIL;
	}

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x68 << 1) | I2C_MASTER_READ, 0x1);
    if (count > 1) {
        i2c_master_read(cmd, dest, count - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, dest + count - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(_bus, cmd, portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

//void pre_spi_transfer_callback(spi_transaction_t *t)
//{
//	//set CS pin low
//	//gpio_set_level((gpio_num_t)PIN_NUM_CS,0);
//}
//
//void post_spi_transfer_callback(spi_transaction_t *t)
//{
//	//set CS pin high
//	//gpio_set_level((gpio_num_t)PIN_NUM_CS,1);
//}
//
//esp_err_t MPU9250_SPI::initializeBus(spi_device_handle_t *bus){
//	esp_err_t ret;
//
//	buscfg.miso_io_num=PIN_NUM_MISO;
//	buscfg.mosi_io_num=PIN_NUM_MOSI;
//	buscfg.sclk_io_num=PIN_NUM_CLK;
//	buscfg.quadwp_io_num=-1;
//	buscfg.quadhd_io_num=-1;
//	buscfg.flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_MISO | SPICOMMON_BUSFLAG_MOSI;
//	buscfg.intr_flags = 0;
//	buscfg.max_transfer_sz = 8;
//
//	devcfg.clock_speed_hz=SPI_HIGH_FREQUENCY; //Clock out at 10 MHz
//	devcfg.mode=0,                            //SPI mode 0
//	devcfg.spics_io_num=PIN_NUM_CS;           //CS pin
//	devcfg.queue_size=7;                      //We want to be able to queue 7 transactions at a time
//	devcfg.pre_cb=pre_spi_transfer_callback;  	  //Specify pre-transfer callback to handle D/C line
//	devcfg.post_cb=post_spi_transfer_callback;
//	devcfg.flags = SPI_DEVICE_HALFDUPLEX;
//
////	gpio_config_t IOConf = {
////			.pin_bit_mask = PIN_NUM_CS,
////			.mode = GPIO_MODE_OUTPUT,
////			.pull_up_en = GPIO_PULLUP_ENABLE,
////			.pull_down_en = GPIO_PULLDOWN_DISABLE,
////			.intr_type =GPIO_INTR_DISABLE,
////	};
////	gpio_config(&IOConf);
//
//	//Initialize the SPI bus
//	ret=spi_bus_initialize(IMU_HOST, &buscfg, DMA_CHAN);
//	ESP_ERROR_CHECK(ret);
//	//Attach the IMU to the SPI bus
//	ret=spi_bus_add_device(IMU_HOST, &devcfg, bus);
//	ESP_ERROR_CHECK(ret);
//
//	return ret;
//}
//
///* writes a byte to MPU9250 register given a register address and data */
//bool MPU9250_SPI::writeRegister(uint8_t subAddress, uint8_t data){
//	uint8_t buff;
//	esp_err_t ret;
//	static spi_transaction_t trans;
//
//	trans.addr = subAddress;
//	trans.tx_data[0] = data;
//	trans.length = 8;
//
//	ret = spi_device_queue_trans(_bus, &trans, portMAX_DELAY);
//	assert(ret==ESP_OK);
//
//	//_bus->Write(subAddress, data);
//	vTaskDelay(10); // need to slow down how fast I write to MPU9250
//
//	/* read back the register */
//	//buff = _bus->Read(subAddress | 0x80); // set top bit to perform read
//	buff = trans.rx_data[0];
//
//	/* check the read back register against the written register */
//	if(buff == data) {
//		return true;
//	}
//	else{
//		return false;
//	}
//}
//
///* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */
//void MPU9250_SPI::readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest)
//{
//	esp_err_t ret;
//	static spi_transaction_t trans;
//	trans.addr = subAddress | 0x80;
//	trans.rx_buffer = dest;
//	trans.length = count*8;
//
//	ret = spi_device_queue_trans(_bus, &trans, portMAX_DELAY);
//	assert(ret==ESP_OK);
//	//_bus->Read(subAddress | 0x80, dest, count);
//}
//
//void MPU9250_SPI::setBusLowSpeed()
//{
//
//	//_bus->ReconfigureFrequency(SPI_LOW_FREQUENCY);
//}
//
//void MPU9250_SPI::setBusHighSpeed()
//{
//	//_bus->ReconfigureFrequency(SPI_HIGH_FREQUENCY);
//}
