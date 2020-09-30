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
 
#ifndef MPU9250_BUS_H
#define MPU9250_BUS_H
#include "driver/i2c.h"  // I2C library
#include "driver/spi_master.h" // SPI Library

class MPU9250_Bus
{
	public:
		static const int I2C_FREQUENCY = 1000000;			// 400 kHz
		static const int SPI_LOW_FREQUENCY = 1000000;		// 1 MHz
		static const int SPI_HIGH_FREQUENCY = 1000000;		// 10 MHz

	public:
		virtual ~MPU9250_Bus() {};
		//virtual esp_err_t initializeBus(spi_device_handle_t *bus);
		virtual esp_err_t initializeBus(i2c_port_t *bus);
		virtual esp_err_t writeRegister(uint8_t subAddress, uint8_t data) { return false; };
		virtual esp_err_t readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest) { return false; };
		virtual void setBusLowSpeed() {};
		virtual void setBusHighSpeed() {};
};

class MPU9250_I2C : public MPU9250_Bus
{
	public:
		MPU9250_I2C(i2c_port_t * bus) : _bus(*bus) {};
		~MPU9250_I2C() {};

		/* Configure and Initialize I2C bus */
		esp_err_t initializeBus(i2c_port_t *bus);

		/* writes a byte to MPU9250 register given a register address and data */
		esp_err_t writeRegister(uint8_t subAddress, uint8_t data);

		/* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */
		esp_err_t readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);

	private:
		i2c_port_t _bus;

		#define I2C_MASTER_SCL_IO			15     			/*!< gpio number for I2C master clock */
		#define I2C_MASTER_SDA_IO			14				/*!< gpio number for I2C master data  */
		#define I2C_MASTER_TX_BUF_DISABLE	0               /*!< I2C master doesn't need buffer */
		#define I2C_MASTER_RX_BUF_DISABLE	0               /*!< I2C master doesn't need buffer */
};

//class MPU9250_SPI : public MPU9250_Bus
//{
//	public:
//		MPU9250_SPI(spi_device_handle_t * bus) : _bus(*bus) {};
//		~MPU9250_SPI() {};
//
//		/* Configure and Initialize SPI bus */
//		esp_err_t initializeBus(spi_device_handle_t *bus);
//
//		/* writes a byte to MPU9250 register given a register address and data */
//		bool writeRegister(uint8_t subAddress, uint8_t data);
//
//		/* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */
//		void readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
//
//		void setBusLowSpeed();
//
//		void setBusHighSpeed();
//
//	private:
//		#define IMU_HOST    VSPI_HOST
//		#define DMA_CHAN    0
//
//		#define PIN_NUM_MISO 15
//		#define PIN_NUM_MOSI 2
//		#define PIN_NUM_CLK  14
//		#define PIN_NUM_CS   13
//
//		spi_device_handle_t _bus;
//		spi_bus_config_t buscfg;
//	    spi_device_interface_config_t devcfg;
//};

#endif
