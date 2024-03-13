/*
 * I2C_BitBang.cpp
 *
 *  Created on: Nov 11, 2022
 *      Author: hugo
 */

#include "I2C_BitBang.hpp"


I2C_BitBang::I2C_BitBang() {
	slaveAddress = 0xFF; //The address is only 7-bits, so this not valid address!!! But we use to indicate that the address is not transmitted yet!

	error = I2CBB_ERROR_NONE;
	//i2cMode = MODE_I2C;			//default is I2c Mode. There is also SCCB mode (e.g. for cam OV7670)
	speedSelect = SPEED_100k;	//default speed 100kHz
	stretchTime_us = 5000;		//in microseconds

	rxBufferIndex = 0;


	/*initalizing timing according to i2c standards
	 * https://www.analog.com/en/technical-articles/i2c-timing-definition-and-specification-guide-part-2.html#
	 */
	init_timings();

}

I2C_BitBang::~I2C_BitBang() {

}

void I2C_BitBang::begin() {
	rxBufferIndex = 0;
}

uint8_t I2C_BitBang::requestFrom(uint8_t address, uint8_t quantity) {
	slaveAddress = address;	//7bit address
	i2c_bus_init();
	i2c_start_cond();

	bool     nack = 1;
	if (slaveAddress != 0xFF) {
		if (i2c_write_byte((slaveAddress << 1) | 0x01)) {
			slaveAddress = 0xFF; //The address is transmitted
			i2c_stop_cond();
			return nack;
		}
		slaveAddress = 0xFF; //The address is transmitted
	}

	int j = 0;
	for (j = 0; j < quantity; j++)
	{
		readBuffer[j] = i2c_read_byte(0x00); // read data
	}
	i2c_stop_cond();

	// set rx buffer iterator vars
	rxBufferIndex = 0;

	//rxBufferLength = read;
	return j;
}



int I2C_BitBang::read(void) {

	  int value = -1;

	  // get each successive byte on each call
	  if(rxBufferIndex < 48){
	    value = readBuffer[rxBufferIndex];
	    ++rxBufferIndex;
	  }

	  return value;
}


void I2C_BitBang::beginTransmission(uint8_t address) {
	slaveAddress = address;	//7bit address
	i2c_bus_init();
	i2c_start_cond();
}

size_t I2C_BitBang::write(uint8_t data) {
	bool     nack = 1;
	if (slaveAddress != 0xFF) {
		if (i2c_write_byte((slaveAddress << 1) & 0xFE)) {
			slaveAddress = 0xFF; //The address is transmitted
			i2c_stop_cond();
			return nack;
		}
		slaveAddress = 0xFF; //The address is transmitted
	}
	if (!i2c_write_byte(data))
		return 0;
	i2c_stop_cond();
	return nack;
}

uint8_t I2C_BitBang::endTransmission(void) {
	i2c_stop_cond();
	return 0;
}

///////////////////////////////////////////////////////////////////////////////////
// Write a byte to I2C bus. Return 0 if ack by the slave.
bool I2C_BitBang::i2c_write_byte(unsigned char byte) {
  unsigned bit;
  bool     nack;

  for (bit = 0; bit < 8; ++bit)
  {
	i2c_write_bit((byte & 0x80) != 0);
	byte <<= 1;
  }

  nack = i2c_read_bit();

  set_SDA();

  return nack; //0-ack, 1-nack
}

// Read a byte from I2C bus
unsigned char I2C_BitBang::i2c_read_byte(bool nack) {
  unsigned char byte = 0;
  unsigned char bit;

  for (bit = 0; bit < 8; ++bit) {
    byte = (byte << 1) | i2c_read_bit();
  }

  i2c_write_bit(nack);
  set_SDA();

  return byte;
}


/**
  * @brief  Checks if target device is ready for communication.
  * @note   This function is used with Memory devices
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @retval  status
  */
bool I2C_BitBang::IsDeviceReady(uint16_t DevAddress) {
	uint8_t ack = 0;

	i2c_start_cond();

    uint8_t i;
    for (i = 0; i < 8; i++)
    {
    	i2c_write_bit(DevAddress & 0x80); // write the most-significant bit
    	DevAddress <<= 1;
    }

    ack = i2c_read_bit();

   	i2c_stop_cond();

    return !ack; //0-ack, 1-nack
}

bool I2C_BitBang::writeData(uint8_t reg, uint8_t *pData, uint16_t size) {



	//return !ack; //0-ack, 1-nack
}

//void init_timings(void);	//initialize timing structure t according to i2c standard
//void arbitration_lost(void);
//void clock_stretching(uint32_t t_us);
