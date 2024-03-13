/*
 * I2C_BitBang_Driver.cpp
 *
 *  Created on: Nov 11, 2022
 *      Author: hugo
 */

//#include "main.h"
#include "I2C_BitBang.hpp"

#define SW_I2C_SDA_Pin GPIO_PIN_9
#define SW_I2C_SDA_GPIO_Port GPIOC
#define SW_I2C_SCL_Pin GPIO_PIN_8
#define SW_I2C_SCL_GPIO_Port GPIOA

#define I2C_CLEAR_SDA HAL_GPIO_WritePin(SW_I2C_SDA_GPIO_Port, SW_I2C_SDA_Pin, GPIO_PIN_RESET);
#define I2C_SET_SDA HAL_GPIO_WritePin(SW_I2C_SDA_GPIO_Port, SW_I2C_SDA_Pin, GPIO_PIN_SET);
#define I2C_CLEAR_SCL HAL_GPIO_WritePin(SW_I2C_SCL_GPIO_Port, SW_I2C_SCL_Pin, GPIO_PIN_RESET);
#define I2C_SET_SCL HAL_GPIO_WritePin(SW_I2C_SCL_GPIO_Port, SW_I2C_SCL_Pin, GPIO_PIN_SET);

#define I2C_DELAY DWT_Delay_us(5); // 5 microsecond delay

/*
 * timings are in hns units (0.1 um units)
 * e.g. 47 means 4.7 us
 */
#define MARGIN_100	-10
#define MARGIN_400  -6
#define MARGIN_50    10
#define MARGIN_10    0

#if (MARGIN_400 < -6)
#error "In I2Cbitbang class: MARGIN_100 need to be larger than -6"
#endif
#if (MARGIN_100 < -36)
#error "In I2Cbitbang class: MARGIN_100 need to be larger than 36"
#endif

//ToDo for pin init!!!

void I2C_BitBang::i2c_gpio_init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SW_I2C_SDA_GPIO_Port, SW_I2C_SDA_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SW_I2C_SCL_GPIO_Port, SW_I2C_SCL_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin : SW_I2C_SDA_Pin */
	GPIO_InitStruct.Pin = SW_I2C_SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(SW_I2C_SDA_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SW_I2C_SCL_Pin */
	GPIO_InitStruct.Pin = SW_I2C_SCL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(SW_I2C_SCL_GPIO_Port, &GPIO_InitStruct);

}



void I2C_BitBang::i2c_bus_init(void)
{
	error = I2CBB_ERROR_NONE;
	set_SDA();
	set_SCL();

	//TODO confirm that SDA and SCL are set, otherwise bus error
}

bool I2C_BitBang::read_SCL(void)  // Return current level of SCL line, 0 or 1
{

	if (HAL_GPIO_ReadPin(SW_I2C_SCL_GPIO_Port, SW_I2C_SCL_Pin) == GPIO_PIN_SET)
	        return 1;
	    else
	        return 0;

	return 0;
}
bool I2C_BitBang::read_SDA(void)  // Return current level of SDA line, 0 or 1
{

	if (HAL_GPIO_ReadPin(SW_I2C_SDA_GPIO_Port, SW_I2C_SDA_Pin) == GPIO_PIN_SET)
	        return 1;
	    else
	        return 0;
	return 0;
}

void I2C_BitBang::set_SCL(void)   // Do not drive SCL (set pin high-impedance)
{
	I2C_SET_SCL
}

void I2C_BitBang::clear_SCL(void) // Actively drive SCL signal low
{
	I2C_CLEAR_SCL
}

void I2C_BitBang::set_SDA(void)   // Do not drive SDA (set pin high-impedance)
{
	I2C_SET_SDA
}

void I2C_BitBang::clear_SDA(void) // Actively drive SDA signal low
{
	I2C_CLEAR_SDA
}


void I2C_BitBang::i2c_start_cond(void) {
	if (started)
	{
		// if started, do a restart condition
		// set SDA to 1
		set_SDA();
		I2C_Delay.hns(t.susta);
		set_SCL();

		clock_stretching(stretchTime_us);
		// Repeated start setup time, minimum 4.7us
		I2C_Delay.hns(t.susta);
	}

	//TODO implement arbitration
	/*
	conf_SDA(I2CBB_INPUT);
	if (read_SDA() == 0)
	{
	arbitration_lost();
	}
	conf_SDA(I2CBB_OUTPUT);
	*/

	// SCL is high, set SDA from 1 to 0.
	clear_SDA();
	I2C_Delay.hns(t.hdsta);
	clear_SCL();
	I2C_Delay.hns(t.low);
	started = true;
}

void I2C_BitBang::i2c_stop_cond(void) {
	// set SDA to 0
	clear_SDA();
	//  I2C_delay();
	I2C_Delay.hns(t.susto);

	set_SCL();

	clock_stretching(stretchTime_us);

	// Stop bit setup time, minimum 4us
	I2C_Delay.hns(t.susto);

	// SCL is high, set SDA from 0 to 1
	set_SDA();
	I2C_Delay.hns(t.buff);

	//TODO implement arbitration
	/*  if (read_SDA() == 0) {
	arbitration_lost();
	}
	*/

	started = false;
}

void I2C_BitBang::i2c_write_bit(bool bit) {
	if (bit) {
		set_SDA();
	}
	else {
		clear_SDA();
	}

	// SDA change propagation delay
	I2C_Delay.hns(t.sudat);

	// Set SCL high to indicate a new valid SDA value is available
	set_SCL();

	// Wait for SDA value to be read by slave, minimum of 4us for standard mode
	I2C_Delay.hns(t.high);

	clock_stretching(stretchTime_us);

	//TODO implement arbitration
	// SCL is high, now data is valid
	// If SDA is high, check that nobody else is driving SDA
	/*  conf_SDA(I2CBB_INPUT);
	if (bit && (read_SDA() == 0)) {
	arbitration_lost();
	}
	conf_SDA(I2CBB_OUTPUT);
	*/
	// Clear the SCL to low in preparation for next change
	clear_SCL();
	I2C_Delay.hns(t.low);
}

bool I2C_BitBang::i2c_read_bit(void) {
	bool bit;

	// Let the slave drive data
	set_SDA();

	// Wait for SDA value to be written by slave, minimum of 4us for standard mode
	I2C_Delay.hns(t.dvdat);

	// Set SCL high to indicate a new valid SDA value is available
	set_SCL();
	clock_stretching(stretchTime_us);

	// Wait for SDA value to be written by slave, minimum of 4us for standard mode
	I2C_Delay.hns(t.high);

	// SCL is high, read out bit
	bit = read_SDA();

	// Set SCL low in preparation for next operation
	clear_SCL();

	I2C_Delay.hns(t.low);

	return bit;
}

void I2C_BitBang::clock_stretching(uint32_t t_us)
{
	  while ( (read_SCL() == 0) && (t_us > 0) )
	  {
		I2C_Delay.us(1);
		t_us--;
		if (t_us == 0)
			error |= I2CBB_ERROR_STRETCH_TOUT;
	  }
}

void I2C_BitBang::init_timings(void) {
	switch (speedSelect )
	{
		case SPEED_400k:
			t.hdsta = 6 + MARGIN_400;
			t.susto = 6 + MARGIN_400;
			t.susta = 6 + MARGIN_400;
			t.sudat = 1;
			t.dvdat = 9 + MARGIN_400;
			t.dvack = 9 + MARGIN_400;
			t.high = 6 + MARGIN_400;
			t.low = 13 + MARGIN_400;
			t.buff = 13 + MARGIN_400;
			break;
		case SPEED_10k:
			t.hdsta = 400 + MARGIN_10;
			t.susto = 400 + MARGIN_10;
			t.susta = 470 + MARGIN_10;
			t.sudat = 30;
			t.dvdat = 360 + MARGIN_10;
			t.dvack = 360 + MARGIN_10;
			t.high = 400 + MARGIN_10;
			t.low = 470 + MARGIN_10;
			t.buff = 470 + MARGIN_10;
			break;
		case SPEED_50k:
			t.hdsta = 80 + MARGIN_50;
			t.susto = 80 + MARGIN_50;
			t.susta = 94 + MARGIN_50;
			t.sudat = 6;
			t.dvdat = 72 + MARGIN_50;
			t.dvack = 72 + MARGIN_50;
			t.high = 80 + MARGIN_50;
			t.low = 94 + MARGIN_50;
			t.buff = 94 + MARGIN_50;
			break;
		case SPEED_100k:
		default:
			t.hdsta = 40 + MARGIN_100;
			t.susto = 40 + MARGIN_100;
			t.susta = 47 + MARGIN_100;
			//t.sudat = 3;
			t.sudat = 30;
			t.dvdat = 36 + MARGIN_100;
			t.dvack = 36 + MARGIN_100;
			//t.high = 40 + MARGIN_100;
			t.high = 80 + MARGIN_100;
			t.low = 47 + MARGIN_100;
			t.buff = 47 + MARGIN_100;
			break;
	}
}

