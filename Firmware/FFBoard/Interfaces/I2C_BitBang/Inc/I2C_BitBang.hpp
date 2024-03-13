/*
 * I2C_BitBang.hpp
 *
 *  Created on: Nov 10, 2022
 *  Modified on: 11.02.2024
 *      Author: hugo
 */

#ifndef I2C_BITBANG_INC_I2C_BITBANG_HPP_
#define I2C_BITBANG_INC_I2C_BITBANG_HPP_

#include <stdint.h>
#include <stddef.h>
#include "BitBang_Delay.hpp"


#define	I2CBB_ERROR_NONE 			0x0UL
#define	I2CBB_ERROR_I2CBB 			0x1UL
#define	I2CBB_ERROR_NACK 			0x2UL
#define	I2CBB_ERROR_STRETCH_TOUT	0x4UL
#define	I2CBB_ERROR_ARBITRATION 	0x8UL

typedef enum
{
	SPEED_100k = 0,
	SPEED_400k = 1,
	SPEED_50k = 2,
	SPEED_10k = 3
}i2cbbSpeed_t;

typedef enum
{
	ACK_CHECK = 0,
	ACK_IGNORE = 1	//for SCCB interface
}ackMode_t;

class I2C_BitBang : private BitBang_Delay {
public:
	//Libary Native Functions
	I2C_BitBang();
	~I2C_BitBang();
	bool writeData(uint8_t reg, uint8_t *pData, uint16_t size);

	void i2c_gpio_init(void);

	//Arduino TwoWire Functions copy-cat'ish
	void    begin();
	uint8_t requestFrom(uint8_t address, uint8_t quantity);
	int    read(void);
	void    beginTransmission(uint8_t address);
	size_t write(uint8_t data);
	uint8_t endTransmission(void);

	bool IsDeviceReady(uint16_t DevAddress);

private:
	//void i2c_gpio_init(void);
	void i2c_bus_init(void);
	bool read_SCL(void);  // Return current level of SCL line, 0 or 1
	bool read_SDA(void);  // Return current level of SDA line, 0 or 1
	void set_SCL(void);   // Do not drive SCL (set pin high-impedance)
	void clear_SCL(void); // Actively drive SCL signal low
	void set_SDA(void);   // Do not drive SDA (set pin high-impedance)
	void clear_SDA(void); // Actively drive SDA signal low
	BitBang_Delay I2C_Delay;	//delay

	void i2c_start_cond(void);
	void i2c_stop_cond(void);
	void i2c_write_bit(bool bit);
	bool i2c_read_bit(void);
	bool i2c_write_byte(unsigned char byte);
	void init_timings(void);	//initialize timing structure t according to i2c standard
	unsigned char i2c_read_byte(bool nack);
	void arbitration_lost(void);
	void clock_stretching(uint32_t t_us);


	i2cbbSpeed_t speedSelect;
	uint8_t readBuffer[48];
	uint8_t rxBufferIndex;
	ackMode_t ackMode;
	uint8_t slaveAddress;
	uint32_t error;
	uint32_t stretchTime_us;
	bool started; // global data
	struct t 			/*timing according to i2c standards*/
	{
		uint16_t hdsta;		//hold time for start condition /the minimum time the DTA should be low before SCL goes low/
		uint16_t susta;		//set-up time for repeated START condition
		uint16_t susto;		//setup time for stop condition
		uint16_t sudat;		//the minimum amount of time required for SDA to have reached a stable level before an SCL transition takes place
		uint16_t dvdat;		//data validity time
		uint16_t dvack;		//acknoqledge validity time
		uint16_t high;		//high period of SCL
		uint16_t low;		//low period of SCL /IMPORTANT !!!must be > dvdat and > sudat/
		uint16_t buff;		//buffer time /the bus free time between stop and start conditions/
	}t;

};




//using TwoWire = I2C_BitBang;
/*
class TwoWire
{
private:
    static uint8_t rxBuffer[];
    static size_t  rxBufferIndex;
    static size_t  rxBufferLength;

    static uint8_t txAddress;
    static uint8_t txBuffer[];
    static size_t  txBufferIndex;
    static size_t  txBufferLength;

    static uint8_t transmitting;
    static void (*user_onRequest)(void);
    static void (*user_onReceive)(size_t);
    static void onRequestService(void);
    static void onReceiveService(uint8_t*, size_t);

public:
    TwoWire();
    void    begin(int sda, int scl);
    void    begin(int sda, int scl, uint8_t address);
    void    pins(int sda, int scl) __attribute__((deprecated));  // use begin(sda, scl) in new code
    void    begin();
    void    begin(uint8_t);
    void    begin(int);
    void    setClock(uint32_t);
    void    setClockStretchLimit(uint32_t);
    void    beginTransmission(uint8_t);
    void    beginTransmission(int);
    uint8_t endTransmission(void);
    uint8_t endTransmission(uint8_t);
    size_t  requestFrom(uint8_t address, size_t size, bool sendStop);
    uint8_t status();

    uint8_t requestFrom(uint8_t, uint8_t);
    uint8_t requestFrom(uint8_t, uint8_t, uint8_t);
    uint8_t requestFrom(int, int);
    uint8_t requestFrom(int, int, int);

    virtual size_t write(uint8_t);
    virtual size_t write(const uint8_t*, size_t);
    virtual int    available(void);
    virtual int    read(void);
    virtual int    peek(void);
    virtual void   flush(void);
    void           onReceive(void (*)(int));     // arduino api
    void           onReceive(void (*)(size_t));  // legacy esp8266 backward compatibility
    void           onRequest(void (*)(void));

//    using Print::write; TTW
};
*/
/*
#ifdef __cplusplus
}
#endif
*/




#endif /* I2C_BITBANG_INC_I2C_BITBANG_HPP_ */
