/*
 *
 * 		I2C_hejnasa.h
 *		author: Łukasz Hejna
 *		09.2020r.
 * 
 */

#ifndef hejnas_I2C_h
#define hejnas_I2C_h

#include "Arduino.h"

#define delay_1us __asm__ __volatile__ ("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t")

#define  i2cbitdelay 3

#define  I2C_ACK  1
#define  I2C_NAK  0

#define i2c_scl_lo   digitalWrite(_scl,LOW)
#define i2c_scl_hi   digitalWrite(_scl,HIGH)
#define i2c_sda_lo   digitalWrite(_sda,LOW)
#define i2c_sda_hi   digitalWrite(_sda,HIGH)

#define thST        delayMicros(4)
#define twSCLL      delayMicros(1)
#define twSCLH      delayMicros(3)
#define tSUSP       delayMicros(4)
#define tSUSR       delayMicros(4)

#define I2C_ST      thST;I2C_sda_0;thST;I2C_scl_0;thST;I2C_sda_1;delayMicros(10)
#define I2C_SP      tSUSP;I2C_sda_0;tSUSP;I2C_scl_1;tSUSP;I2C_sda_1;tSUSP
#define I2C_SR      I2C_sda_1;tSUSR

class hejnas_i2c
{
	
	public:
		hejnas_i2c(uint8_t scl, uint8_t sda, uint8_t address);
    hejnas_i2c(uint8_t scl, uint8_t sda);

    size_t write(uint8_t _register, uint8_t _data);

    uint8_t read(uint8_t _register, uint8_t *_data, size_t quantity);
		
	private:
    uint8_t beginTransmission();
    uint8_t beginTransmission(uint8_t address);
    uint8_t requestFrom();
    uint8_t requestFrom(uint8_t address);
    uint8_t endTransmission(void);
  
		uint8_t _address = 0x00;
    uint8_t _scl;
    uint8_t _sda;
    
    uint8_t i2c_write( uint8_t c );
    uint8_t i2c_read(uint8_t ack);
    
    uint8_t i2c_readbit(void);
    void i2c_writebit( uint8_t c );
    
    void i2c_start(void);
    void i2c_stop(void);

    uint8_t initialized;   
     
    //uint8_t i2c_read_8bit( uint8_t ack );

    void _delay_us(uint32_t ile_us);
		
};//end of class hejnas_i2c

#endif
