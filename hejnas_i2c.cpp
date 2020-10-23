/*
 * 		I2C_hejnasa.h
 *		author: Łukasz Hejna
 *		09.2020r.
 *
 * jest to biblioteka obsługująca i2c jedynie jedynie z poziomu software
 * nie wykożystywane są przerwania, timery itp.
 */

#include "hejnas_i2c.h"


/////////////////////////////////////////////////////////////////////////
// PUBLIC   PUBLIC   PUBLIC   PUBLIC   PUBLIC   PUBLIC   PUBLIC   PUBLIC
/////////////////////////////////////////////////////////////////////////



/**********************************************************************
 *
 * inicjacja biblioteki bez domyślnego adresu
 * 
 **********************************************************************/
hejnas_i2c::hejnas_i2c(uint8_t scl, uint8_t sda)
{
  _scl = scl;
  _sda = sda;
  
  pinMode(_scl, OUTPUT);
  pinMode(_sda, OUTPUT);

  i2c_sda_hi;
  i2c_scl_hi;
 
}//end of hejnas_i2c::hejnas_i2c(uint8_t scl, uint8_t sda)



/**********************************************************************
 * 
 * inicjacja biblioteki z deklaracją domyślnego adresu
 * 
 **********************************************************************/
hejnas_i2c::hejnas_i2c(uint8_t scl, uint8_t sda, uint8_t address)
{
  _scl = scl;
  _sda = sda;
  
  pinMode(_scl, OUTPUT);
  pinMode(_sda, OUTPUT);

  i2c_sda_hi;
  i2c_scl_hi;

  _address = address;
 
}//end of hejnas_i2c::hejnas_i2c(uint8_t scl, uint8_t sda, uint8_t address)



/**********************************************************************
 * 
 * wpisuje jeden bajt do zadeklarowanego rejestru
 * 
 **********************************************************************/
size_t hejnas_i2c::write(uint8_t _register, uint8_t _data)
{
    beginTransmission(_address);
    i2c_write(_register);
    i2c_write(_data);
    endTransmission();
    
}//end of hejnas_i2c::write



/**********************************************************************
 * 
 * odczytuje quantity bitów rozpoczynając od zadeklarowanego rejestru
 * 
 **********************************************************************/
uint8_t hejnas_i2c::read(uint8_t _register, uint8_t *_data, size_t quantity)
{
  uint8_t return_value = 1;
  
  beginTransmission(_address);
  i2c_write(_register);
  endTransmission();

  requestFrom(_address);
  int i=0;
  for(i=0;i<quantity;i++)
  {
    if(i!=(quantity - 1))_data[i] = i2c_read(I2C_ACK);
      else _data[i] = i2c_read(I2C_NAK);
  }  
  endTransmission();
  
  return return_value;
  
}//end of hejnas_i2c::read(uint8_t _register, uint8_t *_data, size_t quantity)



/////////////////////////////////////////////////////////////////////////
//   PRIVATE   PRIVATE   PRIVATE   PRIVATE   PRIVATE   PRIVATE   PRIVATE
/////////////////////////////////////////////////////////////////////////


/**********************************************************************
 * 
 * beginTransmission
 * 
 **********************************************************************/
uint8_t hejnas_i2c::beginTransmission()
{
  i2c_start();  
  uint8_t rc = i2c_write((_address<<1) | 0);  
  initialized = rc;
  
  return rc; 
  
}//end of hejnas_i2c::beginTransmission()


uint8_t hejnas_i2c::beginTransmission(uint8_t address)
{
  i2c_start();  
  uint8_t rc = i2c_write((address<<1) | 0);  
  initialized = rc;
  
  return rc; 
  
}//end of hejnas_i2c::beginTransmission(uint8_t address)



/**********************************************************************
 * 
 *  uint8_t requestFrom
 *  
 **********************************************************************/
uint8_t hejnas_i2c::requestFrom()
{
  i2c_start();  
  uint8_t rc = i2c_write((_address<<1) | 1);
  
  initialized = rc;
  
  return rc; 
  
}//end of requestFrom(uint8_t address);


uint8_t hejnas_i2c::requestFrom(uint8_t address)
{
  i2c_start();  
  uint8_t rc = i2c_write((address<<1) | 1);
  
  initialized = rc;
  
  return rc; 
  
}//end of requestFrom(uint8_t address);



/**********************************************************************
 * 
 * 
 * 
 **********************************************************************/
uint8_t hejnas_i2c::endTransmission(void)
{
    i2c_stop();
    return initialized;   // Use the result of beginTransmission()
}





/**********************************************************************
 * 
 * 
 * 
 **********************************************************************/
void hejnas_i2c::i2c_start(void)
{
    // set both to high at the same time
    //I2C_DDR &=~ (_BV( I2C_SDA ) | _BV( I2C_SCL ));
    //*_sclDirReg &=~ (_sdaBitMask | _sclBitMask);
    i2c_sda_hi;
    i2c_scl_hi;

    _delay_us(i2cbitdelay);

    i2c_sda_lo;
    _delay_us(i2cbitdelay);

    i2c_scl_lo;
    _delay_us(i2cbitdelay);
}


/*
 * 
 * 
 */
void hejnas_i2c::i2c_stop(void)
{
    i2c_scl_hi;
    _delay_us(i2cbitdelay);

    i2c_sda_hi;
    _delay_us(i2cbitdelay);
}


/**********************************************************************
 * 
 * 
 * 
 **********************************************************************/
uint8_t hejnas_i2c::i2c_read( uint8_t ack )
{
    uint8_t res = 0;

    for ( uint8_t i=0;i<8;i++) {
        res <<= 1;
        res |= i2c_readbit();
    }

    if ( ack )
        i2c_writebit( 0 );
    else
        i2c_writebit( 1 );

    _delay_us(i2cbitdelay);

    return res;
}


/**********************************************************************
 * 
 * 
 * 
 **********************************************************************/
uint8_t hejnas_i2c::i2c_write( uint8_t c )
{
    for ( uint8_t i=0;i<8;i++) {
        i2c_writebit( c & 128 );
        c<<=1;
    }

    return i2c_readbit();
}


/**********************************************************************
 * 
 * 
 * 
 **********************************************************************/
uint8_t hejnas_i2c::i2c_readbit(void)
{
    i2c_sda_hi;
    i2c_scl_hi;
    _delay_us(i2cbitdelay);

    uint8_t c = digitalRead(_sda);

    i2c_scl_lo;
    _delay_us(i2cbitdelay);

    return c;
}


/**********************************************************************
 * 
 * 
 * 
 **********************************************************************/
void hejnas_i2c::i2c_writebit( uint8_t c )
{
    if ( c > 0 ) {
        i2c_sda_hi;
    } else {
        i2c_sda_lo;
    }

    i2c_scl_hi;
    _delay_us(i2cbitdelay);

    i2c_scl_lo;
    _delay_us(i2cbitdelay);

    if ( c > 0 ) {
        i2c_sda_lo;
    }
    _delay_us(i2cbitdelay);
}


/**********************************************************************
 * 
 * 
 * 
 **********************************************************************/
//int hejnas_i2c::read( uint8_t ack )
//{
//  return i2c_read_8bit( ack );
//}


/**********************************************************************
 * 
 * 
 * 
 **********************************************************************/
//uint8_t hejnas_i2c::requestFrom(uint8_t address)
//{
//    i2c_start();
//    uint8_t rc = i2c_write((address<<1) | 1); // set read bit
//    return rc;
//}


/**********************************************************************
 * 
 * 
 * 
 **********************************************************************/
//uint8_t hejnas_i2c::requestFrom(uint8_t address, uint8_t quantity)
//{
//    return requestFrom( (uint8_t) address);

    // Ignore 'quantity', since SoftI2CMaster::requestFrom() just sets the start of read adresses,
    // so it's the same for any number of bytes.
//    (void)quantity;
//}


/**********************************************************************
 * 
 * czekaj około 1us
 * 
 **********************************************************************/
void hejnas_i2c::_delay_us(uint32_t ile_us)
{
  for(uint32_t i_delay_us = 0; i_delay_us < ile_us; i_delay_us++)delay_1us;
    
}//end of hejnas_i2c::_delay_us
