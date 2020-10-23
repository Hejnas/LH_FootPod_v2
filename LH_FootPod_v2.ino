/*
 * _scl = 1;  //pin dla footpod na nrf51822  
 * _sda = 0;  //pin dla footpod na nrf51822
 * 
 * 
 */
#include <BLEPeripheral.h>
#include "hejnas_i2c.h"

#define ProjectName "\nLH_FootPod_v2.ino\n"

//
//  deklaracja przerwania LSM6DS3_int_1 pin 06
//
const byte LSM6DS3_int_1_PIN = 3;
volatile byte LSM6DS3_int_1_state = LOW;

//
// deklaracja i2c
//
const int sdaPin = 0;
const int sclPin = 1;
hejnas_i2c i2c = hejnas_i2c( sclPin, sdaPin, 0x6B);

// define pins (varies per shield/board)
#define BLE_REQ   10
#define BLE_RDY   2
#define BLE_RST   9

int16_t     rawX = 0;
int16_t     rawY = 0;
int16_t     rawZ = 0;
uint8_t     accelRange = 16;      //Max G force readable.  Can be: 2, 4, 8, 16
uint8_t     raw[6];
float       accelX_ms2;   //przyspieszenie X w m/s2
float       accelY_ms2;   //przyspieszenie Y w m/s2
float       accelZ_ms2;   //przyspieszenie Z w m/s2
float       accel_ms2;    //długość wektora przyspieszenia wypadkowego minu przyspieszenie ziemskie s=(X^2+Y^2+Z^2)^0,5-gn w m/s2

//zwiększenie dokładności pomiarów
int32_t    rawX_srednia, rawY_srednia, rawZ_srednia;
uint8_t     ile_pomiarow = 10;
uint8_t     licznik_pomiarow = 0;


const float gn = 9.80665;  //gn = 9,80665 m/s^2

uint16_t    inst_speed = 0x0100;     //  Unit is in m/s with a resolution of 1/256 (256/3.6)*kmph
uint8_t     inst_cadence = 160;   //  Unit is in 1/minute (or RPM) with a resolutions of 1 1/min (or 1 RPM)

uint32_t    aktualnyCzas = 0;
uint32_t    zapamietanyCzasBLE = 0;
uint16_t    interwalCzasuBLE = 250;
uint32_t    zapamietanyCzasLSM6DS3  = 0;
uint16_t    interwalCzasuLSM6DS3  = 10;

BLEPeripheral            blePeripheral                  = BLEPeripheral(BLE_REQ, BLE_RDY, BLE_RST);
BLEService               RSCService                     = BLEService("1814");  
BLECharacteristic        RSCMeasurementCharacteristics  = BLECharacteristic("2A53", BLENotify, 4); //org.bluetooth.characteristic.rsc_measurement" uuid="2A53"
BLECharCharacteristic    RSCFeatureCharacteristics      = BLECharCharacteristic("2A54", BLERead);

void setup()
{ 
  Serial.begin(115200);
  delay(500);
  Serial.println(ProjectName);
 
  BLE_init();
  
  initAccel();
  
}

void loop()
{
  aktualnyCzas = millis();  //zapamiętanie czasu w ms

  if(digitalRead(LSM6DS3_int_1_PIN) == HIGH) //jeżeli LSM6DS3 zgłosi że ma pomiar
  {
    readRaw();  //odczytanie wartości raw
    licznik_pomiarow++;

    //sumowanie rawX, rawY, rawZ
    rawX_srednia = rawX_srednia + rawX;    
    rawY_srednia = rawY_srednia + rawY;
    rawZ_srednia = rawZ_srednia + rawZ;

    //jeżeli wykonano wlasciwa liczbe pomiarow do sredniej to ja oblicz
    if(licznik_pomiarow == ile_pomiarow)
    {
      licznik_pomiarow = 0; //zerowanie licznika pomiarow

      //obliczenie srednij
      rawX_srednia = rawX_srednia / 10;    
      rawY_srednia = rawY_srednia / 10;
      rawZ_srednia = rawZ_srednia / 10;

      //wpisanie do rawX, rawY, rawZ usrednionych pomiarow
      rawX = rawX_srednia;
      rawY = rawY_srednia;
      rawZ = rawZ_srednia;

      rawX_srednia = 0;
      rawY_srednia = 0;
      rawZ_srednia = 0;
      
      //obliczenie wartości przyspieszeń
      readAccel();

      Serial.print(aktualnyCzas);
      Serial.print(";");
      Serial.print(accelX_ms2,4);
      Serial.print(";");
      Serial.print(accelY_ms2,4);
      Serial.print(";");
      Serial.print(accelZ_ms2,4);
      Serial.print(";");
      Serial.print(accel_ms2,4);
      Serial.print("\n");
      
    }    
  }
    

  // BLE jest aktualizowane co czas = interwalCzasuBLE
  if(aktualnyCzas > zapamietanyCzasBLE + interwalCzasuBLE )
  {
    zapamietanyCzasBLE = aktualnyCzas;   //zapamiętanie nowego czsu odniesienia
    zapamietanyCzasBLE = zapamietanyCzasBLE / interwalCzasuBLE;
    zapamietanyCzasBLE = zapamietanyCzasBLE * interwalCzasuBLE;
    
    inst_speed = uint16_t(accel_ms2);
    inst_speed = inst_speed<<8;
    accel_ms2 = accel_ms2 - inst_speed;
    accel_ms2 = accel_ms2 * 256;
    inst_speed = inst_speed | uint8_t(accel_ms2);
    //Serial.print(";0x");
    //Serial.print(inst_speed,HEX);
    //Serial.print("\n");

    
    // listen for BLE peripherals to connect:
    BLECentral central = blePeripheral.central();  
    byte charArray[4] = {
        0b1100,
        (unsigned byte)inst_speed, (unsigned byte)(inst_speed >> 8),
        (unsigned byte)inst_cadence};  
    RSCMeasurementCharacteristics.setValue(charArray,4);

  }

}


/************************************************************************
 * 
 * BLE_init
 * 
 ************************************************************************/
 void BLE_init()
 {
  // set advertised local name and service UUID
  blePeripheral.setLocalName("Hejnas FootPod");
  blePeripheral.setDeviceName("Hejnas FootPod");
  blePeripheral.setAdvertisedServiceUuid(RSCService.uuid());  // add the service UUID
  blePeripheral.addAttribute(RSCService);   // Add the BLE Heart Rate service
  blePeripheral.addAttribute(RSCMeasurementCharacteristics);
  blePeripheral.addAttribute(RSCFeatureCharacteristics);
  RSCFeatureCharacteristics.setValue(0);

  // begin initialization
  blePeripheral.begin();

  Serial.println(F("BLE RSC Peripheral"));
  
 }//end of BLE_init


/************************************************************************
 * 
 * initAccel
 * 
 ************************************************************************/
 void initAccel()
 {
  //INT1_CTRL (0Dh) INT1 pad control register
  i2c.write(0x0D, 0x01);    //Accelerometer data-ready on INT1 pad


  /* -------------------------------------------------------------
   *  
   * CTRL1_XL (10h) Linear acceleration sensor control register 1
   * 
   * ------------------------------------------------------------- */

  //  Output data rate and power mode selection. Default value: 0000
  //  (0000: Power-down; 0001: 12.5 Hz; 0010: 26 Hz; 0011: 52 Hz; 0100: 104 Hz; 0101: 208 Hz;
  //   0110: 416 Hz; 0111: 833 Hz; 1000: 1.66 kHz; 1001: 3.33 kHz; 1010: 6.66 kHz
  uint8_t ODR_XL = 0b0101;

  //  Accelerometer full-scale selection. Default value: 00.
  //  (00: ±2 g; 01: ±16 g; 10: ±4 g; 11: ±8 g)
  uint8_t FS_XL = 0b01;

  //Anti-aliasing filter bandwidth selection. Default value: 00
  //(00: 400 Hz; 01: 200 Hz; 10: 100 Hz; 11: 50 Hz)
  uint8_t BW_XL = 0b00;
  
  i2c.write(0x10, ODR_XL << 4 | FS_XL << 2 | BW_XL);

  /* -------------------------------------------------------------
   *  
   * CTRL2_G (11h) Angular rate sensor control register 2
   * 
   * ------------------------------------------------------------- */

  i2c.write(0x11, 0x00);    //gyro power down

  //CTRL4_C (13h) Control register 4
  //Accelerometer bandwidth determined by setting BW_XL[1:0] in CTRL1_XL (10h) register.)
  i2c.write(0x13, 0x80);    //

  pinMode(LSM6DS3_int_1_PIN, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(LSM6DS3_int_1_PIN), LSM6DS3_int_1, RISING);
  
 }//end off initAccel


void LSM6DS3_int_1()
{
  LSM6DS3_int_1_state = !LSM6DS3_int_1_state;
  
}//end of LSM6DS3_int_1()

/************************************************************************
 * 
 *    readAccel
 *    
 ************************************************************************/
void readAccel()
{
  accelX_ms2 = calcAccel_ms2(rawX);
  accelY_ms2 = calcAccel_ms2(rawY);
  accelZ_ms2 = calcAccel_ms2(rawZ);

  accel_ms2 = sqrt(accelX_ms2*accelX_ms2 + accelY_ms2*accelY_ms2 + accelZ_ms2*accelZ_ms2);
  accel_ms2 = accel_ms2 - gn;
  
}//end off readAccel();


void readRaw()
{
  i2c.read(0x28,raw,6);
  
  rawX = uint16_t(raw[0]) | uint16_t(raw[1])<<8;
  rawY = uint16_t(raw[2]) | uint16_t(raw[3])<<8;
  rawZ = uint16_t(raw[4]) | uint16_t(raw[5])<<8;
  
}//end of readRaw


/* **********************************************************************
 *  
 *  calcAccel m/s^2
 *  
 * **********************************************************************/
float calcAccel_ms2(int16_t input)
{
  float output = (float)input * 0.061 * (accelRange >> 1) / 1000;
  output = output * gn;
  return output;
}
