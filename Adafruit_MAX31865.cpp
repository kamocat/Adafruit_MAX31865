/*************************************************** 
  This is a library for the Adafruit PT100/P1000 RTD Sensor w/MAX31865

  Designed specifically to work with the Adafruit RTD Sensor
  ----> https://www.adafruit.com/products/3328

  This sensor uses SPI to communicate, 4 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_MAX31865.h"
#ifdef __AVR
  #include <avr/pgmspace.h>
#elif defined(ESP8266)
  #include <pgmspace.h>
#endif

#include <stdlib.h>
#include <SPI.h>

static SPISettings max31865_spisettings = SPISettings(500000, MSBFIRST, SPI_MODE1);


// Software (bitbang) SPI
Adafruit_MAX31865::Adafruit_MAX31865(int8_t spi_cs, int8_t spi_mosi, int8_t spi_miso, int8_t spi_clk) {
  _sclk = spi_clk;
  _cs = spi_cs;
  _miso = spi_miso;
  _mosi = spi_mosi;

}

// Hardware SPI init
Adafruit_MAX31865::Adafruit_MAX31865(int8_t spi_cs) {
  _cs = spi_cs;
  _sclk = _miso = _mosi = -1;
}

boolean Adafruit_MAX31865::begin(max31865_numwires_t wires,
  float RTDnominal, float refResistor) {
  RnomInv = 1./RTDnominal;
  Rref = refResistor/32768.;
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);

  if (_sclk != -1) {
    //define pin modes
    pinMode(_sclk, OUTPUT); 
    digitalWrite(_sclk, LOW);
    pinMode(_mosi, OUTPUT); 
    pinMode(_miso, INPUT);
  } else {
    //start and configure hardware SPI
    SPI.begin();
  }  

  for (uint8_t i=0; i<16; i++) {
    // readRegister8(i);
  }

  setWires(wires);
  enableBias(false);
  autoConvert(false);
  clearFault();

  //Serial.print("config: "); Serial.println(readRegister8(MAX31856_CONFIG_REG), HEX);
  return true;
}


uint8_t Adafruit_MAX31865::readFault(void) {
  return readRegister8(MAX31856_FAULTSTAT_REG);
}

void Adafruit_MAX31865::clearFault(void) {
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  t &= ~0x2C;
  t |= MAX31856_CONFIG_FAULTSTAT;
  writeRegister8(MAX31856_CONFIG_REG, t);
}

void Adafruit_MAX31865::enableBias(boolean b) {
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  if (b) {
    t |= MAX31856_CONFIG_BIAS;       // enable bias
  } else {
    t &= ~MAX31856_CONFIG_BIAS;       // disable bias
  }
  writeRegister8(MAX31856_CONFIG_REG, t);
}

void Adafruit_MAX31865::autoConvert(boolean b) {
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  if (b) {
    t |= MAX31856_CONFIG_MODEAUTO;       // enable autoconvert
  } else {
    t &= ~MAX31856_CONFIG_MODEAUTO;       // disable autoconvert
  }
  writeRegister8(MAX31856_CONFIG_REG, t);
}

void Adafruit_MAX31865::setWires(max31865_numwires_t wires ) {
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  if (wires == MAX31865_3WIRE) {
    t |= MAX31856_CONFIG_3WIRE;
  } else {
    // 2 or 4 wire
    t &= ~MAX31856_CONFIG_3WIRE;
  }
  writeRegister8(MAX31856_CONFIG_REG, t);
}

float  Adafruit_MAX31865::temperature(uint8_t order) {
  // http://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf
  // http://www.mosaic-industries.com/embedded-systems/microcontroller-projects/temperature-measurement/platinum-rtd-sensors/resistance-calibration-table

  float Rt, temp;

  Rt = readRTD();
  Rt *= Rref;
  
  // Serial.print("\nResistance: "); Serial.println(Rt, 8);
  if( order == 4 ){
    // 4th-order rational function
    float Z1, Z2, Z3, Z4;
    Z1 = -RTD_A;
    Z2 = RTD_A * RTD_A - (4 * RTD_B);
    Z3 = (4 * RTD_B) * RnomInv;
    Z4 = 2 * RTD_B;
    temp = Z2 + (Z3 * Rt);
    temp = (sqrt(temp) + Z1) / Z4;
  } else if( order == 5 ){
    // 5th-order polynomial.
    // Not sure where these coeffecients come from
    Rt *= 100*RnomInv;      // normalize to 100 ohm
    float rpoly = Rt;
    temp = -242.02;
    temp += 2.2228 * rpoly;
    rpoly *= Rt;  // square
    temp += 2.5859e-3 * rpoly;
    rpoly *= Rt;  // ^3
    temp -= 4.8260e-6 * rpoly;
    rpoly *= Rt;  // ^4
    temp -= 2.8183e-8 * rpoly;
    rpoly *= Rt;  // ^5
    temp += 1.5243e-10 * rpoly;
  } else {
    // First-order approximation. Very fast, and good for most purposes
    temp = (Rt*RnomInv - 1) * (1e6/3850.);
  }

  return temp;
}

uint16_t Adafruit_MAX31865::readRTD (void) {
  clearFault();
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  if( !t & MAX31856_CONFIG_BIAS ){
    enableBias(true);
    delay(13);
  }
  if( !t & MAX31856_CONFIG_MODEAUTO ){
    t |= MAX31856_CONFIG_1SHOT;      
    writeRegister8(MAX31856_CONFIG_REG, t);
    delay(65);
  }
  // If you're using autoconvert, we assume that you are
  // either using the ~DRDY pin, or have set up delays
  // somewhere else in your code.

  uint16_t rtd = readRegister16(MAX31856_RTDMSB_REG);

  // remove fault
  rtd >>= 1;

  return rtd;
}

/**********************************************/

uint8_t Adafruit_MAX31865::readRegister8(uint8_t addr) {
  uint8_t ret = 0;
  readRegisterN(addr, &ret, 1);

  return ret;
}

uint16_t Adafruit_MAX31865::readRegister16(uint8_t addr) {
  uint8_t buffer[2] = {0, 0};
  readRegisterN(addr, buffer, 2);

  uint16_t ret = buffer[0];
  ret <<= 8;
  ret |=  buffer[1];
  
  return ret;
}


void Adafruit_MAX31865::readRegisterN(uint8_t addr, uint8_t buffer[], uint8_t n) {
  addr &= 0x7F; // make sure top bit is not set

  if (_sclk == -1)
    SPI.beginTransaction(max31865_spisettings);
  else 
    digitalWrite(_sclk, LOW);

  digitalWrite(_cs, LOW);

  spixfer(addr);

  //Serial.print("$"); Serial.print(addr, HEX); Serial.print(": ");
  while (n--) {
    buffer[0] = spixfer(0xFF);
    //Serial.print(" 0x"); Serial.print(buffer[0], HEX);
    buffer++;
  }
  //Serial.println();

  if (_sclk == -1)
    SPI.endTransaction();

  digitalWrite(_cs, HIGH);
}


void Adafruit_MAX31865::writeRegister8(uint8_t addr, uint8_t data) {
  if (_sclk == -1)
    SPI.beginTransaction(max31865_spisettings);
  else 
    digitalWrite(_sclk, LOW);

  digitalWrite(_cs, LOW);

  spixfer(addr | 0x80);   // make sure top bit is set
  spixfer(data);

  //Serial.print("$"); Serial.print(addr, HEX); Serial.print(" = 0x"); Serial.println(data, HEX);

  if (_sclk == -1)
    SPI.endTransaction();

  digitalWrite(_cs, HIGH);
}



uint8_t Adafruit_MAX31865::spixfer(uint8_t x) {
  if (_sclk == -1)
    return SPI.transfer(x);

  // software spi
  //Serial.println("Software SPI");
  uint8_t reply = 0;

  for (int i=7; i>=0; i--) {
    reply <<= 1;
    digitalWrite(_sclk, HIGH);
    digitalWrite(_mosi, x & (1<<i));
    digitalWrite(_sclk, LOW);
    if (digitalRead(_miso))
      reply |= 1;
  }

  return reply;
}
