/*
  SC16IS750.cpp - Library for interfacing with SC16IS750 i2c to serial and gpio port expander with Arduino
  Created by Eric O'Neill, July 30, 2015

  Modified by Victor Aprea, July 13, 2020 to add support for Dual UART SC16IS752
  and to support baudrate calculation in place
*/

#include "SC16IS750.h"
#include "Arduino.h"
#include "Wire.h"

SC16IS750::SC16IS750(int address) {
  _outputRegVal = 0x00;
  _inputRegVal = 0x00;
  _deviceAddress = address;
  _channel = 0;
  _crystalFreqMHz = 14.7456f;  // default crystal
}

//-------------------- private functions ---------------------------
byte SC16IS750::patchChannel(byte subAddress) {
  subAddress &= 0b11111001;       // clear bits 2:1
                                  // then set bits 2:1 to _channel
  subAddress |= (_channel << 1);  // |= 0b00000000 or 0b00000010
  return subAddress;
}

byte SC16IS750::readRegister(byte regAddress) {
  // patch in the channel number
  regAddress = patchChannel(regAddress);

  Wire.beginTransmission(_deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t) _deviceAddress, (uint8_t) 1);
  if (Wire.available()) {
    return Wire.read();  // read values in the input register
  } else {
    Serial.print("failed to read");
  }
}

void SC16IS750::writeRegister(byte regAddress, byte data) {
  regAddress = patchChannel(regAddress);
  Wire.beginTransmission(_deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

void SC16IS750::configureUart(uint32_t baudRate) {
  writeRegister(LCR, 0x80);     // 0x80 to program baudrate

  uint16_t brr = (uint16_t) round((_crystalFreqMHz * 1e6) /
    (baudRate * 16.0f));
  uint8_t brrh = ((brr >> 8) & 0xff);
  uint8_t brrl = (brr & 0xff);
  writeRegister(DLL, brrl);     // 0x60 =   9600 baud for XTAL = 14.7456 MHz
                                // 0x08 = 115200 baud for XTAL = 14.7456 MHz
  writeRegister(DLH, brrh);     // 0x00 in most cases

  writeRegister(LCR, 0xBF);     // access EFR register

  // writeRegister(EFR, 0xD0);  // enable enhanced registers
  writeRegister(EFR, 0x10);     // enables the enhanced fn IER[7:4], FCR[5:4],
                                // and MCR[7:5] so that they can be modified.
  writeRegister(LCR, 0x03);     // 8 data bit, 1 stop bit, no parity

  // writeRegister(FCR, 0x06);  // reset TXFIFO, reset RXFIFO, non FIFO mode
  // writeRegister(FCR, 0x01);  // enable FIFO mode
  writeRegister(IER, 0x00);     // this configures Polled mode (see 7.5.2)
  writeRegister(FCR, 0x07);     // reset TXFIFO, reset RXFIFO, FIFO enabled
  // writeRegister(EFCR, 0x00);
}

bool SC16IS750::uartConnected() {
  const char TEST_CHARACTER = 'H';

  writeRegister(SPR, TEST_CHARACTER);

  return (readRegister(SPR) == TEST_CHARACTER);
}

//-------------------- public functions ---------------------------

// must configure pins to be inputs or outputs before using them.
// Output: 1, Input: 0
void SC16IS750::configurePins(byte pinConfig) {
  _pinConfig = pinConfig;
  writeRegister(IODIR, pinConfig);
  writeRegister(IOCTRL, 0x00);
  writeRegister(IOINTMSK, 0xFF);
}

int SC16IS750::writePin(int pin, bool val) {
  if (((_pinConfig >> pin) & 0x01) == 0) {
    return -1;  // pin config is set to input
  }
  Wire.beginTransmission(_deviceAddress);
  Wire.write(IOSTATE);
  // set bit at pin
  if (val) {
    _outputRegVal |= 1 << pin;
  } else {
    // clear bit at pin
    _outputRegVal &= ~(1 << pin);
  }
  Wire.write(_outputRegVal);
  if (Wire.endTransmission() == 0) {
    return 1;
  } else {
    return -1;
  }
}

int SC16IS750::readPin(int pin) {
  if ((_pinConfig >> pin) == 0) {
    return -1;  // pin configuration is set to output
  }
  Wire.beginTransmission(_deviceAddress);
  Wire.write(IOSTATE);
  Wire.endTransmission();

  Wire.requestFrom((uint8_t) _deviceAddress, (uint8_t) 1);
  if (Wire.available()) {
    _inputRegVal = Wire.read();  // read values in the input register
  }
  // return value from input register corresponding
  // to the pin specified in parameter
  return (_inputRegVal >> pin) & 0x01;
}

byte SC16IS750::available() {
  /*

    Get the number of bytes (characters) available for reading.

    This is data that's already arrived and stored in the receive
    buffer (which holds 64 bytes).

   */
  // This alternative just checks if there's data but doesn't
  // return how many characters are in the buffer:
  //    readRegister(LSR) & 0x01
  return readRegister(RXLVL);
}

byte SC16IS750::txBufferSize() {
  return readRegister(TXLVL);
}

int SC16IS750::read() {
  return readRegister(RHR);
}

void SC16IS750::write(byte value) {
  /*

    Write byte to UART.

   */
  while (readRegister(TXLVL) == 0) {
    // Wait for space in TX buffer
  }
  writeRegister(THR, value);
}

// legal values are 0 and 1
// and 1 is only valid if the target chip supports it (e.g. SC16IS752)
void SC16IS750::setChannel(uint8_t channel) {
  if (channel < 2) {
  _channel = channel;
  }
}

void SC16IS750::setCyrstalFrequencyMHz(float freqMHz) {
  _crystalFreqMHz = freqMHz;
}

void SC16IS750::setDeviceAddress(uint8_t address) {
  _deviceAddress = address;
}
