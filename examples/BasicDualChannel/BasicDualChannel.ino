#include "SC16IS750.h"

// this sketch will print all incoming data from either serial port @ 9600 baud
// it toggles sending data to channel 1 and channel 2 delimited by newlines

SC16IS750 i2cUart;

void setup() {
  Serial.begin(115200);

  i2cUart.setCyrstalFrequencyMHz(14.7456f); // can omit, default
  i2cUart.setChannel(0);
  i2cUart.configureUart(9600);
  Serial.print("UART Channel 0 ... ");
  if (i2cUart.uartConnected()) {
    Serial.println("OK");
  } else {
    Serial.println("Failed");
  }

  i2cUart.setChannel(1);
  i2cUart.configureUart(9600);
  Serial.print("UART Channel 1 ... ");
  if (i2cUart.uartConnected()) {
    Serial.println("OK");
  } else {
    Serial.println("Failed");
  }
}

uint8_t txPort = 0;

void loop() {
  i2cUart.setChannel(0);
  if (i2cUart.available() > 0) {
    Serial.print("CH0: ");
    while (i2cUart.available() > 0) {
      Serial.write(i2cUart.read());
    }
    Serial.println();
  }

  i2cUart.setChannel(1);
  if (i2cUart.available() > 0) {
    Serial.print("CH1: ");
    while (i2cUart.available() > 0) {
      Serial.write(i2cUart.read());
    }
    Serial.println();
  }

  if (Serial.available() > 0) {
    while (Serial.available() > 0) {
      byte ch = Serial.read();
      if (ch == '\n') {
        txPort = 1 - txPort;         // don't do this for single uart
        i2cUart.setChannel(txPort);
      }
      i2cUart.write(ch);
    }
  }

}
