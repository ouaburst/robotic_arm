/* 
 * Arduino code to send and receive I2C data
 * Tested on Adafruit Feather M0+ Express and Raspberry Pi Model 4
 * 
 * SDA <--> SDA
 * SCL <--> SCL
 * GND <--> GND
 * 
 * Sets built-in LED (1 = on, 0 = off) on Feather when requested 
 * and responds with data received
 */

#include <Wire.h>
#define SLAVE_ADDRESS 0x04       // I2C address for Arduino
#define LED 13                   // Built-in LED
int i2cData = 0;                 // the I2C data received

char rx_byte = 0;
int sendValue;

void setup(){
  Serial.begin(19200);
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
}
void loop() {
  // Everything happens in the interrupts

  if (Serial.available() > 0) {   
    rx_byte = Serial.read();    

    if ((rx_byte >= '0') && (rx_byte <= '9')) {
      Serial.print("Number sended: ");
      Serial.println(rx_byte);
      sendValue = rx_byte;
      
    }
  }
}
// Handle reception of incoming I2C data
void receiveData(int byteCount) {
  while (Wire.available()) {
    i2cData = Wire.read();

    Serial.print("Recived: ");   
    Serial.println(i2cData);       
  }
}
// Handle request to send I2C data
void sendData() { 
  Wire.write(sendValue);
}
