/* 16-12-13 Small Utility Program to configure the JY-MCU Bluetooth Board 
 
 Based on the following resources
 @from http://club.dx.com/forums/Forums.dx/threadid.1166641
 @see http://arduino.cc/en/Reference/SoftwareSerial
 
 From http://ecno92.blogspot.co.uk/2012/11/jy-mcu-linvor-at-commands-change-name.html
 
 AT+VERSION
 Returns the software version of the module
 OKlinvorV1.x
 
 AT+BAUDx
 Sets the baud rate of the module 
 The command AT+BAUD8 sets the 
 baud rate to 115200
 1 >> 1200, 2 >> 2400, 3 >> 4800, 4 >> 9600 (Default), 5 >> 19200, 6 >> 38400 
 7 >> 57600, 8 >> 115200, 9 >> 230400
 
 AT+NAMEOpenPilot
 Sets the name of the module
 Any name can be specified up to 20 characters
 OKsetname
 
 AT+PINxxxx
 Sets the pairing password of the device
 Any 4 digit number can be used, the default 
 pincode is 1234
 OKsetPIN
 
 AT+PN
 Sets the parity of the module
 AT+PN >> No parity check
 OK None
 
 */

#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 11); // RX, TX

void setup()
{
  Serial.begin(9600);
  Serial.println("JY-MCU Configuration");
  mySerial.begin(9600);
  //mySerial.begin(115200); //Change for New JY-MCU


  delay(1000);
  mySerial.print("AT");
  delay(1000);
  mySerial.print("AT+VERSION");
  delay(1000);

  mySerial.print("AT+NAMENode3");
  delay(1000);
  mySerial.print("AT+PIN0000");
  delay(1000);
  //Re-ordered as this was happening to early in process. 
  //mySerial.print("AT+BAUD8");
  //delay(1000);

  Serial.println("Initial Config Complete");
}

void loop() // run over and over
{
  if (mySerial.available())
    Serial.write(mySerial.read());
  if (Serial.available())
    mySerial.write(Serial.read());
} 

