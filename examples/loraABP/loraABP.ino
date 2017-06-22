/*
 * loraABP.ino
 * 
 * A simple example to use the Arduino LoRa Node shield.
 * This example join the LoRaWan network and send periodically
 * packets to the server. The activation method used is
 * Activation By Personalization (ABP).
 * If you register an application on a server with the same
 * keys used in this example you will be able to see the
 * packets sent from the node.
 * Change the keys below to fit the application in your server.
 * 
 * Note : If you're using this example with an Arduino Primo download
 * the ArduinoLowPower library from the library manager.
 *
 * This example code is in the public domain.
 * 
 * created April 2017
 * by chiara@arduino.org
 *
 ***********************************************************************
 * NOTE:
 * Frequency is defined in config.h file inside the src folder of this
 * library. Edit this file to select the frequency that fit your region.
 ***********************************************************************
 *
 */

// Application's keys
const char * deviceAddress         = "26011AD0";
const char * networkSessionKey     = "F60F30CF0900CE09E07301104E02B0D3";
const char * applicationSessionKey = "0CE04D0B30C80970D30A00A101D01001";

#include "LoRaNode.h"

void setup() {  
  pinMode(31, OUTPUT);
  digitalWrite(31, HIGH);

  Serial.begin(9600);

  //register the keys for this application
  node.joinABP(deviceAddress, networkSessionKey, applicationSessionKey);
  //register callback for incoming messages
  node.onReceive(readMsg);
  //begin initialization
  node.begin();    
  //show the node's main parameters
  node.showStatus();
}

void loop() {
  //send a frame every 10 seconds
  char frame[] = {0x00, 0x00, 0x00, 0xA0};

  //             data , dataLength   , port
  node.sendFrame(frame, sizeof(frame), 2);

  //wait 10 seconds sleeping to save power
  node.sleep(10000);
 }

void readMsg(unsigned char * rcvData, int dim, int port){
  Serial.print("data = ");
  for(int i=0; i<dim; i++)
    Serial.print(rcvData[i], HEX);
  Serial.println();
  Serial.println("on port " + String(port));
}