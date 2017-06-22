/*
 * loraOtaa.ino
 * 
 * A simple example to use the Arduino LoRa Node shield.
 * This example join the LoRaWan network and send periodically
 * packets to the server. The activation method used is
 * Over The Air Activation (OTAA). With this method you won't
 * be able to send any packet until the activation procedure
 * is complete (in this case the begin function is a blocking
 * function that returns only when the activation is complete).
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
 
#include "LoRaNode.h"

// Application's keys
const char * appEui = "70B3D57EF0003411";
const char * appKey = "B304CB0BAA52A353E819725F894D34F8";
const char * devEui = "00000000B266BE73";

void setup() {
  pinMode(31, OUTPUT);
  digitalWrite(31, HIGH);

  Serial.begin(9600);

  node.joinOTAA(appEui, appKey, devEui);

  //register callback for incoming messages
  node.onReceive(readMsg);

  //show the node's main parameters
  node.showStatus();
  
  Serial.println("joining...");
  
  //begin initialization and attempt join
  node.begin();  

  Serial.println("joined!");  
}

void loop() {
  // send an unconfirmed frame every 10 seconds
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
