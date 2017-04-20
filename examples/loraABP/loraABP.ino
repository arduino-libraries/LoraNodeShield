/*
 * Note : If you're using this example with an Arduino Primo download
 * the ArduinoLowPower library from the library manager.
 *
 */
 
#include "LoRaNode.h"

void setup() {  
  pinMode(31, OUTPUT);
  digitalWrite(31, HIGH);

  Serial.begin(9600);

  //            devAddr  , NetworkSessionKey                 , AppSessionKey   
  node.joinABP("26011Ad0", "F60F30Cf0900ce09E07301104E02b0D3", "0CE04d0b30C80970D30a00A101d01001");
  //register callback for incoming messages
  node.onReceive(readMsg);
  //begin initialization
  node.begin();    
  
  node.showStatus();
}

void loop() {
  //send a confirmed frame every 10 seconds
  char frame[] = {0x00, 0x00, 0x00, 0xA0};

  //             data , dataLength   , port, confirmed
  node.sendFrame(frame, sizeof(frame), 2);

  delay(5000);
 }

void readMsg(unsigned char * rcvData, int dim, int port){
  Serial.print("data = ");
  for(int i=0; i<dim; i++)
    Serial.print(rcvData[i], HEX);
  Serial.println();
  Serial.println("on port " + String(port));
}
