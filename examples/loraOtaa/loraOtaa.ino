#include "SPI.h"
#include "LoRaNode.h"

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
 node.showStatus();
  Serial.println("joining...");
  //begin initialization and attempt join
  node.begin();  
  Serial.println("joined!");

}

void loop() {
 // send an unconfirmed frame every 10 seconds
  char frame[] = {0x00, 0x00, 0x00, 0xA0};

//               data , dataLength   , port
  node.sendFrame(frame, sizeof(frame), 2);

  delay(10000);
}

void readMsg(unsigned char * rcvData, int dim, int port){
  Serial.print("data = ");
  for(int i=0; i<dim; i++)
    Serial.print(rcvData[i], HEX);
  Serial.println();
  Serial.println("on port " + String(port));
}
