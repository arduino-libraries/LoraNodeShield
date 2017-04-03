#include "SPI.h"
#include "LoRaNode.h"

const char * devAddr = "26011AD0";
const char * nwkSessionKey = "F60F30Cf0900ce09E07301104E02b0D3";
const char * appSessionKey = "0CE04d0b30C80970D30a00A101d01001";

int ledState = LOW;
int lastButtonState = LOW;
int buttonState;
long lastDebounceTime = 0;
long debounceDelay = 50;

const int led = 6;
const int button = A2;


void setup() {  
  pinMode(31, OUTPUT);
  digitalWrite(31, HIGH);
  
  pinMode(button, INPUT);
  pinMode(led, OUTPUT);
  
  Serial.begin(9600);
   
  node.joinABP(devAddr, nwkSessionKey, appSessionKey);
  //register callback for incoming messages
  node.onReceive(readMsg);
  //begin initialization
  node.begin();    
  
  node.showStatus();
}

void loop() {
   char frame[] = {0x00, 0x00, 0x00, 0xA0};

  //debounce button to send the frame just once at pressure
  int reading = digitalRead(button);
  if (reading != lastButtonState) {
      lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {

      if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == HIGH) {
        //send
        node.sendFrame(frame, sizeof(frame), 2);
        Serial.println("data sent");
      }
    }
  }
    lastButtonState = reading;
}

void readMsg(unsigned char * rcvData, int dim, int port){
  // toggle the led if the desidered value is received
  if(rcvData[0] == 'a'){
    ledState = !ledState;
    digitalWrite(led, ledState);
    }
}
