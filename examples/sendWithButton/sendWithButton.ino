#include "SPI.h"
#include "LoRaNode.h"
#include <Wire.h>
#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

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

char frame[] = {0x00};

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
  
  mlx.begin();
}

void loop() {

  //debounce button to send the frame just once at pressure
  int reading = digitalRead(button);
  if (reading != lastButtonState) {
      lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {

      if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == HIGH) {
        // read a value from the temperature sensor and send it
        int temp = mlx.readAmbientTempC();
        frame[0] = temp;
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