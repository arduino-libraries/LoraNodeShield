/*
 * TinkerkitExample.ino
 * 
 * This example shows how to use the connectors on the shield
 * to interact with the LoRaWan network.
 * Connect a tinkerkit button to IN2, a tinkerkit LDR sensor
 * to IN3, a tinkerkit buzzer to OUT5 and a tinkerkit led to
 * OUT6.
 * Every time the button is pressed a packet cointaining the
 * value read from the LDR sensor and a count of the number
 * of button pressure will be sent (note that the packet is
 * sent in the Cayenne format). If the node receive a packet
 * with value 'a' the buzzer will sound. Any other received
 * packet will change the led's status.

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

const char * appEui = "00250C0000010001";
const char * appKey = "0E708C34BBDA282AA8691318BCD06BBB";
const char * devEui = "00250C010000062F";

volatile int ledState = LOW;
int lastButtonState = LOW;
int buttonState;
char buttonCnt = 0;
long lastDebounceTime = 0;
long debounceDelay = 50;

const int led = 6;
const int button = A2;
const int buzzer = 5;
const int ldrSensor = A3;

//              | cayenne button |  illuminance senor    |
char frame[7] = {0x00, 0x01, 0x00, 0x01, 0x65, 0x00, 0x00};

void setup() {  
  pinMode(31, OUTPUT);
  digitalWrite(31, HIGH);
  
  pinMode(button, INPUT);
  pinMode(led, OUTPUT);
  pinMode(ldrSensor, INPUT);
  digitalWrite(ldrSensor, ledState);
  
  Serial.begin(9600);

  node.joinOTAA(appEui, appKey, devEui);
  //register callback for incoming messages
  node.onReceive(readMsg);
  //begin initialization
  node.begin();    
  
  node.showStatus();
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
        // read a value from the LDR sensor and send it
        int light = analogRead(ldrSensor);
        // illuminance value has to be 2 byte long (MSB)
        frame[5] = (light & 0xFF00) >> 8;
        frame[6] = light & 0x00FF;
        // send button count also
        frame[2] = ++buttonCnt;
      
        node.sendFrame(frame, sizeof(frame), 2);
      }
    }
  }
    lastButtonState = reading;
}

void readMsg(unsigned char * rcvData, int dim, int port){
  // make a sound if the desidered value is received
  if(rcvData[0] == 'a'){
    tone(buzzer, 262, 500);
   }
   // toggle the led if anything else was received 
   else{
    ledState = !ledState;
    digitalWrite(led, ledState);
   }
}
