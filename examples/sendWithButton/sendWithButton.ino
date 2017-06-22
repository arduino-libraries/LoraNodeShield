/*
 * sendWithButton.ino
 * 
 * This example shows how to use the connectors on the shield
 * to interact with the LoRaWan network.
 * Connect a tinkerkit temperature sensor to TWI connector, 
 * a tinkerkit button to IN2 and a tinkerkit led to OUT6.
 * Every time the button is pressed a packet cointaining the
 * value read from the temp sensor and a count of the number
 * of button pressure will be sent (note that the packet is
 * sent in the Cayenne format).Every time the node receive a
 * packet with the 'a' value the led will change status.

 * Change the keys below to fit the application in your server.
 * 
 * Note : If you're using this example with an Arduino Primo download
 * the ArduinoLowPower library from the library manager.
 *
 * To use the temperature value download the Adafruit_MLX90614
 * library from the library manager.
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

#include <Wire.h>
#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

const char * appEui = "00250C0000010001";
const char * appKey = "0E708C34BBDA282AA8691318BCD06BBB";
const char * devEui = "00250C010000062F";


int ledState = LOW;
int lastButtonState = LOW;
int buttonState;
char buttonCnt = 0;
long lastDebounceTime = 0;
long debounceDelay = 50;

const int led = 6;
const int button = A2;

//              | cayenne button | cayenne temp sensor   |
char frame[7] = {0x00, 0x01, 0x00, 0x01, 0x67, 0x00, 0x00};

void setup() {  
  pinMode(31, OUTPUT);
  digitalWrite(31, HIGH);
  
  pinMode(button, INPUT);
  pinMode(led, OUTPUT);
  
  Serial.begin(9600);
   

  //node.joinABP(devAddr, nwkSessionKey, appSessionKey);
  node.joinOTAA(appEui, appKey, devEui);
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
        float temp = mlx.readAmbientTempC();
        // temperature value has to be 2 byte long (MSB)
        // example: 27.25°C => 2725
        int temperature = temp * 100;
        frame[5] = (temperature & 0xFF00) >> 8;
        frame[6] = temperature & 0x00FF;
        // send button count also
        frame[2] = ++buttonCnt;
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