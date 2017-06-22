/*
  Copyright (c) 2016 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef __LORA_NODE_H_
#define __LORA_NODE_H_

#include "Arduino.h"
#define USE_MODEM_LORA

#include "config.h"
#include "boards/arduino/board.h"

#ifdef __cplusplus
extern "C"{
#endif //__cplusplus

#include "mac/region/Region.h"
#include "radio/radio.h"
#include "system/gpio.h"
#include "system/timer.h"
#include "mac/LoRaMac.h"

#if defined ( REGION_EU868 )
 #include "mac/LoRaMacTest.h"
#endif

#include "boards/mcu/arduino/utilities.h"

#ifdef __cplusplus
}
#endif //__cplusplus

#if defined ( REGION_EU868 )

#define LC4                { 867100000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC5                { 867300000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC6                { 867500000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC7                { 867700000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC8                { 867900000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC9                { 868800000, 0, { ( ( DR_7 << 4 ) | DR_7 ) }, 2 }
#define LC10               { 868300000, 0, { ( ( DR_6 << 4 ) | DR_6 ) }, 1 }

#endif

#define LORAWAN_APP_DATA_MAX_SIZE                   64
#define LORAWAN_DEFAULT_DATARATE                    DR_0


/*!
 * LoRaWAN compliance tests support data
 */

typedef void (*LoRaNodeEventHandler)(unsigned char * buffer, int buffSize, int port);

enum SF{
	SF7,
	SF8,
	SF9,
	SF10,
	SF11,
	SF12
};

class LoRaNode {
 public:
	LoRaNode();
	
	void onReceive(LoRaNodeEventHandler handler);	//function for registering a callback to be called when a message is received
	
	void begin();  //begin initialization function
	void joinOTAA(const char *appEui, const char *appKey); //function to configure the otaa parameters
	void joinOTAA(const char *appEui, const char *appKey, const char *devEui);
	void joinABP(const char * devAddr, const char * nwkSKey, const char * appSKey); //function to configure the ABP parameters
	void sendFrame(char frame[], int dim, int port = 1, bool confirmed = false);  //function to send a frame
	void poll(int port, bool confirm = false);	//function to send a dummy frame in order to open the receive windows
	void showStatus();	//print node parameters
	void sleep(uint32_t ms);
	
	void setSF(SF sf);
	void setTxPower(int power);

	bool                     nextTx;
	bool                     joined;
	LoRaNodeEventHandler     onReceiveCallback;
	const char *             _appEui;
	const char *             _appKey;
	const char *             _devEui;
	uint8_t                  _appDataSize; //set function
	uint8_t                  _appData[LORAWAN_APP_DATA_MAX_SIZE];
	uint8_t	                 _appPort;
	bool                     _confirmed;

  private:
	bool send(int port, bool confirmed);
  
	bool                     _otaa;
	const char *             _devAddr;
	const char *             _nwkSKey;
	const char *             _appSKey;
    LoRaMacPrimitives_t      _LoRaMacPrimitives;
    LoRaMacCallback_t        _LoRaMacCallbacks;
    MibRequestConfirm_t      _mibReq;
	bool                     _initialized;
	uint8_t                  _dr;
	uint8_t                  _power;

};

extern LoRaNode node;

#endif __LORA_NODE_H_