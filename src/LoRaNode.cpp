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

#include "LoRaNode.h"

#ifdef ARDUINO_ARCH_NRF52
  #include "ArduinoLowPower.h"
#endif

#define NR_OF_TRIALS	48

struct ComplianceTest_s
{
    bool Running;
    uint8_t State;
    bool IsTxConfirmed;
    uint8_t AppPort;
    uint8_t AppDataSize;
    uint8_t *AppDataBuffer;
    uint16_t DownLinkCounter;
    bool LinkCheck;
    uint8_t DemodMargin;
    uint8_t NbGateways;
}ComplianceTest;

static uint8_t IsTxConfirmed = false;
static uint8_t AppPort = 1;
static uint8_t AppDataSize = 16;

void convertAddress(uint32_t& DevAddr, const char * devAddr){
	// address is MSB
	uint8_t first;
	uint8_t second;
	uint8_t third;
	uint8_t fourth;
	first   = (devAddr[7] >= 'A' && devAddr[7] <= 'F') ? (devAddr[7] - 'A' + 10) : (devAddr[7] >= 'a' && devAddr[7] <= 'f') ? (devAddr[7] - 'a' + 10) : (devAddr[7] - '0');
	first  += (devAddr[6] >= 'A' && devAddr[6] <= 'F') ? ((devAddr[6] - 'A' + 10) << 4) : (devAddr[6] >= 'a' && devAddr[6] <= 'f') ? ((devAddr[6] - 'a' + 10) << 4) : ((devAddr[6] - '0') << 4);
	second  = (devAddr[5] >= 'A' && devAddr[5] <= 'F') ? (devAddr[5] - 'A' + 10) : (devAddr[5] >= 'a' && devAddr[5] <= 'f') ? (devAddr[5] - 'a' + 10) : (devAddr[5] - '0');
	second += (devAddr[4] >= 'A' && devAddr[4] <= 'F') ? ((devAddr[4] - 'A' + 10) << 4) : (devAddr[4] >= 'a' && devAddr[4] <= 'f') ? ((devAddr[4] - 'a' + 10) << 4) : ((devAddr[4] - '0') << 4);
	third   = (devAddr[3] >= 'A' && devAddr[3] <= 'F') ? (devAddr[3] - 'A' + 10) : (devAddr[3] >= 'a' && devAddr[3] <= 'f') ? (devAddr[3] - 'a' + 10) : (devAddr[3] - '0');
	third  += (devAddr[2] >= 'A' && devAddr[2] <= 'F') ? ((devAddr[2] - 'A' + 10) << 4) : (devAddr[2] >= 'a' && devAddr[2] <= 'f') ? ((devAddr[2] - 'a' + 10) << 4) : ((devAddr[2] - '0') << 4);
	fourth  = (devAddr[1] >= 'A' && devAddr[1] <= 'F') ? (devAddr[1] - 'A' + 10) : (devAddr[1] >= 'a' && devAddr[1] <= 'f') ? (devAddr[1] - 'a' + 10) : (devAddr[1] - '0');
	fourth += (devAddr[0] >= 'A' && devAddr[0] <= 'F') ? ((devAddr[0] - 'A' + 10) << 4) : (devAddr[0] >= 'a' && devAddr[0] <= 'f') ? ((devAddr[0] - 'a' + 10) << 4) : ((devAddr[0] - '0') << 4);
	DevAddr = first | (second << 8) | (third << 16)  | (fourth << 24);
}

void convertKey(uint8_t* skey, const char * _skey, uint8_t len){
	for(int i = 0; i < len; i++){
		skey[i] = (_skey[(i*2) + 1] >= 'A' && _skey[(i*2) + 1] <= 'F') ? (_skey[(i*2) + 1] - 'A' + 10) : (_skey[(i*2) + 1] >= 'a' && _skey[(i*2) + 1] <= 'f') ? (_skey[(i*2) + 1] - 'a' + 10) : (_skey[(i*2) + 1] - '0');
		skey[i] += (_skey[(i*2)] >= 'A' && _skey[(i*2)] <= 'F') ? ((_skey[(i*2)] - 'A' + 10) << 4) : (_skey[(i*2)] >= 'a' && _skey[(i*2)] <= 'f') ? ((_skey[(i*2)] - 'a' + 10) << 4) : ((_skey[(i*2)] - '0') << 4);
	}
}

static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{
 if( mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( mcpsConfirm->McpsRequest )
        {
            case MCPS_UNCONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                break;
            }
            case MCPS_CONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                // Check AckReceived
                // Check NbTrials
                break;
            }
            case MCPS_PROPRIETARY:
            {
                break;
            }
            default:
                break;
        }
    }
    node.nextTx = true;
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication( McpsIndication_t *mcpsIndication )
{
    if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        return;
    }

    switch( mcpsIndication->McpsIndication )
    {
        case MCPS_UNCONFIRMED:
        {
            break;
        }
        case MCPS_CONFIRMED:
        {
            break;
        }
        case MCPS_PROPRIETARY:
        {
            break;
        }
        case MCPS_MULTICAST:
        {
            break;
        }
        default:
            break;
    }

    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot

    if( mcpsIndication->RxData == true )
    {
		if(node.onReceiveCallback)
			node.onReceiveCallback(mcpsIndication->Buffer, mcpsIndication->BufferSize, mcpsIndication->Port);
        switch( mcpsIndication->Port )
        {
        case 1: // The application LED can be controlled on port 1 or 2
        case 2:
            if( mcpsIndication->BufferSize == 1 )
            {
                // AppLedStateOn = mcpsIndication->Buffer[0] & 0x01;
               // GpioWrite( &Led3, ( ( AppLedStateOn & 0x01 ) != 0 ) ? 0 : 1 );
            }
            break;
        case 224:
            if( ComplianceTest.Running == false )
            {
                // Check compliance test enable command (i)
                if( ( mcpsIndication->BufferSize == 4 ) &&
                    ( mcpsIndication->Buffer[0] == 0x01 ) &&
                    ( mcpsIndication->Buffer[1] == 0x01 ) &&
                    ( mcpsIndication->Buffer[2] == 0x01 ) &&
                    ( mcpsIndication->Buffer[3] == 0x01 ) )
                {
                    IsTxConfirmed = false;
                    AppPort = 224;
                    AppDataSize = 2;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.LinkCheck = false;
                    ComplianceTest.DemodMargin = 0;
                    ComplianceTest.NbGateways = 0;
                    ComplianceTest.Running = true;
                    ComplianceTest.State = 1;
                    
                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = true;
                    LoRaMacMibSetRequestConfirm( &mibReq );

#if defined( REGION_EU868 )
                    LoRaMacTestSetDutyCycleOn( false );
#endif
                }
            }
            else
            {
                ComplianceTest.State = mcpsIndication->Buffer[0];
                switch( ComplianceTest.State )
                {
                case 0: // Check compliance test disable command (ii)
                    IsTxConfirmed = node._confirmed;//LORAWAN_CONFIRMED_MSG_ON;
                    AppPort = node._appPort;//LORAWAN_APP_PORT;
                    AppDataSize = node._appDataSize;//LORAWAN_APP_DATA_SIZE;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.Running = false;

                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
					mibReq.Param.AdrEnable = 1;//LORAWAN_ADR_ON;
                    LoRaMacMibSetRequestConfirm( &mibReq );
#if defined( REGION_EU868 )
                    LoRaMacTestSetDutyCycleOn(true /*LORAWAN_DUTYCYCLE_ON*/);
#endif
                       break;
                case 1: // (iii, iv)
                    AppDataSize = 2;
                    break;
                case 2: // Enable confirmed messages (v)
                    IsTxConfirmed = true;
                    ComplianceTest.State = 1;
                    break;
                case 3:  // Disable confirmed messages (vi)
                    IsTxConfirmed = false;
                    ComplianceTest.State = 1;
                    break;
                case 4: // (vii)
                    AppDataSize = mcpsIndication->BufferSize;

                    node._appData[0] = 4;
                    for( uint8_t i = 1; i < AppDataSize; i++ )
                    {
                        node._appData[i] = mcpsIndication->Buffer[i] + 1;
                    }
                    break;
                case 5: // (viii)
                    {
                        MlmeReq_t mlmeReq;
                        mlmeReq.Type = MLME_LINK_CHECK;
                        LoRaMacMlmeRequest( &mlmeReq );
                    }
                    break;
                case 6: // (ix)
                    {
                        MlmeReq_t mlmeReq;

                        // Disable TestMode and revert back to normal operation
                        IsTxConfirmed = node._confirmed;//LORAWAN_CONFIRMED_MSG_ON;
						AppPort = node._appPort;//LORAWAN_APP_PORT;
                        AppDataSize = node._appDataSize;//LORAWAN_APP_DATA_SIZE;
                        ComplianceTest.DownLinkCounter = 0;
                        ComplianceTest.Running = false;

                        MibRequestConfirm_t mibReq;
                        mibReq.Type = MIB_ADR;
                        mibReq.Param.AdrEnable = true;//LORAWAN_ADR_ON;
                        LoRaMacMibSetRequestConfirm( &mibReq );
#if defined( REGION_EU868 )
                        LoRaMacTestSetDutyCycleOn(true /*LORAWAN_DUTYCYCLE_ON */);
#endif

                        mlmeReq.Type = MLME_JOIN;
						
						uint8_t DevEui[8];
						if(node._devEui == NULL)
							BoardGetUniqueId( DevEui );
						else
							convertKey(DevEui, node._devEui, 8);

                        mlmeReq.Req.Join.DevEui = DevEui;
                        mlmeReq.Req.Join.AppEui = (unsigned char*)node._appEui;
                        mlmeReq.Req.Join.AppKey = (unsigned char*)node._appKey;
                        mlmeReq.Req.Join.NbTrials = NR_OF_TRIALS;

                        LoRaMacMlmeRequest( &mlmeReq );
                        // DeviceState = DEVICE_STATE_SLEEP;

                    }
                    break;
                case 7: // (x)
                    {
                        if( mcpsIndication->BufferSize == 3 )
                        {
                            MlmeReq_t mlmeReq;
                            mlmeReq.Type = MLME_TXCW;
                            mlmeReq.Req.TxCw.Timeout = ( uint16_t )( ( mcpsIndication->Buffer[1] << 8 ) | mcpsIndication->Buffer[2] );
                            LoRaMacMlmeRequest( &mlmeReq );
                        }
						else if( mcpsIndication->BufferSize == 7 )
                        {
                            MlmeReq_t mlmeReq;
                            mlmeReq.Type = MLME_TXCW_1;
                            mlmeReq.Req.TxCw.Timeout = ( uint16_t )( ( mcpsIndication->Buffer[1] << 8 ) | mcpsIndication->Buffer[2] );
                            mlmeReq.Req.TxCw.Frequency = ( uint32_t )( ( mcpsIndication->Buffer[3] << 16 ) | ( mcpsIndication->Buffer[4] << 8 ) | mcpsIndication->Buffer[5] ) * 100;
                            mlmeReq.Req.TxCw.Power = mcpsIndication->Buffer[6];
                            LoRaMacMlmeRequest( &mlmeReq );
                        }
                        ComplianceTest.State = 1;
                    }
                    break;
                default:
                    break;
                }
            }
            break;
        default:
            break;
        }
    }
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] mlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
        switch( mlmeConfirm->MlmeRequest )
        {
            case MLME_JOIN:
            {
				            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Status is OK, node has joined the network
				node.joined = true;
            }
            else
            {
                // Join was not successful. Try to join again
                MlmeReq_t mlmeReq;
		
				uint8_t DevEui[8];
				uint8_t AppEui[8];
				uint8_t AppKey[16];
				if(node._devEui == NULL)
					BoardGetUniqueId( DevEui );
				else
					convertKey(DevEui, node._devEui, 8);
				convertKey(AppEui, node._appEui, 8);
				convertKey(AppKey, node._appKey, 16);
				
				mlmeReq.Type = MLME_JOIN;

				mlmeReq.Req.Join.DevEui = DevEui;
				mlmeReq.Req.Join.AppEui = AppEui;
				mlmeReq.Req.Join.AppKey = AppKey;
				mlmeReq.Req.Join.NbTrials = NR_OF_TRIALS;
				if( node.nextTx == true )
				{
					LoRaMacMlmeRequest( &mlmeReq );
				}

            }
            break;
            }
            case MLME_LINK_CHECK:
            {
              if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
              {
                // Check DemodMargin
                // Check NbGateways
                if( ComplianceTest.Running == true )
                {
                    ComplianceTest.LinkCheck = true;
                    ComplianceTest.DemodMargin = mlmeConfirm->DemodMargin;
                    ComplianceTest.NbGateways = mlmeConfirm->NbGateways;
                }
              }
                break;
            }
            default:
                break;
        }
    // }
    node.nextTx = true;
}


LoRaNode::LoRaNode() : 
	nextTx(true),
	joined(false),
	onReceiveCallback(NULL),
	_appDataSize(16),
	_appEui(NULL),
	_appKey(NULL),
	_devEui(NULL),
	_devAddr(NULL),
	_nwkSKey(NULL),
	_appSKey(NULL),
	_initialized(false),
	_dr(DR_0)
{
#ifdef REGION_EU868
	_power = 1;
#elif defined (REGION_US915) | defined (REGION_US915_HYBRID)
	_power = 5;
#endif
}

void LoRaNode::onReceive(LoRaNodeEventHandler handler){
	onReceiveCallback = handler;
}

void LoRaNode::joinOTAA(const char *appEui, const char *appKey){
	_otaa = true;
	_appEui = appEui;
	_appKey = appKey;
	_devEui = NULL;
}

void LoRaNode::joinOTAA(const char *appEui,const char *appKey, const char *devEui){
	_otaa = true;
	_appEui = appEui;
	_appKey = appKey;
	_devEui = devEui;
}

void LoRaNode::joinABP(const char * devAddr, const char * nwkSKey, const char * appSKey){
	_otaa = false;
	_devAddr = devAddr;
	_nwkSKey = nwkSKey;
	_appSKey = appSKey;
}

void LoRaNode::begin(){
	BoardInitMcu( );
    BoardInitPeriph( );
	_initialized = true;
	
	_LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
	_LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
	_LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
#if defined( REGION_EU868 )
                LoRaMacInitialization( &_LoRaMacPrimitives, &_LoRaMacCallbacks, LORAMAC_REGION_EU868, _dr, _power );
#elif defined( REGION_US915 )
                LoRaMacInitialization( &_LoRaMacPrimitives, &_LoRaMacCallbacks, LORAMAC_REGION_US915, _dr, _power );
#elif defined( REGION_US915_HYBRID )
                LoRaMacInitialization( &_LoRaMacPrimitives, &_LoRaMacCallbacks, LORAMAC_REGION_US915_HYBRID, _dr, _power );
#else
    #error "Please define a region in the compiler options."
#endif	
	
	_mibReq.Type = MIB_ADR;
	_mibReq.Param.AdrEnable = 1;//LORAWAN_ADR_ON;
	LoRaMacMibSetRequestConfirm( &_mibReq );

	_mibReq.Type = MIB_PUBLIC_NETWORK;
	_mibReq.Param.EnablePublicNetwork = true;//LORAWAN_PUBLIC_NETWORK;
	LoRaMacMibSetRequestConfirm( &_mibReq );

#if defined( REGION_EU868 )
	LoRaMacTestSetDutyCycleOn( true/*LORAWAN_DUTYCYCLE_ON*/ );

	LoRaMacChannelAdd( 3, ( ChannelParams_t )LC4 );
	LoRaMacChannelAdd( 4, ( ChannelParams_t )LC5 );
	LoRaMacChannelAdd( 5, ( ChannelParams_t )LC6 );
	LoRaMacChannelAdd( 6, ( ChannelParams_t )LC7 );
	LoRaMacChannelAdd( 7, ( ChannelParams_t )LC8 );
	LoRaMacChannelAdd( 8, ( ChannelParams_t )LC9 );
	LoRaMacChannelAdd( 9, ( ChannelParams_t )LC10 );

	_mibReq.Type = MIB_RX2_CHANNEL;
	_mibReq.Param.Rx2Channel = ( Rx2ChannelParams_t ){ 869525000, DR_3 };
	LoRaMacMibSetRequestConfirm( &_mibReq );
#endif

	
	if(_otaa){ //OTAA		
		MlmeReq_t mlmeReq;
		
		uint8_t DevEui[8];
		uint8_t AppEui[8];
		uint8_t AppKey[16];
		if(_devEui == NULL)
			BoardGetUniqueId( DevEui );
		else
			convertKey(DevEui, _devEui, 8);
		convertKey(AppEui, _appEui, 8);
		convertKey(AppKey, _appKey, 16);

		
		mlmeReq.Type = MLME_JOIN;

		mlmeReq.Req.Join.DevEui = DevEui;
		mlmeReq.Req.Join.AppEui = AppEui;
		mlmeReq.Req.Join.AppKey = AppKey;
		mlmeReq.Req.Join.NbTrials = NR_OF_TRIALS;
		do{
			LoRaMacMlmeRequest( &mlmeReq );
			delay(7000);
      
		}
		
		while(!joined);
	}
	else{ //ABP
		uint32_t DevAddr;
		// Choose a random device address if not already defined in Comissioning.h
		if( _devAddr == NULL )
		{
			// Random seed initialization
			srand1( BoardGetRandomSeed( ) );

			// Choose a random device address
			DevAddr = randr( 0, 0x01FFFFFF );
		}
		else
			convertAddress(DevAddr, _devAddr);
		uint8_t NwkSKey[16];
		uint8_t AppSKey[16];
		convertKey(NwkSKey, _nwkSKey, 16);
		convertKey(AppSKey, _appSKey, 16);
		_mibReq.Type = MIB_NET_ID;
		_mibReq.Param.NetID = 0;//LORAWAN_NETWORK_ID;
		LoRaMacMibSetRequestConfirm( &_mibReq );

		_mibReq.Type = MIB_DEV_ADDR;
		_mibReq.Param.DevAddr = DevAddr;
		LoRaMacMibSetRequestConfirm( &_mibReq );

		_mibReq.Type = MIB_NWK_SKEY;
		_mibReq.Param.NwkSKey = NwkSKey;
		LoRaMacMibSetRequestConfirm( &_mibReq );

		_mibReq.Type = MIB_APP_SKEY;
		_mibReq.Param.AppSKey = AppSKey;
		LoRaMacMibSetRequestConfirm( &_mibReq );

		_mibReq.Type = MIB_NETWORK_JOINED;
		_mibReq.Param.IsNetworkJoined = true;
		LoRaMacMibSetRequestConfirm( &_mibReq );

	} 
}

void LoRaNode::sendFrame(char frame[], int dim, int port, bool confirmed){
		//TODO: check maximum length
		for(int i = 0; i < dim; i++)
			_appData[i] = frame[i];

		AppPort = port;
		AppDataSize = dim;
		nextTx = send(port, confirmed);
}

void LoRaNode::poll(int port, bool confirm) {
  char payload[] = { 0x00 };
  return sendFrame(payload, 1, port, confirm);
}

void LoRaNode::showStatus(){
	Serial.println();
	Serial.println("LoRa Node parameters:");
	Serial.print("Frequency:        ");
#ifdef REGION_EU868
	Serial.print(868);
#else
	Serial.print(915);
#endif
	Serial.println(" MHz");

	if(_initialized){	// print these data only if begin function was called
	
		//FXOSC = 32MHz
		uint8_t msb = Radio.Read(0x02);
		uint8_t lsb = Radio.Read(0x03);
		uint32_t br = (msb<<8) | lsb;
		Serial.print("BitRate:          ");
		Serial.print(32000000/br);
		Serial.println(" b/s");
		
		Serial.print("Datarate          SF");
		Serial.println((Radio.Read(0x1E) & 0b11110000)>>4);
		
		Serial.print("Bandwidth:        ");
		uint8_t bw = ((Radio.Read(0x1D) & 0b11110000) >> 4);
		switch(bw){
			case 0: 
				Serial.println("7800 Hz");
			break;
			case 1: 
				Serial.println("10400 Hz");
			break;
			case 2: 
				Serial.println("15600 Hz");
			break;
			case 3: 
				Serial.println("20800 Hz");
			break;
			case 4: 
				Serial.println("31250 Hz");
			break;
			case 5: 
				Serial.println("41700 Hz");
			break;
			case 6: 
				Serial.println("62500 Hz");
			break;
			case 7: 
				Serial.println("125000 Hz");
			break;
			case 8: 
				Serial.println("250000 Hz");
			break;
			case 9: 
				Serial.println("500000 Hz");
			break;
		}
	}				
	
	Serial.println();
	if(_otaa){
		if(_appKey == NULL && _appEui == NULL && _devEui == NULL){
			Serial.println("No parameters defined yet.");
			return;
		}
		Serial.print("Application Key: {");
		for(int i = 0; i < 32; i += 2){
			Serial.print(" 0x");
			Serial.print(_appKey[i]);
			Serial.print(_appKey[i+1]);
		}
		Serial.println(" }");
		Serial.print("Application Eui: {");
		for(int i = 0; i < 16; i += 2){
			Serial.print(" 0x");
			Serial.print(_appEui[i]);
			Serial.print(_appEui[i+1]);
		}
		Serial.println(" }");
		if(_devEui != NULL){
			Serial.print("Device Eui:      {");
			for(int i = 0; i < 16; i += 2){
				Serial.print(" 0x");
				Serial.print(_devEui[i]);
				Serial.print(_devEui[i+1]);
			}
			Serial.println(" }");
		}
		else{
			uint8_t DevEui[8];
			BoardGetUniqueId( DevEui );
			Serial.print("Device Eui:      {");
			for(int i = 0; i < 8; i++){
				Serial.print(" 0x");
				if(DevEui[i] <= 0x0F)
					Serial.print(0);
				Serial.print(DevEui[i], HEX);
			}
			Serial.println(" }");
		}
		if(joined)
			Serial.println("The node has joined the network.");
		else
			Serial.println("The node hasn't joined the network yet.");
	}
	else{ //ABP
		if(_devAddr == NULL && _nwkSKey == NULL && _appSKey == NULL){
			Serial.println("No parameters defined yet.");
			return;
		}
		Serial.print("Device address:          0x");
		Serial.println(_devAddr);
		Serial.print("Network Session Key:     {");
		for(int i = 0; i < 32; i += 2){
			Serial.print(" 0x");
			Serial.print(_nwkSKey[i]);
			Serial.print(_nwkSKey[i+1]);
		}
		Serial.println(" }");
		Serial.print("Application Session Key: {");
		for(int i = 0; i < 32; i += 2){
			Serial.print(" 0x");
			Serial.print(_appSKey[i]);
			Serial.print(_appSKey[i+1]);
		}
		Serial.println(" }");
	} 
	Serial.println();
}


void LoRaNode::sleep(uint32_t ms){
#ifdef ARDUINO_ARCH_NRF52
  LowPower.sleep(ms);
#else
  delay(ms);
#endif
}

void LoRaNode::setSF(SF sf){
#ifdef REGION_EU868
	switch(sf){
		case SF7:
			_dr = DR_5;
		break;
		case SF8:
			_dr = DR_4;
		break;
		case SF9:
			_dr = DR_3;
		case SF10:
			_dr = DR_2;
		break;
		case SF11:
			_dr = DR_1;
		break;
		case SF12:
			_dr = DR_0;
		break;
		default:
			_dr = DR_0;
		break;
	}
#elif defined (REGION_US915_HYBRID) | defined (REGION_US915)
	switch(sf){
		case SF7:
			_dr = DR_3;
		break;
		case SF8:
			_dr = DR_2;
		break;
		case SF9:
			_dr = DR_1;
		case SF10:
			_dr = DR_0;
		break;
		default:
			_dr = DR_0;
		break;
	}
#endif	
}

void LoRaNode::setTxPower(int power){

#ifdef REGION_EU868
	if(power >= 16){
		_power = 0;
		return;
	}
	if(power >= 14){
		_power = 1;
		return;
	}
	if(power >= 12){
		_power = 2;
		return;
	}
	if(power >= 10){
		_power = 3;
		return;
	}
	if(power >= 8){
		_power = 4;
		return;
	}
	if(power >= 6){
		_power = 5;
		return;
	}
	if(power >= 4){
		_power = 6;
		return;
	}
	else{
		_power = 7;
		return;
	}
	
#elif defined (REGION_US915_HYBRID) | defined (REGION_US915)
	if(power >= 30){
		_power = 0;
		return;
	}
	if(power >= 28){
		_power = 1;
		return;
	}
	if(power >= 26){
		_power = 2;
		return;
	}
	if(power >= 24){
		_power = 3;
		return;
	}
	if(power >= 22){
		_power = 4;
		return;
	}
	if(power >= 20){
		_power = 5;
		return;
	}
	if(power >= 18){
		_power = 6;
		return;
	}
	if(power >= 16){
		_power = 7;
		return;
	}
	if(power >= 14){
		_power = 8;
		return;
	}


#endif
}

// private

bool LoRaNode::send(int port, bool confirmed){
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;

    if( LoRaMacQueryTxPossible(AppDataSize, &txInfo) != LORAMAC_STATUS_OK )
    {
      // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
    }
    else
    {
        if(confirmed == false)
        {
			mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppPort;
            mcpsReq.Req.Unconfirmed.fBuffer = _appData;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppPort;
            mcpsReq.Req.Confirmed.fBuffer = _appData;
            mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Confirmed.NbTrials = NR_OF_TRIALS;
            mcpsReq.Req.Confirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
    }

    if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
    {
        return false;
    }
    return true;
}



LoRaNode node;
