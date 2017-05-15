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

#ifndef __BOARD_H__
#define __BOARD_H__

//-----------modified
#include "Arduino.h"


#include "rtc-board.h"

#ifdef __cplusplus
extern "C"{
#endif

#include "system/gpio.h"
#include "sx1276-board.h"
#include "system/spi.h"
#ifdef __cplusplus
}
#endif

//------------------------

/*!
 * Generic definition
 */
#ifndef SUCCESS
#define SUCCESS                                     1
#endif

#ifndef FAIL
#define FAIL                                        0
#endif

/*!
 * Board IO Extender pins definitions
 */
#define LED_13                                       13


/*!
 * Board MCU pins definitions
 */
#define RADIO_RESET                                 P_A0

#define RADIO_MOSI                                  P_11
#define RADIO_MISO                                  P_12
#define RADIO_SCLK                                  P_13
#define RADIO_NSS                                   P_10

#define RADIO_DIO_0                                 P_2
#define RADIO_DIO_1                                 P_3
#define RADIO_DIO_2                                 P_4
#define RADIO_DIO_3                                 P_7
#define RADIO_DIO_4                                 P_8
#define RADIO_DIO_5                                 P_9

#define RADIO_ANT_SWITCH                            P_A4

/*!
 * LED GPIO pins objects
 */
extern Gpio_t Led13;

//#define USE_RADIO_DEBUG

#if defined( USE_RADIO_DEBUG )
extern Gpio_t DbgPin1;
extern Gpio_t DbgPin2;
#endif

#ifdef __cplusplus
extern "C"{
#endif
/*!
 * \brief Disable interrupts
 *
 * \remark IRQ nesting is managed
 */
void BoardDisableIrq( void );

/*!
 * \brief Enable interrupts
 *
 * \remark IRQ nesting is managed
 */
void BoardEnableIrq( void );

#ifdef __cplusplus
}
#endif

/*!
 * \brief Initializes the target board peripherals.
 */
void BoardInitMcu( void );

/*!
 * \brief Initializes the boards peripherals.
 */
void BoardInitPeriph( void );

/*!
 * \brief De-initializes the target board peripherals to decrease power
 *        consumption.
 */
void BoardDeInitMcu( void );

/*!
 * \brief Get the current battery level
 *
 * \retval value  battery level ( 0: very low, 254: fully charged )
 */
uint8_t BoardGetBatteryLevel( void );

/*!
 * Returns a pseudo random seed generated using the MCU Unique ID
 *
 * \retval seed Generated pseudo random seed
 */
uint32_t BoardGetRandomSeed( void );

/*!
 * \brief Gets the board 64 bits unique ID
 *
 * \param [IN] id Pointer to an array that will contain the Unique ID
 */
void BoardGetUniqueId( uint8_t *id );

/*!
 * \brief Get the board power source
 *
 * \retval value  power source ( 0: USB_POWER,  1: BATTERY_POWER )
 */
uint8_t GetBoardPowerSource( void );

#endif // __BOARD_H__
