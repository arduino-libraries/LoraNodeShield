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

#include "board.h"

/*!
 * Unique Devices IDs register set (nRF52)
 */
#define         ID1                                 ( 0x10000060 )
#define         ID2                                 ( 0x10000064 )

/*!
 * IO Extander pins objects
 */
Gpio_t Led13;

#if defined( USE_RADIO_DEBUG )
Gpio_t DbgPin1;
Gpio_t DbgPin2;
#endif

/*!
 * Nested interrupt counter.
 *
 * \remark Interrupt should only be fully disabled once the value is 0
 */
static uint8_t IrqNestLevel = 0;

void BoardDisableIrq( void )
{
    __disable_irq( );
    IrqNestLevel++;
}

void BoardEnableIrq( void )
{
    IrqNestLevel--;
    if( IrqNestLevel == 0 )
    {
        __enable_irq( );
    }
}

void BoardInitPeriph( void )
{
    /* Init the GPIO extender pins */
    // GpioInit( &Led13, IRQ_MPL3115, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );

    // Switch LED 13 OFF
    // GpioWrite( &Led13, 1 );
    RtcInit();
}

static void BoardUnusedIoInit( void )
{
    //TODO
}

void BoardInitMcu( void )
{
    BoardUnusedIoInit( );

#if defined( USE_RADIO_DEBUG )
    GpioInit( &DbgPin1, P_A1, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &DbgPin2, P_A5, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
#endif
    SpiInit( &SX1276.Spi, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, NC );
    SX1276IoInit( );
}

void BoardDeInitMcu( void )
{
    // Gpio_t ioPin;

    // GpioInit( &ioPin, OSC_HSE_IN, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
    // GpioInit( &ioPin, OSC_HSE_OUT, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 1 );

    // GpioInit( &ioPin, OSC_LSE_IN, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 1 );
    // GpioInit( &ioPin, OSC_LSE_OUT, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 1 );
}

uint32_t BoardGetRandomSeed( void )
{
    // return ( ( *( uint32_t* )ID1 ) ^ ( *( uint32_t* )ID2 ) ^ ( *( uint32_t* )ID3 ) );
}

void BoardGetUniqueId( uint8_t *id )
{
    id[7] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID2 ) ) >> 24;
    id[6] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID2 ) ) >> 16;
    id[5] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID2 ) ) >> 8;
    id[4] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID2 ) );
    id[3] = ( ( *( uint32_t* )ID2 ) ) >> 24;
    id[2] = ( ( *( uint32_t* )ID2 ) ) >> 16;
    id[1] = ( ( *( uint32_t* )ID2 ) ) >> 8;
    id[0] = ( ( *( uint32_t* )ID2 ) );
}

uint8_t BoardGetBatteryLevel( void )
{

}

uint8_t GetBoardPowerSource( void )
{
//TODO
}

#ifdef USE_FULL_ASSERT
/*
 * Function Name  : assert_failed
 * Description    : Reports the name of the source file and the source line number
 *                  where the assert_param error has occurred.
 * Input          : - file: pointer to the source file name
 *                  - line: assert_param error line source number
 * Output         : None
 * Return         : None
 */
void assert_failed( uint8_t* file, uint32_t line )
{
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while( 1 )
    {
    }
}
#endif
