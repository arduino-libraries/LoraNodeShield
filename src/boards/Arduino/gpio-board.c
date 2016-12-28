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
#include "gpio-board.h"

void GpioMcuInit( Gpio_t *obj, PinNames pin, PinModes mode, PinConfigs config, PinTypes type, uint32_t value )
{
    if( pin == NC )
    {
        return;
    }
    obj->pin = pin;
	
    obj->pinIndex = obj->pin;
    obj->port = 0; //Arduino Primo only
	
    if( mode == PIN_INPUT )
    {
        if(type == PIN_PULL_UP)
		    pinMode(obj->pinIndex, INPUT_PULLUP);
        else
        pinMode(obj->pinIndex, INPUT);
    }
    else if(mode == PIN_OUTPUT) // mode ouptut
    {
        pinMode(obj->pinIndex, OUTPUT);
        // Sets initial output value
        digitalWrite(obj->pinIndex, value);
    }
}

void GpioMcuSetInterrupt( Gpio_t *obj, IrqModes irqMode, IrqPriorities irqPriority, GpioIrqHandler *irqHandler )
{
    uint32_t mode;

    if( irqHandler == NULL )
    {
        return;
    }
	
    if(irqMode == IRQ_RISING_EDGE)
        mode = RISING;
    else if(irqMode == IRQ_FALLING_EDGE)
        mode = FALLING;
    else
        mode = CHANGE;
	
    attachInterrupt(obj->pinIndex, irqHandler, mode);
}

void GpioMcuRemoveInterrupt( Gpio_t *obj )
{
    detachInterrupt(obj->pinIndex);
}

void GpioMcuWrite( Gpio_t *obj, uint32_t value )
{
    if( ( obj == NULL ) || ( obj->port == NULL ) )
    {
        // assert_param( FAIL );
    }
    // Check if pin is not connected
    if( obj->pin == NC )
    {
        return;
    }
	
    digitalWrite(obj->pinIndex, value);
}

void GpioMcuToggle( Gpio_t *obj )
{
    if( ( obj == NULL ) || ( obj->port == NULL ) )
    {
        // assert_param( FAIL );
    }

    // Check if pin is not connected
    if( obj->pin == NC )
    {
        return;
    }
	
    bool val=digitalRead(obj->pinIndex);
    digitalWrite(obj->pinIndex, !val);
}

uint32_t GpioMcuRead( Gpio_t *obj )
{
    if( obj == NULL )
    {
        // assert_param( FAIL );
    }
    // Check if pin is not connected
    if( obj->pin == NC )
    {
        return 0;
    }

    return digitalRead(obj->pinIndex);
}