/*
  Copyright (c) 2016 Arduino. All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#if defined(ARDUINO_ARCH_NRF52)

#include "boards/arduino/board.h"
#include "boards/arduino/rtc-board.h"
#include "nrf_timer.h"

/*!
 * RTC Time base in ms
 */
#define RTC_ALARM_TICK_DURATION                     1          // 1 tick every 32us
#define RTC_ALARM_TICK_PER_MS                       1          // 1/31.25 = tick duration in ms

typedef uint32_t* RtcCalendar_t;
TimerTime_t RtcCalendarContext;
volatile TimerTime_t now;
volatile uint32_t McuWakeUpTime = 0;
volatile bool NonScheduledWakeUp = false;
static bool RtcTimerEventAllowsLowPower = false;
volatile uint32_t timeout;



void RtcInit( void )
{
	uint32_t err_code;
	now=0;

	sd_clock_hfclk_request();
	uint32_t runn = 0;
	do{sd_clock_hfclk_is_running(&runn);}
	while(!runn);

	nrf_timer_mode_set(NRF_TIMER3, NRF_TIMER_MODE_TIMER);
	nrf_timer_bit_width_set(NRF_TIMER3, NRF_TIMER_BIT_WIDTH_32);
	nrf_timer_frequency_set(NRF_TIMER3, NRF_TIMER_FREQ_16MHz);
	uint32_t ticks=nrf_timer_ms_to_ticks(1, NRF_TIMER_FREQ_16MHz);
	nrf_timer_cc_write(NRF_TIMER3, NRF_TIMER_CC_CHANNEL0, ticks);
	nrf_timer_shorts_enable(NRF_TIMER3, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK);

	// enable interrupt
	nrf_timer_int_enable(NRF_TIMER3, NRF_TIMER_INT_COMPARE0_MASK);
	NVIC_SetPriority(TIMER3_IRQn, 2); //high priority
	NVIC_ClearPendingIRQ(TIMER3_IRQn);
	NVIC_EnableIRQ(TIMER3_IRQn);

	nrf_timer_task_trigger(NRF_TIMER3, NRF_TIMER_TASK_START);
}


static TimerTime_t RtcConvertCalendarTickToTimerTime( RtcCalendar_t *calendar )
{

    // TimerTime_t timeCounter = 0;
    // RtcCalendar_t now;
    // double timeCounterTemp = 0.0;

    // Passing a NULL pointer will compute from "now" else,
    // compute from the given calendar value
    if( calendar == NULL )
    {
		return now;
		// rtc.getTime();
		// rtc.getDate();
        // now.CalendarDate = rtc.date;
		// now.CalendarTime = rtc.time;
    }
    else
    {
		return *calendar;
        // now = *calendar;
    }

	//compute the elapsed seconds starting by 01/01/01 00:00:00
	// uint32_t days = rtc.rdn(now.CalendarDate.year, now.CalendarDate.month, now.CalendarDate.day) - rtc.rdn(1,1,1);
	// timeCounter = (now.CalendarTime.hour * SecondsInHour) + (now.CalendarTime.minute * SecondsInMinute) + now.CalendarTime.second;
	// timeCounter += SecondsInDay;
    // Years (calculation valid up to year 2099)
    // for( int16_t i = 0; i < ( now.CalendarDate.Year + now.CalendarCentury ); i++ )
    // {
        // if( ( i == 0 ) || ( i % 4 ) == 0 )
        // {
            // timeCounterTemp += ( double )SecondsInLeapYear;
        // }
        // else
        // {
            // timeCounterTemp += ( double )SecondsInYear;
        // }
    // }

    // Months (calculation valid up to year 2099)*/
    // if( ( now.CalendarDate.Year == 0 ) || ( ( now.CalendarDate.Year + now.CalendarCentury ) % 4 ) == 0 )
    // {
        // for( uint8_t i = 0; i < ( now.CalendarDate.Month - 1 ); i++ )
        // {
            // timeCounterTemp += ( double )( DaysInMonthLeapYear[i] * SecondsInDay );
        // }
    // }
    // else
    // {
        // for( uint8_t i = 0;  i < ( now.CalendarDate.Month - 1 ); i++ )
        // {
            // timeCounterTemp += ( double )( DaysInMonth[i] * SecondsInDay );
        // }
    // }

    // timeCounterTemp += ( double )( ( uint32_t )now.CalendarTime.Seconds +
                     // ( ( uint32_t )now.CalendarTime.Minutes * SecondsInMinute ) +
                     // ( ( uint32_t )now.CalendarTime.Hours * SecondsInHour ) +
                     // ( ( uint32_t )( now.CalendarDate.Date * SecondsInDay ) ) );

    // timeCounterTemp = ( double )timeCounterTemp * RTC_ALARM_TICK_DURATION;

    // timeCounter = round( timeCounterTemp );
    // return ( timeCounter );
}

TimerTime_t RtcGetElapsedAlarmTime( void )
{
    TimerTime_t currentTime = 0;
    TimerTime_t contextTime = 0;

    currentTime = RtcConvertCalendarTickToTimerTime( NULL );
    contextTime = RtcConvertCalendarTickToTimerTime( &RtcCalendarContext );

    if( currentTime < contextTime )
    {
        return( currentTime + ( 0xFFFFFFFF - contextTime ) );
    }
    else
    {
        return( currentTime - contextTime );
    }
}

TimerTime_t RtcGetTimerValue( void )
{
    return( RtcConvertCalendarTickToTimerTime( NULL ) );
}


TimerTime_t RtcComputeElapsedTime( TimerTime_t eventInTime )
{
    TimerTime_t elapsedTime = 0;

    // Needed at boot, cannot compute with 0 or elapsed time will be equal to current time
    if( eventInTime == 0 )
    {
        return 0;
    }

    elapsedTime = RtcConvertCalendarTickToTimerTime( NULL );

    if( elapsedTime < eventInTime )
    { // roll over of the counter
        return( elapsedTime + ( 0xFFFFFFFF - eventInTime ) );
    }
    else
    {
        return( elapsedTime - eventInTime );
    }
}

TimerTime_t RtcComputeFutureEventTime( TimerTime_t futureEventInTime )
{
    return( RtcGetTimerValue( ) + futureEventInTime );
}

TimerTime_t RtcGetAdjustedTimeoutValue( uint32_t timeout )
{
    // if( timeout > McuWakeUpTime )
    // {   // we have waken up from a GPIO and we have lost "McuWakeUpTime" that we need to compensate on next event
        // if( NonScheduledWakeUp == true )
        // {
            // NonScheduledWakeUp = false;
            // timeout -= McuWakeUpTime;
        // }
    // }

    // if( timeout > McuWakeUpTime )
    // {   // we don't go in Low Power mode for delay below 50ms (needed for LEDs)
        // if( timeout < 50 ) // 50 ms
        // {
            // RtcTimerEventAllowsLowPower = false;
        // }
        // else
        // {
            // RtcTimerEventAllowsLowPower = true;
            // timeout -= McuWakeUpTime;
        // }
    // }
    return  timeout;
}


static void RtcStartWakeUpAlarm( uint32_t timeoutValue )
{
	// every 164 ms we loose 1ms
	uint32_t adjustment = timeoutValue / 100;
	uint32_t partialAdjustment = adjustment;
	while(partialAdjustment > 100){
		//if the adjustment is greater than 174 ms we need to calculate the adjustment over the adjustment
		adjustment += partialAdjustment / 100;
		partialAdjustment = partialAdjustment / 100;
	}
	
	timeout = now + timeoutValue/* + adjustment*/;
	RtcCalendarContext = now;
}

void RtcSetTimeout( uint32_t timeout )
{
    RtcStartWakeUpAlarm( timeout );
}


void TIMER3_IRQHandler(void){
	nrf_timer_event_clear(NRF_TIMER3, NRF_TIMER_EVENT_COMPARE0);
	now++;
	if(timeout == now)
		TimerIrqHandler( );
}

#endif