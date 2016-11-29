/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Bleeper board SPI driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include "board.h"
#include "spi-board.h"

#include "SPI.h"

/*!
 * \brief  Find First Set
 *         This function identifies the least significant index or position of the
 *         bits set to one in the word
 *
 * \param [in]  value  Value to find least significant index
 * \retval bitIndex    Index of least significat bit at one
 */
__STATIC_INLINE uint8_t __ffs( uint32_t value )
{
    return( uint32_t )( 32 - __CLZ( value & ( -value ) ) );
}

/*!
 * MCU SPI peripherals enumeration
 */
// typedef enum
// {
    // SPI_1 = ( uint32_t )SPI1_BASE,
    // SPI_2 = ( uint32_t )SPI2_BASE,
// }SPIName;

void SpiInit( Spi_t *obj, PinNames mosi, PinNames miso, PinNames sclk, PinNames nss )
{
   
    if( nss != NC )
    {
        GpioInit( &obj->Nss, nss, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, 0x05U );
    }
    else
    {
        // obj->Spi.Init.NSS = SPI_NSS_SOFT;
    }

    SPI.setDataMode(SPI_MODE1);
    SpiFrequency( obj, 10000000 );	
    if( nss == NC )
    {
        SPI.begin();
    }
    else
    {
        SPI.beginSlave();
    }
}

void SpiDeInit( Spi_t *obj )
{
    SPI.end();
}

void SpiFormat( Spi_t *obj, int8_t bits, int8_t cpol, int8_t cpha, int8_t slave )
{
    // obj->Spi.Init.Direction = SPI_DIRECTION_2LINES;
    // if( bits == SPI_DATASIZE_8BIT )
    // {
        // obj->Spi.Init.DataSize = SPI_DATASIZE_8BIT;
    // }
    // else
    // {
        // obj->Spi.Init.DataSize = SPI_DATASIZE_16BIT;
    // }
    // obj->Spi.Init.CLKPolarity = cpol;
    // obj->Spi.Init.CLKPhase = cpha;
    // obj->Spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    // obj->Spi.Init.TIMode = SPI_TIMODE_DISABLE;
    // obj->Spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    // obj->Spi.Init.CRCPolynomial = 7;

    // if( slave == 0 )
    // {
        // obj->Spi.Init.Mode = SPI_MODE_MASTER;
    // }
    // else
    // {
        // obj->Spi.Init.Mode = SPI_MODE_SLAVE;
    // }
}

void SpiFrequency( Spi_t *obj, uint32_t hz )
{
    uint32_t divisor;

    if(hz > 6000000)
        divisor = SPI_CLOCK_DIV2;
    else if(hz > 3000000)
        divisor = SPI_CLOCK_DIV4;
    else if(hz > 1500000)
        divisor = SPI_CLOCK_DIV8;
    else if(hz > 750000)
        divisor = SPI_CLOCK_DIV16;
    else if(hz > 375000)
        divisor = SPI_CLOCK_DIV32;
    else if(hz > 190000)
        divisor = SPI_CLOCK_DIV64;
    else
        divisor = SPI_CLOCK_DIV128;

    SPI.setClockDivider(divisor);
}

/*FlagStatus*/uint16_t SpiGetFlag( Spi_t *obj, uint16_t flag )
{
    // FlagStatus bitstatus = RESET;

    // Check the status of the specified SPI flag
    // if( ( obj->Spi.Instance->SR & flag ) != ( uint16_t )RESET )
    // {
        // SPI_I2S_FLAG is set
        // bitstatus = SET;
    // }
    // else
    // {
        // SPI_I2S_FLAG is reset
        // bitstatus = RESET;
    // }
    // Return the SPI_I2S_FLAG status
    // return  bitstatus;
}

uint16_t SpiInOut( Spi_t *obj, uint16_t outData )
{
    return SPI.transfer(outData);
}
