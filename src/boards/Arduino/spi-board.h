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
#ifndef __SPI_MCU_H__
#define __SPI_MCU_H__

//-------------modified
// #include "..\..\system\spi.h"
//---------------------
typedef struct Spi_s Spi_t;
/*!
 * SPI driver structure definition
 */
struct Spi_s
{
    // SPI_HandleTypeDef Spi;
    Gpio_t Mosi;
    Gpio_t Miso;
    Gpio_t Sclk;
    Gpio_t Nss;
};

#ifdef __cplusplus
extern "C"{
#endif
uint16_t SpiInOut( Spi_t *obj, uint16_t outData );
uint16_t SpiGetFlag( Spi_t *obj, uint16_t flag );
void SpiFrequency( Spi_t *obj, uint32_t hz );
void SpiFormat( Spi_t *obj, int8_t bits, int8_t cpol, int8_t cpha, int8_t slave );
void SpiDeInit( Spi_t *obj );void SpiInit( Spi_t *obj, PinNames mosi, PinNames miso, PinNames sclk, PinNames nss );
#ifdef __cplusplus
}
#endif


#endif  // __SPI_MCU_H__
