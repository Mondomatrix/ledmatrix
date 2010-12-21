/*
 This file is part of the LEDMatrix firmware.
 
 The LEDMatrix firmware is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 The LEDMatrix firmware is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with the LEDMatrix firmware.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "spi.h"
#include <avr/io.h>
#include <stdint.h>

void SPISetup(void)
{
	DDR_SPI &= ~((1<<DD_MOSI)|(1<<DD_MISO)|(1<<DD_SS)|(1<<DD_SCK));
	// Define the following pins as output
	DDR_SPI |= ((1<<DD_MOSI)|(1<<DD_SS)|(1<<DD_SCK));

	SPCR = ((1<<SPE)|               // SPI Enable
			(0<<SPIE)|              // SPI Interupt Enable
			(0<<DORD)|              // Data Order (0:MSB first / 1:LSB first)
			(1<<MSTR)|              // Master/Slave select   
			(0<<SPR1)|(0<<SPR0)|    // SPI Clock Rate
			(0<<CPOL)|              // Clock Polarity (0:SCK low / 1:SCK hi when idle)
			(0<<CPHA));             // Clock Phase (0:leading / 1:trailing edge sampling)
	SPSR = (1 << SPI2X);

	SPISSOn();
}

void SPISSOn(void)
{
	sbi(PORT_SPI, PORT_SS);
}

void SPISSOff(void)
{
	cbi(PORT_SPI, PORT_SS);
}

void SPIWriteByte(uint8_t dataByte)
{
	SPDR = dataByte;
	while((SPSR & (1<<SPIF))==0);
}

uint8_t SPIReadByte(void)
{
	SPDR = 0;
	while((SPSR & (1<<SPIF))==0);
	return SPDR;
}

uint8_t SPIReadWriteByte(uint8_t dataByte)
{
	SPDR = dataByte;
	while((SPSR & (1<<SPIF))==0);
	return SPDR;
}

void SPIWriteArray(uint8_t* dataArray, uint8_t len)
{
	uint8_t i;      

	for (i = 0; i < len; i++) 
	{
		SPDR = dataArray[i];
		while((SPSR & (1<<SPIF))==0);
	}
}

void SPIReadArray(uint8_t* targetArray, uint8_t len)
{
	uint8_t i;      

	for (i = 0; i < len; i++) 
	{
		SPDR = 0;
		while((SPSR & (1<<SPIF))==0);
		targetArray[i] = SPDR;
	}
}

void SPIReadWriteArray(uint8_t* dataArray, uint8_t* targetArray, uint8_t len)
{
	uint8_t i;      

	for (i = 0; i < len; i++) 
	{
		SPDR = dataArray[i];
		while((SPSR & (1<<SPIF))==0);
		targetArray[i] = SPDR;
	}
}
