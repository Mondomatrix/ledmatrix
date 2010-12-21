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

#ifndef RBGSPI
#define RBGSPI

#include <stdint.h>

#define PORT_SPI    PORTB
#define PORT_SS		2
#define DDR_SPI     DDRB
#define DD_MISO     DDB4
#define DD_MOSI     DDB3
#define DD_SS       DDB2
#define DD_SCK      DDB5

#ifndef cbi
#define cbi(register,bit)	register &= ~(_BV(bit))
#endif
#ifndef sbi
#define sbi(register,bit)	register |= (_BV(bit))
#endif

void SPISetup(void);
void SPISSOn(void);
void SPISSOff(void);
void SPIWriteByte(uint8_t dataByte);
uint8_t SPIReadByte(void);
uint8_t SPIReadWriteByte(uint8_t dataByte);
void SPIWriteArray(uint8_t* dataArray, uint8_t len);
void SPIReadArray(uint8_t* targetArray, uint8_t len);
void SPIReadWriteArray(uint8_t* dataArray, uint8_t* targetArray, uint8_t len);

#endif
