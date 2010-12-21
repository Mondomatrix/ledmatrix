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

#include "auart.h"
#include <stdint.h>

void AUARTSetup()
{
	UCSR0A = ((0 << TXC0) | (1 << U2X0) | (1 << MPCM0));
	UCSR0B = ((1 << RXCIE0) | (0 << TXCIE0) | (0 << UDRIE0) | (1 << RXEN0) | (1 << TXEN0) | (1 << UCSZ02) | (0 << RXB80) | (0 << TXB80));
	UCSR0C = ((0 << UMSEL01) | (0 << UMSEL00) | (0 << UPM01) | (0 << UPM00) | (0 << USBS0) | (1 << UCSZ01) | (1 << UCSZ00) | (0 << UCPOL0));
	UBRR0H = 0;
	UBRR0L = 21;

/*	//set baud rate
	UBRR0H = ((F_CPU / 16 + 38400 / 2) / 38400 - 1) >> 8;
	UBRR0L = ((F_CPU / 16 + 38400 / 2) / 38400 - 1);
	
    // reset config for UART0
	UCSR0A = 0;
	UCSR0B = 0;
	UCSR0C = 0;
	
	//configure UART0
    UCSR0B = _BV(RXEN0)|_BV(TXEN0)|_BV(UCSZ02);
    UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);	
*/
	RS485ControlDir &= ~((1 << RS485ControlPin));
	RS485ControlDir |= ((1 << RS485ControlPin));
	cbi(RS485ControlPort, RS485ControlPin);
}
