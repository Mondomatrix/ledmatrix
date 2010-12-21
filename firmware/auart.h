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

#ifndef RBG_AUART
#define RBG_AUART

#include <stdint.h>
#include <avr/io.h>

#ifndef cbi
#define cbi(register,bit)	register &= ~(_BV(bit))
#endif
#ifndef sbi
#define sbi(register,bit)	register |= (_BV(bit))
#endif

#define RS485ControlPort	PORTD
#define RS485ControlDir		DDRD
#define RS485ControlPin		2

#define AUARTPort	PORTD
#define AUARTDir	DDRD
#define AUARTTXPin	1
#define AUARTRXPin	0

void AUARTSetup();

#endif
