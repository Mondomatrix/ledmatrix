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

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "spi.h"
#include "auart.h"

#ifndef cbi
#define cbi(register,bit)	register &= ~(_BV(bit))
#endif
#ifndef sbi
#define sbi(register,bit)	register |= (_BV(bit))
#endif

void IOSetup(void);
void TimerSetup(void);

void WriteDCPacket(void);
void WriteBrightPacket(void);

void BuildRedBrightPacket(void);
void BuildRedDCPacket(void);
void BuildBlueBrightPacket(void);
void BuildBlueDCPacket(void);
void BuildGreenBrightPacket(void);
void BuildGreenDCPacket(void);

uint8_t RedBright[64];
uint8_t BlueBright[64];
uint8_t GreenBright[64];
uint8_t RedDC[64];
uint8_t BlueDC[64];
uint8_t GreenDC[64];
uint8_t BrightPacket[96];
uint8_t DCPacket[48];

volatile uint8_t CurrentState = 0;
volatile uint8_t ChangeColour = 0;
volatile uint8_t NumBlanks = 0;

#define MaxBlanks 4

#define RedPrepare 0
#define RedSend 1
#define BluePrepare 2
#define BlueSend 3
#define GreenPrepare 4
#define GreenSend 5

#define LEDModePort PORTB
#define LEDMode PB1
#define LEDXLatPort PORTB
#define LEDXLat PB2
#define RedBasePort PORTB
#define RedBase PB0
#define BlueBasePort PORTD
#define BlueBase PD7
#define GreenBasePort PORTD
#define GreenBase PD6
#define BLANKPort PORTD
#define BLANK PD3

#define SERIAL_STATE_WAITING 0
#define SERIAL_STATE_COMMAND 1
#define SERIAL_STATE_DATA 2

#define ADDRESS_ADC_THRESH 185

volatile uint8_t MyAddress = 0;
volatile uint8_t CommandByte = 0;
volatile uint8_t DataBuffer[256];
volatile uint8_t DataCount = 0;
volatile uint8_t BytesReceived = 0;
volatile uint8_t ProcessCommand = 0;

uint8_t SerialLengthFromCommand(uint8_t CmdByte);
void SerialStateMachine(uint8_t SerialByte, uint8_t NinthBit);
uint8_t GetMyAddress(void);

int main(void)
{
	uint8_t i, j, jdot, jdot2, k, kdot, k2;
	
	IOSetup();
	SPISetup();
	AUARTSetup();
	
	ADMUX = 0b01000111;
	ADCSRA = 0b10000110;
	
	MyAddress = GetMyAddress();

	for(i=0;i<128;i++)
	{
		DataBuffer[i] = 0;
		DataBuffer[i+128] = 0;
	}
	
	TimerSetup();
	
	for(i=0;i<64;i++)
	{
		RedBright[i]=255;
		BlueBright[i]=255;
		GreenBright[i]=255;
		RedDC[i]=63;
		BlueDC[i]=31;
		GreenDC[i]=63;
	}

	while(1)
	{
		if(ProcessCommand == 1)
		{
			ProcessCommand = 0;
			//based on command byte, execute the function
			if(CommandByte == 1)
			{
				for(j=0;j<4;j++)
				{
					jdot = j * 16;
					jdot2 = j * 48;
					for(k=0;k<8;k++)
					{
						kdot = 3*k;
						RedBright[k+jdot] = DataBuffer[kdot+jdot2];
						GreenBright[k+jdot] = DataBuffer[kdot+jdot2+1];
						BlueBright[k+jdot] = DataBuffer[kdot+jdot2+2];
					}
					for(k=0;k<8;k++)
					{
						k2 = 7-k;
						kdot = 3*k+24;
						RedBright[8+k2+jdot] = DataBuffer[kdot+jdot2];
						GreenBright[8+k2+jdot] = DataBuffer[kdot+jdot2+1];
						BlueBright[8+k2+jdot] = DataBuffer[kdot+jdot2+2];
					}
				}
			} else if(CommandByte == 2) {
				jdot = DataBuffer[0];
				kdot = jdot % 16;
				jdot = jdot - kdot;
				if(kdot >= 8)
				{
					kdot -= 8;
					RedBright[jdot+(15-kdot)] = DataBuffer[1];
					GreenBright[jdot+(15-kdot)] = DataBuffer[2];
					BlueBright[jdot+(15-kdot)] = DataBuffer[3];
				} else {
					RedBright[jdot+kdot] = DataBuffer[1];
					GreenBright[jdot+kdot] = DataBuffer[2];
					BlueBright[jdot+kdot] = DataBuffer[3];
				}
			}
			CommandByte = 0;
			BytesReceived = 0;
			DataCount = 0;
		}
		
		switch(CurrentState)
		{
			case RedPrepare:
				BuildRedBrightPacket();
				BuildRedDCPacket();
				
				while(ChangeColour == 0)
				{
				}
				ChangeColour = 0;
				
				CurrentState = RedSend;
				break;
			case RedSend:
				sbi(GreenBasePort, GreenBase);
				sbi(BlueBasePort, BlueBase);
				sbi(RedBasePort, RedBase);
				WriteDCPacket();
				WriteBrightPacket();
				cbi(BLANKPort, BLANK);
				cbi(RedBasePort, RedBase);
				TCNT1L = 0;
				TCNT1H = 0;
				CurrentState = BluePrepare;
				break;
			case BluePrepare:
				BuildBlueBrightPacket();
				BuildBlueDCPacket();
				
				while(ChangeColour == 0)
				{
				}
				ChangeColour = 0;
								
				CurrentState = BlueSend;
				break;
			case BlueSend:
				sbi(GreenBasePort, GreenBase);
				sbi(BlueBasePort, BlueBase);
				sbi(RedBasePort, RedBase);
				WriteDCPacket();
				WriteBrightPacket();
				cbi(BLANKPort, BLANK);
				cbi(BlueBasePort, BlueBase);
				TCNT1L = 0;
				TCNT1H = 0;
				CurrentState = GreenPrepare;
				break;
			case GreenPrepare:
				BuildGreenBrightPacket();
				BuildGreenDCPacket();
				
				while(ChangeColour == 0)
				{
				}
				ChangeColour = 0;
								
				CurrentState = GreenSend;
				break;
			case GreenSend:
				sbi(GreenBasePort, GreenBase);
				sbi(BlueBasePort, BlueBase);
				sbi(RedBasePort, RedBase);
				WriteDCPacket();
				WriteBrightPacket();
				cbi(BLANKPort, BLANK);
				cbi(GreenBasePort, GreenBase);
				TCNT1L = 0;
				TCNT1H = 0;
				CurrentState = RedPrepare;
				break;
			default:
				CurrentState = RedPrepare;
				break;
		}
	}
}


uint8_t GetMyAddress(void)
{
	uint8_t addressBuffer = 0;
	uint8_t bit7result = 0;
	//read the address, return it
	
	addressBuffer = PINC & 0x3F;
	addressBuffer |= ((PIND & 0x10) << 2);
	
	ADCSRA |= (1 << ADSC);
	while(ADCSRA & (1 << ADSC));
	bit7result = ADCH;
	
	if(bit7result >= ADDRESS_ADC_THRESH)
		addressBuffer += 128;
	
	return addressBuffer;
}

void SerialStateMachine(uint8_t SerialByte, uint8_t NinthBit)
{
	static uint8_t SerialState = SERIAL_STATE_WAITING;
	
	switch(SerialState)
	{
		case SERIAL_STATE_WAITING:
			//make sure MPCM is set
			//check for address match
			if(SerialByte == MyAddress)
			{
				UCSR0A = ((0 << TXC0) | (1 << U2X0) | (0 << MPCM0));
				SerialState = SERIAL_STATE_COMMAND;
			} else {
				UCSR0A = ((0 << TXC0) | (1 << U2X0) | (1 << MPCM0));
				SerialState = SERIAL_STATE_WAITING;
			}
			//no match, resume
			//if match, clear MPCM, state = STATE_ADDRESS
			break;
		case SERIAL_STATE_COMMAND:
			//check ninth bit. if clear, continue, else, state = SERIAL_STATE_WAITING, set MPCM
			if(NinthBit != 0)
			{
				SerialState = SERIAL_STATE_WAITING;
				UCSR0A = ((0 << TXC0) | (1 << U2X0) | (1 << MPCM0));
			} else {
			//commandbyte = data
				CommandByte = SerialByte;
			//state = SERIAL_STATE_DATA
				SerialState = SERIAL_STATE_DATA;
			//set DataCount based on command
				DataCount = SerialLengthFromCommand(SerialByte);
				BytesReceived = 0;
			}
			break;
		case SERIAL_STATE_DATA:
			//check ninth bit. if clear, continue, else, state = SERIAL_STATE_WAITING, DataCount = 0
			if(NinthBit == 0)
			{
				DataBuffer[BytesReceived] = SerialByte;
			//DataBuffer[BytesReceived] = byte
				BytesReceived++;
				if(DataCount == BytesReceived)
				{
					ProcessCommand = 1;
					SerialState = SERIAL_STATE_WAITING;
					UCSR0A = ((0 << TXC0) | (1 << U2X0) | (1 << MPCM0));
				} else {
					SerialState = SERIAL_STATE_DATA;
				}
			} else {
				SerialState = SERIAL_STATE_WAITING;
				UCSR0A = ((0 << TXC0) | (1 << U2X0) | (1 << MPCM0));
			}
			//bytesreceived++
			//if datacount == bytesrecieved then call command processor, bytesreceived = 0, datacount = 0, state = SERIAL_STATE_WAITING, set MPCM
			//else state = SERIAL_STATE_DATA
			break;
		default:
			SerialState = SERIAL_STATE_WAITING; // also set MPCM
			break;
	}
}

uint8_t SerialLengthFromCommand(uint8_t CmdByte)
{
	//return proper data length for a command
	if(CmdByte == 1)
	{
		return 192;
	} else if(CmdByte == 2) {
		return 4;
	} else {
		return 0;
	}
}


void WriteDCPacket(void)
{
	sbi(LEDModePort, LEDMode);
	_delay_us(1);
	SPIWriteArray(DCPacket, 48);
	sbi(LEDXLatPort, LEDXLat);
	_delay_us(10);
	cbi(LEDXLatPort, LEDXLat);
	_delay_us(1);
	cbi(LEDModePort, LEDMode);
	_delay_us(1);
}

void WriteBrightPacket(void)
{
	cbi(LEDModePort, LEDMode);
	_delay_us(1);
	SPIWriteArray(BrightPacket, 96);
	sbi(LEDXLatPort, LEDXLat);
	_delay_us(10);
	cbi(LEDXLatPort, LEDXLat);
	_delay_us(1);
	cbi(LEDModePort, LEDMode);
	_delay_us(1);	
}

void BuildRedBrightPacket(void)
{
	uint8_t i;
	uint8_t j;
	i = 63;
	for(j=0;j<96;j+=3)
	{
		BrightPacket[j] = RedBright[i];
		i--;
		BrightPacket[j+1] = (0x00 | (RedBright[i] >> 4));
		BrightPacket[j+2] = ((RedBright[i] << 4) | 0x00);
		i--;
	}
}

void BuildRedDCPacket(void)
{
	uint8_t i;
	uint8_t j;
	i = 63;
	for(j=0;j<48;j+=3)
	{
		DCPacket[j] = ((RedDC[i] << 2) | (RedDC[i-1] >> 4));
		i--;
		DCPacket[j+1] = ((RedDC[i] << 4) | (RedDC[i-1] >> 2));
		i--;
		DCPacket[j+2] = ((RedDC[i] << 6) | (RedDC[i-1]));
		i-=2;
	}
}
	
void BuildBlueBrightPacket(void)
{
	uint8_t i;
	uint8_t j;
	i = 63;
	for(j=0;j<96;j+=3)
	{
		BrightPacket[j] = BlueBright[i];
		i--;
		BrightPacket[j+1] = (0x00 | (BlueBright[i] >> 4));
		BrightPacket[j+2] = ((BlueBright[i] << 4) | 0x00);
		i--;
	}
}

void BuildBlueDCPacket(void)
{
	uint8_t i;
	uint8_t j;
	i = 63;
	for(j=0;j<48;j+=3)
	{
		DCPacket[j] = ((BlueDC[i] << 2) | (BlueDC[i-1] >> 4));
		i--;
		DCPacket[j+1] = ((BlueDC[i] << 4) | (BlueDC[i-1] >> 2));
		i--;
		DCPacket[j+2] = ((BlueDC[i] << 6) | (BlueDC[i-1]));
		i-=2;
	}
}

void BuildGreenBrightPacket(void)
{
	uint8_t i;
	uint8_t j;
	i = 63;
	for(j=0;j<96;j+=3)
	{
		BrightPacket[j] = GreenBright[i];
		i--;
		BrightPacket[j+1] = (0x00 | (GreenBright[i] >> 4));
		BrightPacket[j+2] = ((GreenBright[i] << 4) | 0x00);
		i--;
	}
}

void BuildGreenDCPacket(void)
{
	uint8_t i;
	uint8_t j;
	i = 63;
	for(j=0;j<48;j+=3)
	{
		DCPacket[j] = ((GreenDC[i] << 2) | (GreenDC[i-1] >> 4));
		i--;
		DCPacket[j+1] = ((GreenDC[i] << 4) | (GreenDC[i-1] >> 2));
		i--;
		DCPacket[j+2] = ((GreenDC[i] << 6) | (GreenDC[i-1]));
		i-=2;
	}
}

void IOSetup(void)
{
	//set up DDRB and DDRD for our particular hardware arrangement
	//LEDMODE on PB1, XLAT on PB2, REDBASE on PB0, LEDSCLK on PB5, LEDSOUT on PB3, LEDSIN on PB4
	//USART TX on PD1, RX on PD0, PHY_CONTROL on PD2
	//Address bits on PC0-5, PD4
	//GREENBASE on PD6, BLUEBASE on PD7, LEDGSCLK on PD5
	//BLANK on PD3
	DDRB = 0;
	DDRB = ((1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3) | (0 << PB4) | (1 << PB5));
	DDRD = 0;
	DDRD = ((0 << PD0) | (1 << PD1) | (1 << PD2) | (1 << PD3) | (0 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7));
	DDRC = 0;
	DDRC = ((0 << PC0) | (0 << PC1) | (0 << PC2) | (0 << PC3) | (0 << PC4) | (0 << PC5));
}

void TimerSetup(void)
{
	//we use OCR0A in to set frequency, OCR0B is Fast PWM mode to generate GSCLK
	TCCR0A = ((0 << COM0A1) | (0 << COM0A0) | (1 << COM0B1) | (0 << COM0B0) | (1 << WGM01) | (1 << WGM00));
	TCCR0B = ((0 << FOC0A) | (0 << FOC0B) | (1 << WGM02) | (0 << CS02) | (0 << CS01) | (1 << CS00));
	//no interrupts for timer0
	TIMSK0 = ((0 << OCIE0B) | (0 << OCIE0A) | (0 << TOIE0));
	//these values give us a grayscale clock of 4 megahertz with about a 40% duty cycle
	OCR0A = 4;
	OCR0B = 2;
	
	//using Timer1 to generate blanking signal. Set to source T1 pin, which is the same as OCR0B
	//using CTC mode with OCR1A and prescaler set so we get an interrupt every 4096 clocks
	TCCR1A = ((0 << COM1A1) | (0 << COM1A0) | (0 << COM1B1) | (0 << COM1B0) | (0 << WGM11) | (0 << WGM10));
	TCCR1B = ((0 << ICNC1) | (0 << ICES1) | (0 << WGM13) | (1 << WGM12) | (1 << CS12) | (1 << CS11) | (1 << CS10));
	TCCR1C = ((0 << FOC1A) | (0 << FOC1B));
	TIMSK1 = ((0 << ICIE1) | (0 << OCIE1B) | (1 << OCIE1A) | (0 << TOIE1));
	
	//this gives us a count of 4096 in OCR1A
	OCR1AH = 16;
	OCR1AL = 0;
	OCR1BH = 0;
	OCR1BL = 0;
	
	sei();
}

ISR(TIMER1_COMPA_vect)
{
	//this is called every 4096 grayscale clocks
	//set blank high
	sbi(BLANKPort, BLANK);
	//if it's time to change colours, change the state of the state machine. we'll set blank low and reset the timer after we send new data.
	//otherwise, set blank low and reset Timer1 so we get a full 4096 clocks until the next blank
	if((NumBlanks++) == MaxBlanks)
	{
		ChangeColour = 1;
		NumBlanks = 0;
	} else {
		_delay_us(10);
		cbi(BLANKPort, BLANK);
		TCNT1L = 0;
		TCNT1H = 0;
	}
}

ISR(USART_RX_vect)
{
	uint8_t IntDataByte, IntNinthBit;
	IntDataByte = UDR0;
	IntNinthBit = UCSR0A & 0x01;
	SerialStateMachine(IntDataByte, IntNinthBit);
}
