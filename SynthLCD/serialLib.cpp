/*
 * serialLib.cpp
 *
 * Created: 8/27/2015 1:40:48 PM
 *  Author: Sam
 */ 

#include "serialLib.h"
#include "avr/interrupt.h"

char txBuffer[TX_BUFFER_SIZE];
int txBufferReadIndex = 0;
int txBufferWriteIndex = 0;
int txMessagesRemaining = 0;

char rxBuffer[RX_BUFFER_SIZE];
int rxBufferReadIndex = 0;
int rxBufferWriteIndex = 0;
int rxMessagesRemaining = 0;

RXEvent rxEvent = 0;

void serialInit()
{
	UBRR0H = (uint8_t)(BAUD_PRESCALLER>>8);
	UBRR0L = (uint8_t)(BAUD_PRESCALLER);
		
	UCSR0B = (1 << RXCIE0) | (1 << TXCIE0) | (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);
}

void setRXHandler(RXEvent handler)
{
	rxEvent = handler;
}

void putChar(char c)
{
	while(!( UCSR0A & (1<<UDRE0)));
	UDR0 = c;
}

void putString(char* c)
{
	while(*c)
	{
		putChar(*c);
		c++;
	}
}

char getChar()
{
	char ret = rxBuffer[rxBufferReadIndex & RX_BUFFER_SIZE];
	rxBufferReadIndex++;
	rxMessagesRemaining--;
	
	return ret;
}

int bytesToRead()
{
	return rxMessagesRemaining;
}