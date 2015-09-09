/*
 * serialLib.h
 *
 * Created: 8/27/2015 1:41:04 PM
 *  Author: Sam
 */ 


#ifndef SERIALLIB_H_
#define SERIALLIB_H_

#define F_CPU	16000000UL

#define BAUD 31250
#define BAUD_PRESCALLER (((F_CPU / (BAUD * 16UL))) - 1)

typedef void(*RXEvent)(int);

//These must be a power of 2
#define TX_BUFFER_SIZE	256
#define RX_BUFFER_SIZE	256

void serialInit();
void setRXHandler(RXEvent handler);

void putChar(char c);
void putString(char* c);

char getChar();
int bytesToRead();

#endif /* SERIALLIB_H_ */