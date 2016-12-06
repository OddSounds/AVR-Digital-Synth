/*
 * OscEngine.h
 *
 * Created: 10/31/2016 8:47:45 PM
 *  Author: Sam
 */ 


#ifndef OSCENGINE_H_
#define OSCENGINE_H_

#include <stdlib.h>

#include "Globals.h"
#include "WaveTables.h"

#define NUM_OSC		6

#define DISABLE_OSC0()	TCCR0B = 0
#define ENABLE_OSC0()	TCCR0B = 1

#define DISABLE_OSC1()	TCCR1B = 0
#define ENABLE_OSC1()	TCCR1B = 1

#define DISABLE_OSC2()	TCCR2B = 0
#define ENABLE_OSC2()	TCCR2B = 1

#define OSC0_0_DDR	DDRD
#define OSC0_1_DDR	DDRD

#define OSC1_0_DDR	DDRB
#define OSC1_1_DDR	DDRB

#define OSC2_0_DDR	DDRB
#define OSC2_1_DDR	DDRD

#define OSC0_0_PIN	PIND6
#define OSC0_1_PIN	PIND5

#define OSC1_0_PIN	PINB1
#define OSC1_1_PIN	PINB2

#define OSC2_0_PIN	PINB3
#define OSC2_1_PIN	PIND3

#define BUSY_DDR	DDRB
#define BUSY_PIN	PINB0
#define BUSY_PORT	PORTB

void oscSetup(void);
void startOsc(void);

void lfsrUpdate();

#endif /* OSCENGINE_H_ */