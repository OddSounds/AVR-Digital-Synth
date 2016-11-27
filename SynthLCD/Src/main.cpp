/*
* SynthLCD.cpp
*
* Created: 12/31/2014 8:09:54 PM
*  Author: Sam
*/

#define F_CPU	16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "../H/OscEngine.h"

int main(void)
{
	oscSetup();
	startOsc();
	
	while(1)
	{
	}
}