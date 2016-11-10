/*
 * Globals.h
 *
 * Created: 10/31/2016 8:45:34 PM
 *  Author: Sam
 */ 


#ifndef GLOBALS_H_
#define GLOBALS_H_

#include "OscEngine.h"

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

#endif /* GLOBALS_H_ */