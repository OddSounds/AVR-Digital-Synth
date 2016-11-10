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

#define USE_ENVELOPE

#define FILTER_LOW	0
#define FILTER_HIGH	1
#define FILTER_BAND	2

#define DISABLE_TIMER()	cbi(TIMSK2, TOIE2);
#define ENABLE_TIMER()	sbi(TIMSK2, TOIE2);

void oscInit(void);
void oscSetup(void);

void lfoRouteOsc1(void);
void lfoRouteOsc2(void);
void lfoRouteCents1(void);
void lfoRouteCents2(void);
void lfoRouteSemis1(void);
void lfoRouteSemis2(void);
void lfoRoutePhase1(void);
void lfoRoutePhase2(void);
void lfoRouteWave1(void);
void lfoRouteWave2(void);
void lfoRouteCutoff(void);
void lfoRouteFilterMode(void);

inline void lowPassFilter(unsigned char *val);
inline void highPassFilter(unsigned long *val);

inline void noteUpdate(void);

inline void osc1NoteUpdate();
inline void osc1CentsUpdate();

inline void osc2NoteUpdate();
inline void osc2CentsUpdate();

void lfsrUpdate();

#endif /* OSCENGINE_H_ */