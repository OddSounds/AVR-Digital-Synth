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

typedef void(*FunctionPointer)();

unsigned char osc1Note = 36;
unsigned long osc1Freq = 0;
unsigned char osc1PhaccuShort[3] = {0};
unsigned long osc1Phaccu = 0;
unsigned volatile long osc1TWord = 0;
short osc1CentsShift = 0;
char dOsc1CentsShift = 0;
char osc1SemisShift = 0;
char dOsc1SemisShift = 0;
char osc1OctaveShift = 0;
char dOsc1OctaveShift = 0;
char osc1PhaseShift = 0;
unsigned char osc1Weight = 125;
short osc1Out[3] = {0};
bool osc1NoteSync = false;

unsigned long osc2Freq = 0;
unsigned long osc2Phaccu = 0;
unsigned volatile long osc2TWord = 0;
short osc2CentsShift = 0;
char dOsc2CentsShift = 0;
char osc2SemisShift = 0;
char dOsc2SemisShift = 0;
char osc2OctaveShift = 0;
char dOsc2OctaveShift = 0;
char osc2PhaseShift = 0;
char dOsc2PhaseShift = 0;
unsigned char osc2Weight = 125;
short osc2Out[3] = {0};
bool osc2Sync = false;
unsigned char osc2Note = 36;
bool osc2Busy = false;

bool duoMode = true;
bool ringMod = false;

unsigned volatile long lfoTWord = 0;
unsigned long lfoPhaccu = 0;
unsigned short lfoOut[2] = {0};
unsigned long lfoFreq = 0;
unsigned char lfoWaveForm = WAVE_SINE;
unsigned char lfoRoute = 0;
unsigned char lfoPrintFreq = 0;
unsigned char lfoDepth = 255;
char lfoDelta = 0;
bool lfoNoteSync = false;

FunctionPointer lfoRouteFunction = NULL;

unsigned long centsConst;
unsigned long stepConst;
unsigned long refclk;
unsigned long reftime;

unsigned char resMix = 0;
unsigned char prevOutput[4] = {0};
unsigned short prevInput[4] = {0};

bool notePlaying = true;
unsigned char playThisNote = 24;
char keyArray[16] = {0};
char keyArrayWrite = 0;
char keyArrayRead = 0;

unsigned char osc1WaveForm = WAVE_LSAW;
unsigned char osc2WaveForm = WAVE_LSAW;

unsigned char filterMode = FILTER_LOW;
unsigned char filterCutoff = 255;
unsigned char poles = 0;

short lfsrState = 0xACE1;

unsigned short clock_32k = 0;
unsigned short envelopeGain = 0;

unsigned char attackRate = 10;
unsigned char decayRate = 255;
unsigned char sustainLevel = 255;
unsigned char releaseRate = 0;
unsigned char envelopePhase = ATTACK_SLOPE;

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