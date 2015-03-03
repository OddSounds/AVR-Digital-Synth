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

#include "PinDefs.h"
#include "StringTables.h"
#include "WaveTables.h"
#include "FixedLib.h"

#define MENU_OSC1	0
#define MENU_OSC2	1
#define MENU_LFO	2
#define MENU_MIX	3
#define MENU_FILTER	4
#define MENU_ARP	5

#define FILTER_LOW	0
#define FILTER_HIGH	1
#define FILTER_BAND	2

#define TIME_DEBOUNCE	100

//The DC pin tells the LCD if we are sending a command or data
#define LCD_COMMAND 0
#define LCD_DATA  1

//You may find a different size screen, but this one is 84 by 48 pixels
#define LCD_X     84
#define LCD_Y     48

//MIDI Commands
#define NOTE_ON		0x90
#define NOTE_OFF	0x80
#define AFTERTOUCH	13
#define CC			11
#define PC			12
#define PITCH_WHEEL	14

//MIDI CC
#define CC_osc1WaveForm		0x77
#define CC_osc2WaveForm		0x76
#define CC_cents			0x0E
#define CC_semis			0x0F
#define CC_octave			0x10
#define CC_weight			0x11

#define CC_lfoWaveForm	0x72
#define CC_lfoRoute		0x73
#define CC_lfoSpeed		0x12

#define BAUD 31250
#define BAUD_PRESCALLER (((F_CPU / (BAUD * 16UL))) - 1)

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

typedef unsigned short _8s8;
typedef unsigned long _16s16;

typedef void(*FunctionPointer)();

char onScreen[6][13];
char lineUpdate[6];
char screenUpdate = 0;

unsigned char adcSelect = 0;
int adcValue[4];
bool updateADC[4] = {false};

unsigned char commandBytes[3];
unsigned char commandCount = 0;

unsigned long osc1Freq = 0;
volatile unsigned long osc1Phaccu = 0;
volatile unsigned long osc1TWord = 0;
char osc1CentsShift = 0;
char osc1SemisShift = 0;
char osc1PhaseShift = 0;
unsigned char osc1Weight = 255;
volatile unsigned short osc1Out[3] = {0};
volatile bool osc1NoteSync = false;

unsigned long osc2Freq = 0;
volatile unsigned long osc2Phaccu = 0;
volatile unsigned long osc2TWord = 0;
char osc2CentsShift = 0;
char osc2SemisShift = 0;
char osc2OctaveShift = 0;
char osc2PhaseShift = 0;
unsigned char osc2Weight = 0;
volatile unsigned short osc2Out[3] = {0};
volatile bool osc2Sync = false;

unsigned long lfoFreq = 0;
unsigned char lfoWaveForm = WAVE_FLAT;
unsigned char lfoRoute = 0;
unsigned char lfoPrintFreq = 0;
unsigned char lfoDepth = 0;
bool lfoNoteSync = false;

unsigned char menuSelect = MENU_OSC1;
int menuKnobPos[6][4];
unsigned char menuChange[6][4];

unsigned long stepConst;
unsigned long refclk;
unsigned long reftime;

volatile unsigned char alpha = 0;
volatile unsigned char resMix = 0;
volatile unsigned short prevOutput[4] = {0};
volatile unsigned short prevInput[4] = {0};

bool notePlaying = true;
unsigned char playThisNote = 48;

volatile unsigned char osc1WaveForm = WAVE_LSAW;
volatile unsigned char osc2WaveForm = WAVE_RSAW;

unsigned char filterMode = FILTER_LOW;
unsigned char filterCutoff = 255;

bool btnPressed[6] = {0};
bool btnCanPress[6] = {0};
unsigned char btnLastPressed[6] = {0};

unsigned short lfsrState = 0xACE1;

//Function defs
void adcInit(void);

void initLCD(void);
inline void lcdShift(unsigned char data);
inline void lcdWrite(char control, char data);
inline void lcdClear(void);
inline void lcdString(char* str);
inline void lcdCharacter(char character);
inline void gotoXY(char x, char y);

inline void updateScreen(void);
inline void writeLine(unsigned char line, char* str);

inline void osc1MenuUpdate(void);
inline void osc1MenuWaveformUpdate(void);
inline void osc1MenuCentsUpdate(void);
inline void osc1MenuSemisUpdate(void);
inline void osc1MenuSyncUpdate(void);

inline void osc2MenuUpdate(void);
inline void osc2MenuWaveformUpdate(void);
inline void osc2MenuCentsUpdate(void);
inline void osc2MenuSemisUpdate(void);
inline void osc2MenuOctaveUpdate(void);

inline void mixerMenuUpdate(void);
inline void mixerMenuOsc1WeightUpdate(void);
inline void mixerMenuOsc2WeightUpdate(void);
inline void mixerMenuOsc2SyncUpdate(void);
inline void mixerMenuEmptyLine(void);

inline void lfoMenuUpdate(void);
inline void lfoMenuWaveformUpdate(void);
inline void lfoMenuFrequencyUpdate(void);
inline void lfoMenuDepthUpdate(void);
inline void lfoMenuRouteUpdate(void);

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

inline void lowPassFilter(unsigned long *val);
inline void highPassFilter(unsigned long *val);

void uartInit(void);

void oscInit(void);

void btnInit(void);

void setup(void);

void noteUpdate(void);

int main(void)
{
	setup();
	
	osc1MenuUpdate();
	
	for(int i = 0; i < MENU_ARP; i++)
	{
		menuKnobPos[i][0] = 512;
		menuKnobPos[i][1] = 512;
		menuKnobPos[i][2] = 512;
		menuKnobPos[i][3] = 512;
		
		menuChange[i][0] = 0;
		menuChange[i][1] = 0;
		menuChange[i][2] = 0;
		menuChange[i][3] = 0;
	}
	
	while(1)
	{
		//Refresh analog conversion
		sbi(ADCSRA, ADSC);
		
		//Button Reading
		if(!(OSC1_BTN_RD & (1 << OSC1_BTN_PIN)))
		{
			if(!btnPressed[MENU_OSC1] && btnCanPress[MENU_OSC1] && menuSelect != MENU_OSC1)
			{		
				btnLastPressed[MENU_OSC1] = 0;
				
				menuSelect = MENU_OSC1;

				menuChange[menuSelect][0] = 0;
				menuChange[menuSelect][1] = 0;
				menuChange[menuSelect][2] = 0;
				menuChange[menuSelect][3] = 0;
				
				osc1MenuUpdate();
			}
		}
		else
		{
			if(btnPressed[MENU_OSC1])
			btnPressed[MENU_OSC1] = false;
			
			if(!btnCanPress[MENU_OSC1])
			btnLastPressed[MENU_OSC1]++;
			
			if(btnLastPressed[MENU_OSC1] > TIME_DEBOUNCE)
			btnCanPress[MENU_OSC1] = true;
		}
		
		if(!(OSC2_BTN_RD & (1 << OSC2_BTN_PIN)))
		{
			if(!btnPressed[MENU_OSC2] && btnCanPress[MENU_OSC2] && menuSelect != MENU_OSC2)
			{
				btnLastPressed[MENU_OSC2] = 0;			
				
				menuSelect = MENU_OSC2;				
				
				menuChange[menuSelect][0] = 0;
				menuChange[menuSelect][1] = 0;
				menuChange[menuSelect][2] = 0;
				menuChange[menuSelect][3] = 0;	

				osc2MenuUpdate();
			}
		}
		else
		{
			if(btnPressed[MENU_OSC2])
			btnPressed[MENU_OSC2] = false;
					
			if(!btnCanPress[MENU_OSC2])
			btnLastPressed[MENU_OSC2]++;
					
			if(btnLastPressed[MENU_OSC2] > TIME_DEBOUNCE)
			btnCanPress[MENU_OSC2] = true;
		}
		
		if(!(SAVE_BTN_RD & (1 << SAVE_BTN_PIN)))
		{
			if(!btnPressed[MENU_LFO] && btnCanPress[MENU_LFO] && menuSelect != MENU_LFO)
			{
				btnLastPressed[MENU_LFO] = 0;
				
				menuSelect = MENU_LFO;
				
				menuChange[menuSelect][0] = 0;
				menuChange[menuSelect][1] = 0;
				menuChange[menuSelect][2] = 0;
				menuChange[menuSelect][3] = 0;

				lfoMenuUpdate();
			}
		}
		else
		{
			if(btnPressed[MENU_LFO])
			btnPressed[MENU_LFO] = false;
			
			if(!btnCanPress[MENU_LFO])
			btnLastPressed[MENU_LFO]++;
			
			if(btnLastPressed[MENU_LFO] > TIME_DEBOUNCE)
			btnCanPress[MENU_LFO] = true;
		}
		
		if(!(BANK_BTN_RD & (1 << BANK_BTN_PIN)))
		{
			if(!btnPressed[MENU_MIX] && btnCanPress[MENU_MIX] && menuSelect != MENU_MIX)
			{
				btnLastPressed[MENU_MIX] = 0;
				
				menuSelect = MENU_MIX;
				
				menuChange[menuSelect][0] = 0;
				menuChange[menuSelect][1] = 0;
				menuChange[menuSelect][2] = 0;
				menuChange[menuSelect][3] = 0;

				mixerMenuUpdate();
			}
		}
		else
		{
			if(btnPressed[MENU_MIX])
			btnPressed[MENU_MIX] = false;
			
			if(!btnCanPress[MENU_MIX])
			btnLastPressed[MENU_MIX]++;
			
			if(btnLastPressed[MENU_MIX] > TIME_DEBOUNCE)
			btnCanPress[MENU_MIX] = true;
		}

		//Menu Updates
		switch(menuSelect)
		{
			case MENU_OSC1:
			if(updateADC[0] && adcValue[0] < menuKnobPos[MENU_OSC1][0] + 10 && adcValue[0] > (menuKnobPos[MENU_OSC1][0] > 10 ? menuKnobPos[MENU_OSC1][0] - 10 : 0))
			{
				menuChange[MENU_OSC1][0] = 1;
				updateADC[0] = false;
			}
			if(updateADC[1] && adcValue[1] < menuKnobPos[MENU_OSC1][1] + 10 && adcValue[1] > (menuKnobPos[MENU_OSC1][1] > 10 ? menuKnobPos[MENU_OSC1][1] - 10 : 0))
			{
				menuChange[MENU_OSC1][1] = 1;
				updateADC[1] = false;
			}
			if(updateADC[2] && adcValue[2] < menuKnobPos[MENU_OSC1][2] + 10 && adcValue[2] > (menuKnobPos[MENU_OSC1][2] > 10 ? menuKnobPos[MENU_OSC1][2] - 10 : 0))
			{
				menuChange[MENU_OSC1][2] = 1;
				updateADC[2] = false;
			}
			if(updateADC[3] && adcValue[3] < menuKnobPos[MENU_OSC1][3] + 10 && adcValue[3] > (menuKnobPos[MENU_OSC1][3] > 10 ? menuKnobPos[MENU_OSC1][3] - 10 : 0))
			{
				menuChange[MENU_OSC1][3] = 1;
				updateADC[3] = false;
			}
			
			if(menuChange[MENU_OSC1][3] != 0)
			{
				if(adcValue[3] < 146)
				{
					if(osc1WaveForm != WAVE_SINE)
					{
						osc1WaveForm = WAVE_SINE;
						menuKnobPos[MENU_OSC1][3] = 73;
						osc1MenuWaveformUpdate();
					}
				}
				else if(adcValue[3] < 292)
				{
					if(osc1WaveForm != WAVE_TRI)
					{
						osc1WaveForm = WAVE_TRI;
						menuKnobPos[MENU_OSC1][3] = 219;
						osc1MenuWaveformUpdate();
					}
				}
				else if(adcValue[3] < 438)
				{
					if(osc1WaveForm != WAVE_LSAW)
					{
						osc1WaveForm = WAVE_LSAW;
						menuKnobPos[MENU_OSC1][3] = 365;
						osc1MenuWaveformUpdate();
					}
				}
				else if(adcValue[3] < 584)
				{
					if(osc1WaveForm != WAVE_RSAW)
					{
						osc1WaveForm = WAVE_RSAW;
						menuKnobPos[MENU_OSC1][3] = 511;
						osc1MenuWaveformUpdate();
					}
				}
				else if(adcValue[3] < 730)
				{
					if(osc1WaveForm != WAVE_SQU)
					{
						osc1WaveForm = WAVE_SQU;
						menuKnobPos[MENU_OSC1][3] = 584 + 73;
						osc1MenuWaveformUpdate();
					}
				}
				else if(adcValue[3] < 876)
				{
					if(osc1WaveForm != WAVE_NOISE)
					{
						osc1WaveForm = WAVE_NOISE;
						menuKnobPos[MENU_OSC1][3] = 730 + 73;
						osc1MenuWaveformUpdate();
					}
				}
				else if(adcValue[3] < 1023)
				{
					if(osc1WaveForm != WAVE_FLAT)
					{
						osc1WaveForm = WAVE_FLAT;
						menuKnobPos[MENU_OSC1][3] = 876 + 73;
						osc1MenuWaveformUpdate();
					}
				}
			}
			if(menuChange[MENU_OSC1][2] != 0)
			{
				if(osc1SemisShift != (adcValue[2] >> 4) - 32)
				{
					osc1SemisShift = (adcValue[2] >> 4) - 32;
					menuKnobPos[MENU_OSC1][2] = adcValue[2];
					osc1MenuSemisUpdate();
					noteUpdate();
				}
			}
			if(menuChange[MENU_OSC1][1] != 0)
			{
				if(osc1CentsShift != (adcValue[1] >> 4) - 32)
				{
					osc1CentsShift = (adcValue[1] >> 4) - 32;
					menuKnobPos[MENU_OSC1][1] = adcValue[1];
					osc1MenuCentsUpdate();
					noteUpdate();
				}
			}
			if(menuChange[MENU_OSC1][0] != 0)
			{
				if(adcValue[0] < 512)
				{
					if(osc1NoteSync != false)
					{
						osc1NoteSync = false;
						osc1MenuSyncUpdate();
					}
				}
				else if(adcValue[0] < 1023)
				{
					if(osc1NoteSync != true)
					{
						osc1NoteSync = true;
						osc1MenuSyncUpdate();
					}
				}
			}
			break;
			
			case MENU_OSC2:
			if(updateADC[0] && adcValue[0] < menuKnobPos[MENU_OSC2][0] + 10 && adcValue[0] > (menuKnobPos[MENU_OSC2][0] > 10 ? menuKnobPos[MENU_OSC2][0] - 10 : 0))
			{
				menuChange[MENU_OSC2][0] = 1;
				updateADC[0] = false;
			}
			if(updateADC[1] && adcValue[1] < menuKnobPos[MENU_OSC2][1] + 10 && adcValue[1] > menuKnobPos[MENU_OSC2][1] - 10)
			{
				menuChange[MENU_OSC2][1] = 1;
				updateADC[1] = false;
			}
			if(updateADC[2] && adcValue[2] < menuKnobPos[MENU_OSC2][2] + 10 && adcValue[2] > menuKnobPos[MENU_OSC2][2] - 10)
			{
				menuChange[MENU_OSC2][2] = 1;
				updateADC[2] = false;
			}
			if(updateADC[3] && adcValue[3] < menuKnobPos[MENU_OSC2][3] + 10 && adcValue[3] > menuKnobPos[MENU_OSC2][3] - 10)
			{
				menuChange[MENU_OSC2][3] = 1;
				updateADC[3] = false;
			}
			
		if(menuChange[MENU_OSC2][3] != 0)
		{
				if(adcValue[3] < 146)
				{
					if(osc2WaveForm != WAVE_SINE)
					{
						osc2WaveForm = WAVE_SINE;
						menuKnobPos[MENU_OSC2][3] = 73;
						osc2MenuWaveformUpdate();
					}
				}
				else if(adcValue[3] < 292)
				{
					if(osc2WaveForm != WAVE_TRI)
					{
						osc2WaveForm = WAVE_TRI;
						menuKnobPos[MENU_OSC2][3] = 219;
						osc2MenuWaveformUpdate();
					}
				}
				else if(adcValue[3] < 438)
				{
					if(osc2WaveForm != WAVE_LSAW)
					{
						osc2WaveForm = WAVE_LSAW;
						menuKnobPos[MENU_OSC2][3] = 365;
						osc2MenuWaveformUpdate();
					}
				}
				else if(adcValue[3] < 584)
				{
					if(osc2WaveForm != WAVE_RSAW)
					{
						osc2WaveForm = WAVE_RSAW;
						menuKnobPos[MENU_OSC2][3] = 511;
						osc2MenuWaveformUpdate();
					}
				}
				else if(adcValue[3] < 730)
				{
					if(osc2WaveForm != WAVE_SQU)
					{
						osc2WaveForm = WAVE_SQU;
						menuKnobPos[MENU_OSC2][3] = 584 + 73;
						osc2MenuWaveformUpdate();
					}
				}
				else if(adcValue[3] < 876)
				{
					if(osc2WaveForm != WAVE_NOISE)
					{
						osc2WaveForm = WAVE_NOISE;
						menuKnobPos[MENU_OSC2][3] = 730 + 73;
						osc2MenuWaveformUpdate();
					}
				}
				else if(adcValue[3] < 1023)
				{
					if(osc2WaveForm != WAVE_FLAT)
					{
						osc2WaveForm = WAVE_FLAT;
						menuKnobPos[MENU_OSC2][3] = 876 + 73;
						osc2MenuWaveformUpdate();
					}
				}
			}
			if(menuChange[MENU_OSC2][2] != 0)
			{
				if(osc2SemisShift != (adcValue[2] >> 2) - 128)
				{
					osc2SemisShift = (adcValue[2] >> 2) - 128;
					menuKnobPos[MENU_OSC2][2] = adcValue[2];
					osc2MenuSemisUpdate();
				}
			}
			if(menuChange[MENU_OSC2][1] != 0)
			{
				if(osc2CentsShift != (adcValue[1] >> 2) - 128)
				{
					osc2CentsShift = (adcValue[1] >> 2) - 128;
					menuKnobPos[MENU_OSC2][1] = adcValue[1];
					osc2MenuCentsUpdate();
				}
			}
			if(menuChange[MENU_OSC2][0] != 0)
			{
				if(adcValue[0] < 205)
				{
					if(osc2OctaveShift != -2)
					{
						osc2OctaveShift = -2;
						menuKnobPos[MENU_OSC2][0] = 102;
						osc2MenuOctaveUpdate();
					}
				}
				else if(adcValue[0] < 410)
				{
					if(osc2OctaveShift != -1)
					{
						osc2OctaveShift = -1;
						menuKnobPos[MENU_OSC2][0] = 205+102;
						osc2MenuOctaveUpdate();
					}
				}
				else if(adcValue[0] < 615)
				{
					if(osc2OctaveShift != 0)
					{
						osc2OctaveShift = 0;
						menuKnobPos[MENU_OSC2][0] = 410+102;
						osc2MenuOctaveUpdate();
					}
				}
				else if(adcValue[0] < 820)
				{
					if(osc2OctaveShift != 1)
					{
						osc2OctaveShift = 1;
						menuKnobPos[MENU_OSC2][0] = 615+102;
						osc2MenuOctaveUpdate();
					}
				}
				else if(adcValue[0] < 1023)
				{
					if(osc2OctaveShift != 2)
					{
						osc2OctaveShift = 2;
						menuKnobPos[MENU_OSC2][0] = 820+102;
						osc2MenuOctaveUpdate();
					}
				}
			}
			break;
			
			case MENU_LFO:
			if(updateADC[0] && adcValue[0] < menuKnobPos[MENU_LFO][0] + 10 && adcValue[0] > (menuKnobPos[MENU_LFO][0] > 10 ? menuKnobPos[MENU_LFO][0] - 10 : 0))
			{
				menuChange[MENU_LFO][0] = 1;
				updateADC[0] = false;
			}
			if(updateADC[1] && adcValue[1] < menuKnobPos[MENU_LFO][1] + 10 && adcValue[1] > (menuKnobPos[MENU_LFO][1] > 10 ? menuKnobPos[MENU_LFO][1] - 10 : 0))
			{
				menuChange[MENU_LFO][1] = 1;
				updateADC[1] = false;
			}
			if(updateADC[2] && adcValue[2] < menuKnobPos[MENU_LFO][2] + 10 && adcValue[2] > (menuKnobPos[MENU_LFO][2] > 10 ? menuKnobPos[MENU_LFO][2] - 10 : 0))
			{
				menuChange[MENU_LFO][2] = 1;
				updateADC[2] = false;
			}
			if(updateADC[3] && adcValue[3] < menuKnobPos[MENU_LFO][3] + 10 && adcValue[3] > (menuKnobPos[MENU_LFO][3] > 10 ? menuKnobPos[MENU_LFO][3] - 10 : 0))
			{
				menuChange[MENU_LFO][3] = 1;
				updateADC[3] = false;
			}
						
			if(menuChange[MENU_LFO][3] != 0)
			{
				if(adcValue[3] < 171)
				{
					if(lfoWaveForm != WAVE_SINE)
					{
						lfoWaveForm = WAVE_SINE;
						menuKnobPos[MENU_LFO][0] = 85;
						lfoMenuWaveformUpdate();
					}
				}
				else if(adcValue[3] < 341)
				{
					if(lfoWaveForm != WAVE_TRI)
					{
						lfoWaveForm = WAVE_TRI;
						menuKnobPos[MENU_LFO][0] = 171 + 85;
						lfoMenuWaveformUpdate();
					}
				}
				else if(adcValue[3] < 511)
				{
					if(lfoWaveForm != WAVE_LSAW)
					{
						lfoWaveForm = WAVE_LSAW;
						menuKnobPos[MENU_LFO][0] = 341 + 85;
						lfoMenuWaveformUpdate();
					}
				}
				else if(adcValue[3] < 682)
				{
					if(lfoWaveForm != WAVE_RSAW)
					{
						lfoWaveForm = WAVE_RSAW;
						menuKnobPos[MENU_LFO][0] = 511 + 85;
						lfoMenuWaveformUpdate();
					}
				}
				else if(adcValue[3] < 852)
				{
					if(lfoWaveForm != WAVE_SQU)
					{
						lfoWaveForm = WAVE_SQU;
						menuKnobPos[MENU_LFO][0] = 682 + 85;
						lfoMenuWaveformUpdate();
					}
				}
				else if(adcValue[3] < 1023)
				{
					if(lfoWaveForm != WAVE_FLAT)
					{
						lfoWaveForm = WAVE_FLAT;
						menuKnobPos[MENU_LFO][0] = 852 + 85;
						lfoMenuWaveformUpdate();
					}
				}
			}
			if(menuChange[MENU_LFO][2] != 0)
			{
				if((adcValue[2] >> 2) != lfoPrintFreq)
				{
					lfoPrintFreq = adcValue[2] >> 2;
					menuKnobPos[MENU_LFO][0] = adcValue[2];
					lfoMenuFrequencyUpdate();
				}
			}
			if(menuChange[MENU_LFO][1] != 0)
			{
				if((adcValue[1] >> 2) != lfoDepth)
				{
					lfoDepth = adcValue[1] >> 2;
					menuKnobPos[MENU_LFO][0] = adcValue[1];
					lfoMenuDepthUpdate();
				}
			}
			if(menuChange[MENU_LFO][0] != 0)
			{
				if(adcValue[0] < 93)
				{
					if(lfoRoute != ROUTE_OSC1)
					{
						lfoRoute = ROUTE_OSC1;
						lfoMenuRouteUpdate();
					}
				}
				else if(adcValue[0] < 186)
				{
					if(lfoRoute != ROUTE_OSC2)
					{
						lfoRoute = ROUTE_OSC2;
						lfoMenuRouteUpdate();
					}					
				}
				else if(adcValue[0] < 279)
				{
					if(lfoRoute != ROUTE_CENTS1)
					{
						lfoRoute = ROUTE_CENTS1;
						lfoMenuRouteUpdate();
					}
				}
				else if(adcValue[0] < 372)
				{
					if(lfoRoute != ROUTE_CENTS2)
					{
						lfoRoute = ROUTE_CENTS2;
						lfoMenuRouteUpdate();
					}
				}
				else if(adcValue[0] < 465)
				{
					if(lfoRoute != ROUTE_SEMIS1)
					{
						lfoRoute = ROUTE_SEMIS1;
						lfoMenuRouteUpdate();
					}
				}
				else if(adcValue[0] < 558)
				{
					if(lfoRoute != ROUTE_SEMIS2)
					{
						lfoRoute = ROUTE_SEMIS2;
						lfoMenuRouteUpdate();
					}
				}
				else if(adcValue[0] < 651)
				{
					if(lfoRoute != ROUTE_PHASE1)
					{
						lfoRoute = ROUTE_PHASE1;
						lfoMenuRouteUpdate();
					}
				}
				else if(adcValue[0] < 744)
				{
					if(lfoRoute != ROUTE_PHASE2)
					{
						lfoRoute = ROUTE_PHASE2;
						lfoMenuRouteUpdate();
					}
				}
				else if(adcValue[0] < 837)
				{
					if(lfoRoute != ROUTE_WAVE1)
					{
						lfoRoute = ROUTE_WAVE1;
						lfoMenuRouteUpdate();
					}
				}
				else if(adcValue[0] < 930)
				{
					if(lfoRoute != ROUTE_WAVE2)
					{
						lfoRoute = ROUTE_WAVE2;
						lfoMenuRouteUpdate();
					}
				}
				else if(adcValue[0] < 1023)
				{
					if(lfoRoute != ROUTE_OSC1)
					{
						lfoRoute = ROUTE_OSC1;
						lfoMenuRouteUpdate();
					}
				}
			}
			break;		
		}
	}
}

void setup()
{
	//Init fixed point variables
	toFixed(31372.550, refclk);
	reftime = fixedDivide(1, refclk);
	stepConst = fixedDivide(256, refclk);

	toFixed(440, osc1Freq);
	osc1TWord = fixedMultiply(osc1Freq, stepConst);

	toFixed(440.5, osc2Freq);
	osc2TWord = fixedMultiply(osc2Freq, stepConst);
	
	initLCD();
	lcdClear();
	
	adcInit();
	uartInit();
	oscInit();
	btnInit();
	
	sbi(OSC_OUT_DIR, OSC_OUT_PIN);
	
	sbi (TIMSK2,TOIE2);
	sei();

	noteUpdate();
}

void btnInit()
{
	cbi(OSC1_BTN_DIR, OSC1_BTN_PIN);
	sbi(OSC1_BTN_WR, OSC1_BTN_PIN);
	
	cbi(OSC2_BTN_DIR, OSC2_BTN_PIN); //Set OSC2_BTN as an input
	sbi(OSC2_BTN_WR, OSC2_BTN_PIN); //Set the internal pull-up
	
	cbi(SAVE_BTN_DIR, SAVE_BTN_PIN); //Set SAVE_BTN as an input
	sbi(SAVE_BTN_WR, SAVE_BTN_PIN);	//Set the internal pull-up
	
	cbi(BANK_BTN_DIR, BANK_BTN_PIN);
	sbi(BANK_BTN_WR, BANK_BTN_PIN);
	
	cbi(LOCK_BTN_DIR, LOCK_BTN_PIN);
	sbi(LOCK_BTN_WR, LOCK_BTN_PIN);
	
	cbi(EXTRA_BTN_DIR, EXTRA_BTN_PIN);
	sbi(EXTRA_BTN_WR, EXTRA_BTN_PIN);	
}

void adcInit()
{
	DIDR0 = (1 << ADC0D) | (1 << ADC1D) | (1 << ADC2D) | (1 << ADC3D);
	
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS1);
}

void oscInit() {

	// Timer2 Clock Prescaler to : 1
	sbi (TCCR2B, CS20);
	cbi (TCCR2B, CS21);
	cbi (TCCR2B, CS22);

	// Timer2 PWM Mode set to Phase Correct PWM
	cbi (TCCR2A, COM2A0);  // clear Compare Match
	sbi (TCCR2A, COM2A1);

	sbi (TCCR2A, WGM20);  // Mode 1  / Phase Correct PWM
	cbi (TCCR2A, WGM21);
	cbi (TCCR2B, WGM22);
}

void noteUpdate()
{
	osc1Freq = 	keyFreq[playThisNote + osc1SemisShift];
	osc2Freq = keyFreq[playThisNote + osc2SemisShift + (osc2OctaveShift*12)];

	unsigned long centsConst = 0x27*osc1CentsShift;
	osc1Freq = osc1Freq + fixedMultiply(osc1Freq, centsConst);

	centsConst = 0x27*osc2CentsShift;
	osc2Freq = osc2Freq + fixedMultiply(osc2Freq, centsConst);

	osc1TWord = fixedMultiply(osc1Freq, stepConst);
	osc2TWord = fixedMultiply(osc2Freq, stepConst);
}

inline void writeLine(unsigned char line, char* str)
{
	char buf[13];
	memcpy(buf, str, 12);
	buf[12] = '\0';
	
	gotoXY(0, line);
	lcdString(buf);
}

void lcdShift(unsigned char data)
{
	int i = 0;
	
	cbi(LCD_CLK_WR, LCD_CLK_PIN);
	
	for(i = 0; i < 8; i++)
	{
		if(data & (1 << (7-i)))
		{
			sbi(LCD_DATA_WR, LCD_DATA_PIN);
		}
		else
		{
			cbi(LCD_DATA_WR, LCD_DATA_PIN);
		}
		
		sbi(LCD_CLK_WR, LCD_CLK_PIN);
		cbi(LCD_CLK_WR, LCD_CLK_PIN);
	}
}

inline void gotoXY(char x, char y)
{
	lcdWrite(0, (0x80 | x));
	lcdWrite(0, (0x40 | y));
}

inline void osc1MenuUpdate()
{
	char buf[20];
	
	sprintf(buf, "%s", osc1MenuTitle);
	writeLine(0, buf);
	
	osc1MenuWaveformUpdate();
	osc1MenuCentsUpdate();
	osc1MenuSemisUpdate();
	osc1MenuSyncUpdate();
}

inline void osc1MenuWaveformUpdate()
{
	char buf[20];
	
	sprintf(buf, "%s", waveformLine);
	strcat_P(buf, (PGM_P)pgm_read_word(&waveNameTable[osc1WaveForm]));
	strcat(buf, "       ");
	writeLine(2, buf);	
}

inline void osc1MenuSemisUpdate()
{
	char buf[20];
	
	sprintf(buf, "%s", semisLine);
	sprintf(buf + 7, "%d", osc1SemisShift);
	strcat(buf, "       ");
	writeLine(3, buf);
}

inline void osc1MenuCentsUpdate()
{
	char buf[20];
	
	sprintf(buf, "%s", centsLine);
	sprintf(buf + 7, "%d", osc1CentsShift);
	strcat(buf, "       ");
	writeLine(4, buf);
}

inline void osc1MenuSyncUpdate()
{
	char buf[20];
	
	sprintf(buf, "%s", noteSyncLine);
	if(osc1NoteSync != false)
	{
		sprintf(buf + 6, "%s", onString);
	}
	else
	{
		sprintf(buf + 6, "%s", offstring);
	}
	strcat(buf, "       ");
	writeLine(5, buf);
}

inline void osc2MenuUpdate()
{
	char buf[20];
	
	sprintf(buf, "%s", osc2MenuTitle);
	writeLine(0, buf);
	
	osc2MenuWaveformUpdate();
	osc2MenuSemisUpdate();
	osc2MenuCentsUpdate();
	osc2MenuOctaveUpdate();
}

inline void osc2MenuWaveformUpdate()
{
	char buf[20];
	
	sprintf(buf, "%s", waveformLine);
	strcat_P(buf, (PGM_P)pgm_read_word(&waveNameTable[osc2WaveForm]));
	strcat(buf, "       ");
	writeLine(2, buf);	
}

inline void osc2MenuSemisUpdate()
{
	char buf[20];
	
	sprintf(buf, "%s", semisLine);
	sprintf(buf + 7, "%d", osc2SemisShift);
	strcat(buf, "       ");
	writeLine(3, buf);
}

inline void osc2MenuCentsUpdate()
{
	char buf[20];
	
	sprintf(buf, "%s", centsLine);
	sprintf(buf + 7, "%d", osc2CentsShift);
	strcat(buf, "       ");
	writeLine(4, buf);
}

inline void osc2MenuOctaveUpdate()
{
	char buf[20];
	
	sprintf(buf, "%s", octaveLine);
	sprintf(buf + 5, "%d", osc2OctaveShift);
	strcat(buf, "        ");
	writeLine(5, buf);
}

inline void mixerMenuUpdate()
{
	char buf[20];
	
	sprintf(buf, "%s", mixingMenuTitle);
	writeLine(0, buf);

	mixerMenuOsc1WeightUpdate();
	mixerMenuOsc2WeightUpdate();
	mixerMenuOsc2SyncUpdate();
	mixerMenuEmptyLine();
}

inline void mixerMenuOsc1WeightUpdate()
{
	char buf[20];
	
	sprintf(buf, "%s", osc1WeightLine);
	sprintf(buf + 9, "%d", osc1Weight);
	strcat(buf, "    ");
	writeLine(2, buf);
}

inline void mixerMenuOsc2WeightUpdate()
{
	char buf[20];
	
	sprintf(buf, "%s", osc2WeightLine);
	sprintf(buf + 9, "%d", osc2Weight);
	strcat(buf, "    ");
	writeLine(3, buf);
}

inline void mixerMenuOsc2SyncUpdate()
{
	char buf[20];
	
	sprintf(buf, "%s", osc2SyncLine);
	if(osc2Sync != false)
	{
		sprintf(buf + 8, "%s", onString);
	}
	else
	{
		sprintf(buf + 8, "%s", offstring);
	}
	strcat(buf, "    ");
	writeLine(4, buf);
}

inline void mixerMenuEmptyLine()
{
	writeLine(5, "            ");
}

inline void lfoMenuUpdate()
{
	char buf[20];
	
	sprintf(buf, "%s", lfoMenuTitle);
	writeLine(0, buf);

	lfoMenuWaveformUpdate();
	lfoMenuFrequencyUpdate();
	lfoMenuDepthUpdate();
	lfoMenuRouteUpdate();
}

inline void lfoMenuWaveformUpdate()
{
	char buf[20];
	
	sprintf(buf, "%s", waveformLine);
	strcat_P(buf, (PGM_P)pgm_read_word(&waveNameTable[lfoWaveForm]));
	strcat(buf, "    ");
	writeLine(2, buf);
}

inline void lfoMenuFrequencyUpdate()
{
	char buf[20];
	
	sprintf(buf, "%s", frequencyLine);
	sprintf(buf + 6, "%d", lfoPrintFreq);
	strcat(buf, "    ");
	writeLine(3, buf);
}

inline void lfoMenuDepthUpdate()
{
	char buf[20];
	
	sprintf(buf, "%s", depthLine);
	sprintf(buf + 6, "%d", lfoDepth);
	strcat(buf, "    ");
	writeLine(4, buf);
}

inline void lfoMenuRouteUpdate()
{
	char buf[20];
	
	sprintf(buf, "%s", routeLine);
	strcat_P(buf, (PGM_P)pgm_read_word(&routeNameTable[lfoRoute]));
	strcat(buf, "    ");
	writeLine(5, buf);
}

void initLCD()
{
	sbi(LCD_SCE_DIR, LCD_SCE_PIN);
	sbi(LCD_RST_DIR, LCD_RST_PIN);
	sbi(LCD_DC_DIR, LCD_DC_PIN);
	sbi(LCD_DATA_DIR, LCD_DATA_PIN);
	sbi(LCD_CLK_DIR, LCD_CLK_PIN);
	
	cbi(LCD_RST_WR, LCD_RST_PIN);
	sbi(LCD_RST_WR, LCD_RST_PIN);
	
	lcdWrite(LCD_COMMAND, 0x21);
	lcdWrite(LCD_COMMAND, 0xB0);
	lcdWrite(LCD_COMMAND, 0x04);
	lcdWrite(LCD_COMMAND, 0x14);
	
	lcdWrite(LCD_COMMAND, 0x20);
	lcdWrite(LCD_COMMAND, 0x0C);
}

inline void lcdWrite(char control, char data)
{
	if(control == 0)
	{
		cbi(LCD_DC_WR, LCD_DC_PIN);
	}
	else
	{
		sbi(LCD_DC_WR, LCD_DC_PIN);
	}
	
	cbi(LCD_SCE_WR, LCD_SCE_PIN);
	lcdShift(data);
	sbi(LCD_SCE_WR, LCD_SCE_PIN);
}

inline void lcdCharacter(char character)
{
	lcdWrite(LCD_DATA, 0x00);
	
	for(unsigned char index = 0; index < 5; index++)
	{
		lcdWrite(LCD_DATA, ASCII[character - 0x20][index]);
	}
	
	lcdWrite(LCD_DATA, 0x00);
}

inline void lcdString(char* str)
{
	while(*str)
		lcdCharacter(*str++);
}

inline void lcdClear()
{
	for(int index = 0; index < (LCD_X*LCD_Y)/8; index++)
	{
		lcdWrite(LCD_DATA, 0x00);
	}
	
	gotoXY(0,0);
}

void uartInit()
{
	UBRR0H = (uint8_t)(BAUD_PRESCALLER>>8);
	UBRR0L = (uint8_t)(BAUD_PRESCALLER);
	
	UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);
}

inline void lfsrUpdate()
{
	unsigned char lsb = lfsrState & 0x01;
	lfsrState = lfsrState >> 1;
	
	if(lsb == 1)
	{
		lfsrState ^= 0xB400;
	}
}

ISR(USART_RX_vect)
{
	unsigned char msg = UDR0;
	
	if(msg & 0x80) //New command
	{
		commandBytes[0] = msg & 0xF0;
		
		commandCount = 1;
	}
	else
	{
		commandBytes[commandCount] = msg;
		commandCount++;
		
		if(commandCount == 3)
		{
			commandCount = 1;
			
			switch(commandBytes[0])
			{
				case NOTE_ON:
				notePlaying = true;
				break;
				
				case NOTE_OFF:
				notePlaying = false;
				break;
			}
		}
	}
}

ISR(ADC_vect)
{
	adcValue[adcSelect] = ADC;
	updateADC[adcSelect] = true;

	adcSelect++;
	
	if(adcSelect > 3)
	adcSelect = 0;
	
	ADMUX = adcSelect;
}

ISR(TIMER2_OVF_vect)
{
	osc1Phaccu += osc1TWord;
	osc2Phaccu += osc2TWord;
	
	lfsrUpdate();

	if(notePlaying)
	{
		unsigned short fraction = 0;
		unsigned short whole = 0;

		osc1Out[0] = osc1Out[1];
		osc1Out[1] = osc1Out[2];
		osc1Out[2] = pgm_read_byte(analogWaveTable + waveformOffset[osc1WaveForm] + (unsigned char)(*((unsigned char*)(&osc1Phaccu) + 2) + osc1PhaseShift));
		
		if(osc1WaveForm == WAVE_NOISE)
			osc1Out[2] = lfsrState;

		if(osc2Sync && (osc1Out[1] < osc1Out[0] && osc1Out[1] < osc1Out[2]))
		{
			osc2Phaccu = 0;
		}

		osc2Out[2] = pgm_read_byte(analogWaveTable + pgm_read_word(waveformOffset + osc2WaveForm) + (unsigned char)(*((unsigned char*)(&osc2Phaccu) + 2) + osc2PhaseShift));
		
		if(osc2WaveForm = WAVE_NOISE)
			osc2Out[2] = lfsrState;

		unsigned long temp = 0;

		osc1Out[2] *= osc1Weight;
		osc2Out[2] *= osc2Weight;

		temp = osc1Out[2] + osc2Out[2];
		
		/*switch(filterMode)
		{
			case FILTER_LOW:
			lowPassFilter(&temp);
			break;
			
			case FILTER_HIGH:
			highPassFilter(&temp);
			break;
			
			case FILTER_BAND:
			break;
		}*/
		
		OCR2A = *((unsigned char*)&temp + 1);
	}
}

inline void lowPassFilter(unsigned long *val)
{
	*val = (*val - prevOutput[0]);
	*val = (*val*filterCutoff);
	
	*val = *((unsigned short*)((unsigned char*)val + 1)) + prevOutput[0];
	
	prevOutput[0] = *val;
	
	*val = (*val - prevOutput[1]);
	*val = (*val*filterCutoff);
	
	*val = *((unsigned short*)((unsigned char*)val + 1)) + prevOutput[1];
	
	prevOutput[1] = *val;
}

inline void highPassFilter(unsigned long *val)
{
	unsigned long temp = *val;
	
	*val = (prevOutput[0] + *val - prevInput[0]);
	*val = (*val*filterCutoff);
	
	*val = *((unsigned short*)((unsigned char*)val + 1));
	
	prevOutput[0] = *val;
	prevInput[0] = temp;
	
	temp = *val;
	
	*val = (prevOutput[1] + *val - prevInput[1]);
	*val = (*val*filterCutoff);
	
	*val = *((unsigned short*)((unsigned char*)val + 1));
	
	prevOutput[1] = *val;
	prevInput[1] = temp;
}

//Fixed Lib
inline void toFixed(int a, unsigned long &b)
{
	*((unsigned short*)&(b) + 1) = a;
	*((unsigned short*)&(b)) = 0;
}

inline void toFixed(double a, unsigned long &b)
{
	char* doubleingPointer = (char*)&a;
	
	unsigned long fraction = 0;
	fraction = (unsigned char)(*(doubleingPointer + 2) & 0x7F);
	fraction = fraction << 8;
	fraction |= (unsigned char)*(doubleingPointer + 1);
	fraction = fraction << 8;
	fraction |= (unsigned char)*doubleingPointer;
	fraction = fraction >> 7;
	
	unsigned char exponent = 0;
	exponent = (unsigned char)(*(doubleingPointer + 3) & 0x7F);
	exponent = exponent << 1;
	exponent |= (unsigned char)((*(doubleingPointer + 2) & 0x80) >> 7);
	
	unsigned char sign = (unsigned char)(*(doubleingPointer + 3) & 0x80);
	
	b = fraction | 0x10000;
	
	if(exponent < 127)
	{
		b = b >> ((int)exponent - 127);
	}
	else
	{
		b = b << ((int)exponent - 127);
	}
}

inline void toFixed(unsigned short &a, unsigned long &b)
{
	b = a;
	b = b << 8;
}

inline void toFixed(int a, unsigned short &b)
{
	*((unsigned char*)&(b) + 1) = a;
	*((unsigned char*)&(b)) = 0;
}

inline void toFixed(double a, unsigned short &b)
{
	unsigned long temp;
	toFixed(a, temp);

	b = (unsigned short)(temp >> 9);
}

inline void toFixed(unsigned long &a, unsigned short &b)
{
	b = (a >> 8);
}

inline void toFixed(int a, volatile unsigned long &b)
{
	*((unsigned short*)&(b)) = a;
}

inline void toFixed(double a, volatile unsigned long &b)
{
	char* doubleingPointer = (char*)&a;
	
	unsigned long fraction = 0;
	fraction = (unsigned char)(*(doubleingPointer + 2) & 0x7F);
	fraction = fraction << 8;
	fraction |= (unsigned char)*(doubleingPointer + 1);
	fraction = fraction << 8;
	fraction |= (unsigned char)*doubleingPointer;
	fraction = fraction >> 7;
	
	unsigned char exponent = 0;
	exponent = (unsigned char)(*(doubleingPointer + 3) & 0x7F);
	exponent = exponent << 1;
	exponent |= (unsigned char)((*(doubleingPointer + 2) & 0x80) >> 7);
	
	unsigned char sign = (unsigned char)(*(doubleingPointer + 3) & 0x80);
	
	b = fraction | 0x10000;
	
	if(exponent < 127)
	{
		b = b >> ((int)exponent - 127);
	}
	else
	{
		b = b << ((int)exponent - 127);
	}
}

inline void toFixed(unsigned short &a, volatile unsigned long &b)
{
	b = a;
	b = b << 8;
}

inline void toFixed(int a, volatile unsigned short &b)
{
	*((unsigned char*)&(b) + 1) = a;
	*((unsigned char*)&(b)) = 0;
}

inline void toFixed(double a, volatile unsigned short &b)
{
	unsigned long temp;
	char* doubleingPointer = (char*)&a;
	
	unsigned short fraction = 0;
	fraction = (unsigned char)(*(doubleingPointer + 2) & 0x7F);
	fraction = fraction << 8;
	fraction |= (unsigned char)*(doubleingPointer + 1);
	fraction = fraction << 8;
	fraction |= (unsigned char)*doubleingPointer;
	fraction = fraction >> 7;
	
	unsigned char exponent = 0;
	exponent = (unsigned char)(*(doubleingPointer + 3) & 0x7F);
	exponent = exponent << 1;
	exponent |= (unsigned char)((*(doubleingPointer + 2) & 0x80) >> 7);
	
	unsigned char sign = (unsigned char)(*(doubleingPointer + 3) & 0x80);
	
	temp = fraction | 0x10000;
	
	if(exponent < 127)
	{
		temp = temp >> ((int)exponent - 127);
	}
	else
	{
		temp = temp << ((int)exponent - 127);
	}
	
	b = (unsigned short)(temp >> 8);
}

inline void toFixed(unsigned long &a, volatile unsigned short &b)
{
	b = (a >> 8);
}

inline unsigned long fixedAdd(unsigned long &lhs, int rhs)
{
	unsigned long ret;
	*((unsigned short*)&(ret) + 1) = *((unsigned short*)&(lhs) + 1) + rhs;
	*((unsigned short*)&(ret)) = 0;
	
	return ret;
}

inline unsigned long fixedAdd(int lhs, unsigned long &rhs)
{
	unsigned long ret;
	*((unsigned short*)&(ret) + 1) = *((unsigned short*)&(rhs) + 1) + lhs;
	*((unsigned short*)&(ret)) = 0;
	
	return ret;
}

inline unsigned long fixedAdd(unsigned long &lhs, double rhs)
{
	unsigned long ret;
	toFixed(rhs, ret);
	
	ret += lhs;
	
	return ret;
}

inline unsigned long fixedAdd(double lhs, unsigned long rhs)
{
	unsigned long ret;
	toFixed(lhs, ret);
	
	ret += rhs;
	
	return ret;
}

inline unsigned long fixedSub(unsigned long &lhs, int rhs)
{
	unsigned long ret;
	*((unsigned short*)&(ret) + 1) = *((unsigned short*)&(ret) + 1) - rhs;
	*((unsigned short*)&(ret)) = 0;
	
	return ret;
}

inline unsigned long fixedSub(int lhs, unsigned long& rhs)
{
	unsigned long ret;
	*((unsigned short*)&(ret) + 1) = rhs - *((unsigned short*)&(ret) + 1);
	*((unsigned short*)&(ret)) = 0;
	
	return ret;
}

inline unsigned long fixedSub(unsigned long &lhs, double rhs)
{
	unsigned long ret;
	toFixed(rhs, ret);
	
	ret = lhs - ret;
	
	return ret;
}

inline unsigned long fixedSub(double lhs, unsigned long &rhs)
{
	unsigned long ret;
	toFixed(lhs, ret);
	
	ret = ret - rhs;
	
	return ret;
}

inline unsigned long fixedMultiply(unsigned long &lhs, unsigned long &rhs)
{
	unsigned long long temp = lhs;
	temp *= rhs;
	
	return (unsigned long)(temp >> 16);
}

inline unsigned long fixedMultiply(unsigned long &lhs, int rhs)
{
	unsigned long temp;
	toFixed(rhs, temp);
	
	return fixedMultiply(lhs, temp);
}

inline unsigned long fixedMultiply(int lhs, unsigned long &rhs)
{
	unsigned long temp;
	toFixed(lhs, temp);
	
	return fixedMultiply(rhs, temp);
}

inline unsigned long fixedMultiply(unsigned long &lhs, double rhs)
{
	unsigned long temp;
	toFixed(rhs, temp);
	
	return fixedMultiply(lhs, temp);
}

inline unsigned long fixedMultiply(double lhs, unsigned long &rhs)
{
	unsigned long temp;
	toFixed(lhs, temp);
	
	return fixedMultiply(rhs, temp);
}

inline unsigned long fixedDivide(unsigned long &lhs, unsigned long &rhs)
{
	unsigned long long temp = lhs;
	temp = temp << 16;
	
	return (unsigned long)(temp/rhs);
}

inline unsigned long fixedDivide(unsigned long &lhs, int rhs)
{
	unsigned long temp;
	toFixed(rhs, temp);
	
	return fixedDivide(lhs, temp);
}

inline unsigned long fixedDivide(unsigned long lhs, int rhs)
{
	unsigned long temp;
	toFixed(rhs, temp);
	
	return fixedDivide((unsigned long)lhs, (unsigned long)temp);
}

inline unsigned long fixedDivide(int lhs, unsigned long &rhs)
{
	unsigned long temp;
	toFixed(lhs, temp);
	
	return fixedDivide(temp, rhs);
}

inline unsigned long fixedDivide(unsigned long &lhs, double rhs)
{
	unsigned long temp;
	toFixed(rhs, temp);
	
	return fixedDivide((unsigned long)lhs, (unsigned long)temp);
}

inline unsigned long fixedDivide(double lhs, unsigned long &rhs)
{
	unsigned long temp;
	toFixed(lhs, temp);
	
	return fixedDivide((unsigned long)temp, (unsigned long)rhs);
}

inline unsigned short fixedAdd(unsigned short &lhs, int rhs)
{
	unsigned short ret;
	*((unsigned char*)&(ret) + 1) = *((unsigned char*)&(lhs) + 1) + rhs;
	*((unsigned char*)&(ret)) = 0;
	
	return ret;
}

inline unsigned short fixedAdd(int lhs, unsigned short &rhs)
{
	unsigned short ret;
	*((unsigned char*)&(ret) + 1) = *((unsigned char*)&(rhs) + 1) + lhs;
	*((unsigned char*)&(ret)) = 0;
	
	return ret;
}

inline unsigned short fixedAdd(unsigned short &lhs, double rhs)
{
	unsigned short ret;
	toFixed(rhs, ret);
	
	ret = ret + lhs;
	
	return ret;
}

inline unsigned short fixedAdd(double lhs, unsigned short rhs)
{
	unsigned short ret;
	toFixed(lhs, ret);
	
	ret = ret + rhs;
	
	return ret;
}

inline unsigned short fixedSub(unsigned short &lhs, int rhs)
{
	unsigned short ret;
	*((unsigned char*)&(ret) + 1) = *((unsigned char*)&(ret) + 1) - rhs;
	*((unsigned char*)&(ret)) = 0;
	
	return ret;
}

inline unsigned short fixedSub(int lhs, unsigned short& rhs)
{
	unsigned short ret;
	*((unsigned char*)&(ret) + 1) = rhs - *((unsigned char*)&(ret) + 1);
	*((unsigned char*)&(ret)) = 0;
	
	return ret;
}

inline unsigned short fixedSub(unsigned short &lhs, double rhs)
{
	unsigned short ret;
	toFixed(rhs, ret);
	
	ret = lhs - ret;
	
	return ret;
}

inline unsigned short fixedSub(double lhs, unsigned short &rhs)
{
	unsigned short ret;
	toFixed(lhs, ret);
	
	ret = ret - rhs;
	
	return ret;
}

inline unsigned short fixedMultiply(unsigned short &lhs, unsigned short &rhs)
{
	unsigned long temp = lhs;
	temp *= rhs;
	
	return (unsigned short)(temp >> 8);
}

inline unsigned short fixedMultiply(unsigned short &lhs, int rhs)
{
	unsigned short temp;
	toFixed(rhs, temp);
	
	return fixedMultiply(lhs, temp);
}

inline unsigned short fixedMultiply(int lhs, unsigned short &rhs)
{
	unsigned short temp;
	toFixed(lhs, temp);
	
	return fixedMultiply(rhs, temp);
}

inline unsigned short fixedMultiply(unsigned short &lhs, double rhs)
{
	unsigned short temp;
	toFixed(rhs, temp);
	
	return fixedMultiply(lhs, temp);
}

inline unsigned short fixedMultiply(double lhs, unsigned short &rhs)
{
	unsigned short temp;
	toFixed(lhs, temp);
	
	return fixedMultiply(rhs, temp);
}

inline unsigned short fixedDivide(unsigned short &lhs, unsigned short &rhs)
{
	unsigned long temp = lhs;
	temp = temp << 8;
	
	return (unsigned short)(temp/rhs);
}

inline unsigned short fixedDivide(unsigned short &lhs, int rhs)
{
	unsigned short temp;
	toFixed(rhs, temp);
	
	return fixedDivide(lhs, temp);
}

inline unsigned short fixedDivide(int lhs, unsigned short &rhs)
{
	unsigned short temp;
	toFixed(lhs, temp);
	
	return fixedDivide(temp, rhs);
}

inline unsigned short fixedDivide(unsigned short &lhs, double rhs)
{
	unsigned short temp;
	toFixed(rhs, temp);
	
	return fixedDivide(lhs, temp);
}

inline unsigned short fixedDivide(double lhs, unsigned short &rhs)
{
	unsigned short temp;
	toFixed(lhs, temp);
	
	return fixedDivide(temp, rhs);
}

inline void fixedAdd(unsigned long &res, unsigned long &lhs, unsigned short &rhs)
{
	res = rhs;
	res = res << 8;
	
	res += lhs;
}

inline void fixedAdd(unsigned long &res, unsigned short &lhs, unsigned long &rhs)
{
	res = lhs;
	res = res << 8;
	
	res += rhs;
}

inline void fixedAdd(unsigned short &res, unsigned long &lhs, unsigned short &rhs)
{
	res = (lhs >> 8);
	
	res += rhs;
}

inline void fixedAdd(unsigned short &res, unsigned short &lhs, unsigned long &rhs)
{
	res = (rhs >> 8);
	
	res += lhs;
}

inline void fixedSub(unsigned long &res, unsigned long &lhs, unsigned short &rhs)
{
	res = rhs;
	res = res << 8;
	
	res -= lhs;
}

inline void fixedSub(unsigned long &res, unsigned short &lhs, unsigned long &rhs)
{
	res = lhs;
	res = res << 8;
	
	res -= rhs;
}

inline void fixedSub(unsigned short &res, unsigned long &lhs, unsigned short &rhs)
{
	res = (lhs >> 8);
	
	res -= rhs;
}

inline void fixedSub(unsigned short &res, unsigned short &lhs, unsigned long &rhs)
{
	res = (rhs >> 8);
	
	res -= lhs;
}

inline void fixedMultiply(unsigned long &res, unsigned long &lhs, unsigned short &rhs)
{
	toFixed(rhs, res);
	res = fixedMultiply(res, lhs);
}

inline void fixedMultiply(unsigned long &res, unsigned short &lhs, unsigned long &rhs)
{
	toFixed(lhs, res);
	res = fixedMultiply(res, rhs);
}

inline void fixedMultiply(unsigned short &res, unsigned long &lhs, unsigned short &rhs)
{
	toFixed(lhs, res);
	res = fixedMultiply(res, rhs);
}

inline void fixedMultiply(unsigned short &res, unsigned short &lhs, unsigned long &rhs)
{
	toFixed(rhs, res);
	res = fixedMultiply(res, lhs);
}

inline void fixedDivide(unsigned long &res, unsigned long &lhs, unsigned short &rhs)
{
	toFixed(rhs, res);
	res = fixedDivide((unsigned long)lhs, (unsigned long)res);
}

inline void fixedDivide(unsigned long &res, unsigned short &lhs, unsigned long &rhs)
{
	toFixed(lhs, res);
	res = fixedDivide((unsigned long)res, (unsigned long)rhs);
}

inline void fixedDivide(unsigned short &res, unsigned long &lhs, unsigned short &rhs)
{
	toFixed(lhs, res);
	res = fixedDivide(res, rhs);
}

inline void fixedDivide(unsigned short &res, unsigned short &lhs, unsigned long &rhs)
{
	toFixed(rhs, res);
	res = fixedDivide(lhs, res);
}

inline unsigned char fixedToByte(unsigned short &a)
{
	return *((unsigned char*)(&a) + 1);
}

inline unsigned char fixedToByte(unsigned long &a)
{
	return *((unsigned short*)(&a) + 1);
}