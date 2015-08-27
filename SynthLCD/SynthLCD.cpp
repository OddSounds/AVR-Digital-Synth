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

#define MIDI_OFFSET	21

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

unsigned char adcSelect = 0;
int adcValue[4];
bool updateADC[4] = {false};

unsigned char commandBytes[3];
unsigned char commandCount = 0;

unsigned long osc1Freq = 0;
unsigned char osc1Phaccu[3] = {0};
unsigned long osc1CurPhaccu = 0;
unsigned long osc1TWord = 0;
char osc1CentsShift = 0;
char osc1SemisShift = 0;
char osc1PhaseShift = 0;
unsigned char osc1Weight = 255;
unsigned short osc1Out[3] = {0};
bool osc1NoteSync = false;
bool osc1Busy = false;
unsigned char osc1Note = 0;

unsigned long osc2Freq = 0;
unsigned long osc2Phaccu = 0;
unsigned long osc2TWord = 0;
char osc2CentsShift = 0;
char osc2SemisShift = 0;
char osc2OctaveShift = 0;
char osc2PhaseShift = 0;
unsigned char osc2Weight = 255;
unsigned short osc2Out[3] = {0};
bool osc2Sync = false;
unsigned char osc2Note = 0;
bool osc2Busy = false;

bool duoMode = true;
bool ringMod = false;

unsigned long lfoTWord = 0;
unsigned long lfoPhaccu = 0;
unsigned short lfoOut = 0;
unsigned long lfoFreq = 0;
unsigned char lfoWaveForm = WAVE_FLAT;
unsigned char lfoRoute = 0;
unsigned char lfoPrintFreq = 0;
unsigned char lfoDepth = 0;
bool lfoNoteSync = false;

unsigned long stepConst;
unsigned long refclk;
unsigned long reftime;

unsigned char alpha = 0;
unsigned char resMix = 0;
unsigned short prevOutput[4] = {0};
unsigned short prevInput[4] = {0};

bool notePlaying = true;
unsigned char playThisNote = 48;
char keyArray[16] = {0};
char keyArrayWrite = 0;
char keyArrayRead = 0;

unsigned char delayLine[256];
unsigned char delayLineTap = 10;
unsigned char delayWriteIndex = delayLineTap;

unsigned char osc1WaveForm = WAVE_LSAW;
unsigned char osc2WaveForm = WAVE_RSAW;

unsigned char filterMode = FILTER_LOW;
unsigned char filterCutoff = 255;

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
	
	while(1)
	{
		sbi(ADCSRA, ADSC);
		
		_delay_us(100);
	}
}

void setup()
{
	//Init fixed point variables
	toFixed(31372.550, refclk);
	reftime = fixedDivide(1, refclk);
	stepConst = fixedDivide(256, refclk);

	toFixed(880, osc1Freq);
	osc1TWord = fixedMultiply(osc1Freq, stepConst);

	toFixed(880, osc2Freq);
	osc2TWord = fixedMultiply(osc2Freq, stepConst);
	
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
	osc1Freq = 	keyFreq[osc1Note + osc1SemisShift];
	osc2Freq = keyFreq[osc2Note + osc2SemisShift + (osc2OctaveShift*12)];

	unsigned long centsConst = 0x27*osc1CentsShift;
	osc1Freq = osc1Freq + fixedMultiply(osc1Freq, centsConst);

	centsConst = 0x27*osc2CentsShift;
	osc2Freq = osc2Freq + fixedMultiply(osc2Freq, centsConst);

	cli();
	osc1TWord = fixedMultiply(osc1Freq, stepConst);
	osc2TWord = fixedMultiply(osc2Freq, stepConst);
	lfoTWord = fixedMultiply(lfoFreq, stepConst);
	sei();
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

			if(commandBytes[0] == NOTE_ON && commandBytes[2] == 0)
				commandBytes[0] = NOTE_OFF;

			switch(commandBytes[0])
			{
				case NOTE_ON:
				notePlaying = true;
				
				osc2Note = osc1Note = commandBytes[1] - MIDI_OFFSET;
				keyArray[keyArrayWrite] = osc2Note;
				keyArrayWrite++;
				
				noteUpdate();
				break;
				
				case NOTE_OFF:
				if(osc1Note == commandBytes[1] - MIDI_OFFSET)
				{
					if(keyArrayWrite == 0)
					{
						notePlaying = false;
					}
					else
					{
						keyArrayWrite--;
						osc2Note = osc1Note = keyArray[keyArrayWrite];
					}
				}
				
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
	if(notePlaying)
	{
		lfsrUpdate();
		
		osc1Phaccu[0] = osc1Phaccu[1];
		osc1Phaccu[1] = osc1Phaccu[2];
		osc1Phaccu[2] = *((unsigned char*)(&osc1CurPhaccu)+2);
		
		osc1CurPhaccu += osc1TWord;

		osc1Out[0] = osc1Out[1];
		osc1Out[1] = osc1Out[2];
		osc1Out[2] = pgm_read_byte(analogWaveTable + waveformOffset[osc1WaveForm] + osc1Phaccu[2] + osc1PhaseShift);
		
		if(osc1WaveForm == WAVE_NOISE)
		osc1Out[2] = lfsrState;

		osc2Phaccu += osc2TWord;

		if(osc2Sync && (osc1Phaccu[1] < osc1Phaccu[0] &&  osc1Phaccu[1] < osc1Phaccu[2]))
		osc2Phaccu = osc1CurPhaccu;

		if(osc2WaveForm != WAVE_NOISE)
			osc2Out[2] = pgm_read_byte(analogWaveTable + waveformOffset[osc2WaveForm] + (unsigned char)*((unsigned char*)(&osc2Phaccu)+2) + osc2PhaseShift);
		else
			osc2Out[2] = lfsrState;

		lfoPhaccu += lfoTWord;
		lfoOut = pgm_read_byte(analogWaveTable + waveformOffset[lfoWaveForm] + (unsigned char)*((unsigned char*)(&lfoPhaccu)+2));
		lfoOut *= lfoDepth;

		unsigned long temp = 0;

		osc1Out[2] *= osc1Weight;
		osc2Out[2] *= osc2Weight;

		if(ringMod)
		temp = osc1Out[2] ^ osc2Out[2];
		else
		{
			temp = osc1Out[2];
			
			if(*((unsigned char*)(&osc1Out[2]) + 1)  < 128 != *((unsigned char*)(&osc2Out[2]) + 1)  < 128)
			{
				__asm__("");
				temp -= (0x8000 - osc2Out[2]);
				__asm__("");
			}
			else
			{
				temp += osc2Out[2];
			}
		}
		
		lowPassFilter(&temp);
		lowPassFilter(&temp);
		lowPassFilter(&temp);
		lowPassFilter(&temp);
		
		//out = delayLine[delayWriteIndex];
		delayLine[delayWriteIndex] = *((unsigned char*)(&temp) + 1);
		delayWriteIndex--;
		if(delayWriteIndex > delayLineTap)
			delayWriteIndex = delayLineTap;
		
		OCR2A = *((unsigned char*)(&temp) + 1);
	}
}

inline void lowPassFilter(unsigned long *val)
{
	*val = (*val - prevOutput[0]);
	*val = (*val*filterCutoff);
	
	*val = *((unsigned short*)((unsigned char*)val + 1)) + prevOutput[0];
	
	prevOutput[0] = *val;
}

inline void highPassFilter(unsigned long *val)
{
	unsigned long temp = *val;
	
	*val = (prevOutput[0] + *val - prevInput[0]);
	*val = (*val*filterCutoff);
	
	*val = *((unsigned short*)((unsigned char*)val + 1));
	
	prevOutput[0] = *val;
	prevInput[0] = temp;
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