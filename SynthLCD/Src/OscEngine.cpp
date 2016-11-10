#include <avr/io.h>
#include <avr/interrupt.h>

#include "../H/OscEngine.h"
#include "../H/WaveTables.h"
#include "../H/FixedLib.h"
#include "../H/PinDefs.h"

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

void oscInit()
{

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

void oscSetup()
{
	//Init fixed point variables
	toFixed(31372.550, refclk);
	reftime = fixedDivide(1, refclk);
	stepConst = fixedDivide(256, refclk);
		
	toFixed(0.05, lfoFreq);
	lfoTWord = fixedMultiply(lfoFreq, stepConst);
	
	sbi(OSC_OUT_DIR, OSC_OUT_PIN);
		
	sbi (TIMSK2,TOIE2);
}

inline void osc1NoteUpdate()
{
	if(osc1Note + (osc1SemisShift + dOsc1SemisShift) + (osc1OctaveShift*12)  < 0)
	{
		cli();
		osc1TWord = keyFreq[0];
		sei();
	}
	else if(osc1Note + (osc1SemisShift + dOsc1SemisShift) + (osc1OctaveShift*12) > 87)
	{
		cli();
		osc1TWord = keyFreq[87];
		sei();
	}
	else
	{
		cli();
		osc1TWord = keyFreq[osc1Note + (osc1SemisShift + dOsc1SemisShift) + (osc1OctaveShift*12)];
		sei();
	}
}

inline void osc1CentsUpdate()
{
	unsigned long cents1Coef = 0x27*(osc1CentsShift + dOsc1CentsShift);
	
	cli();
	cents1Coef *= osc1TWord;
	cents1Coef = ((long)cents1Coef) >> 16;
	osc1TWord += cents1Coef;
	sei();
}

inline void osc2NoteUpdate()
{
	if(osc2Note + (osc2SemisShift + dOsc2SemisShift) + (osc2OctaveShift*12)  < 0)
	{
		cli();
		osc2TWord = keyFreq[0];
		sei();
	}
	else if(osc2Note + (osc2SemisShift + dOsc2SemisShift) + (osc2OctaveShift*12) > 87)
	{
		cli();
		osc2TWord = keyFreq[87];
		sei();
	}
	else
	{
		cli();
		osc2TWord = keyFreq[osc2Note + (osc2SemisShift + dOsc2SemisShift) + (osc2OctaveShift*12)];
		sei();
	}
}

inline void osc2CentsUpdate()
{
	unsigned long cents2Coef = 0x27*(osc2CentsShift + dOsc2CentsShift);
	
	cli();
	cents2Coef *= osc2TWord;
	cents2Coef = ((long)cents2Coef) >> 16;
	osc2TWord += cents2Coef;
	sei();
}

void noteUpdate()
{
	sei();
	lfoTWord = fixedMultiply(lfoFreq, stepConst);
	cli();
	
	osc1NoteUpdate();
	osc1CentsUpdate();
	
	osc2NoteUpdate();
	osc2CentsUpdate();
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

ISR(TIMER2_OVF_vect)
{	
	lfsrUpdate();	
	if(notePlaying)
	{
		clock_32k++;
		
		if(clock_32k > 32000)
			clock_32k = 0;
		
		osc1Phaccu += osc1TWord;
		
		osc1PhaccuShort[0] = osc1PhaccuShort[1];
		osc1PhaccuShort[1] = osc1PhaccuShort[2];
		osc1PhaccuShort[2] = (unsigned char)*((unsigned char*)(&osc1Phaccu) + 2);

		osc1Out[2] = (short)pgm_read_byte(analogWaveTable + waveformOffset[osc1WaveForm] + (unsigned char)(osc1PhaccuShort[2] + osc1PhaseShift));
	
		if(osc1Out[2] & 0x80)
			*((char*)(&osc1Out[2]) + 1) = 0xFF;
	
		if(osc1WaveForm == WAVE_NOISE)
			osc1Out[2] = lfsrState;
		
		osc2Phaccu += osc2TWord;

		if(osc2Sync && osc1PhaccuShort[1] < osc1PhaccuShort[0] &&  osc1PhaccuShort[1] < osc1PhaccuShort[2])
		{
			osc2Phaccu = osc1Phaccu;
		}

		if(osc2WaveForm != WAVE_NOISE)
			osc2Out[2] = (short)pgm_read_byte(analogWaveTable + waveformOffset[osc2WaveForm] + (unsigned char)*((unsigned char*)(&osc2Phaccu)+2) + osc2PhaseShift);
		else
			osc2Out[2] = lfsrState;

		if(osc2Out[2] & 0x80)
			*((char*)(&osc2Out[2]) + 1) = 0xFF;

		/*lfoPhaccu += lfoTWord;
		
		lfoOut[0] = lfoOut[1];
		lfoOut[1] = pgm_read_byte(analogWaveTable + waveformOffset[lfoWaveForm] + (unsigned char)*((unsigned char*)(&lfoPhaccu)+2));
		lfoOut[1] -= 128;*/

		long temp = 0;
		
		osc1Out[2] *= osc1Weight;
		osc2Out[2] *= osc2Weight;

		if(ringMod)
			temp = osc1Out[2] ^ osc2Out[2];
		else
		{
			temp = osc1Out[2];
			temp += osc2Out[2];
			
			temp += 0x8000;
		
			if(temp > 0xFF00)
				temp = 0xFF00;
		
			if(temp < 0)
				temp = 0;			
		}
		
		lowPassFilter((unsigned char*)(&temp + 1));
		
		/*switch(poles)
		{
			case 0:
			__asm__("nop");
			break;
			
			case 4:
			lowPassFilter((unsigned char*)(&temp + 1));
			
			case 3:
			lowPassFilter((unsigned char*)(&temp + 1));
			
			case 2:
			lowPassFilter((unsigned char*)(&temp + 1));
			
			case 1:
			lowPassFilter((unsigned char*)(&temp + 1));
			break;
		}*/
		
//Straight linear envelope
#ifdef USE_ENVELOPE 
		if(envelopePhase == ATTACK_SLOPE)
		{
			if(attackRate > 0)
			{
				envelopeGain += attackRate;
				
				if(*((unsigned char*)(&envelopeGain) + 1) >= 0xFF) //Because the largest attackRate can be is 255, envelopeGain will never change by more than 1
				{												   //The equals would be fine on its own
					envelopeGain = 0xFF00;
					envelopePhase++;
				}
			}
			else
			{
				envelopePhase++;
				envelopeGain = 0xFF00;
			}
		}
		if(envelopePhase == DECAY_SLOPE)
		{
			if(decayRate > 0)
			{
				envelopeGain -= decayRate;
				
				if(*((unsigned char*)(&envelopeGain) + 1) <= sustainLevel)
				{
					*((unsigned char*)(&envelopeGain) + 1) = sustainLevel;
					*((unsigned char*)(&envelopeGain)) = 0;
					
					//If the sustain level is 0, there's no point in going through the sustain sequence
					//or the release stage for that matter
					//Just jump to waiting for a new note
					if(sustainLevel == 0)
						envelopePhase = WAITING;
					else
						envelopePhase++;
				}
			}
			else if(decayRate == 0)
			{
				*((unsigned char*)(&envelopeGain) + 1) = sustainLevel;
				*((unsigned char*)(&envelopeGain)) = 0;
				
				if(sustainLevel == 0)
					envelopePhase = WAITING;
				else
					envelopePhase++;
				
			}
		}
		if(envelopePhase == RELEASE_SLOPE)
		{
			if(releaseRate > 0)
			{
				envelopeGain -= releaseRate;
				
				if(*((unsigned char*)(&envelopeGain) + 1) <= 0)
				{
					*((unsigned short*)(&envelopeGain)) = 0;
					
					envelopePhase++;
					//The only thing that can move the envelope from the waiting stage to attack is a new note
				}
			}
			else
			{
				*((unsigned short*)(&envelopeGain)) = 0;
				
				envelopePhase++;
			}
		}
		
		temp = (*((unsigned char*)(&temp) + 1))*(*((unsigned char*)(&envelopeGain) + 1)); //Apply envelope
		
#endif
		
		/*if(lfoOut[0] != lfoOut[1] && lfoRouteFunction != NULL)
			lfoRouteFunction();*/
		
		OCR2A = *(((unsigned char*)&temp) + 1);
	}
}

inline void lowPassFilter(unsigned char *val)
{
	//y_n+1 = y_n + alpha*(x_n - y_n)
	short diff = *val - prevOutput[0];
	
	unsigned long temp = diff;
	temp *= filterCutoff;
	
	diff = (short)(*(char*)(&temp + 1));
	diff += prevOutput[0];
	
	*val = (unsigned char)diff;
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

void lfoRouteCents1()
{
	bool isNegative = lfoOut[1] < 0;
	lfoOut[1] *= lfoOut[1] > 0 ? 1 : -1;
	unsigned short temp = lfoOut[1]*lfoDepth;
	
	if(isNegative)
	{
		dOsc1CentsShift = -1*(*(((char*)&temp) + 1));
		lfoOut[1] *= -1;
	}
	else
		dOsc1CentsShift = *(((char*)&temp) + 1);
	
	osc1NoteUpdate();
	osc1CentsUpdate();
}

void lfoRouteSemis1()
{
	bool isNegative = lfoOut[1] < 0;
	lfoOut[1] *= lfoOut[1] > 0 ? 1 : -1;
	unsigned short temp = lfoOut[1]*lfoDepth;
	
	if(isNegative)
	{
		dOsc1SemisShift = -1*(*(((char*)&temp) + 1));
		lfoOut[1] *= -1;
	}
	else
		dOsc1SemisShift = *(((char*)&temp) + 1);
		
	osc1NoteUpdate();
	osc1CentsUpdate();
}

void lfoRouteCents2()
{
	bool isNegative = lfoOut[1] < 0;
	lfoOut[1] *= lfoOut[1] > 0 ? 1 : -1;
	unsigned short temp = lfoOut[1]*lfoDepth;
	
	if(isNegative)
	{
		dOsc2CentsShift = -1*(*(((char*)&temp) + 1));
		lfoOut[1] *= -1;
	}
	else
		dOsc2CentsShift = *(((char*)&temp) + 1);
	
	osc2NoteUpdate();
	osc2CentsUpdate();
}

void lfoRouteSemis2()
{
	bool isNegative = lfoOut[1] < 0;
	lfoOut[1] *= lfoOut[1] > 0 ? 1 : -1;
	unsigned short temp = lfoOut[1]*lfoDepth;
	
	if(isNegative)
	{
		dOsc2SemisShift = -1*(*(((char*)&temp) + 1));
		lfoOut[1] *= -1;
	}
	else
		dOsc2SemisShift = *(((char*)&temp) + 1);
	
	osc2NoteUpdate();
	osc2CentsUpdate();
}

void lfoRouteCutoff()
{
	bool isNegative = lfoOut[1] < 0;
	lfoOut[1] *= lfoOut[1] > 0 ? 1 : -1;
	unsigned short temp = lfoOut[1]*lfoDepth;
}

//Fixed Lib
inline void toFixed(int a, unsigned long &b)
{
	*((unsigned short*)&(b) + 1) = a;
	*((unsigned short*)&(b)) = 0;
}

inline void toFixed(double a, unsigned long &b)
{
	b = (unsigned long)(a*65536);
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
	unsigned long long temp = 0;
	temp = lhs;
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