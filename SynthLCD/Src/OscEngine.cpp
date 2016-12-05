#include <avr/io.h>
#include <avr/interrupt.h>

#include "../H/OscEngine.h"
#include "../H/WaveTables.h"
#include "../H/FixedLib.h"
#include "../H/PinDefs.h"

typedef void(*FunctionPointer)();

FunctionPointer lfoRouteFunction = NULL;

extern "C"
{
	unsigned long centsConst;
	unsigned long stepConst;
	unsigned long refclk;
	unsigned long reftime;

	unsigned char oscWaveForm[NUM_OSC];
	unsigned long oscTuningWord[NUM_OSC];
	unsigned long oscPhaseAccum[NUM_OSC];
	unsigned char oscPhaseShift[NUM_OSC];

	uint16_t lfsrState[] = {0xACE1, 0xACE1, 0xACE1};
}

void oscSetup()
{	
	//Setup oscillator 0
	TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM00);
	TIMSK0 = (1 << TOIE0);
	
	//Setup oscillator 1
	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
	
	//Setup oscillator 2
	TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM20);
	
	//Setup pin directions
	sbi(OSC0_0_DDR, OSC0_0_PIN);
	sbi(OSC0_1_DDR, OSC0_1_PIN);
	
	sbi(OSC1_0_DDR, OSC1_0_PIN);
	sbi(OSC1_1_DDR, OSC1_1_PIN);
	
	sbi(OSC2_0_DDR, OSC2_0_PIN);
	sbi(OSC2_1_DDR, OSC2_1_PIN);
}

//Time critical function
void startOsc()
{
	//Timer offsets.
	TCNT0 = 0;
	TCNT1 = 2;
	TCNT2 = 4;
	//All three need to overflow at the same time
	
	ENABLE_OSC0();
	ENABLE_OSC1();
	ENABLE_OSC2();
	
	sei();
}

inline void lfsrUpdate()
{
	asm("lds r10, lfsrState" "\n\t"
		"lds r11, (lfsrState + 1)" "\n\t"
		"lds r12, (lfsrState + 2)" "\n\t"
		"lds r13, (lfsrState + 3)" "\n\t"
		"lds r14, (lfsrState + 4)" "\n\t"
		"lds r15, (lfsrState + 5)" "\n\t"
		"ldi r16, 0xB4" "\n\t"
		"ldi r17, 0xB4" "\n\t"
		"ldi r18, 0xB4" "\n\t"
		
		"clc" "\n\t"
		"ror r11" "\n\t"
		"ror r10" "\n\t"
		"sbrc r10, 0" "\n\t"
		"eor r11, r16" "\n\t"
		
		"clc" "\n\t"
		"ror r13" "\n\t"
		"ror r12" "\n\t"
		"sbrc r12, 0" "\n\t"
		"eor r13, r17" "\n\t"
		
		"clc" "\n\t"
		"ror r15" "\n\t"
		"ror r14" "\n\t"
		"sbrc r14, 0" "\n\t"
		"eor r15, r18" "\n\t"
		
		"sts lfsrState, r10" "\n\t"
		"sts (lfsrState + 1), r11" "\n\t"
		"sts (lfsrState + 2), r12" "\n\t"
		"sts (lfsrState + 3), r13" "\n\t"
		"sts (lfsrState + 4), r14" "\n\t"
		"sts (lfsrState + 5), r15" "\n\t"
		:
		:
		:);
	
	/*lfsrState[0] = lfsrState[0] >> 1;
	if(lfsrState[0] & 0x01)
		lfsrState[0] ^= 0xB400;
		
	lfsrState[1] = lfsrState[1] >> 1;
	if(lfsrState[1] & 0x01)
		lfsrState[1] ^= 0xC400;
		
	lfsrState[2] = lfsrState[2] >> 1;
	if(lfsrState[2] & 0x01)
		lfsrState[2] ^= 0xD400;*/
}

//Oscillator 0
ISR(TIMER0_OVF_vect, ISR_NAKED)
{
	lfsrUpdate();
	
	asm("_OSC0_MATH: " "\n\t"
		"lds r16, oscPhaseAccum" "\n\t"			//Load in phase accumulator
		"lds r17, (oscPhaseAccum + 1)" "\n\t"
		"lds r18, (oscPhaseAccum + 2)" "\n\t"
		"lds r19, (oscPhaseAccum + 3)" "\n\t"
		
		"lds r20, oscTuningWord" "\n\t"			//Load in tuning word
		"lds r21, (oscTuningWord + 1)" "\n\t"
		"lds r22, (oscTuningWord + 2)" "\n\t"
		"lds r23, (oscTuningWord + 3)" "\n\t"
		
		"add r16, r20" "\n\t"			//Add tuning word to phase accumulator
		"adc r17, r21" "\n\t"
		"adc r18, r22" "\n\t"
		"adc r19, r23" "\n\t"
		
		"lds r24, oscWaveForm" "\n\t"			//Load wave form type
		
		"cpi r24, %0" "\n\t"			//Compare with WAVE_NOISE
		"breq _OSC0_NOISE" "\n\t"
		:
		: ""(WAVE_NOISE)
		: "r16", "r17", "r18", "r19", "r20", "r21", "r22", "r23", "r24");
		
	asm("lds r25, %0" "\n\t"			//Load in phase shift
		
		"lsl r24" "\n\t"				//oscWaveForm[0] * 2
		
		"add r26, r24" "\n\t"			//waveFormOffset[oscWaveFrom[0]]
		"adc r27, r1" "\n\t"			//R1 should be 0. Just need to take care of the carry
		
		"ld r28, X+" "\n\t"				//Load in wave form offset
		"ld r29, X" "\n\t"
		
		"add r30, r28" "\n\t"			//Add wave offset to wave table
		"adc r31, r29" "\n\t"
		
		"add r25, r19" "\n\t"			//Add the phase shift to the current phase accumulator
		
		"add r30, r25" "\n\t"			//Add the phase to the wave table
		"adc r31, r1" "\n\t"
		
		"lpm r24, Z" "\n\t"				//Read amplitude from program memory
		
		"out 0x27, r24" "\n\t"			//Out to OCR0A
		"rjmp _OSC1_MATH" "\n\t"
		
		"_OSC0_NOISE: " "\n\t"
		"out 0x27, r10" "\n\t"
		:
		: ""(oscPhaseShift[0]), "x"(waveformOffset), "z"(analogWaveTable)
		: "r16", "r17", "r18", "r19", "r20", "r21", "r22", "r23", "r24");
	
	asm("_OSC1_MATH:" "\n\t"
		"sts (oscPhaseAccum), r16" "\n\t"		//Save the old phase accum
		"sts (oscPhaseAccum + 1), r17" "\n\t"
		"sts (oscPhaseAccum + 2), r18" "\n\t"
		"sts (oscPhaseAccum + 3), r19" "\n\t"
		
		"lds r16, (oscPhaseAccum + 4)" "\n\t"			//Load in phase accumulator
		"lds r17, (oscPhaseAccum + 5)" "\n\t"
		"lds r18, (oscPhaseAccum + 6)" "\n\t"
		"lds r19, (oscPhaseAccum + 7)" "\n\t"
		
		"lds r20, (oscTuningWord + 4)" "\n\t"			//Load in tuning word
		"lds r21, (oscTuningWord + 5)" "\n\t"
		"lds r22, (oscTuningWord + 6)" "\n\t"
		"lds r23, (oscTuningWord + 7)" "\n\t"
		
		"add r16, r20" "\n\t"			//Add tuning word to phase accumulator
		"adc r17, r21" "\n\t"
		"adc r18, r22" "\n\t"
		"adc r19, r23" "\n\t"
		
		"lds r24, oscWaveForm" "\n\t"			//Load wave form type
		
		"cpi r24, %0" "\n\t"			//Compare with WAVE_NOISE
		"breq _OSC1_NOISE" "\n\t"
		:
		: ""(WAVE_NOISE)
		: "r16", "r17", "r18", "r19", "r20", "r21", "r22", "r23", "r24");

	asm("lds r25, %0" "\n\t"			//Load in phase shift
	
		"lsl r24" "\n\t"				//oscWaveForm[0] * 2
	
		"add r26, r24" "\n\t"			//waveFormOffset[oscWaveFrom[0]]
		"adc r27, r1" "\n\t"			//R1 should be 0. Just need to take care of the carry
	
		"ld r28, X+" "\n\t"				//Load in wave form offset
		"ld r29, X" "\n\t"
	
		"add r30, r28" "\n\t"			//Add wave offset to wave table
		"adc r31, r29" "\n\t"
	
		"add r25, r19" "\n\t"			//Add the phase shift to the current phase accumulator
	
		"add r30, r25" "\n\t"			//Add the phase to the wave table
		"adc r31, r1" "\n\t"
	
		"lpm r24, Z" "\n\t"				//Read amplitude from program memory
	
		"out 0x27, r24" "\n\t"			//Out to OCR0A
		"rjmp _OSC2_MATH" "\n\t"
	
		"_OSC1_NOISE: " "\n\t"
		"out 0x27, r11" "\n\t"
		:
		: ""(oscPhaseShift[1]), "x"(waveformOffset), "z"(analogWaveTable)
		: "r16", "r17", "r18", "r19", "r20", "r21", "r22", "r23", "r24");

	asm("_OSC2_MATH:" "\n\t"
		"sts (oscPhaseAccum + 4), r16" "\n\t"		//Save the old phase accum
		"sts (oscPhaseAccum + 5), r17" "\n\t"
		"sts (oscPhaseAccum + 6), r18" "\n\t"
		"sts (oscPhaseAccum + 7), r19" "\n\t"
		:
		:
		:);

	/*oscPhaseAccum[0] += oscTuningWord[0];
	if(oscWaveForm[0] != WAVE_NOISE)
	{
		OCR0A = pgm_read_byte(analogWaveTable + waveformOffset[oscWaveForm[0]] + (unsigned char)(*(((unsigned char*)&oscPhaseAccum[0]) + 3) + oscPhaseShift[0]));
	}
	else
		OCR0A = (unsigned char)lfsrState[0];
	
	/*oscPhaseAccum[1] += oscTuningWord[1];
	if(oscWaveForm[1] != WAVE_NOISE)
		OCR0B = pgm_read_byte(analogWaveTable + waveformOffset[oscWaveForm[1]] + (unsigned char)(*(((unsigned char*)&oscPhaseAccum[1]) + 3) + oscPhaseShift[1]));
	else
		OCR0B = *((unsigned char*)&lfsrState[0] + 1);
		
	oscPhaseAccum[2] += oscTuningWord[2];
	if(oscWaveForm[2] != WAVE_NOISE)
		OCR1A = pgm_read_byte(analogWaveTable + waveformOffset[oscWaveForm[2]] + (unsigned char)(*(((unsigned char*)&oscPhaseAccum[2]) + 3) + oscPhaseShift[2]));
	else
		OCR1A = (uint8_t)lfsrState[1];
	
	oscPhaseAccum[3] += oscTuningWord[3];
	if(oscWaveForm[3] != WAVE_NOISE)
		OCR1B = pgm_read_byte(analogWaveTable + waveformOffset[oscWaveForm[3]] + (unsigned char)(*(((unsigned char*)&oscPhaseAccum[3]) + 3) + oscPhaseShift[3]));
	else
		OCR1B = *((unsigned char*)&lfsrState[1] + 1);
	
	oscPhaseAccum[4] += oscTuningWord[4];
	if(oscWaveForm[4] != WAVE_NOISE)
		OCR2A = pgm_read_byte(analogWaveTable + waveformOffset[oscWaveForm[4]] + (unsigned char)(*(((unsigned char*)&oscPhaseAccum[4]) + 3) + oscPhaseShift[4]));
	else
		OCR2A = (uint8_t)lfsrState[2];	
	
	oscPhaseAccum[5] += oscTuningWord[5];
	if(oscWaveForm[5] != WAVE_NOISE)
		OCR2B = pgm_read_byte(analogWaveTable + waveformOffset[oscWaveForm[5]] + (unsigned char)(*(((unsigned char*)&oscPhaseAccum[5]) + 3) + oscPhaseShift[5]));
	else
		OCR2B = *((unsigned char*)&lfsrState[2] + 1);*/
	
	reti();
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