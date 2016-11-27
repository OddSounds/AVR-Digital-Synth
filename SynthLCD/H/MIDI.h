/*
 * MIDI.h
 *
 * Created: 11/9/2016 7:35:31 PM
 *  Author: Sam
 */ 


#ifndef MIDI_H_
#define MIDI_H_

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


#endif /* MIDI_H_ */