/*
 * PinDefs.h
 *
 * Created: 12/31/2014 8:11:32 PM
 *  Author: Sam
 */ 


#ifndef PINDEFS_H_
#define PINDEFS_H_



#define LCD_CLK_DIR		DDRD
#define LCD_CLK_WR		PORTD
#define LCD_CLK_RD		PIND
#define LCD_CLK_PIN		PORTD4

#define LCD_DATA_DIR	DDRD
#define LCD_DATA_WR		PORTD
#define LCD_DATA_RD		PIND
#define LCD_DATA_PIN	PORTD5

#define LCD_DC_DIR		DDRB
#define LCD_DC_WR		PORTB
#define LCD_DC_RD		PINB
#define LCD_DC_PIN		PORTB5

#define LCD_RST_DIR		DDRC
#define LCD_RST_WR		PORTC
#define LCD_RST_RD		PINC
#define LCD_RST_PIN		PORTC5

#define LCD_SCE_DIR		DDRC
#define LCD_SCE_WR		PORTC
#define LCD_SCE_RD		PINC
#define LCD_SCE_PIN		PORTC4

#define OSC_OUT_DIR		DDRB
#define OSC_OUT_WR		PORTB
#define OSC_OUT_RD		PINB
#define OSC_OUT_PIN		PORTB3

#define OSC1_BTN_DIR	DDRD
#define OSC1_BTN_WR		PORTD
#define OSC1_BTN_RD		PIND
#define OSC1_BTN_PIN	PORTD6

#define OSC2_BTN_DIR	DDRD
#define OSC2_BTN_WR		PORTD
#define OSC2_BTN_RD		PIND
#define OSC2_BTN_PIN	PORTD7

#define SAVE_BTN_DIR	DDRB
#define SAVE_BTN_WR		PORTB
#define SAVE_BTN_RD		PINB
#define SAVE_BTN_PIN	PORTB0

#define BANK_BTN_DIR	DDRB
#define BANK_BTN_WR		PORTB
#define BANK_BTN_RD		PINB
#define BANK_BTN_PIN	PORTB4

#define LOCK_BTN_DIR	DDRB
#define LOCK_BTN_WR 	PORTB
#define LOCK_BTN_RD 	PINB
#define LOCK_BTN_PIN	PORTB2

#define EXTRA_BTN_DIR	DDRB
#define EXTRA_BTN_WR	PORTB
#define EXTRA_BTN_RD	PINB
#define EXTRA_BTN_PIN	PORTB1

#endif /* PINDEFS_H_ */