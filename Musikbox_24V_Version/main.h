/*
 * main.h
 *
 * Created: 29.01.2017 03:02:49
 *  Author: kmaier
 */ 


#ifndef MAIN_H_
#define MAIN_H_

#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "lcd.h"


//############################################################
// PIN CONFIG
//############################################################

//OUTPUT

//LCD_POWER, power for LC display, saving power
#define LCD_POWER_OUT_PORT		PORTB
#define LCD_POWER_OUT_DDR		DDRB
#define LCD_POWER_OUT_PIN		4

//SHDN, hold on power
#define SHDN_OUT_PORT			PORTD
#define SHDN_OUT_DDR			DDRC
#define SHDN_OUT_PIN			4

//AKKU_RELAY, switch akku to charge controller=1
#define AKKU_RELAY_OUT_PORT		PORTD
#define AKKU_RELAY_OUT_DDR		DDRD
#define AKKU_RELAY_OUT_PIN		6

//POWER_RELAYs, switch between mains=0 and akku=1
#define POWER_RELAY_OUT_PORT	PORTD
#define POWER_RELAY_OUT_DDR		DDRD
#define POWER_RELAY_OUT_PIN		5

//ADC_ENABLE
#define ADC_ENABLE_OUT_PORT		PORTB
#define ADC_ENABLE_OUT_DDR		DDRB
#define ADC_ENABLE_OUT_PIN		7

//LOAD_SWITCH
#define LOAD_SWITCH_OUT_PORT	PORTD
#define LOAD_SWITCH_OUT_DDR		DDRD
#define LOAD_SWITCH_OUT_PIN		1

//INPUT

//ON/OFF SENSING
#define ONOFF_BUTTON_IN_PORT	PIND
#define ONOFF_BUTTON_IN_DDR		DDRD
#define ONOFF_BUTTON_IN_PIN		2

//POWER_SENS
#define POWER_SENS_IN_PORT		PIND
#define POWER_SENS_IN_DDR		DDRD
#define POWER_SENS_IN_PIN		0

//LCD_SHOW
#define LCD_SHOW_IN_PORT		PIND
#define LCD_SHOW_IN_DDR			DDRD
#define LCD_SHOW_IN_PIN			3


//############################################################
// ADC CONFIG
//############################################################
//ADC_CHANNELS
#define ADC_CHN_POWER_12U	1
#define ADC_CHN_POWER_24U	0

//ADC Gains, float multiplication with GAIN is real value
#define GAIN_POWER_12_U		 (1.0/1024.0*3.3*430/100*13.18/13.00)
#define GAIN_POWER_24_U		 (1.0/1024.0*3.3*437/47/26.04*25.94)
#define GAIN_AKKU0_I		 (1)
#define GAIN_AKKU1_I		 (1)
#define GAIN_CHARGE_CTR0_U	 (1)
#define GAIN_CHARGE_CTR1_U	 (1)

#define zero_pole_iir_a		 0.01

//############################################################
// LCD CONFIG
//############################################################
//in lcd.h

//############################################################
// TYPEDEFs
//############################################################

enum{
	ADC_PINS_U_24_V_POWER,
	ADC_PINS_U_12_V_POWER,
	ADC_PINS_U_CHRG_CTR0,
	ADC_PINS_U_CHRG_CTR1,
	ADC_PINS_I_AKKU0,
	ADC_PINS_I_AKKU1,
	
	ADC_NUM_OF_CHANNELS
}e_ADC_PINS;

enum{
	STATE_AKKU,
	STATE_MAINS,
	STATE_EXTERNAL,
}e_STATES;


//############################################################
// FUNCTION PROTOTYPES
//############################################################

void config_io();
void read_ADC(void);

void set_ADC_channel(int channel);
void print_number_to_lcd(float nr, int width, int prec);
void zero_pole_iir(float* avg, float new);

void power_state_akku_en();
void power_state_akku_du();
void power_state_akku_ex();

void power_state_mains_en();
void power_state_mains_du();
void power_state_mains_ex();

void power_state_ext_en();
void power_state_ext_du();
void power_state_ext_ex();

void lcd_show_mains();
void lcd_show_akku();

#endif /* MAIN_H_ */