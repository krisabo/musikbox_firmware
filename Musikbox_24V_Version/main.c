/*
 * Musikbox_24V_Version.c
 *
 * Created: 29.01.2017 02:59:14
 * Author : kmaier
 */ 


#include "main.h"

float f_u_24v_power_last = -1;
float f_u_12v_power_last = -1;
float f_u_chrg_ctr0_last = -1;
float f_u_chrg_ctr1_last = -1;
float f_i_akku0_last	 = -1;
float f_i_akku1_last	 = -1;

float f_u_24v_power_avg = -1;
float f_u_12v_power_avg = -1;
float f_u_chrg_ctr0_avg = -1;
float f_u_chrg_ctr1_avg = -1;
float f_i_akku0_avg = -1;
float f_i_akku1_avg = -1;

int cur_adc_chn = 0;
int adc_channel_map[ADC_NUM_OF_CHANNELS];

int power_state=-1;
int power_state_sig = -1;

int main(void)
{
	//power saving
	PRR |= (1<<PRTWI) | (1<<PRSPI) | (1<<PRUSART0);
	
	//config pins
	config_io();

	//delay needed to prevent re-switching on
	_delay_ms(100);	
	PORTD |= (1<<SHDN_OUT_PIN);
	_delay_ms(2);
			
	//automatic switching to mains if mains is connected, good?
	POWER_RELAY_OUT_PORT |= (1 << POWER_RELAY_OUT_PIN);	
	ADC_ENABLE_OUT_PORT |= (1 << ADC_ENABLE_OUT_PIN);	
	
	LCD_POWER_OUT_PORT	|= (1<<LCD_POWER_OUT_PIN);
	lcd_init();    
	lcd_clear();
	lcd_setcursor(0,0);
	lcd_string("    WELCOME!    ");
	lcd_setcursor(0,1);
	lcd_string(" Kris ist super "); 
	           
	//ADC config
	//reference to 3.3V VCC
	ADMUX |= (1 << REFS0);	
	//prescaling ADC and initilization
	ADCSRA |= (0 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADSC) | (1 << ADEN);
	//read all channels
	read_ADC();
	
	f_u_24v_power_avg = f_u_24v_power_last;
	f_u_12v_power_avg = f_u_12v_power_last;
	f_u_chrg_ctr0_avg = f_u_chrg_ctr0_last;
	f_u_chrg_ctr1_avg = f_u_chrg_ctr1_last;
	f_i_akku0_avg	  = f_i_akku0_last;
	f_i_akku1_avg     = f_i_akku1_last;
	
	//wait until ONOFF is released
	while(ONOFF_BUTTON_IN_PORT & (1<<ONOFF_BUTTON_IN_PIN)){
		_delay_us(10);
	}
	
	
	int k;
	
    while (1) 
    {			
		//shut off logic
		k=0;
		while(ONOFF_BUTTON_IN_PORT & (1<<ONOFF_BUTTON_IN_PIN)){
			//debouncing
			_delay_us(100);
			k++;
			if(k>=100){
				//shut down
				PORTD &= ~(1<<SHDN_OUT_PIN);
				_delay_ms(100);
			}
		}			
		
		//measurements
		read_ADC();
		zero_pole_iir(&f_u_12v_power_avg, f_u_12v_power_last);
		zero_pole_iir(&f_u_24v_power_avg, f_u_24v_power_last);
		zero_pole_iir(&f_u_chrg_ctr0_avg, f_u_chrg_ctr0_last);
		zero_pole_iir(&f_u_chrg_ctr1_avg, f_u_chrg_ctr1_last);
		zero_pole_iir(&f_i_akku0_avg,	  f_i_akku0_last);
		zero_pole_iir(&f_i_akku1_avg,	  f_i_akku1_last);
		
		//determine power config
		//in future, maybe add external akku source, needs a new signal input logic
		//state machine input signal logic
		if(POWER_SENS_IN_PORT & (1<<POWER_SENS_IN_PIN)){
			power_state_sig = STATE_MAINS;
		}else{
			power_state_sig = STATE_AKKU;	
		}
		
		// :( this should be an automatic generated piece of code
		// state machine logic
		switch(power_state_sig){
			
			case STATE_MAINS:
				switch(power_state){
					case STATE_MAINS:
						break;
					case STATE_AKKU:
						power_state_akku_ex();
						power_state_mains_en();
						break;
					case STATE_EXTERNAL:						
						power_state_ext_ex();
						power_state_mains_en();		
					default:						
						power_state_mains_en();
						break;
				}
				power_state = STATE_MAINS;	
				break;
				
			case STATE_AKKU:
				switch(power_state){
					case STATE_MAINS:
						power_state_mains_ex();
						power_state_akku_en();
						break;
					case STATE_AKKU:
						break;
					case STATE_EXTERNAL:
						power_state_ext_ex();
						power_state_akku_en();
					default:
						power_state_akku_en();
						break;
				}
				power_state = STATE_AKKU;
				break;
				
			case STATE_EXTERNAL:
				switch(power_state){
					case STATE_MAINS:
						power_state_mains_ex();
						power_state_ext_en();
					break;
					case STATE_AKKU:
						power_state_akku_ex();
						power_state_ext_en();
						break;
					case STATE_EXTERNAL:
						break;
					default:
						power_state_ext_en();
						break;
				}
				power_state = STATE_EXTERNAL;
				
			default:
				break;
			}
			power_state_sig = -1;
		
		
		switch(power_state){
			case STATE_MAINS:
				power_state_mains_du();
				break;
			case STATE_AKKU:
				power_state_akku_du();
				break;
			case STATE_EXTERNAL:
				power_state_ext_du();
				break;
			default:
				break;
		}
		
		//TODO: go to sleep?
		_delay_ms(10);
		}
    
}


void power_state_akku_en(){
	AKKU_RELAY_OUT_PORT &= ~(1 << AKKU_RELAY_OUT_PIN);		
}
void power_state_akku_du(){
	
}
void power_state_akku_ex(){
	
}

void power_state_mains_en(){
	//AKKU to charge
	AKKU_RELAY_OUT_PORT |= (1 << AKKU_RELAY_OUT_PIN);
	//always enable lcd during mains powered
	LCD_POWER_OUT_PORT	|= (1<<LCD_POWER_OUT_PIN);
	lcd_init();
	lcd_clear();
}
void power_state_mains_du(){
		
	lcd_setcursor(0,0);
	lcd_string("MAINS POWERED");
	lcd_setcursor(0,1);
	print_number_to_lcd(f_u_24v_power_avg, 4, 1);
	lcd_string("V ");
	print_number_to_lcd(f_u_12v_power_last, 4, 1);
	lcd_string("V");
	
}
void power_state_mains_ex(){
	
}



void power_state_ext_en(){}
void power_state_ext_du(){}
void power_state_ext_ex(){}


void read_ADC(){
	//iterate over all ADC channels
	for (int i=0; i<ADC_NUM_OF_CHANNELS; i++)
	{
		//start of conversion
		ADCSRA |= (1 << ADSC);
	
		//wait until EOC
		while ((ADCSRA & (1<<ADSC)) != 0);
		
		switch(cur_adc_chn){
			case ADC_PINS_U_24_V_POWER:
				f_u_24v_power_last = ADC*GAIN_POWER_24_U;
				break;
			case ADC_PINS_U_12_V_POWER:
				f_u_12v_power_last = ADC*GAIN_POWER_12_U;
				break;
			case ADC_PINS_U_CHRG_CTR0:
				f_u_chrg_ctr0_last = ADC*GAIN_CHARGE_CTR0_U;
				break;
			case ADC_PINS_U_CHRG_CTR1:
				f_u_chrg_ctr1_last = ADC*GAIN_CHARGE_CTR1_U;
				break;
			case ADC_PINS_I_AKKU0:
				f_i_akku0_last = ADC * GAIN_AKKU0_I;
				break;
			case ADC_PINS_I_AKKU1:
				f_i_akku1_last = ADC * GAIN_AKKU1_I;
				break;
			default:
				break;			
		}				
		//next channel
		cur_adc_chn = (cur_adc_chn+1) % ADC_NUM_OF_CHANNELS;
		set_ADC_channel(adc_channel_map[cur_adc_chn]);
	}
}

void zero_pole_iir(float* avg, float new){
	(*avg) = (1-zero_pole_iir_a)*(*avg)+zero_pole_iir_a * new;
}


void config_io(){
		
		//output
		SHDN_OUT_DDR		|= (1 << SHDN_OUT_PIN);
		POWER_RELAY_OUT_DDR |= (1 << POWER_RELAY_OUT_PIN);
		ADC_ENABLE_OUT_DDR	|= (1 << ADC_ENABLE_OUT_PIN);
		AKKU_RELAY_OUT_DDR	|= (1 << AKKU_RELAY_OUT_PIN);
		LCD_POWER_OUT_DDR	|= (1 << LCD_POWER_OUT_PIN);		
		LOAD_SWITCH_OUT_DDR |= (1 << LOAD_SWITCH_OUT_PIN);
		
		//input
		ONOFF_BUTTON_IN_DDR	&= ~(1 << ONOFF_BUTTON_IN_PIN);
		POWER_SENS_IN_PORT 	&= ~(1 << POWER_SENS_IN_PIN);	
		LCD_SHOW_IN_PORT	&= ~(1 << LCD_SHOW_IN_PIN);
		//i am to lazy to make a def
		PORTD |= (1<<LCD_SHOW_IN_PIN);
		
		//ADC channels
		adc_channel_map[ADC_PINS_U_24_V_POWER] = 0;
		adc_channel_map[ADC_PINS_U_12_V_POWER] = 1;
		//adc_channel_map[ADC_PINS_U_CHRG_CTR0] = 2
		//adc_channel_map[ADC_PINS_U_CHRG_CTR1] = 3
		//adc_channel_map[ADC_PINS_I_AKKU0] = 4
		//adc_channel_map[ADC_PINS_I_AKKU1] = 5
		
}


void print_number_to_lcd(float nr, int width, int prec){
	char buffer[16];
	dtostrf(nr, width, prec, buffer);
	lcd_string(buffer);
	
}

void set_ADC_channel(int channel){
	ADMUX &= ~(0x0F << MUX0);
	ADMUX |= ((channel % 6) << MUX0);
} 