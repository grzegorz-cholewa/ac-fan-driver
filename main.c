/* AC FAN DRIVER */

#include <asf.h>
#define F_CPU 8000000UL

/* GLOBAL VARIABLES */


/* FUNCTION PROTOTYPES */

void led_blink(uint8_t count, uint32_t wait_time_ms);

/* FUNCTION DEFINITIONS */

void led_blink(uint8_t count, uint32_t wait_time_ms)
{
	for (int i = count; i--; i > 0)
	{
		LED_On(LED0);
		delay_ms(wait_time_ms);
		LED_Off(LED0);
		delay_ms(wait_time_ms);
	}
}

void adc_init()
{
	// Select Vref=AVcc
	ADMUX |= (1<<REFS0); // AVCC with external cap at AREF pin
	// enable ADC
	ADCSRA |= 1<<ADEN;
	//set prescaller to 128
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}

uint16_t adc_read(uint8_t ADCchannel)
{
	//select ADC channel with safety mask
	ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F);
	//single conversion mode
	ADCSRA |= (1<<ADSC);
	// wait until ADC conversion is complete
	while( ADCSRA & (1<<ADSC) );
	return ADC;
}

int main (void)
{
	/* set board io port */
	board_init();
	delay_init();
	adc_init();
	
	led_blink(5, 100);
	
	uint16_t result0 = 0;
	uint16_t result1 = 0;
	uint16_t result2 = 0;
	uint16_t result3 = 0;
	uint16_t result4 = 0;
	uint16_t result5 = 0;
	
	while(1)
	{
		result0 = adc_read(0);
		result1 = adc_read(1);
		result2 = adc_read(2);
		result3 = adc_read(3);
		result4 = adc_read(4);
		result5 = adc_read(5);
	}
	
}
