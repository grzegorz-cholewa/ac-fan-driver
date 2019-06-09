/* AC FAN DRIVER */

#include <asf.h>
#define F_CPU 8000000UL

/* TYPE DEFINITIONS */
typedef struct
{
	uint16_t value_0;
	uint16_t value_1;
	uint16_t value_2;
	uint16_t value_3;
	uint16_t value_4;
	uint16_t value_5;
} sensor_values_t;

/* GLOBAL VARIABLES */

/* FUNCTION PROTOTYPES */

void led_blink(uint8_t count, uint32_t wait_time_ms);
void adc_init(void);
uint16_t adc_read(uint8_t ADCchannel);
void read_sensor_values(sensor_values_t * sensor_values);

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

void adc_init(void)
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

void read_sensor_values(sensor_values_t * sensor_values)
{
	sensor_values->value_0 = adc_read(0);
	sensor_values->value_1 = adc_read(1);
	sensor_values->value_2 = adc_read(2);
	sensor_values->value_3 = adc_read(3);
	sensor_values->value_4 = adc_read(4);
	sensor_values->value_5 = adc_read(5);
}

int main (void)
{
	/* set board io port */
	board_init();
	delay_init();
	adc_init();
	
	led_blink(5, 100);
	
	sensor_values_t sensor_values;

	while(1)
	{
		read_sensor_values(&sensor_values);
	}
	
}
