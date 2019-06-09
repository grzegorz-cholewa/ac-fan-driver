/* AC FAN DRIVER */
/* The driver is based on ATMega328 and the purpose is to drive AC regulator circuit for 2 AC fans */
/* depending on readings from zero-crossing detection input and voltages from 6 thermistors */

#include <asf.h>
#define F_CPU 8000000UL

/* APP CONFIG */
#define ADC_READ_PERIOD_MS 1000

/* PIN DEFINITIONS */
#define LED_PIN IOPORT_CREATE_PIN(PORTB, 5)
#define ZERO_CROSSING_PIN IOPORT_CREATE_PIN(PORTB, 0)
#define FAN1_DRIVE_PIN IOPORT_CREATE_PIN(PORTB, 1)
#define FAN2_DRIVE_PIN IOPORT_CREATE_PIN(PORTB, 2)

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
void gpio_init(void);
void led_blink(uint8_t count, uint32_t on_off_cycle_period_ms);
void adc_init(void);
uint16_t adc_read(uint8_t ADCchannel);
bool zero_crossing_read(void);
void read_sensor_values(sensor_values_t * sensor_values);

/* FUNCTION DEFINITIONS */
void gpio_init(void)
{
	ioport_configure_pin(LED_PIN, IOPORT_DIR_OUTPUT |  IOPORT_INIT_LOW);
	ioport_configure_pin(GPIO_PUSH_BUTTON_0, IOPORT_DIR_INPUT | IOPORT_PULL_UP);

	ioport_configure_pin(ZERO_CROSSING_PIN, IOPORT_DIR_INPUT | IOPORT_PULL_UP);
	ioport_configure_pin(FAN1_DRIVE_PIN, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	ioport_configure_pin(FAN2_DRIVE_PIN, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);	
}

void led_blink(uint8_t blink_count, uint32_t on_off_cycle_period_ms)
{
	while (blink_count > 0)
	{
		LED_On(LED_PIN);
		delay_ms(on_off_cycle_period_ms/2);
		LED_Off(LED_PIN);
		delay_ms(on_off_cycle_period_ms/2);
		blink_count--;
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

bool zero_crossing_read(void)
{
	return ioport_get_pin_level(ZERO_CROSSING_PIN);
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
	gpio_init();
	delay_init();
	adc_init();
	sensor_values_t sensor_values;
	
	led_blink(5, 300);
	
	while(1)
	{
		if (zero_crossing_read())
			led_blink(1, 500);
			
		read_sensor_values(&sensor_values);
		delay_ms(ADC_READ_PERIOD_MS);
	}
	
}
