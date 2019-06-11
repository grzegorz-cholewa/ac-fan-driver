/* AC FAN DRIVER */
/* The driver is based on ATMega328 and the purpose is to drive AC regulator circuit for 2 AC fans */
/* depending on readings from zero-crossing detection input and voltages from 6 thermistors */

#include <asf.h>
#define F_CPU 8000000UL

/* CONFIG SECTION */
#define ADC_READ_PERIOD_MS 1000
#define ADC_SENSOR_NUMBER 6
#define TRIAC_PULSE_WIDTH_US 2000

/* PIN DEFINITIONS */
#define LED_PIN IOPORT_CREATE_PIN(PORTB, 5)
#define ZERO_CROSSING_PIN IOPORT_CREATE_PIN(PORTD, 2) // that pin is always a source for INT0 interrupt
#define FAN1_DRIVE_PIN IOPORT_CREATE_PIN(PORTD, 0)
#define FAN2_DRIVE_PIN IOPORT_CREATE_PIN(PORTD, 1)

#define FAN1 0
#define FAN2 1

/* TYPE DEFINITIONS */
typedef struct
{
	uint16_t adc_values[ADC_SENSOR_NUMBER];
	uint8_t temperatures[ADC_SENSOR_NUMBER];
} sensor_values_t;

typedef enum 
{
	PULSE_IDLE,
	PULSE_WAITING,
	PULSE_ACTIVE
} pulse_state_t;

/* GLOBAL VARIABLES */
uint32_t clock_speed = 16000000
pulse_state_t pulse_state[2] = {PULSE_IDLE, PULSE_IDLE};

/* FUNCTION PROTOTYPES */
void gpio_init(void);
void led_blink(uint8_t count, uint32_t on_off_cycle_period_ms);
void interrupt_init(void);
void adc_init(void);
void timer0_init(void);
uint16_t adc_read(uint8_t ADCchannel);
void read_sensors(sensor_values_t * sensor_values);
uint8_t get_temperature(uint16_t adc_value);
void generate_driving_pulse(uint8_t fan_number, uint8_t fan_power);

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

void interrupt_init(void)
{
	    EICRA |= (1 << ISC01);    // set INT0 to trigger on falling edge
	    EIMSK |= (1 << INT0);     // activates INT0
	    sei();                    // turn on interrupts
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

void timer0a_start(uint32_t time_us)
{
        uint32_t prescaler_value = 256;
		uint8_t value = (clock_speed/prescaler_value)*(time_us*1000000)-1;
		
		TCCR0A |= (1 << WGM01); // set the Timer Mode to CTC
		OCR0A = value; // set the value
		TIMSK0 |= (1 << OCIE0A); // set the ISR COMPA vect
		sei(); // enable interrupts
        TCCR0A |= (1 << CS02); // set prescaler to 256 and start the timer
        TIFR0 |= (1 << TOV0); // reset the overflow flag
}

void timer0b_start(uint32_t time_us)
{
	uint32_t prescaler_value = 256;
	uint8_t value = (clock_speed/prescaler_value)*(time_us*1000000)-1;
	
	TCCR0B |= (1 << WGM01); // set the Timer Mode to CTC
	OCR0B = value; // set the value
	TIMSK0 |= (1 << OCIE0A); // set the ISR COMPA vect
	sei(); // enable interrupts
	TCCR0B |= (1 << CS02); // set prescaler to 256 and start the timer
	TIFR0 |= (1 << TOV0); // reset the overflow flag
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

void read_sensors(sensor_values_t * sensor_values)
{
	for (int i = 0; i < ADC_SENSOR_NUMBER; i++)
	{
		sensor_values->adc_values[i] = adc_read(i);
		sensor_values->temperatures[i] = get_temperature(sensor_values->adc_values[i]);
	}
}

uint8_t get_temperature(uint16_t adc_value)
{
	// it's a mock formula
	return adc_value/10;
}

uint32_t get_pulse_delay_us(uint8_t fan_number)
{
	if (fan_number == FAN1)
	{
		// mock, 5ms
		return 5000;
	}
	if (fan_number == FAN2)
	{
		// mock, 5ms
		return 5000;
	}
}

void generate_driving_pulse(uint8_t fan_1_power, uint8_t fan_2_power)
{
	// it's a mock, for indication only
	gpio_set_pin_high(FAN1_DRIVE_PIN);
	delay_ms(100);
	gpio_set_pin_low(FAN1_DRIVE_PIN);
	led_blink(1, 200);
}

int main (void)
{
	gpio_init();
	delay_init();
	interrupt_init();
	adc_init();
	sensor_values_t sensor_values;
	led_blink(3, 300);
	
	while(1)
	{	
 		read_sensors(&sensor_values);
		delay_ms(ADC_READ_PERIOD_MS);		
	}
}

/* ISR for zero-crossing detection */
ISR (INT0_vect) 
{
	pulse_state[FAN1] = PULSE_WAITING;
	pulse_state[FAN2] = PULSE_WAITING;
	timer0a_start(get_pulse_delay_us(FAN1));
	timer0b_start(get_pulse_delay_us(FAN2));
}

ISR (TIMER0_COMPA_vect)  // timer0 A overflow interrupt
{
	
	if (pulse_state[FAN1] = PULSE_WAITING)
	{
		timer0a_start(TRIAC_PULSE_WIDTH_US);
		pulse_state[FAN1] = PULSE_ACTIVE;
	}
	
	TCCR0 = 0x00; // stop the timer
	gpio_set_pin_high(FAN1_DRIVE_PIN);
}

ISR (TIMER0_COMPB_vect)  // timer0 B overflow interrupt
{
	if (pulse_state[FAN2] = PULSE_WAITING)
	{
		timer0b_start(TRIAC_PULSE_WIDTH_US);
		pulse_state[FAN2] = PULSE_ACTIVE;
	}
	
	TCCR0 = 0x00; // stop the timer
	gpio_toggle_pin(FAN2_DRIVE_PIN);
}