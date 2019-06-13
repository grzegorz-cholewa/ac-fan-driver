/* AC FAN DRIVER */
/* The driver is based on ATMega328 and the purpose is to drive AC regulator circuit for 2 AC fans */
/* depending on readings from zero-crossing detection input and voltages from 6 thermistors */

#include <asf.h>
#define F_CPU 8000000UL

/* CONFIG SECTION */
#define ADC_READ_PERIOD_MS 1000
#define ADC_SENSOR_NUMBER 6
#define TRIAC_DRIVING_RESOLUTION_US 100

/* PIN DEFINITIONS */
#define LED_PIN IOPORT_CREATE_PIN(PORTB, 5)
#define ZERO_CROSSING_PIN IOPORT_CREATE_PIN(PORTD, 2) // a source for INT0 interrupt
#define FAN1_DRIVE_PIN IOPORT_CREATE_PIN(PORTD, 0) // signal for gate of triac driving fan1
#define FAN2_DRIVE_PIN IOPORT_CREATE_PIN(PORTD, 1) // signal for gate of triac driving fan2

/* TYPE DEFINITIONS */
typedef struct
{
	uint16_t adc_values[ADC_SENSOR_NUMBER];
	uint8_t temperatures[ADC_SENSOR_NUMBER];
} sensors_t;

typedef enum 
{
	GATE_IDLE,
	GATE_ACTIVE,
} gate_state_t;

typedef struct 
{
	uint8_t index; // index
	uint8_t active_state_percent; // values from 0 to 256
	uint32_t activation_delay_us; // time from zero-crossing to gate activation 
} fan_gate_t;

/* GLOBAL VARIABLES */
sensors_t sensor_values;
fan_gate_t fan1 = {0, 0, 0};
fan_gate_t fan2 = {1, 0, 0};
uint32_t clock_speed = 16000000;
uint32_t pulse_delay_counter_us = 0;

/* FUNCTION PROTOTYPES */
void gpio_init(void);
void led_blink(uint8_t count, uint32_t on_off_cycle_period_ms);
void interrupt_init(void);
void timer_start(uint32_t time_us);
uint16_t adc_value_read(uint8_t adc_channel);
void adc_init(void);
uint16_t adc_value_read(uint8_t adc_channel);;
uint8_t get_temperature(uint16_t adc_value);
void read_sensors(sensors_t * sensor_values);
uint8_t get_active_state_percent(fan_gate_t fan, sensors_t * sensor_values);
uint32_t get_gate_delay_us(fan_gate_t fan);
void set_gate_state(fan_gate_t * fan, gate_state_t pulse_state);
void update_input_data(void);
void drive_triac_gate(void);

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
	ADMUX |= (1<<REFS0); // VREF set to AVCC with external cap at AREF pin
	ADCSRA |= 1<<ADEN; // enable ADC
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // set prescaller to 128
} 

void timer_start(uint32_t time_us)
{
    uint32_t prescaler_value = 8;
	uint32_t second_us = 1000000;
	uint8_t value = time_us*(clock_speed/prescaler_value)/second_us-1; // for 100us timer value is 199
		
    OCR1A = 0x3D08; // 1526

    TCCR1B |= (1 << WGM12);
    // Mode 4, CTC on OCR1A

    TIMSK1 |= (1 << OCIE1A);
    //Set interrupt on compare match

    TCCR1B |= (1 << CS12) | (1 << CS10);
    // set prescaler to 1024 and start the timer


    sei();
    // enable interrupts
    
}

uint16_t adc_value_read(uint8_t adc_channel)
{
	
	ADMUX = (ADMUX & 0xF0) | (adc_channel & 0x0F); // select ADC channel with safety mask
	ADCSRA |= (1<<ADSC); // single conversion mode
	while( ADCSRA & (1<<ADSC) ); // wait until ADC conversion is complete
	return ADC;
}

uint8_t get_temperature(uint16_t adc_value)
{
	// MOCK FORMULA
	return adc_value/10;
}

void read_sensors(sensors_t * sensor_values)
{
	for (int i = 0; i < ADC_SENSOR_NUMBER; i++)
	{
		sensor_values->adc_values[i] = adc_value_read(i);
		sensor_values->temperatures[i] = get_temperature(sensor_values->adc_values[i]);
	}
}

uint8_t get_active_state_percent(fan_gate_t fan, sensors_t * sensor_values)
{
	// TBD consider how sensor data affects each fan
	
	// return sensor_values->temperatures[0]; // MOCK FORMULA
	return 16; // MOCK VALUE
}

uint32_t get_gate_delay_us(fan_gate_t fan)
{
	uint32_t maximum_gate_delay_us = 10000; // each half-sine lasts 10ms, so delay can be up to 10ms
	return (100-fan.active_state_percent)*maximum_gate_delay_us/100; // 
}

void set_gate_state(fan_gate_t * fan, gate_state_t pulse_state)
{
	if (fan->index == fan1.index)
	{
		if (pulse_state == GATE_ACTIVE)
			gpio_set_pin_high(FAN1_DRIVE_PIN);
		if (pulse_state == GATE_IDLE)
			gpio_set_pin_low(FAN1_DRIVE_PIN);
	}
	if (fan->index == fan2.index)
	{
		if (pulse_state == GATE_ACTIVE)
			gpio_set_pin_high(FAN2_DRIVE_PIN);
		if (pulse_state == GATE_IDLE)
			gpio_set_pin_low(FAN2_DRIVE_PIN);
	}
}

void update_input_data(void)
{
	read_sensors(&sensor_values);
	fan1.active_state_percent = get_active_state_percent(fan1, &sensor_values);
	fan2.active_state_percent = get_active_state_percent(fan2, &sensor_values);
	fan1.activation_delay_us = get_gate_delay_us(fan1);
	fan2.activation_delay_us = get_gate_delay_us(fan2);
}

void drive_triac_gate(void)
{
	if (pulse_delay_counter_us >= fan1.activation_delay_us)
	{
		set_gate_state(&fan1, GATE_ACTIVE);
	}
	if (pulse_delay_counter_us >= fan2.activation_delay_us)
	{
		set_gate_state(&fan2, GATE_ACTIVE);
	}		
}


int main (void)
{
	gpio_init();
	delay_init();
	interrupt_init();
	adc_init();
	led_blink(3, 300);
	update_input_data();
	timer_start(TRIAC_DRIVING_RESOLUTION_US);
	
	while(1)
	{	
		static uint32_t loop_counter = 0;
		drive_triac_gate();
		loop_counter++;
		if 	(loop_counter >= clock_speed)
		{
			loop_counter = 0;
			update_input_data();
			led_blink(1,200);
		}
	}
}

/* ISR for zero-crossing detection */
ISR (INT0_vect) 
{
	set_gate_state(&fan1, GATE_IDLE);
	set_gate_state(&fan2, GATE_IDLE);
	pulse_delay_counter_us = 0;
	//led_blink(2, 500);
}

/* ISR for periodical timer overflow */
ISR (TIMER1_COMPA_vect)
{
	pulse_delay_counter_us += TRIAC_DRIVING_RESOLUTION_US;
	led_blink(1,2000);
}

