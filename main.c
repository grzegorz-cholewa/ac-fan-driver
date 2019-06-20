/* AC FAN DRIVER */
/* The driver is based on ATMega328 and the purpose is to drive AC regulator circuit for 2 AC fans */
/* depending on readings from zero-crossing detection input and voltages from 6 thermistors */

#include <asf.h>
#include <temperature.h>

/* CONFIG SECTION */
#define F_CPU 8000000UL
#define TRIAC_DRIVING_RESOLUTION_US 100
#define TARGET_TEMPERATURE 70
#define PID_CONST_P (3)
#define PID_CONST_I (0.1)

/* PIN DEFINITIONS */
#define LED_PIN IOPORT_CREATE_PIN(PORTB, 5)
#define ZERO_CROSSING_PIN IOPORT_CREATE_PIN(PORTD, 2) // a source for INT0 interrupt
#define FAN1_DRIVE_PIN IOPORT_CREATE_PIN(PORTD, 0) // signal for gate of triac driving fan1
#define FAN2_DRIVE_PIN IOPORT_CREATE_PIN(PORTD, 1) // signal for gate of triac driving fan2
#define MIN_FAN_ACTIVE_PERIOD_PERCENT 5
#define MAX_FAN_ACTIVE_PERIOD_PERCENT 95

typedef enum 
{
	GATE_IDLE,
	GATE_ACTIVE,
} gate_state_t;

typedef struct 
{
	const uint8_t index; // index
	const uint8_t main_temp_sensor_index;
	uint8_t active_state_percent; // values from 0 to 256
	uint32_t activation_delay_us; // time from zero-crossing to gate activation 
	gate_state_t state;
} fan_gate_t;

/* GLOBAL VARIABLES */
sensors_t sensor_values;
fan_gate_t fan1 = {0, 0, 0, 0, GATE_IDLE};
fan_gate_t fan2 = {1, 4, 0, 0, GATE_IDLE};
uint32_t clock_speed = 16000000;
uint32_t pulse_delay_counter_us = 0;

/* FUNCTION PROTOTYPES */
void gpio_init(void);
void led_blink(uint8_t count, uint32_t on_off_cycle_period_ms);
void interrupt_init(void);
void timer_start(uint32_t time_us);
uint32_t get_gate_delay_us(fan_gate_t * fan);
void set_gate_state(fan_gate_t * fan, gate_state_t pulse_state);
void update_input_data(void);
void drive_triac_gate(fan_gate_t * fan);
void pid_regulator(fan_gate_t * fan, sensors_t * sensor_values);

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

void timer_start(uint32_t time_us)
{
    uint32_t prescaler_value = 8;
	uint32_t second_us = 1000000;
	uint8_t value = time_us*(clock_speed/prescaler_value)/second_us-1; // for 100us timer value is 199
		
    OCR1A = value; 
    TCCR1B |= (1 << WGM12); // Mode 4, CTC on OCR1A
    TIMSK1 |= (1 << OCIE1A); // set interrupt on compare match
    TCCR1B |= (1 << CS11); // set prescaler
    sei(); // enable interrupts
}

uint32_t get_gate_delay_us(fan_gate_t * fan)
{
	uint32_t maximum_gate_delay_us = 10000; // each half-sine lasts 10ms, so delay can be up to 10ms
	return (100-fan->active_state_percent)*maximum_gate_delay_us/100; // 
}

void set_gate_state(fan_gate_t * fan, gate_state_t state)
{
	fan->state = state;
	if ((state == GATE_ACTIVE) && (fan->active_state_percent>MIN_FAN_ACTIVE_PERIOD_PERCENT))
	{
		if (fan->index == fan1.index)
			gpio_set_pin_high(FAN1_DRIVE_PIN);
		if (fan->index == fan2.index)
			gpio_set_pin_high(FAN2_DRIVE_PIN);
	}
	
	if ((state == GATE_IDLE) && (fan->active_state_percent<MAX_FAN_ACTIVE_PERIOD_PERCENT))
	{
		if (fan->index == fan1.index)
			gpio_set_pin_low(FAN1_DRIVE_PIN);
		if (fan->index == fan2.index)
			gpio_set_pin_low(FAN2_DRIVE_PIN);
	}
}

void update_input_data(void)
{
	read_temperatures(&sensor_values);
	pid_regulator(&fan1, &sensor_values);
	pid_regulator(&fan2, &sensor_values);
	fan1.activation_delay_us = get_gate_delay_us(&fan1);

	fan2.activation_delay_us = get_gate_delay_us(&fan2);
}

void drive_triac_gate(fan_gate_t * fan)
{
	if ( (fan->state == GATE_IDLE) && (pulse_delay_counter_us >= fan->activation_delay_us) )
	{
		set_gate_state(fan, GATE_ACTIVE);
	}
}

void pid_regulator(fan_gate_t * fan, sensors_t * sensor_values)
{
	static int current_temp;
	static int error;
	static int integral;
	int active_state_percent;
	
	current_temp = sensor_values->temperatures[fan->main_temp_sensor_index];
	
	if(current_temp < 0) // system error: temperature too low
	{
		// TBD send error to main MCU
		fan->active_state_percent = 0;
		return;
	}
	else if(current_temp > 90) // system error: temperature too high
	{
		// TBD send error to main MCU
		fan->active_state_percent = 100;
		return;
	}
	
	error = current_temp - TARGET_TEMPERATURE; // negative number means that temperature is lower than target
	integral = integral + error;
	
	active_state_percent =  PID_CONST_P * error  + PID_CONST_I * integral;

	fan->active_state_percent = active_state_percent;
};


int main (void)
{
	gpio_init();
	adc_init();
	led_blink(3, 300);
	update_input_data();
	set_gate_state(&fan1, GATE_IDLE);
	set_gate_state(&fan2, GATE_IDLE);
	timer_start(TRIAC_DRIVING_RESOLUTION_US);
	interrupt_init();
	
	while(1)
	{	
		static uint32_t loop_counter = 0;
		loop_counter++;
		if 	(loop_counter >= 100000)
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
	pulse_delay_counter_us = 0;
	set_gate_state(&fan1, GATE_IDLE);
	set_gate_state(&fan2, GATE_IDLE);

}

/* ISR for periodical timer overflow */
ISR (TIMER1_COMPA_vect)
{
	pulse_delay_counter_us += TRIAC_DRIVING_RESOLUTION_US; // TEMP VALUE
	drive_triac_gate(&fan1);
	drive_triac_gate(&fan2);
}

