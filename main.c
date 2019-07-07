/* AC FAN DRIVER */
/* The driver is based on ATMega328 and the purpose is to drive AC regulator circuit for 2 AC fans */
/* depending on readings from zero-crossing detection input and voltages from 6 thermistors */

#include <asf.h>
#include <temperature.h>
#include <math.h> 

/* CONFIG SECTION */
#define HALF_SINE_PERIOD_US 10000
#define TRIAC_DRIVING_RESOLUTION_US 100
#define ZERO_CROSSING_OFFSET_US 260
#define GATE_PULSE_TIME_US 500
#define MIN_FAN_VOLTAGE 50 // this is min value for triac gate driver
#define MAX_FAN_VOLTAGE 225 // // this is max value for triac gate driver
#define FAN_FULL_ON_VOLTAGE
#define MIN_WORKING_TEMPERATURE 0 // exceeding this value results in sending error alert
#define MAX_WORKING_TEMPERATURE 90 // exceeding this value results in sending error alert
#define MAX_GATE_DELAY_US 9500
#define MIN_GATE_DELAY_US 500
#define TARGET_TEMPERATURE 70
#define PID_KP 1
#define PID_TIME_CONST_S 5
#define TEMPERATURE_SAMPLING_PERIOD_S 1
#define PROPORTIONAL_OUTPUT_REGULATION 1 // activates proportional regulation instead of PI

/* PIN DEFINITIONS */
#define LED_PIN IOPORT_CREATE_PIN(PORTB, 5)
#define ZERO_CROSSING_PIN IOPORT_CREATE_PIN(PORTD, 2) // a source for INT0 interrupt
#define FAN1_DRIVE_PIN IOPORT_CREATE_PIN(PORTD, 0) // signal for gate of triac driving fan1
#define FAN2_DRIVE_PIN IOPORT_CREATE_PIN(PORTD, 1) // signal for gate of triac driving fan2

/* CONSTANTS DEFINES */
#define PI (3.14)

typedef enum
{
	WORK_STATE_FORCE_OFF,
	WORK_STATE_FORCE_FULL_ON,
	WORK_STATE_AUTO
} module_work_state;

typedef enum 
{
	GATE_IDLE_BEFORE_PULSE,
	GATE_ACTIVE,
	GATE_IDLE_AFTER_PULSE
} gate_state_t;

typedef struct 
{
	const uint8_t index; // index
	const uint8_t main_temp_sensor_index;
	uint32_t mean_voltage; // values from 0 to 256
	uint32_t activation_delay_us; // time from zero-crossing to gate activation 
	gate_state_t state;
} fan_gate_t;

/* GLOBAL VARIABLES */
module_work_state work_state = WORK_STATE_FORCE_OFF;
sensors_t sensor_values;
fan_gate_t fan1 = {0, 0, 0, 0, GATE_IDLE_BEFORE_PULSE};
fan_gate_t fan2 = {1, 1, 0, 0, GATE_IDLE_BEFORE_PULSE};
uint32_t clock_speed = 16000000;
uint32_t gate_pulse_delay_counter_us = 0;
uint32_t pid_pulse_delay_counter_us = 0;

/* FUNCTION PROTOTYPES */
void gpio_init(void);
void led_blink(uint8_t count, uint32_t on_off_cycle_period_ms);
void interrupt_init(void);
void timer_start(uint32_t time_us);
uint32_t get_gate_delay_us(fan_gate_t * fan);
void set_gate_state(fan_gate_t * fan, gate_state_t pulse_state);
void update_input_data(void);
void drive_fan(fan_gate_t * fan);
void pid_regulator(fan_gate_t * fan, sensors_t * sensor_values);

/* FUNCTION DEFINITIONS */
void gpio_init(void)
{
	ioport_configure_pin(LED_PIN, IOPORT_DIR_OUTPUT |  IOPORT_INIT_LOW);
	ioport_configure_pin(GPIO_PUSH_BUTTON_0, IOPORT_DIR_INPUT | IOPORT_PULL_UP);
	ioport_configure_pin(ZERO_CROSSING_PIN, IOPORT_DIR_INPUT | IOPORT_PULL_UP);
	ioport_configure_pin(FAN1_DRIVE_PIN, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	ioport_configure_pin(FAN2_DRIVE_PIN, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);	
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
	    EICRA |= (1 << ISC01 || 1 << ISC00);    // set INT0 to trigger on rising edge
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
	double activation_angle_rad = acos(fan->mean_voltage/230.0); // acos function input is double, value from -1 to 1
	uint32_t gate_delay = HALF_SINE_PERIOD_US*activation_angle_rad/(PI/2.0);
	
	if (gate_delay > MAX_GATE_DELAY_US)
		return MAX_GATE_DELAY_US;
	if (gate_delay < MIN_GATE_DELAY_US)
		return MIN_GATE_DELAY_US;
	
	//return 8000 - ZERO_CROSSING_OFFSET_US; // const delay, for debugging
	return gate_delay - ZERO_CROSSING_OFFSET_US;
}

void set_gate_state(fan_gate_t * fan, gate_state_t state)
{
	if (fan->state == state)
	{
		return; // no state change
	}
	
	fan->state = state;
	if (state == GATE_ACTIVE)
	{
		if (fan->index == fan1.index)
			gpio_set_pin_low(FAN1_DRIVE_PIN); // optotransistor is active low
		if (fan->index == fan2.index)
			gpio_set_pin_low(FAN2_DRIVE_PIN);
	}
	
	if (state != GATE_ACTIVE)
	{
		if (fan->index == fan1.index)
			gpio_set_pin_high(FAN1_DRIVE_PIN);
		if (fan->index == fan2.index)
			gpio_set_pin_high(FAN2_DRIVE_PIN);
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

void drive_fan(fan_gate_t * fan)
{
	switch (work_state)
	{
		case WORK_STATE_AUTO:
		{
			if ( (fan->state == GATE_IDLE_BEFORE_PULSE) && (gate_pulse_delay_counter_us >= fan->activation_delay_us) )
			{
				if (fan->mean_voltage>=MIN_FAN_VOLTAGE)
					set_gate_state(fan, GATE_ACTIVE);
			}
			if ( (fan->state == GATE_ACTIVE) && (gate_pulse_delay_counter_us >= (fan->activation_delay_us + GATE_PULSE_TIME_US)) )
			{
				if (fan->mean_voltage<=MAX_FAN_VOLTAGE)
					set_gate_state(fan, GATE_IDLE_AFTER_PULSE);
			}
			break;
		}
		case WORK_STATE_FORCE_FULL_ON:
		set_gate_state(fan, GATE_ACTIVE);
		break;
		
		case WORK_STATE_FORCE_OFF:
		set_gate_state(fan, GATE_IDLE_BEFORE_PULSE);
		break;
	}
}

void pid_regulator(fan_gate_t * fan, sensors_t * sensor_values)
{
	static float PID_KI = PID_KP/PID_TIME_CONST_S;
	static int current_temp;
	static int error;
	static int integral;
	int mean_voltage;
	
	current_temp = sensor_values->temperatures[fan->main_temp_sensor_index];

	if(current_temp < MIN_WORKING_TEMPERATURE) // system error: temperature too low
	{
		// TBD send error to main MCU
	}
	else if(current_temp > MAX_WORKING_TEMPERATURE) // system error: temperature too high
	{
		// TBD send error to main MCU
	}
	
	error = TARGET_TEMPERATURE - current_temp; // negative number means that temperature is higher then target
	integral = integral + error;
	
	#ifdef PROPORTIONAL_OUTPUT_REGULATION // PROPORTIONAL REGULATION, FOR TESTING ONLY
   
	mean_voltage = 115 - 5*error;
	//mean_voltage = current_temp *3;
	// mean_voltage = 75; // for setting const value

	#else // use PID regulator
	
	mean_voltage =  - PID_KP * error - PID_KI * integral;
	
	#endif
	
	if (mean_voltage >= MAX_FAN_VOLTAGE)
		mean_voltage = MAX_FAN_VOLTAGE;
	
	if (mean_voltage <= MIN_FAN_VOLTAGE)
		mean_voltage = MIN_FAN_VOLTAGE;
	
	fan->mean_voltage = mean_voltage;
};


int main (void)
{
	gpio_init();
	adc_init();
	led_blink(3, 300);
	update_input_data();
	timer_start(TRIAC_DRIVING_RESOLUTION_US);
	interrupt_init();
	work_state = WORK_STATE_AUTO;
	
	while(1)
	{	
		if 	(pid_pulse_delay_counter_us >= TEMPERATURE_SAMPLING_PERIOD_S*1000000)
		{
			update_input_data();
			pid_pulse_delay_counter_us = 0;
			led_blink(1,200);
		}
	}
}

/* ISR for zero-crossing detection */
ISR (INT0_vect) 
{
	if (work_state == WORK_STATE_AUTO)
	{
		set_gate_state(&fan1, GATE_IDLE_BEFORE_PULSE);
		set_gate_state(&fan2, GATE_IDLE_BEFORE_PULSE);
	}
	gate_pulse_delay_counter_us = 0;
}

/* ISR for periodical timer overflow */
ISR (TIMER1_COMPA_vect)
{
	gate_pulse_delay_counter_us += TRIAC_DRIVING_RESOLUTION_US;
	pid_pulse_delay_counter_us += TRIAC_DRIVING_RESOLUTION_US;
	drive_fan(&fan1);
	drive_fan(&fan2);
}

