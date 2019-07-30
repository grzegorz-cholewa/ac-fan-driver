/* AC FAN DRIVER */
/* The driver is based on ATMega328 and the purpose is to drive AC regulator circuit for 2 AC fans */
/* depending on readings from zero-crossing detection input and voltages from 6 thermistors */

/* STANDARD INCLUDES */
#include <asf.h>
#include <math.h> 

/* MODULES */
#include <config.h>
#include <temperature.h>

/* PIN DEFINITIONS */
#define LED_PIN IOPORT_CREATE_PIN(PORTB, 5) // evalboard LED
#define LED_RED_PIN IOPORT_CREATE_PIN(PORTD, 3)
#define ZERO_CROSSING_PIN IOPORT_CREATE_PIN(PORTD, 2) // a source for INT0 interrupt
#define FAN1_DRIVE_PIN IOPORT_CREATE_PIN(PORTD, 5) // signal for gate of triac driving fan1
#define FAN2_DRIVE_PIN IOPORT_CREATE_PIN(PORTD, 6) // signal for gate of triac driving fan2
#define FAN3_DRIVE_PIN IOPORT_CREATE_PIN(PORTD, 7) // signal for gate of triac driving fan3

/* TYPE DEFINITIONS */
typedef enum
{
	WORK_STATE_FORCE_OFF,
	WORK_STATE_FORCE_FULL_ON,
	WORK_STATE_AUTO
} module_work_state;

typedef enum 
{
	GATE_IDLE,
	GATE_ACTIVE,
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
uint32_t clock_speed = 16000000;
uint32_t gate_pulse_delay_counter_us = 0;
uint32_t pid_pulse_delay_counter_us = 0;
sensors_t sensor_values;
bool drive_triacs_flag_pending = false;

/* FUNCTION PROTOTYPES */
void drive_fan(fan_gate_t * fan);
uint32_t get_gate_delay_us(uint8_t mean_voltage);
void gpio_init(void);
void interrupt_init(void);
void led_blink(uint8_t count, uint32_t on_off_cycle_period_ms);
uint8_t pid_regulator(int current_temp, uint16_t debug_adc_read);
void set_gate_state(fan_gate_t * fan, gate_state_t pulse_state);
void timer_start(uint32_t time_us);
void send_and_indicate_error(void);
void update_working_parameters(fan_gate_t * fan);

/* FUNCTION DEFINITIONS */
void drive_fan(fan_gate_t * fan)
{
	int temp_gate_pulse_delay_counter_us = gate_pulse_delay_counter_us;
	
	switch (work_state)
	{
		case WORK_STATE_AUTO:
		{
			if (fan->mean_voltage<MIN_FAN_VOLTAGE)
			{
				set_gate_state(fan, GATE_IDLE);
				break;
			}
			
			if (fan->mean_voltage>=MAX_FAN_VOLTAGE)
			{
				set_gate_state(fan, GATE_ACTIVE);
				break;
			}
			
			if ( (temp_gate_pulse_delay_counter_us >= fan->activation_delay_us) && (temp_gate_pulse_delay_counter_us <= (fan->activation_delay_us + GATE_PULSE_TIME_US)) )
			{
					set_gate_state(fan, GATE_ACTIVE);
			}
			else
			{
				set_gate_state(fan, GATE_IDLE);
			}
			break;
		}
		case WORK_STATE_FORCE_FULL_ON:
			set_gate_state(fan, GATE_ACTIVE);
		break;
		
		case WORK_STATE_FORCE_OFF:
			set_gate_state(fan, GATE_IDLE);
		break;
	}
}

uint32_t get_gate_delay_us(uint8_t mean_voltage)
{
	double activation_angle_rad = acos(mean_voltage/230.0); // acos function input is double, value from -1 to 1
	uint32_t gate_delay = HALF_SINE_PERIOD_US*activation_angle_rad/(PI/2.0);
	
	if (gate_delay > MAX_GATE_DELAY_US)
		gate_delay = MAX_GATE_DELAY_US;
		
	if (gate_delay < MIN_GATE_DELAY_US)
		gate_delay = MIN_GATE_DELAY_US;

	return gate_delay - ZERO_CROSSING_OFFSET_US;
}

void gpio_init(void)
{
	ioport_configure_pin(LED_PIN, IOPORT_DIR_OUTPUT |  IOPORT_INIT_LOW);
	ioport_configure_pin(GPIO_PUSH_BUTTON_0, IOPORT_DIR_INPUT | IOPORT_PULL_UP);
	ioport_configure_pin(ZERO_CROSSING_PIN, IOPORT_DIR_INPUT | IOPORT_PULL_UP);
	ioport_configure_pin(FAN1_DRIVE_PIN, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	ioport_configure_pin(FAN2_DRIVE_PIN, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	ioport_configure_pin(FAN3_DRIVE_PIN, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
}

void interrupt_init(void)
{
	EICRA |= (1 << ISC01 || 1 << ISC00);    // set INT0 to trigger on rising edge
	EIMSK |= (1 << INT0);     // activates INT0
	sei();                    // turn on interrupts
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

uint8_t pid_regulator(int current_temp, uint16_t debug_adc_read)
{
	static float PID_KI = PID_KP/PID_TIME_CONST_S;
	static int error;
	static int integral;
	uint8_t mean_voltage;

	if(current_temp < MIN_WORKING_TEMPERATURE) // system error: temperature too low
	{
		send_and_indicate_error();
	}
	else if(current_temp > MAX_WORKING_TEMPERATURE) // system error: temperature too high
	{
		send_and_indicate_error();
	}
	
	error = TARGET_TEMPERATURE - current_temp; // negative number means that temperature is higher then target
	integral = integral + error;
	
	#ifdef MOCK_OUTPUT_VOLTAGE_REGULATION // FOR DEBUG ONLY
	mean_voltage = debug_adc_read/4; // voltage proportional to adc read
	//mean_voltage = 75; // const value
	
	#else // use PID regulator
	mean_voltage =  - PID_KP * error - PID_KI * integral;
	#endif
	
	if (mean_voltage >= MAX_FAN_VOLTAGE)
	mean_voltage = FAN_FULL_ON_VOLTAGE;
	
	if (mean_voltage <= MIN_FAN_VOLTAGE)
	mean_voltage = FAN_OFF_VOLTAGE;
	
	return mean_voltage;
};

void set_gate_state(fan_gate_t * fan, gate_state_t state)
{
	if (fan->state == state)
	{
		return; // no state change
	}
	
	fan->state = state;
	
	if (state == GATE_ACTIVE)
	{
		if (fan->index == 0)
			gpio_set_pin_low(FAN1_DRIVE_PIN); // optotransistor is active low
		if (fan->index == 1)
			gpio_set_pin_low(FAN2_DRIVE_PIN);
		if (fan->index == 2)
			gpio_set_pin_low(FAN3_DRIVE_PIN);
	}
	
	if (state != GATE_ACTIVE)
	{
		if (fan->index == 0)
			gpio_set_pin_high(FAN1_DRIVE_PIN);
		if (fan->index == 1)
			gpio_set_pin_high(FAN2_DRIVE_PIN);
		if (fan->index == 2)
			gpio_set_pin_high(FAN3_DRIVE_PIN);
	}
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

void send_and_indicate_error(void)
{
	// TBD send error to main MCU
	LED_On(LED_RED_PIN);	
}


void update_working_parameters(fan_gate_t * fan)
{
	read_temperatures(&sensor_values);
	fan->mean_voltage = pid_regulator(sensor_values.temperatures[fan->main_temp_sensor_index], sensor_values.adc_values[fan->main_temp_sensor_index]);
	fan->activation_delay_us = get_gate_delay_us(fan->mean_voltage);
}

int main (void)
{
	work_state = WORK_STATE_AUTO;
	static fan_gate_t fan1 = {0, 0, 0, 0, GATE_IDLE};
	static fan_gate_t fan2 = {1, 1, 0, 0, GATE_IDLE};
	static fan_gate_t fan3 = {2, 2, 0, 0, GATE_IDLE};

	gpio_init();
	adc_init();
	interrupt_init();
	led_blink(3, 300);
	
	timer_start(TRIAC_DRIVING_RESOLUTION_US);
	update_working_parameters(&fan1);
	update_working_parameters(&fan2);
	update_working_parameters(&fan3);
	
	while(1)
	{
		if (drive_triacs_flag_pending == true)
		{
			drive_fan(&fan1);
			drive_fan(&fan2);
			drive_fan(&fan3);
			drive_triacs_flag_pending = false;		
		}
		else if(pid_pulse_delay_counter_us >= WORKING_PARAMETERS_UPDATE_PERIOD_US)
		{
			update_working_parameters(&fan1);
			update_working_parameters(&fan2);
			update_working_parameters(&fan3);
			pid_pulse_delay_counter_us = 0;
		}
	}
}

/* ISR for zero-crossing detection */
ISR (INT0_vect)
{
	gate_pulse_delay_counter_us = 0;
}

/* ISR for periodical timer overflow */
ISR (TIMER1_COMPA_vect)
{
	gate_pulse_delay_counter_us += TRIAC_DRIVING_RESOLUTION_US;
	pid_pulse_delay_counter_us += TRIAC_DRIVING_RESOLUTION_US;
	drive_triacs_flag_pending = true;
}
