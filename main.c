/* AC FAN DRIVER */
/* The driver is based on ATMega328 and the purpose is to drive AC regulator circuit for 2 AC fans */
/* depending on readings from zero-crossing detection input and voltages from 6 thermistors */

/* INCLUDES */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <asf.h> 
#include <config.h>
#include <temperature.h>


/* TYPE DEFINITIONS */
typedef enum
{
	WORK_STATE_FORCE_OFF,
	WORK_STATE_FORCE_FULL_ON,
	WORK_STATE_AUTO
} module_work_state_t;

typedef enum 
{
	GATE_IDLE,
	GATE_ACTIVE,
} gate_state_t;

typedef struct 
{
	port_pin_t pin;
	const uint8_t main_temp_sensor_index;
	uint32_t mean_voltage; // values from 0 to 256
	uint32_t activation_delay_us; // time from zero-crossing to gate activation 
	gate_state_t state;
} fan_gate_t;

/* GLOBAL VARIABLES */
module_work_state_t work_state = WORK_STATE_FORCE_OFF;
uint32_t gate_pulse_delay_counter_us = 0;
uint32_t pid_pulse_delay_counter_us = 0;
sensors_t sensor_values;
char uart_tx_buffer[200];

/* FUNCTION PROTOTYPES */
void drive_fan(fan_gate_t * fan_gate_array, uint8_t array_length);
uint32_t get_gate_delay_us(uint8_t mean_voltage);
void gpio_init(void);
void interrupt_init(void);
void led_blink(uint8_t count, uint32_t on_off_cycle_period_ms);
uint8_t pid_regulator(int current_temp, uint16_t debug_adc_read);
void set_gate_state(fan_gate_t * fan, gate_state_t pulse_state);
void timer_start(uint32_t time_us);
void send_and_indicate_error(void);
void update_working_parameters(fan_gate_t * fan_gate_array, uint8_t array_length);
void usart_init(void);
void uart_transmit_char(unsigned char);
void uart_transmit_string(char * string);
void rs_transmitter_enable(void);
void rs_transmitter_disable(void);
void send_debug_info(fan_gate_t * fan_gate_array);

/* FUNCTION DEFINITIONS */
void drive_fan(fan_gate_t * fan_gate_array, uint8_t array_length)
{
	for (uint8_t i = 0; i < FAN_NUMBER; i++)
	{
		cli(); // enter critical section
		switch (work_state)
		{
			case WORK_STATE_AUTO:
			{
				if (fan_gate_array[i].mean_voltage<=MIN_FAN_VOLTAGE)
				{
					set_gate_state(&fan_gate_array[i], GATE_IDLE);
					break;
				}
				
				if (fan_gate_array[i].mean_voltage>=MAX_FAN_VOLTAGE)
				{
					set_gate_state(&fan_gate_array[i], GATE_ACTIVE);
					break;
				}
				
				if ( (gate_pulse_delay_counter_us >= fan_gate_array[i].activation_delay_us) && (gate_pulse_delay_counter_us <= (fan_gate_array[i].activation_delay_us + GATE_PULSE_TIME_US)) )
				{

					set_gate_state(&fan_gate_array[i], GATE_ACTIVE);
				}
				else
				{
					set_gate_state(&fan_gate_array[i], GATE_IDLE);
				}
				break;
			}
			case WORK_STATE_FORCE_FULL_ON:
			set_gate_state(&fan_gate_array[i], GATE_ACTIVE);
			break;
			
			case WORK_STATE_FORCE_OFF:
			set_gate_state(&fan_gate_array[i], GATE_IDLE);
			break;
		}
		sei(); // leave critical section
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
	ioport_configure_pin(RS_DRIVER_ENABLE_PIN, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
}

void interrupt_init(void)
{
	EICRA |= (1 << ISC00 ) | (1 << ISC01);    // set INT0 to trigger on rising edge
	EIMSK |= (1 << INT0);     // activate INT0
	sei();                    // activate interrupts
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
	mean_voltage = debug_adc_read/4; // voltage proportional to ADC read
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
		gpio_set_pin_low(fan->pin); // optotransistor is active low
	}
	
	if (state != GATE_ACTIVE)
	{
		gpio_set_pin_high(fan->pin);
	}
}

void timer_start(uint32_t time_us)
{
	uint32_t prescaler_value = 8;
	uint32_t second_us = 1000000;
	uint8_t value = time_us*(F_CPU/prescaler_value)/second_us-1; // for 100us timer value is 199
	
	OCR1A = value;
	TCCR1B |= (1 << WGM12); // Mode 4, CTC on OCR1A
	TIMSK1 |= (1 << OCIE1A); // set interrupt on compare match
	TCCR1B |= (1 << CS11); // set prescaler
	sei(); // enable interrupts
}

void send_and_indicate_error(void)
{
	// TBD drive error pin
	// led_blink(1, 200);	
}

void update_working_parameters(fan_gate_t * fan_gate_array, uint8_t array_length)
{
	read_temperatures(&sensor_values);
	
	for (uint8_t i = 0; i < FAN_NUMBER; i++)
	{
		fan_gate_array[i].mean_voltage = pid_regulator(sensor_values.temperatures[fan_gate_array[i].main_temp_sensor_index], sensor_values.adc_values[fan_gate_array[i].main_temp_sensor_index]);
		fan_gate_array[i].activation_delay_us = get_gate_delay_us(fan_gate_array[i].mean_voltage);
	}
}

void usart_init(void)
{
	/*Set baud rate */
	UBRR0H = (unsigned char)(MYUBRR>>8);
	UBRR0L = (unsigned char)MYUBRR;
	UCSR0B = (1<<RXEN0)|(1<<TXEN0); // enable receiver and transmitter
	UCSR0C = (1<<USBS0)|(3<<UCSZ00); // set frame format: 8data, 2stop bit
	
	UCSR0B |= USART_TXC_bm; // enable tx complete interrupt
}

void uart_transmit_char(unsigned char data)
{
	rs_transmitter_enable();
	
	while (!(UCSR0A & (1<<UDRE0))) // wait for empty transmit buffer 
	;

	UDR0 = data; // put data to send buffer
}

void uart_transmit_string(char * string_to_send)
{
	memcpy(uart_tx_buffer, string_to_send, sizeof(uart_tx_buffer));
	uart_transmit_char('\n'); // first char is needed to make TX interrupt finished work
}

void rs_transmitter_enable(void)
{
	gpio_set_pin_high(RS_DRIVER_ENABLE_PIN);
}

void rs_transmitter_disable(void)
{
	gpio_set_pin_low(RS_DRIVER_ENABLE_PIN);
}

void send_debug_info(fan_gate_t * fan_gate_array)
{
	char debug_info[sizeof(uart_tx_buffer)];
	snprintf(debug_info, sizeof(debug_info), 
		"NTC1 temp: %d\nNTC2 temp: %d\nNTC3 temp: %d\nFAN1 voltage: %ld\nFAN2 voltage: %ld\nFAN3 voltage: %ld\n", 
		sensor_values.temperatures[fan_gate_array[0].main_temp_sensor_index],
		sensor_values.temperatures[fan_gate_array[1].main_temp_sensor_index],
		sensor_values.temperatures[fan_gate_array[2].main_temp_sensor_index],
		fan_gate_array[0].mean_voltage,
		fan_gate_array[1].mean_voltage,
		fan_gate_array[2].mean_voltage );
	uart_transmit_string(debug_info);
}


int main (void)
{
	work_state = WORK_STATE_AUTO;

	static fan_gate_t fan_gate_array[FAN_NUMBER] = {{FAN1_DRIVE_PIN, 0, 0, 0, GATE_IDLE}, {FAN2_DRIVE_PIN, 1, 0, 0, GATE_IDLE}, {FAN3_DRIVE_PIN, 2, 0, 0, GATE_IDLE}};
	
	gpio_init();
	usart_init();
	adc_init();
	interrupt_init();
	led_blink(3, 50);

	timer_start(GATE_DRIVING_TIMER_RESOLUTION_US);
	update_working_parameters(fan_gate_array, FAN_NUMBER);
	
	while(1)
	{
		drive_fan(fan_gate_array, FAN_NUMBER);		
		
		if(pid_pulse_delay_counter_us >= WORKING_PARAMETERS_UPDATE_PERIOD_US)
		{
			update_working_parameters(fan_gate_array, FAN_NUMBER);
			pid_pulse_delay_counter_us = 0;
			#ifdef SEND_DEBUG_INFO_OVER_RS
				send_debug_info(fan_gate_array);
			#endif
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
	gate_pulse_delay_counter_us += GATE_DRIVING_TIMER_RESOLUTION_US;
	pid_pulse_delay_counter_us += GATE_DRIVING_TIMER_RESOLUTION_US;
}

ISR(USART0_TX_vect) // USART TX complete interrupt
{
	static uint16_t buffer_index = 0;
	if (uart_tx_buffer[buffer_index] != 0x00)
	{
		uart_transmit_char(uart_tx_buffer[buffer_index]);
		buffer_index++;
	}
	else
	{
		 buffer_index = 0;
		 // rs_transmitter_disable();
	}
	
}

ISR(USART0_RX_vect)
{
	led_blink(1, 50);
}
