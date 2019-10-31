/* AC FAN DRIVER */
/* The driver is based on ATMega328 and the purpose is to drive AC regulator circuit for 2 AC fans */
/* depending on readings from zero-crossing detection input and voltages from 6 thermistors */

/* INCLUDES */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <asf.h> 
#include <config.h>
#include <ntc.h>
#include <rs485.h>
#include <modbus.h>

/* TYPE DEFINITIONS */
typedef enum
{
	WORK_STATE_AUTO,
	WORK_STATE_MANUAL
} channel_work_state_t;

typedef enum 
{
	GATE_IDLE,
	GATE_ACTIVE,
} gate_state_t;

typedef struct 
{
	port_pin_t gate_pin;
	const uint8_t temp_sensor_index;
	channel_work_state_t work_state;
	int16_t setpoint;
	uint16_t output_power; // values from 0 to 256
	uint32_t activation_delay_us; // time from zero-crossing to gate activation 
	gate_state_t state;
} channel_t;

/* GLOBAL VARIABLES */
uint32_t gate_pulse_delay_counter_us = 0;
uint32_t pi_pulse_delay_counter_us = 0;
uint32_t rx_time_interval_counter = 0;
sensors_t sensor_values;
bool modbus_request_pending_flag = false;
uint16_t temperature_error_state = 0;
uint8_t incoming_modbus_frame[RS_RX_BUFFER_SIZE];
uint8_t modbus_frame_byte_counter = 0;
static struct register_t modbus_registers[REGISTERS_RANGE];

/* FUNCTION PROTOTYPES */
uint16_t check_temperatures(sensors_t * sensor_array);
void drive_fans(channel_t * channel_array, uint8_t array_length);
uint32_t get_gate_delay_us(uint8_t output_power);
void gpio_init(void);
void interrupt_init(void);
void led_blink(uint8_t count, uint32_t on_off_cycle_period_ms);
uint16_t pi_regulator(int current_temp, uint16_t target_temperature, uint16_t debug_adc_read);
void set_gate_state(channel_t * fan, gate_state_t pulse_state);
void timer_start(uint32_t time_us);
void update_working_parameters(channel_t * channel_array, uint8_t array_length);
void send_debug_info(channel_t * channel_array);
void init_modbus_registers(channel_t * channel_array);
void update_modbus_registers(channel_t * channel_array);
void update_app_data(channel_t * channel_array);
uint8_t power_percent_to_voltage(int16_t power);
uint8_t voltage_to_power_percent(uint8_t voltage);

/* FUNCTION DEFINITIONS */
uint16_t check_temperatures(sensors_t * sensor_array)
{	
	for (int i = 0; i < ADC_SENSOR_NUMBER; i++)
	{
		int16_t temperature = sensor_array->temperatures[i];
		if(temperature > MAX_WORKING_TEMPERATURE)
		{
			return TEMPERATURE_STATUS_ERROR;
		}

		if(temperature < MIN_WORKING_TEMPERATURE)
		{
			return TEMPERATURE_STATUS_ERROR;
		}

	}
	return TEMPERATURE_STATUS_OK;
}

void drive_fans(channel_t * channel_array, uint8_t array_length)
{
	for (uint8_t i = 0; i < FAN_NUMBER; i++)
	{
		cli(); // enter critical section

		if (channel_array[i].output_power<=MIN_OUTPUT_POWER)
		{
			set_gate_state(&channel_array[i], GATE_IDLE);
		}
				
		else if (channel_array[i].output_power>=MAX_OUTPUT_POWER)
		{
			set_gate_state(&channel_array[i], GATE_ACTIVE);
		}
				
		else if ( (gate_pulse_delay_counter_us >= channel_array[i].activation_delay_us) && (gate_pulse_delay_counter_us <= (channel_array[i].activation_delay_us + GATE_PULSE_TIME_US)) )
		{
			set_gate_state(&channel_array[i], GATE_ACTIVE);
		}
		else
		{
			set_gate_state(&channel_array[i], GATE_IDLE);
		}

		sei(); // leave critical section
	}
}

uint32_t get_gate_delay_us(uint8_t output_power)
{
	uint8_t mean_voltage = power_percent_to_voltage(output_power);
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

uint16_t pi_regulator(int current_temp, uint16_t target_temperature, uint16_t debug_adc_read)
{
	static float PI_KI = PI_KP/PI_TIME_CONST_S;
	static int error;
	static int integral;
	uint16_t output_power;
	
	error = INIT_TARGET_TEMPERATURE - current_temp; // negative number means that temperature is higher then target
	integral = integral + error;
	
	#ifdef MOCK_OUTPUT_VOLTAGE_REGULATION // FOR DEBUG ONLY
		output_power = debug_adc_read/4; // voltage proportional to ADC read
	
	#else // use PID regulator
		output_power =  - PI_KP * error - PI_KI * integral;
	#endif
	
	if (output_power >= MAX_OUTPUT_POWER)
		output_power = FULL_ON_OUTPUT_POWER;
	
	if (output_power <= MIN_OUTPUT_POWER)
		output_power = FULL_OF_OUTPUT_POWER;
	
	return output_power;
};

void set_gate_state(channel_t * fan, gate_state_t state)
{
	if (fan->state == state)
	{
		return; // no state change
	}
	
	fan->state = state;
	
	if (state == GATE_ACTIVE)
	{
		gpio_set_pin_low(fan->gate_pin); // optotransistor is active low
	}
	
	if (state != GATE_ACTIVE)
	{
		gpio_set_pin_high(fan->gate_pin);
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

void update_working_parameters(channel_t * channel_array, uint8_t array_length)
{
	read_temperatures(&sensor_values);
	temperature_error_state = check_temperatures(&sensor_values);
	
	for (uint8_t i = 0; i < FAN_NUMBER; i++)
	{
		if (channel_array[i].work_state == WORK_STATE_AUTO)
		{
			channel_array[i].output_power = pi_regulator(sensor_values.temperatures[channel_array[i].temp_sensor_index],
											channel_array[i].setpoint,
											sensor_values.adc_values[channel_array[i].temp_sensor_index]);			
		}
		channel_array[i].activation_delay_us = get_gate_delay_us(channel_array[i].output_power);
	}
}

void send_debug_info(channel_t * channel_array)
{
	char debug_info[50];
	snprintf(debug_info, sizeof(debug_info),
	//"ADC:  %d %d %d %d %d %d\nTEMP: %d %d %d %d %d %d\nC1 %d C2 %d C3 %d ER %d\n",
	"T: %d %d %d %d %d %d V %d %d %d ER %d\n",
	//sensor_values.adc_values[0], sensor_values.adc_values[1], sensor_values.adc_values[2], sensor_values.adc_values[3], sensor_values.adc_values[4], sensor_values.adc_values[5],
	sensor_values.temperatures[0], sensor_values.temperatures[1], sensor_values.temperatures[2], sensor_values.temperatures[3], sensor_values.temperatures[4], sensor_values.temperatures[5],
	channel_array[0].output_power, channel_array[1].output_power, channel_array[2].output_power,
	temperature_error_state
	);
	if (rs485_ready_to_send())
		rs485_transmit_byte_array((uint8_t *)debug_info, strlen(debug_info));
}

void init_modbus_registers(channel_t * channel_array)
{
	modbus_registers[0].active = true;
	modbus_registers[1].active = true;
	modbus_registers[2].active = true;
	modbus_registers[3].active = true;
	modbus_registers[4].active = true;
	modbus_registers[5].active = true;
	modbus_registers[5].active = true;
	modbus_registers[6].active = false;
	modbus_registers[7].active = false;
	modbus_registers[8].active = false;
	modbus_registers[9].active = true;
	modbus_registers[10].active = true;
	modbus_registers[11].active = true;
	modbus_registers[12].active = true;
	modbus_registers[13].active = true;
	modbus_registers[14].active = true;
	modbus_registers[15].active = true;
	modbus_registers[16].active = true;
	modbus_registers[17].active = true;
	modbus_registers[18].active = true;
	update_modbus_registers(channel_array);
}


void update_modbus_registers(channel_t * channel_array)
{
	modbus_registers[0].value = channel_array[0].output_power;
	modbus_registers[1].value = channel_array[1].output_power;
	modbus_registers[2].value = channel_array[2].output_power;
	modbus_registers[3].value = channel_array[0].setpoint;
	modbus_registers[4].value = channel_array[1].setpoint;
	modbus_registers[5].value = channel_array[2].setpoint;
	modbus_registers[9].value = sensor_values.temperatures[0];
	modbus_registers[10].value = sensor_values.temperatures[1];
	modbus_registers[11].value = sensor_values.temperatures[2];
	modbus_registers[12].value = sensor_values.temperatures[3];
	modbus_registers[13].value = sensor_values.temperatures[4];
	modbus_registers[14].value = sensor_values.temperatures[5];
	modbus_registers[15].value = channel_array[0].output_power;
	modbus_registers[16].value = channel_array[1].output_power;
	modbus_registers[17].value = channel_array[2].output_power;
	modbus_registers[18].value = temperature_error_state;
}

void update_app_data(channel_t * channel_array)
{
	if (modbus_registers[0].value > 100)
		channel_array[0].work_state = WORK_STATE_AUTO;
	else
	{
		channel_array[0].work_state = WORK_STATE_MANUAL;
		channel_array[0].output_power = modbus_registers[0].value;
	}
		
	if (modbus_registers[1].value > 100)
		channel_array[1].work_state = WORK_STATE_AUTO;
	else
	{
		channel_array[1].work_state = WORK_STATE_MANUAL;
		channel_array[1].output_power = modbus_registers[1].value;			
	}
	
	if (modbus_registers[0].value > 100)
		channel_array[0].work_state = WORK_STATE_AUTO;
	else
	{
		channel_array[0].work_state = WORK_STATE_MANUAL;
		channel_array[0].output_power = modbus_registers[0].value;
	}
		
	channel_array[0].setpoint = modbus_registers[3].value;

	channel_array[1].setpoint = modbus_registers[4].value;

	channel_array[2].setpoint = modbus_registers[5].value;
}


int main (void)
{
	static channel_t channel_array[FAN_NUMBER] = {
		{FAN1_DRIVE_PIN, 0, WORK_STATE_AUTO, INIT_TARGET_TEMPERATURE, 0, 0, GATE_IDLE},
		{FAN2_DRIVE_PIN, 1, WORK_STATE_AUTO, INIT_TARGET_TEMPERATURE, 0, 0, GATE_IDLE},
		{FAN3_DRIVE_PIN, 2, WORK_STATE_AUTO, INIT_TARGET_TEMPERATURE, 0, 0, GATE_IDLE}
	};
	
	gpio_init();
	adc_init();
	interrupt_init();
	rs485_init();
	led_blink(3, 50);
	timer_start(GATE_DRIVING_TIMER_RESOLUTION_US);
	update_working_parameters(channel_array, FAN_NUMBER);
	modbus_init(modbus_registers);
	init_modbus_registers(channel_array);
	
	while(1)
	{
		drive_fans(channel_array, FAN_NUMBER);		
		
		if(pi_pulse_delay_counter_us >= WORKING_PARAMETERS_UPDATE_PERIOD_US)
		{
			update_working_parameters(channel_array, FAN_NUMBER);
			pi_pulse_delay_counter_us = 0;
			#ifdef SEND_DEBUG_INFO_OVER_RS
				send_debug_info(channel_array);
			#endif
		}
		
		if (modbus_request_pending_flag == true)
		{
			update_modbus_registers(channel_array); // TBD - not needed on control
			if (modbus_frame_byte_counter > 0)
				led_blink(2, 100);
			int8_t request_type = modbus_process_frame(incoming_modbus_frame, modbus_frame_byte_counter);
			
			modbus_request_pending_flag = false;
			
			if (request_type == REQUEST_TYPE_WRITE)
			{
				update_app_data(channel_array);
			}
		}
	}
}

uint8_t power_percent_to_voltage(int16_t power)
{
	return power * MAX_FAN_VOLTAGE /  100;
}

uint8_t voltage_to_power_percent(uint8_t voltage)
{
	return 100 * voltage / MAX_FAN_VOLTAGE;
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
	pi_pulse_delay_counter_us += GATE_DRIVING_TIMER_RESOLUTION_US;
	rx_time_interval_counter += GATE_DRIVING_TIMER_RESOLUTION_US;
	
	if ( (rx_time_interval_counter > TIME_BETWEEN_MODBUS_FRAMES_US) && (!rs485_rx_buffer_empty()) )
	{
		rs485_get_frame(incoming_modbus_frame, RS_RX_BUFFER_SIZE);
		modbus_request_pending_flag = true;
		modbus_frame_byte_counter = 0;
	}
}

/* ISR for UART TX complete interrupt */ 
ISR(USART0_TX_vect) 
{
	rs485_transmit_from_buffer();
}

/* ISR for UART RX interrupt */ 
ISR(USART0_RX_vect)
{
	if ((UCSR1A & (UDRE0 | FE0 | DOR0))==0) // data register, frame error, overrun check
	{
		if (rs485_get_byte_to_buffer())
		{
			modbus_frame_byte_counter++;
		}
		else
		{
			led_blink(1, 50);
		}
	}
	else
	{
		led_blink(2, 50);
	}
	
	rx_time_interval_counter = 0;
}
