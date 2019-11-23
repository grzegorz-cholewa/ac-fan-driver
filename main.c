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

#define WORK_STATE_MANUAL 0
#define WORK_STATE_AUTO 1

#define TEMPERATURE_STATUS_NO_ERROR 0
#define TEMPERATURE_STATUS_ERROR 1

typedef enum 
{
	GATE_IDLE,
	GATE_ACTIVE,
} gate_state_t;

typedef struct 
{
	port_pin_t gate_pin;
	const uint8_t temp_sensor_index;
	uint8_t work_state;
	int16_t setpoint;
	int16_t output_voltage_percent;
	uint32_t activation_delay_us; // time from zero-crossing to gate activation 
	gate_state_t state;
} channel_t;

/* GLOBAL VARIABLES */
uint32_t gate_pulse_delay_counter_us = 0;
uint32_t update_parameter_timer_counter_us = 0;
uint32_t rx_time_interval_counter = 0;
sensors_t sensor_values;
bool modbus_request_pending_flag = false;
int16_t temperature_error_state = TEMPERATURE_STATUS_NO_ERROR;
uint8_t incoming_modbus_frame[RS_RX_BUFFER_SIZE];
uint16_t modbus_frame_byte_counter = 0;
static struct register_t modbus_registers[REGISTERS_NUMBER];
static channel_t channel_array[OUTPUT_CHANNELS_NUMBER] = {
	{FAN1_DRIVE_PIN, 0, WORK_STATE_AUTO, INIT_CHANNEL_SETPOINT_C*10, 0, 0, GATE_IDLE},
	{FAN2_DRIVE_PIN, 1, WORK_STATE_AUTO, INIT_CHANNEL_SETPOINT_C*10, 0, 0, GATE_IDLE},
	{FAN3_DRIVE_PIN, 2, WORK_STATE_AUTO, INIT_CHANNEL_SETPOINT_C*10, 0, 0, GATE_IDLE}
};

/* FUNCTION PROTOTYPES */
int16_t check_temperatures(sensors_t * sensor_array);
void drive_fans(void);
uint32_t get_gate_delay_us(uint16_t output_power); //uint16_t zamiast uint8_t
void gpio_init(void);
void interrupt_init(void);
void led_blink(uint8_t count, uint32_t on_off_cycle_period_ms);
int16_t pi_regulator(uint8_t channel, int16_t current_temp, int16_t target_temperature, uint16_t debug_adc_read);
void set_gate_state(channel_t * fan, gate_state_t pulse_state);
void timer_start(uint32_t time_us);
void update_working_parameters(void);
void send_debug_info(void);
void init_modbus_registers(void);
void update_modbus_registers(void);
void update_app_data(void);
uint8_t power_percent_to_voltage(int16_t power);

/* FUNCTION DEFINITIONS */
int16_t check_temperatures(sensors_t * sensor_array)
{	
	for (int i = 0; i < ADC_SENSOR_NUMBER; i++)
	{
		int16_t temperature = sensor_array->temperatures[i];
		if ( (temperature > MAX_WORKING_TEMPERATURE) || (temperature < MIN_WORKING_TEMPERATURE) )
		{
			gpio_set_pin_high(ERROR_OUT_PIN);
			gpio_set_pin_high(LED_PIN);
			return TEMPERATURE_STATUS_ERROR;	
		}
	}
	gpio_set_pin_low(LED_PIN);
	return TEMPERATURE_STATUS_NO_ERROR;
}

void drive_fans(void)
{
	for (uint8_t i = 0; i < OUTPUT_CHANNELS_NUMBER; i++)
	{
		if (channel_array[i].output_voltage_percent <= MIN_OUTPUT_VOLTAGE_DAGPERCENT)
		{
			set_gate_state(&channel_array[i], GATE_IDLE);
		}
				
		else if (channel_array[i].output_voltage_percent >= MAX_OUTPUT_VOLTAGE_DAGPERCENT)
		{
			set_gate_state(&channel_array[i], GATE_ACTIVE);
		}
				
		else if ( (gate_pulse_delay_counter_us >= channel_array[i].activation_delay_us) && (gate_pulse_delay_counter_us <= (channel_array[i].activation_delay_us + GATE_PULSE_MIN_TIME_US)) )
		{
			set_gate_state(&channel_array[i], GATE_ACTIVE);
		}
		else
		{
			set_gate_state(&channel_array[i], GATE_IDLE);
		}
	}
}

uint32_t get_gate_delay_us(uint16_t output_voltage_percent)
{
	uint16_t mean_voltage = (output_voltage_percent*23)/100; //output_voltage_percent * MAX_FAN_VOLTAGE / 1000; //power_percent_to_voltage(output_power);
	double activation_angle_rad = acos(mean_voltage/230.0); // acos function input is double, value from -1 to 1
	uint32_t gate_delay = HALF_SINE_PERIOD_US*activation_angle_rad/(PI/2.0);
	
	if (gate_delay > MAX_GATE_DELAY_US)
		gate_delay = MAX_GATE_DELAY_US;
		
	if (gate_delay < MIN_GATE_DELAY_US)
		gate_delay = MIN_GATE_DELAY_US;

	return gate_delay - ZERO_CROSSING_DETECTION_OFFSET_US;
}

void gpio_init(void)
{
	ioport_configure_pin(LED_PIN, IOPORT_DIR_OUTPUT |  IOPORT_INIT_LOW);
	ioport_configure_pin(GPIO_PUSH_BUTTON_0, IOPORT_DIR_INPUT | IOPORT_PULL_UP);
	ioport_configure_pin(ZERO_CROSSING_PIN, IOPORT_DIR_INPUT | IOPORT_PULL_UP);
	ioport_configure_pin(FAN1_DRIVE_PIN, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	ioport_configure_pin(FAN2_DRIVE_PIN, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	ioport_configure_pin(FAN3_DRIVE_PIN, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	ioport_configure_pin(ERROR_OUT_PIN, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
}

void interrupt_init(void)
{
	EICRA |= (1 << ISC00 ) | (1 << ISC01); // set INT0 to trigger on rising edge
	EIMSK |= (1 << INT0); // activate INT0
	sei(); // activate interrupts
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

/*
int16_t pi_regulator(uint8_t channel, int16_t current_temp, int16_t setpoint, uint16_t debug_adc_read)
{
	int16_t error;
	static int16_t integral_error[3] = {0, 0, 0};
	int16_t output_voltage_percent;
	
	error = setpoint - current_temp;
	
	integral_error[channel] = integral_error[channel] + error;
	
	output_voltage_percent = (-1) * (PI_KP * error  + integral_error[channel]/TIME_CONST);
	
	if (output_voltage_percent > MAX_OUTPUT_VOLTAGE_PERCENT)
		output_voltage_percent = FULL_ON_OUTPUT_VOLTAGE_PERCENT;
	
	if (output_voltage_percent < MIN_OUTPUT_VOLTAGE_PERCENT)
		output_voltage_percent = FULL_OFF_OUTPUT_VOLTAGE_PERCENT;
	
	#ifdef MOCK_OUTPUT_VOLTAGE_REGULATION // FOR DEBUG ONLY
		output_voltage_percent = debug_adc_read/8;
	#endif
	
	return output_voltage_percent;
};*/

int16_t pi_regulator(uint8_t channel, int16_t current_temp, int16_t setpoint, uint16_t debug_adc_read)
{
	int16_t error;
	static int16_t integral_error[3] = {0, 0, 0};
	int16_t output_voltage_percent; //TODO tysieczne zamiast setnych (procent)
	
	error = current_temp - setpoint;
		
	integral_error[channel] = integral_error[channel] + error;
	
	if ((error>0) && (integral_error[channel] <350*TIME_CONST))
	integral_error[channel] = 350*TIME_CONST;
	
	if (integral_error[channel] > 1000*TIME_CONST)
	integral_error[channel] = 1000*TIME_CONST;
	if (integral_error[channel] < 0)
	integral_error[channel] = 0;

	output_voltage_percent = PI_KP * error  + integral_error[channel]/TIME_CONST;
	
	if (output_voltage_percent > MAX_OUTPUT_VOLTAGE_DAGPERCENT)
	output_voltage_percent = FULL_ON_OUTPUT_VOLTAGE_PERCENT;
	
	if (output_voltage_percent < MIN_OUTPUT_VOLTAGE_DAGPERCENT)
	output_voltage_percent = FULL_OFF_OUTPUT_VOLTAGE_PERCENT;
	
	//if (channel ==0)
	//{
	//	modbus_registers[10].value = integral_error[channel]/TIME_CONST/10;
	//}
	
	/*
	#ifdef MOCK_OUTPUT_VOLTAGE_REGULATION // FOR DEBUG ONLY
	output_voltage_percent = debug_adc_read/8;
	#endif
	*/
	
	return output_voltage_percent;
};


void set_gate_state(channel_t * fan, gate_state_t pulse_state)
{
	if (fan->state == pulse_state)
	{
		return; // no state change
	}
	
	fan->state = pulse_state;
	
	if (pulse_state == GATE_ACTIVE)
	{
		gpio_set_pin_low(fan->gate_pin); // optotransistor is active low
	}
	
	if (pulse_state != GATE_ACTIVE)
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

void update_working_parameters(void)
{
	ntc_read_temperatures(&sensor_values);
	temperature_error_state = check_temperatures(&sensor_values);
	
	for (uint8_t i = 0; i < OUTPUT_CHANNELS_NUMBER; i++)
	{
		if (channel_array[i].work_state == WORK_STATE_AUTO)
		{
			channel_array[i].output_voltage_percent = pi_regulator(i, sensor_values.temperatures[i],
											channel_array[i].setpoint,
											sensor_values.adc_values[i]);			
		}
		channel_array[i].activation_delay_us = get_gate_delay_us(channel_array[i].output_voltage_percent);
	}
}

/*
void send_debug_info(void)
{
	char debug_info[100];
	snprintf(debug_info, sizeof(debug_info),
	"ADC %d %d %d %d %d %d\nT %d %d %d %d %d %d\nV %d %d %d\nER %d\n",
	sensor_values.adc_values[0], sensor_values.adc_values[1], sensor_values.adc_values[2], sensor_values.adc_values[3], sensor_values.adc_values[4], sensor_values.adc_values[5],
	sensor_values.temperatures[0], sensor_values.temperatures[1], sensor_values.temperatures[2], sensor_values.temperatures[3], sensor_values.temperatures[4], sensor_values.temperatures[5],
	channel_array[0].output_voltage_percent, channel_array[1].output_voltage_percent, channel_array[2].output_voltage_percent,
	temperature_error_state
	);
	if (rs485_ready_to_send())
		rs485_transmit_byte_array((uint8_t *)debug_info, strlen(debug_info));
}
*/

void init_modbus_registers(void)
{
	for (int i = 0; i < REGISTERS_NUMBER; i++)
	{
		modbus_registers[i].active = true;
	}
}

void update_modbus_registers(void)
{
	modbus_registers[0].value = channel_array[0].work_state;
	modbus_registers[1].value = channel_array[1].work_state;
	modbus_registers[2].value = channel_array[2].work_state;
	modbus_registers[3].value = channel_array[0].output_voltage_percent/10;
	modbus_registers[4].value = channel_array[1].output_voltage_percent/10;
	modbus_registers[5].value = channel_array[2].output_voltage_percent/10;
	modbus_registers[6].value = channel_array[0].setpoint/10;
	modbus_registers[7].value = channel_array[1].setpoint/10;
	modbus_registers[8].value = channel_array[2].setpoint/10;
	modbus_registers[9].value = sensor_values.temperatures[0]/10;
	modbus_registers[10].value = sensor_values.temperatures[1]/10;
	modbus_registers[11].value = sensor_values.temperatures[2]/10;
	modbus_registers[12].value = sensor_values.temperatures[3]/10;
	modbus_registers[13].value = sensor_values.temperatures[4]/10;
	modbus_registers[14].value = sensor_values.temperatures[5]/10;
	modbus_registers[15].value = temperature_error_state;
}

void update_app_data(void)
{
	channel_array[0].work_state = modbus_registers[0].value;
	channel_array[1].work_state = modbus_registers[1].value;
	channel_array[2].work_state = modbus_registers[2].value;
	
	if ((modbus_registers[3].value) != channel_array[0].output_voltage_percent/10) // if output value changed
	{
		channel_array[0].work_state = WORK_STATE_MANUAL;
		channel_array[0].output_voltage_percent = modbus_registers[3].value*10;
	}
	
	if ((modbus_registers[4].value) != channel_array[1].output_voltage_percent/10) // if output value changed
	{
		channel_array[1].work_state = WORK_STATE_MANUAL;
		channel_array[1].output_voltage_percent = modbus_registers[4].value*10;
	}
		
	if ((modbus_registers[5].value) != channel_array[2].output_voltage_percent/10) // if output value changed
	{
		channel_array[2].work_state = WORK_STATE_MANUAL;
		channel_array[2].output_voltage_percent = modbus_registers[5].value*10;
	}
	
	channel_array[0].setpoint = modbus_registers[6].value*10;
	channel_array[1].setpoint = modbus_registers[7].value*10;
	channel_array[2].setpoint = modbus_registers[8].value*10;
}

/*uint8_t power_percent_to_voltage(int16_t power)
{
	return power * MAX_FAN_VOLTAGE /  100;
}*/



int main (void)
{
	gpio_init();
	ntc_adc_init();
	interrupt_init();
	rs485_init();
	led_blink(3, 50);
	timer_start(MAIN_TIMER_RESOLUTION_US);
	update_working_parameters();
	modbus_init(modbus_registers);
	init_modbus_registers();
	
	while(1)
	{
		if(update_parameter_timer_counter_us >= WORKING_PARAMETERS_UPDATE_PERIOD_US)
		{
			update_working_parameters();
			update_parameter_timer_counter_us = 0;
			#ifdef SEND_DEBUG_INFO_OVER_RS
				send_debug_info();
			#endif
		}
		
		if (modbus_request_pending_flag == true)
		{
			update_modbus_registers(); 
			int8_t request_type = modbus_process_frame(incoming_modbus_frame, modbus_frame_byte_counter);
			
			if (request_type == REQUEST_TYPE_WRITE)
			{
				update_app_data();
			}
			
			modbus_request_pending_flag = false;
			modbus_frame_byte_counter = 0;
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
	drive_fans();
	
	gate_pulse_delay_counter_us += MAIN_TIMER_RESOLUTION_US;
	update_parameter_timer_counter_us += MAIN_TIMER_RESOLUTION_US;
	rx_time_interval_counter += MAIN_TIMER_RESOLUTION_US;
	
	if ( (rx_time_interval_counter > MAX_TIME_BETWEEN_MODBUS_FRAMES_US) && (!rs485_rx_buffer_empty()) )
	{
		rs485_get_frame(incoming_modbus_frame, RS_RX_BUFFER_SIZE);
		modbus_request_pending_flag = true;
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
	if ( (rx_time_interval_counter > MAX_TIME_BETWEEN_MODBUS_FRAMES_US) && (!rs485_rx_buffer_empty()) )
	{
		rs485_get_frame(incoming_modbus_frame, RS_RX_BUFFER_SIZE);
		modbus_request_pending_flag = true;
		return;
	}
	rx_time_interval_counter = 0;
	
	if (modbus_request_pending_flag == false)
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
	}	
}
