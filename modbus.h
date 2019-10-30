#include <rs485.h>

#ifndef MODBUS_H_
#define MODBUS_H_

#define TEMPERATURE_STATUS_OK 0x000
#define TEMPERATURE_STATUS_ERROR 0x001

#define DEVICE_ID 0x01
#define FUNC_READ 0x03
#define FUNC_WRITE 0x06

/* DATA ADDRESSES */
#define ADDR_OFFSET 40001
//#define CTRL_REGISTERS_NUMBER 6
#define REGISTERS_RANGE 19
#define CTRL_CH1_POWER_OFFSET 0
#define CTRL_CH2_POWER_OFFSET 1
#define CTRL_CH3_POWER_OFFSET 2
#define CTRL_NTC1_SETPOINT_OFFSET 3
#define CTRL_NTC2_SETPOINT_OFFSET 4
#define CTRL_NTC3_SETPOINT_OFFSET 5
#define INFO_REGISTERS_OFFSET 9
#define INFO_REGISTERS_NUMBER 10
#define INFO_NTC1_TEMPERATURE_OFFSET 9
#define INFO_NTC2_TEMPERATURE_OFFSET 10
#define INFO_NTC3_TEMPERATURE_OFFSET 11
#define INFO_NTC4_TEMPERATURE_OFFSET 12
#define INFO_NTC5_TEMPERATURE_OFFSET 13
#define INFO_NTC6_TEMPERATURE_OFFSET 14
#define INFO_CH1_VOLTAGE_OFFSET 15
#define INFO_CH2_VOLTAGE_OFFSET 16
#define INFO_CH3_VOLTAGE_OFFSET 17
#define INFO_ERROR_STATUS 18

#define REQUEST_TYPE_READ 0
#define REQUEST_TYPE_WRITE 1

typedef struct 
{
	uint8_t register_position;
	int16_t value_to_set;	
}control_params;

struct register_t
{
	bool active;
	int16_t value;
};

uint8_t get_high_byte(uint16_t two_byte);

uint8_t get_low_byte(uint16_t two_byte);

uint16_t get_short(uint8_t * first_byte_pointer);

void modbus_send_info_response(struct register_t * first_register, uint8_t registers_number);

void modbus_get_info_registers(struct register_t  * data, uint16_t data_length);

int8_t modbus_process_frame(uint8_t * frame, uint16_t frame_size);

control_params modbus_get_control_params(void);

void modbus_init(struct register_t * modbus_registers_pointer);

#endif /* MODBUS_H_ */