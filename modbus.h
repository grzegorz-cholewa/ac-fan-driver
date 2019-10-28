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
#define CTRL_CH1_POWER 40001
#define CTRL_CH2_POWER 40002
#define CTRL_CH3_POWER 40003
#define CTRL_NTC1_SETPOINT 40004
#define CTRL_NTC2_SETPOINT 40005
#define CTRL_NTC3_SETPOINT 40006
#define INFO_REGISTERS_OFFSET 9
#define INFO_REGISTERS_NUMBER 10
#define INFO_NTC1_TEMPERATURE 40010
#define INFO_NTC2_TEMPERATURE 40011
#define INFO_NTC3_TEMPERATURE 40012
#define INFO_NTC4_TEMPERATURE 40013
#define INFO_NTC5_TEMPERATURE 40014
#define INFO_NTC6_TEMPERATURE 40015
#define INFO_CH1_VOLTAGE 40016
#define INFO_CH2_VOLTAGE 40017
#define INFO_CH3_VOLTAGE 40018
#define INFO_ERROR_STATUS 40019

uint8_t get_first_byte(uint16_t two_byte);

uint8_t get_second_byte(uint16_t two_byte);

void modbus_send_control_response(void);

void modbus_send_info_response(int16_t * info_registers, uint8_t registers_number);

void modbus_get_info_registers(int16_t * data, uint16_t data_length);

void modbus_process_frame(uint8_t * frame, uint16_t frame_size);

#endif /* MODBUS_H_ */