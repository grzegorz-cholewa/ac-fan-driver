#include <rs485.h>

#ifndef MODBUS_H_
#define MODBUS_H_

#define TEMPERATURE_STATUS_OK 0x000
#define TEMPERATURE_STATUS_ERROR 0x001

#define DEVICE_ID 0x01
#define FUNC_READ 0x03
#define FUNC_WRITE 0x06

/* DATA ADDRESSES */
#define REGISTERS_RANGE 19
#define REQUEST_TYPE_READ 0
#define REQUEST_TYPE_WRITE 1

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

void modbus_init(struct register_t * modbus_registers_pointer);

bool are_registers_valid(struct register_t * first_register, uint8_t registers_number);

#endif /* MODBUS_H_ */