#include <rs485.h>

#ifndef MODBUS_H_
#define MODBUS_H_

#define DEVICE_ID 0x01
#define FUNC_READ 0x03
#define FUNC_WRITE 0x06

/* DATA ADDRESSES */
#define REGISTERS_NUMBER 16
#define MAX_REGISTERS_OFFSET (REGISTERS_NUMBER-1)
#define REQUEST_TYPE_READ 0
#define REQUEST_TYPE_WRITE 1

struct register_t
{
	bool active;
	int16_t value;
};

void modbus_init(struct register_t * modbus_registers_pointer);

int8_t modbus_process_frame(uint8_t * frame, uint16_t frame_size);

uint8_t get_high_byte(uint16_t two_byte);

uint8_t get_low_byte(uint16_t two_byte);

uint16_t get_short(uint8_t * first_byte_pointer);

bool are_registers_valid(struct register_t * first_register, uint8_t registers_number);

void send_info_response(struct register_t * first_register, uint8_t registers_number);

void get_info_registers(struct register_t  * data, uint16_t data_length);


#endif /* MODBUS_H_ */