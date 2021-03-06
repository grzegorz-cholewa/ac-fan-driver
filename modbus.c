#include <asf.h>
#include <string.h>
#include <crc.h>
#include <modbus.h>
#include <rs485.h>

#define MAX_REGISTERS_OFFSET (REGISTERS_NUMBER-1)
#define MODBUS_FUNCTION_READ 0x03
#define MODBUS_FUNCTION_WRITE 0x06
#define FRAME_ERROR_LENGTH -1
#define FRAME_ERROR_CRC -2

/* STATIC FUNCTION DECLARATIONS */
uint8_t get_high_byte(uint16_t two_byte);
uint8_t get_low_byte(uint16_t two_byte);
uint16_t get_short_little_endian(uint8_t * first_byte_pointer);
uint16_t get_short_big_endian(uint8_t * first_byte_pointer);
bool are_registers_valid(struct register_t * first_register, uint8_t registers_number);
void send_info_response(struct register_t * first_register, uint8_t registers_number);
void get_info_registers(struct register_t  * data, uint16_t data_length);
struct register_t * modbus_registers;

/* GLOBAL VARIABLES */
uint8_t control_request_head[] = {DEVICE_ID, MODBUS_FUNCTION_WRITE};
uint8_t info_request_head[] = {DEVICE_ID, MODBUS_FUNCTION_READ};


void modbus_init(struct register_t * modbus_registers_pointer)
{
	modbus_registers = modbus_registers_pointer;
}

uint8_t get_high_byte(uint16_t two_byte) 
{
	return ((two_byte >> 8) & 0xFF); // MSB 
}

uint8_t get_low_byte(uint16_t two_byte) 
{
	
	return (two_byte & 0xFF); // LSB
}

uint16_t get_short_little_endian(uint8_t * first_byte_pointer) // first byte is low byte
{
	return (short) (*(first_byte_pointer+1) << 8 | *(first_byte_pointer));
}

uint16_t get_short_big_endian(uint8_t * first_byte_pointer) // first byte is high byte
{
	return (short) (*first_byte_pointer << 8 | *(first_byte_pointer+1));
}

int8_t modbus_process_frame(uint8_t * frame, uint16_t frame_size)
{
	uint16_t crc_calculated;
	uint16_t crc_received;
	
	if (memcmp(frame, info_request_head, sizeof(control_request_head)) == 0)
	{
		// check CRC
		crc_calculated = crc16_modbus(frame, frame_size-2);
		crc_received = get_short_little_endian(frame+frame_size-2);
		if (crc_calculated != crc_received)
		return FRAME_ERROR_CRC;
		
		uint16_t first_address_offset = get_short_big_endian(frame+2);
		uint16_t registers_number = get_short_big_endian(frame+4);
		
		if (first_address_offset + registers_number-1 > MAX_REGISTERS_OFFSET)
		return -1; // index out of range
		
		if ( are_registers_valid(modbus_registers + first_address_offset, registers_number) )
		{
			send_info_response(modbus_registers + first_address_offset, registers_number);
			return REQUEST_TYPE_READ;
		}
	}
	
	else if ( memcmp(frame, control_request_head, sizeof(control_request_head)) == 0 )
	{
		// check CRC
		crc_calculated = crc16_modbus(frame, frame_size-2);
		crc_received = get_short_little_endian(frame+frame_size-2);
		if (crc_calculated != crc_received)
		return FRAME_ERROR_CRC;
		
		uint16_t register_offset = get_short_big_endian(frame+2);
		int16_t value_to_set = get_short_big_endian(frame+4);
		
		if (register_offset > MAX_REGISTERS_OFFSET)
		return FRAME_ERROR_LENGTH;
		
		if ( are_registers_valid(modbus_registers + register_offset, 1) )
		{
			(modbus_registers + register_offset)->value = value_to_set;
			rs485_transmit_byte_array(frame, frame_size); // send echo as response
			return REQUEST_TYPE_WRITE;
		}
	}
	return -1;
}

void send_info_response(struct register_t * first_register, uint8_t registers_number)
{ 
	uint8_t response_buffer[5 + 2 * REGISTERS_NUMBER]; // max array size is needed when all registers are read back
	uint8_t frame_len = 3 + 2*registers_number + 2;
	
	/* Add constant elements */
	*(response_buffer + 0) = DEVICE_ID;
	*(response_buffer + 1) = MODBUS_FUNCTION_READ;
	*(response_buffer + 2) = 2*registers_number;
	
	/* Add data registers */
	for (int i = 0; i < registers_number; i++)
	{
		*(response_buffer + 3 + 2*i) =  get_high_byte((first_register + i)->value);
		*(response_buffer + 3 + 2*i + 1) =  get_low_byte((first_register + i)->value);
	}

	/* Add CRC */
	uint16_t crc_value = crc16_modbus(response_buffer, frame_len-2);
	*(response_buffer + 3 + 2*registers_number) = get_low_byte(crc_value);
	*(response_buffer + 4 + 2*registers_number) = get_high_byte(crc_value);
	
	rs485_transmit_byte_array(response_buffer, frame_len);
}

void get_info_registers(struct register_t  * data, uint16_t data_length)
{
	memcpy(modbus_registers, data, data_length);
}

bool are_registers_valid(struct register_t * first_register, uint8_t registers_number)
{
	for (int i = 0; i < registers_number; i++)
	{
		if ( (first_register + i)->active == false ) // found register that is not initialized
			return false;
	}
	return true;
}
