#include <asf.h>
#include <rs485.h>
#include <modbus.h>
#include <crc.h>
#include <string.h>

struct register_t * modbus_registers;

uint8_t control_request_head[] = {DEVICE_ID, FUNC_WRITE};
uint8_t info_request_head[] = {DEVICE_ID, FUNC_READ};

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

uint16_t get_short(uint8_t * first_byte_pointer)
{
	return (short) (*first_byte_pointer << 8 | *(first_byte_pointer+1));
}

void send_info_response(struct register_t * first_register, uint8_t registers_number)
{ 
	uint8_t response_buffer[5 + 2 * REGISTERS_NUMBER]; // max array size is needed when all registers are read back
	uint8_t frame_len = 3 + 2*registers_number + 2;
	
	/* Add constant elements */
	*(response_buffer + 0) = DEVICE_ID;
	*(response_buffer + 1) = FUNC_READ;
	*(response_buffer + 2) = registers_number;
	
	/* Add data registers */
	for (int i = 0; i < registers_number; i++)
	{
		*(response_buffer + 3 + 2*i) =  get_high_byte((first_register + i)->value);
		*(response_buffer + 3 + 2*i + 1) =  get_low_byte((first_register + i)->value);
	}

	/* Add CRC */
	uint16_t crc_value = usMBCRC16(response_buffer, frame_len-2);
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

int8_t modbus_process_frame(uint8_t * frame, uint16_t frame_size)
{
	if (memcmp(frame, info_request_head, sizeof(control_request_head)) == 0)
	{
        uint16_t first_address_offset = get_short(frame+2);
        uint16_t registers_number = get_short(frame+4);
		
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
		uint16_t register_offset = get_short(frame+2);
		int16_t value_to_set = get_short(frame+4);
		
		if (register_offset > MAX_REGISTERS_OFFSET)
			return -1;
		
		if ( are_registers_valid(modbus_registers + register_offset, 1) )
		{
			(modbus_registers + register_offset)->value = value_to_set;
			rs485_transmit_byte_array(frame, frame_size); // send echo as response
			return REQUEST_TYPE_WRITE;
		}
	}

	return -1;
}