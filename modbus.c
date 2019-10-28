#include <asf.h>
#include <rs485.h>
#include <modbus.h>
#include <crc.h>
#include <string.h>

int16_t info_registers[INFO_REGISTERS_NUMBER];

uint8_t control_request_head[] = {DEVICE_ID, FUNC_WRITE};
uint8_t info_request_head[] = {DEVICE_ID, FUNC_READ};
	
uint8_t get_first_byte(uint16_t two_byte) 
{
	return (two_byte & 0xFF); // LSB	 
}

uint8_t get_second_byte(uint16_t two_byte) 
{
	return ((two_byte >> 8) & 0xFF); // MSB
}

void modbus_send_control_response(void)
{
	
}

void modbus_send_info_response(int16_t * info_registers, uint8_t registers_number)
{ 
	uint8_t response_buffer[5 + 2 * INFO_REGISTERS_NUMBER];
	uint8_t frame_len = 3 + 2*registers_number + 2;
	
	/* Add constant elements */
	*(response_buffer + 0) = DEVICE_ID;
	*(response_buffer + 1) = FUNC_READ;
	*(response_buffer + 2) = INFO_REGISTERS_NUMBER;
	
	/* Add data registers */
	for (int i = 0; i < registers_number; i++)
	{
		*(response_buffer + 3 + 2*i) =  get_first_byte(* info_registers + i);
		*(response_buffer + 3 + 2*i + 1) =  get_second_byte(* info_registers + i);
	}

	/* Add CRC */
	uint16_t crc_value = usMBCRC16(response_buffer, frame_len-2);
	*(response_buffer + 3 + 2*registers_number) = get_first_byte(crc_value);
	*(response_buffer + 3 + 2*registers_number) = get_second_byte(crc_value);
	
	rs485_transmit_byte_array(response_buffer, frame_len);
}

void modbus_get_info_registers(int16_t * data, uint16_t data_length)
{
	memcpy(info_registers, data, data_length);
}	

void modbus_process_frame(uint8_t * frame, uint16_t frame_size)
{
	if (memcmp(frame, control_request_head, sizeof(control_request_head)) == 0)
	{
		modbus_send_control_response();
	}
	
	else if (memcmp(frame, info_request_head, sizeof(control_request_head)) == 0)
	{
		modbus_send_info_response(info_registers, INFO_REGISTERS_NUMBER);
	}
}