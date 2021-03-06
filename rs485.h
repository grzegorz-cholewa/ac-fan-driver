
#ifndef RS485_H_
#define RS485_H_

void rs485_init(void);
void rs485_transmit_byte_array(uint8_t * byte_array, uint16_t array_size);
void rs485_transmit_string(char * string);
void rs485_transmit_from_buffer(void);
bool rs485_get_byte_to_buffer(void);
void rs485_get_frame(uint8_t * dest_array, uint8_t array_size);
bool rs485_ready_to_send(void);
bool rs485_rx_buffer_empty(void);

#endif /* RS485_H_ */