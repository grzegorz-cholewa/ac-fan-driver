/*
 * rs485.h
 *
 * Created: 24/10/2019 17:30:51
 *  Author: LENOVO
 */ 


#ifndef RS485_H_
#define RS485_H_
#define UART_TX_BUFFER_SIZE 200

void rs485_init(void);
void rs485_transmit_byte(uint8_t data);
void rs485_transmit_string(char * string);
void rs485_transmit_byte_array(uint8_t * byte_array, uint16_t array_size);
void rs485_transmitter_enable(void);
void rs485_transmitter_disable(void);
void rs485_transmit_from_buffer(void);

#endif /* RS485_H_ */