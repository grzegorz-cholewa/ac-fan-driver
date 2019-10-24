#include <asf.h>
#include <string.h>
#include <config.h>
#include <rs485.h>

uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE];

void rs485_init(void)
{
	UBRR0H = (unsigned char)(MYUBRR>>8); // set baud rate
	UBRR0L = (unsigned char)MYUBRR; // set baud rate
	UCSR0B = (1<<RXEN0)|(1<<TXEN0); // enable receiver and transmitter
	UCSR0C = (1<<USBS0)|(3<<UCSZ00); // set frame format: 8data, 2stop bit
	
	UCSR0B |= USART_TXC_bm; // enable TX complete interrupt
	
	ioport_configure_pin(RS_DRIVER_ENABLE_PIN, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
}

void rs485_transmit_byte(uint8_t data)
{
	while (!(UCSR0A & (1<<UDRE0))) // wait for empty transmit buffer
	;

	UDR0 = data; // put data to send buffer
}

void rs485_transmit_string(char * string_to_send)
{
	memcpy(uart_tx_buffer, string_to_send, sizeof(uart_tx_buffer));
	rs485_transmitter_enable();
	rs485_transmit_byte('\n'); // first char is needed to make TX interrupt finished work
}

void rs485_transmit_byte_array(uint8_t * byte_array, uint16_t array_size)
{
	memcpy(uart_tx_buffer, byte_array, array_size);
	rs485_transmitter_enable();
	rs485_transmit_byte('\n'); // first char is needed to make TX interrupt finished work
}

void rs485_transmitter_enable(void)
{
	gpio_set_pin_high(RS_DRIVER_ENABLE_PIN);
}

void rs485_transmitter_disable(void)
{
	gpio_set_pin_low(RS_DRIVER_ENABLE_PIN);
}

void rs485_transmit_from_buffer(void)
{
	static uint16_t buffer_index = 0;
	if (uart_tx_buffer[buffer_index] != 0x00)
	{
		rs485_transmit_byte(uart_tx_buffer[buffer_index]);
		buffer_index++;
	}
	else
	{
		buffer_index = 0;
		rs485_transmitter_disable();
	}
}