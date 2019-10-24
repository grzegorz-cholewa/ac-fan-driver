#include <asf.h>
#include <string.h>
#include <config.h>
#include <rs485.h>

uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE];
uint16_t bytes_to_transmit = 0;
bool transmit_pending = false;

void rs485_init(void)
{
	UBRR0H = (unsigned char)(MYUBRR>>8); // set baud rate
	UBRR0L = (unsigned char)MYUBRR; // set baud rate
	UCSR0B = (1<<RXEN0)|(1<<TXEN0); // enable receiver and transmitter
	UCSR0C = (1<<USBS0)|(3<<UCSZ00); // set frame format: 8data, 2stop bit
	
	UCSR0B |= USART_RXC_bm; // enable RX interrupt
	UCSR0B |= USART_TXC_bm; // enable TX complete interrupt
	
	ioport_configure_pin(RS_DRIVER_ENABLE_PIN, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
}

void rs485_transmit_byte(uint8_t data)
{
	while (!(UCSR0A & (1<<UDRE0))) // wait for empty transmit buffer
	;

	UDR0 = data; // put data to send buffer
}

void rs485_transmit_byte_array(uint8_t * byte_array, uint16_t array_size)
{
	memcpy(uart_tx_buffer, byte_array, array_size);
	bytes_to_transmit = array_size;
	rs485_transmitter_enable();
	rs485_transmit_from_buffer();
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
	if (buffer_index < bytes_to_transmit)
	{
		rs485_transmit_byte(uart_tx_buffer[buffer_index]);
		buffer_index++;
	}
	else
	{
		// memset(uart_tx_buffer, 0, UART_TX_BUFFER_SIZE); // probably not needed
		bytes_to_transmit = 0;
		buffer_index = 0;
		rs485_transmitter_disable();
	}
}