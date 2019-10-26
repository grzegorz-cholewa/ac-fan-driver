#include <asf.h>
#include <string.h>
#include <config.h>
#include <rs485.h>

uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE];
uint8_t * tx_buffer_pointer = uart_tx_buffer;

uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
uint8_t * rx_buffer_pointer = uart_rx_buffer;

uint16_t bytes_to_transmit = 0;

void rs485_init(void)
{
	UBRR0H = (unsigned char)(MYUBRR>>8); // set baud rate
	UBRR0L = (unsigned char)MYUBRR; // set baud rate
	
	//UCSR0B = (1<<RXEN0)|(1<<TXEN0); // enable receiver and transmitter
	usart_tx_enable(&USART0);
	usart_rx_enable(&USART0);
	
	UCSR0C = (1<<USBS0)|(3<<UCSZ00); // set frame format: 8data, 2stop bit
	
	usart_rx_complete_interrupt_enable(&USART0); // enable RX interrupt
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
	rs485_transmitter_enable();
	memcpy(uart_tx_buffer, byte_array, array_size);
	bytes_to_transmit = array_size;
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
	if (bytes_to_transmit > (tx_buffer_pointer-uart_tx_buffer))
	{
		rs485_transmit_byte(*tx_buffer_pointer);
		tx_buffer_pointer++;
	}
	else
	{
		delay_us(150);
		rs485_transmitter_disable();
		bytes_to_transmit = 0;
		tx_buffer_pointer = uart_tx_buffer;
	}
}

bool rs485_rx_buffer_full()
{
	if (rx_buffer_pointer <= uart_rx_buffer + UART_RX_BUFFER_SIZE)
		return false;
	else
		return true;
}

bool rs485_get_byte_to_buffer()
{
	if (!rs485_rx_buffer_full())
	{
		uint8_t received_byte = usart_get(&USART0);
		*rx_buffer_pointer = received_byte;
		rx_buffer_pointer++;
		return true;
	}
	else
		return false;
}