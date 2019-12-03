#include <asf.h>
#include <string.h>
#include <config.h>
#include <rs485.h>

/* STATIC FUNCTION PROTOTYPES */
void transmit_byte(uint8_t data);
void transmitter_enable(void);
void transmitter_disable(void);
bool rx_buffer_full(void);

/* GLOBAL VARIABLES */
uint8_t uart_tx_buffer[RS_TX_BUFFER_SIZE];
uint8_t uart_rx_buffer[RS_RX_BUFFER_SIZE];
uint8_t * tx_buffer_pointer = uart_tx_buffer;
uint8_t * rx_buffer_pointer = uart_rx_buffer;
uint16_t bytes_to_transmit = 0;

/* FUNCTION DEFINITIONS */
void rs485_init(void)
{
	UBRR0H = (unsigned char)(MYUBRR>>8); // set baud rate
	UBRR0L = (unsigned char)MYUBRR; // set baud rate
	
	UCSR0B = (1<<RXEN0)|(1<<TXEN0); // enable receiver and transmitter
	
	UCSR0C = (1<<USBS0)|(3<<UCSZ00); // set frame format: 8data, 2stop bit
	
	usart_rx_complete_interrupt_enable(&USART0); // enable RX interrupt
	UCSR0B |= USART_TXC_bm; // enable TX complete interrupt
	
	ioport_configure_pin(RS_DRIVER_ENABLE_PIN, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
}

void transmit_byte(uint8_t data)
{
	while (!(UCSR0A & (1<<UDRE0))) // wait for empty transmit buffer
	;
	UDR0 = data; // put data to send buffer
}

void rs485_transmit_byte_array(uint8_t * byte_array, uint16_t array_size)
{
	transmitter_enable();
	memcpy(uart_tx_buffer, byte_array, array_size);
	bytes_to_transmit = array_size;
	rs485_transmit_from_buffer();
}

void transmitter_enable(void)
{
	gpio_set_pin_high(RS_DRIVER_ENABLE_PIN);
}

void transmitter_disable(void)
{
	gpio_set_pin_low(RS_DRIVER_ENABLE_PIN);
}

void rs485_transmit_from_buffer(void)
{
	if (bytes_to_transmit > (tx_buffer_pointer-uart_tx_buffer))
	{
		transmit_byte(*tx_buffer_pointer);
		tx_buffer_pointer++;
	}
	else
	{
		transmitter_disable();
		bytes_to_transmit = 0;
		tx_buffer_pointer = uart_tx_buffer;
	}
}

bool rs485_ready_to_send(void)
{
	if (tx_buffer_pointer == uart_tx_buffer)
		return true;
	else
		return false;
}

bool rx_buffer_full(void)
{
	if (rx_buffer_pointer <= uart_rx_buffer + RS_RX_BUFFER_SIZE)
		return false;
	else
		return true;
}

bool rs485_rx_buffer_empty(void)
{
	if (rx_buffer_pointer == uart_rx_buffer)
		return true;
	else
		return false;	
}

bool rs485_get_byte_to_buffer()
{
	if (!rx_buffer_full())
	{
		uint8_t received_byte = usart_get(&USART0);
		*rx_buffer_pointer = received_byte;
		rx_buffer_pointer++;
		return true;
	}
	else
		return false;
}

void rs485_get_frame(uint8_t * dest_array, uint8_t array_size)
{
	memcpy(dest_array, uart_rx_buffer, array_size);
	memset(uart_rx_buffer, 0, RS_TX_BUFFER_SIZE);
	rx_buffer_pointer = uart_rx_buffer;
}