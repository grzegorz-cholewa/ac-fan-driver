
#ifndef CONFIG_H_
#define CONFIG_H_

/* DEBUG SWITCHES */
//#define MOCK_OUTPUT_VOLTAGE_REGULATION 1 // activates proportional regulation instead of PI
//#define SEND_DEBUG_INFO_OVER_RS 1
//#define ON_EVALBOARD 1
#ifdef ON_EVALBOARD
	#define LED_PIN IOPORT_CREATE_PIN(PORTB, 5)
	#define F_CPU 16000000
#else
	#define LED_PIN IOPORT_CREATE_PIN(PORTD, 3)
#endif

/* WORKING PARAMETERS */
#define OUTPUT_CHANNELS_NUMBER 3
#define MAIN_TIMER_RESOLUTION_US 100
#define HALF_SINE_PERIOD_US 10000
#define GATE_PULSE_MIN_TIME_US 100
#define MIN_WORKING_TEMPERATURE 0 // exceeding this value results in error alert
#define MAX_WORKING_TEMPERATURE 90 // exceeding this value results in error alert
#define MAX_FAN_VOLTAGE 230
#define FULL_OFF_OUTPUT_VOLTAGE_PERCENT 0
#define FULL_ON_OUTPUT_VOLTAGE_PERCENT 100
#define ZERO_CROSSING_DETECTION_OFFSET_US 260
#define MIN_GATE_DELAY_US 500
#define MAX_GATE_DELAY_US 9500
#define MIN_OUTPUT_VOLTAGE_DAGPERCENT 350 // if value is less then that, FULL_OFF_OUTPUT_POWER is set instead
#define MAX_OUTPUT_VOLTAGE_DAGPERCENT 1000 // if value is bigger then that, FULL_ON_OUTPUT_VOLTAGE_PERCENT is set

/* PI PARAMETERS */
#define WORKING_PARAMETERS_UPDATE_PERIOD_US 1000000 // also period of triggering PI regulator
#define INIT_CHANNEL_SETPOINT_C 65
#define PI_KP 5
#define TIME_CONST 10

/* RS485 PARAMETERS */
#define RS_BAUD_RATE 9600
#define MYUBRR (F_CPU/16/RS_BAUD_RATE - 1)
#define RS_TX_BUFFER_SIZE 100
#define RS_RX_BUFFER_SIZE 50
#define MAX_TIME_BETWEEN_MODBUS_FRAMES_US 4500 //((1000000/RS_BAUD_RATE)*11*4) //>((1000000/RS_BAUD_RATE)*11*4) //min 3.5 char between messages

/* PIN DEFINITIONS */
#define ZERO_CROSSING_PIN IOPORT_CREATE_PIN(PORTD, 2) // source for INT0 interrupt
#define FAN1_DRIVE_PIN			IOPORT_CREATE_PIN(PORTD, 5) // signal for gate of triac driving fan1
#define FAN2_DRIVE_PIN			IOPORT_CREATE_PIN(PORTD, 6) // signal for gate of triac driving fan2
#define FAN3_DRIVE_PIN			IOPORT_CREATE_PIN(PORTD, 7) // signal for gate of triac driving fan3
#define RS_DRIVER_ENABLE_PIN	IOPORT_CREATE_PIN(PORTB, 0) // driver enable control pin (high is driver enable)
#define ERROR_OUT_PIN			IOPORT_CREATE_PIN(PORTD, 4) // error indication

/* CONSTANTS */
#define PI (3.14)

#endif /* CONFIG_H_ */