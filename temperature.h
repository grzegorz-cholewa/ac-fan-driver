#ifndef TEMPERATURE_H_
#define TEMPERATURE_H_

#include <compiler.h>

#define ADC_SENSOR_NUMBER 6
#define ADC_READ_PERIOD_MS 1000

/* TYPE DEFINITIONS */
typedef struct
{
	uint16_t adc_values[ADC_SENSOR_NUMBER];
	int temperatures[ADC_SENSOR_NUMBER];
} sensors_t;


/* 10-bit ADC init */
void adc_init(void);

/* single ADC read from selected channels, returns 10-bit value */
uint16_t adc_value_read(uint8_t adc_channel);

/* reads temperature from all sensors */
void read_temperatures(sensors_t * sensor_values);

/* converts 10bit ADC read value to temperature in Celsius degree */
int adc_to_temperature(uint16_t adc_value);

#endif /* TEMPERATURE_H_ */