#ifndef TEMPERATURE_H_
#define TEMPERATURE_H_

#include <compiler.h>

#define ADC_SENSOR_NUMBER 6
#define ADC_READ_PERIOD_MS 1000

/* TYPE DEFINITIONS */
typedef struct
{
	uint16_t adc_values[ADC_SENSOR_NUMBER];
	int16_t temperatures[ADC_SENSOR_NUMBER];
} sensors_t;

/* 10-bit ADC init */
void ntc_adc_init(void);

/* reads temperature from all sensors */
void ntc_read_temperatures(sensors_t * sensor_values);

#endif /* TEMPERATURE_H_ */