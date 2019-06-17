#ifndef TEMPERATURE_H_
#define TEMPERATURE_H_

#include <compiler.h>

/*
* \brief converts 10bit ADC read value to temperature in Celcius degree
*
*/
int adc_to_temperature(uint16_t adc_value);

#endif /* TEMPERATURE_H_ */