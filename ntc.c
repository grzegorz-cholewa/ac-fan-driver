#include <ntc.h>

/* STATIC FUNCTIONS DECLARATIONS */
/* converts 10bit ADC read value to temperature in Celsius degree */
int16_t adc_to_temperature(uint16_t adc_value);

/* single ADC read from selected channels, returns 10-bit value */
uint16_t adc_value_read(uint8_t adc_channel);


// lookup table for temperature reads; index is ADC value and values are in Celcius Degree
// this table is for: NTC 1k@25C with 200Ohm pull-down, beta = 3730
int NTC_table[513] = {
	629, 526, 423, 374, 343, 321, 304, 290, 279,
	269, 261, 254, 247, 241, 236, 231, 226, 222,
	218, 215, 211, 208, 205, 202, 200, 197, 195,
	192, 190, 188, 186, 184, 182, 180, 179, 177,
	175, 174, 172, 171, 169, 168, 166, 165, 164,
	163, 161, 160, 159, 158, 157, 156, 155, 154,
	152, 152, 151, 150, 149, 148, 147, 146, 145,
	144, 143, 143, 142, 141, 140, 139, 139, 138,
	137, 136, 136, 135, 134, 134, 133, 132, 132,
	131, 130, 130, 129, 128, 128, 127, 127, 126,
	126, 125, 124, 124, 123, 123, 122, 122, 121,
	121, 120, 120, 119, 119, 118, 118, 117, 117,
	116, 116, 115, 115, 114, 114, 113, 113, 112,
	112, 112, 111, 111, 110, 110, 109, 109, 109,
	108, 108, 107, 107, 107, 106, 106, 105, 105,
	105, 104, 104, 103, 103, 103, 102, 102, 102,
	101, 101, 101, 100, 100, 99, 99, 99, 98,
	98, 98, 97, 97, 97, 96, 96, 96, 95, 95, 95,
	94, 94, 94, 93, 93, 93, 92, 92, 92, 92, 91,
	91, 91, 90, 90, 90, 89, 89, 89, 88, 88, 88,
	88, 87, 87, 87, 86, 86, 86, 86, 85, 85, 85,
	84, 84, 84, 84, 83, 83, 83, 82, 82, 82, 82,
	81, 81, 81, 81, 80, 80, 80, 79, 79, 79, 79,
	78, 78, 78, 78, 77, 77, 77, 77, 76, 76, 76,
	76, 75, 75, 75, 75, 74, 74, 74, 73, 73, 73,
	73, 72, 72, 72, 72, 71, 71, 71, 71, 70, 70,
	70, 70, 70, 69, 69, 69, 69, 68, 68, 68, 68,
	67, 67, 67, 67, 66, 66, 66, 66, 65, 65, 65,
	65, 64, 64, 64, 64, 63, 63, 63, 63, 62, 62,
	62, 62, 62, 61, 61, 61, 61, 60, 60, 60, 60,
	59, 59, 59, 59, 58, 58, 58, 58, 58, 57, 57,
	57, 57, 56, 56, 56, 56, 55, 55, 55, 55, 54,
	54, 54, 54, 53, 53, 53, 53, 53, 52, 52, 52,
	52, 51, 51, 51, 51, 50, 50, 50, 50, 49, 49,
	49, 49, 48, 48, 48, 48, 47, 47, 47, 47, 46,
	46, 46, 46, 45, 45, 45, 45, 44, 44, 44, 44,
	43, 43, 43, 43, 42, 42, 42, 42, 41, 41, 41,
	41, 40, 40, 40, 40, 39, 39, 39, 38, 38, 38,
	38, 37, 37, 37, 37, 36, 36, 36, 36, 35, 35,
	35, 34, 34, 34, 34, 33, 33, 33, 32, 32, 32,
	31, 31, 31, 31, 30, 30, 30, 29, 29, 29, 28,
	28, 28, 28, 27, 27, 27, 26, 26, 26, 25, 25,
	25, 24, 24, 24, 23, 23, 22, 22, 22, 21, 21,
	21, 20, 20, 20, 19, 19, 18, 18, 18, 17, 17,
	16, 16, 16, 15, 15, 14, 14, 13, 13, 12, 12,
	12, 11, 11, 10, 10, 9, 9, 8, 8, 7, 6, 6,
	5, 5, 4, 3, 3, 2, 2, 1, 0, 0, -1, -2, -3,
	-3, -4, -5, -6, -7, -8, -9, -10, -11, -12,
	-13, -14, -15, -17, -18, -20, -21, -23, -25,
	-27, -30, -33, -36, -41, -46, -55, -64
};


void ntc_adc_init(void)
{
	ADMUX |= (1<<REFS0); // VREF set to AVCC with external cap at AREF pin
	ADCSRA |= 1<<ADEN; // enable ADC
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // set prescaller to 128
}

uint16_t adc_value_read(uint8_t adc_channel)
{
	ADMUX = (ADMUX & 0xF0) | (adc_channel & 0x0F); // select ADC channel with safety mask
	ADCSRA |= (1<<ADSC); // single conversion mode
	while( ADCSRA & (1<<ADSC) ); // wait until ADC conversion is complete
	return ADC;
}

void ntc_read_temperatures(sensors_t * sensor_values)
{
	for (int i = 0; i < ADC_SENSOR_NUMBER; i++)
	{
		sensor_values->adc_values[i] = adc_value_read(i);
		sensor_values->temperatures[i] = adc_to_temperature(sensor_values->adc_values[i]);
	}
}

int16_t adc_to_temperature(uint16_t adc_value){
	return NTC_table[adc_value/2]; // table consists of 512 elements, while 10-bit ADC have max value 1024
};

