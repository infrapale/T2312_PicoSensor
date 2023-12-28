#ifndef __MEASURE_H__
#define __MEASURE_H__

bool measure_initialize(void);

/// @note  call approximately 1/sec
void measure_state_machine(void);

bool measure_read_bme(void);

float measure_get_bme_temperature(void);

float measure_get_bme_humidity(void);


float measure_get_ldr1(void);

float measure_get_ldr2(void);




#endif