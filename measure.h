#ifndef __MEASURE_H__
#define __MEASURE_H__

typedef enum
{
  MEAS_TEMPERATURE = 0,
  MEAS_HUMIDITY,
  MEAS_PRESSURE,
  MEAS_LDR1,
  MEAS_LDR2,
  MEAS_NBR_OF
} measure_et;

bool measure_initialize(void);

/// @note  call approximately 1/sec
void measure_state_machine(void);

//bool measure_read_bme(void);

bool measure_is_active(measure_et mindx);

bool measure_is_updated(measure_et mindx);

float measure_get_value(measure_et mindx);


#endif