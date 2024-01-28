#include "main.h"
#include "measure.h"

//Adafruit_PCT2075 PCT2075;
Adafruit_BME680 bme; // I2C

typedef struct 
{
    bool bme_is_ok;
    uint32_t bme_last_read;
    uint16_t state;
    uint16_t prev_state;
    uint32_t time_to_next;
    uint32_t sec_cntr;
} measure_ctrl_st;

typedef struct 
{
  float value;
  bool active;
  bool updated;

} measure_st;

measure_ctrl_st mctrl =
{
    .bme_is_ok = false,
    .bme_last_read = 0,
    .state = 0,
    .prev_state = 255,
    .time_to_next = 0,
    .sec_cntr = 0,

};

measure_st measure[MEAS_NBR_OF] = 
{
  [MEAS_TEMPERATURE]  = {0.0, true, false},
  [MEAS_HUMIDITY]     = {0.0, true, false},
  [MEAS_PRESSURE]     = {0.0, false, false},
  #ifdef VILLA_ASTRID_PIHA
  [MEAS_LDR1]         = {0.0, true, false},
  [MEAS_LDR2]         = {0.0, true, false},
  #else
  [MEAS_LDR1]         = {0.0, false, false},
  [MEAS_LDR2]         = {0.0, false, false},
  #endif
};

bool measure_initialize(void)
{
    bme = Adafruit_BME680(&Wire1); 
   

    if (!bme.begin(0x77)) 
    {
        Serial.println("Could not find a valid BME680 sensor, check wiring!");
        mctrl.bme_is_ok = false;
        // while (1);
    }
    else 
    {
        mctrl.bme_is_ok = true;
        Serial.println("BME680 Sensor found");

        // Set up oversampling and filter initialization
        bme.setTemperatureOversampling(BME680_OS_8X);
        bme.setHumidityOversampling(BME680_OS_2X);
        bme.setPressureOversampling(BME680_OS_4X);
        bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
        bme.setGasHeater(320, 150); // 320*C for 150 ms
    }
    return mctrl.bme_is_ok;
}

bool measure_read_bme(void)
{
    if (mctrl.bme_is_ok)
    {
        mctrl.bme_is_ok =  bme.performReading();
    }
    return mctrl.bme_is_ok;
}

void measure_i2c_power(bool pwr_on)
{
  Serial.printf("Power %d\n",pwr_on);
  #ifdef VILLA_ASTRID_TUPA
  if (pwr_on) digitalWrite(PIN_I2C_PWR_EN,HIGH);
  else digitalWrite(PIN_I2C_PWR_EN,LOW);
  #endif
}

/// @note  call approximately 1/sec
void measure_state_machine(void)
{
  mctrl.sec_cntr++;
  if (mctrl.sec_cntr > mctrl.time_to_next)
  {
    if (mctrl.prev_state != mctrl.state)
    {
      Serial.printf("Measure State %d -> %d\n",mctrl.prev_state, mctrl.state);
      mctrl.prev_state = mctrl.state;
    }
    switch(mctrl.state)
    {
      case 0:
        measure_i2c_power(true);
        mctrl.time_to_next += 5;
        mctrl.state++;
        break;
      case 1:
        mctrl.time_to_next = mctrl.sec_cntr + 2;
        if (measure_initialize()) 
        { 
          mctrl.state++;
        }
        else
        {
          measure_i2c_power(false);
          mctrl.state--;
        } 
        break;
      case 2:
        if (measure_read_bme())
        {
          if (measure[MEAS_TEMPERATURE].active)
          {
            measure[MEAS_TEMPERATURE].value = bme.temperature;
            measure[MEAS_TEMPERATURE].updated = true;
          }
          if (measure[MEAS_HUMIDITY].active)
          {
            measure[MEAS_HUMIDITY].value = bme.humidity;
            measure[MEAS_HUMIDITY].updated = true;
          }
          if (measure[MEAS_PRESSURE].active)
          {
            measure[MEAS_PRESSURE].value = bme.pressure;
            measure[MEAS_PRESSURE].updated = true;
          }
          if (measure[MEAS_LDR1].active)
          {
            measure[MEAS_LDR1].value = (float)analogRead(A0);
            measure[MEAS_LDR1].updated = true;
          }
          if (measure[MEAS_LDR2].active)
          {
            measure[MEAS_LDR2].value = (float)analogRead(A1);
            measure[MEAS_LDR2].updated = true;
          }
          mctrl.bme_last_read = mctrl.sec_cntr;
        }
        mctrl.state++;
        break;
      case 3:  
        measure_i2c_power(false);
        mctrl.time_to_next += 30;
        mctrl.state = 0;
        break;
    }

  }
}


bool measure_is_active(measure_et mindx)
{
  return measure[mindx].active;
}

bool measure_is_updated(measure_et mindx)
{
  return measure[mindx].updated;
}

float measure_get_value(measure_et mindx)
{
  return measure[mindx].value;
}
