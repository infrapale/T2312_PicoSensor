#include "main.h"

//Adafruit_PCT2075 PCT2075;
Adafruit_BME680 bme; // I2C

typedef struct 
{
    bool bme_is_ok;
    uint16_t state;
    uint32_t time_to_next;
    uint32_t sec_cntr;
} measure_ctrl_st;

measure_ctrl_st mctrl =
{
    .bme_is_ok = false,
    .state = 0,
    .time_to_next = 0,
    .sec_cntr = 0,

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

/// @note  call approximately 1/sec
void measure_state_machine(void)
{
  mctrl.sec_cntr++;
  if (mctrl.sec_cntr > mctrl.time_to_next)
  {
    switch(mctrl.state)
    {
      case 0:
        digitalWrite(PIN_I2C_PWR_EN,LOW);
        delay(500);
        digitalWrite(PIN_I2C_PWR_EN,HIGH);
        mctrl.time_to_next += 5;
        break;
      case 1:
        if (measure_initialize())
        {
          mctrl.state++;
          mctrl.time_to_next = mctrl.sec_cntr;

        }
        break;
    }

  }
}

float measure_get_bme_temperature(void)
{
    if (mctrl.bme_is_ok) return bme.temperature;
    else return 0.0;
}

float measure_get_bme_humidity(void)
{
    if (mctrl.bme_is_ok) return bme.humidity;
    else return 0.0;
}



