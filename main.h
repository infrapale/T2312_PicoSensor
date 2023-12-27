#ifndef __MAIN_H__
#define __MAIN_H__

#define VILLA_ASTRID_TUPA
// #define VILLA_ASTRID_PIHA
// #define VILLA_ASTRID_KHH


#define PIN_SERIAL1_TX  (0u)
#define PIN_SERIAL1_RX  (1u)
#define PIN_I2C_PWR_EN  (15u)
#define PIN_LDR1        (A0)
#define PIN_LDR2        (A1)

#include "Adafruit_BME680.h"



typedef enum
{
   FEED_PUB_TEMPERATURE = 0,
   FEED_PUB_HUMIDITY,
   FEED_PUB_LDR1,
   FEED_PUB_LDR2,
   FEED_PUB_NBR_OF
} feed_pub_et;


#endif