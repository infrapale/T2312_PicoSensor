/**
 * T2312_Pico_Sensor
 * Generic Adafruit IO sensor using Pi Cow
 * Pico W Adafruit IO MQTT subcribe and publish
 * https://learn.adafruit.com/mqtt-adafruit-io-and-you/intro-to-adafruit-mqtt
 * https://github.com/infrapale/pico_arduino_sdk.git
 *
 */

//#define PIRPANA
#define LILLA_ASTRID
//#define VILLA_ASTRID
#include <stdint.h>
#include "stdio.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "secrets.h"
//#include <Adafruit_PCT2075.h>
// #include <Adafruit_Sensor.h>
#include "main.h"
#include "measure.h"
//#include "Adafruit_BME680.h"


#include "Wire.h"
#include <Adafruit_SleepyDog.h>

// WiFi parameters
#define WLAN_SSID       WIFI_SSID
#define WLAN_PASS       WIFI_PASS

// Adafruit IO
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    IO_USERNAME
#define AIO_KEY         IO_KEY
#define AIO_PUBLISH_INTERVAL_ms  (60000*5)

#define SEALEVELPRESSURE_HPA (1013.25)


typedef struct
{
  uint16_t  mqtt_fault;
  uint16_t  bme_fault;
  uint16_t  ldr_fault;  
} fault_cntr_st;

typedef struct 
{
    uint8_t     publ_indx;
    uint32_t    next_sec;
    uint32_t    next_ping;
    uint32_t    next_measure;
    uint32_t    sec_cntr;
    uint16_t    set_temp;
    bool        heat_on;
    fault_cntr_st   fault_cntr;
} control_st;

typedef struct
{
    Adafruit_MQTT_Publish *pub_feed;
    char      name[20];
    uint32_t  interval;
    uint32_t  next_sec;
    float     value;

} my_pub_st;

WiFiClient client;


// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
// infrapale/feeds/lillaastrid.studio-temp
// infrapale/feeds/lillaastrid.studio-set-tmp
// infrapale/feeds/villaastrid.khh-temperature
// infrapale/feeds/villaastrid.khh-set-temperature
// infrapale/feeds/villaastrid.tupa-temp
// infrapale/feeds/villaastrid.tupa-hum
// infrapale/feeds/villaastrid.ulko-temp
// infrapale/feeds/villaastrid.ulko-hum
// infrapale/feeds/villaastrid.ulko-ldr-1
// infrapale/feeds/villaastrid.ulko-ldr-2

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
#ifdef VILLA_ASTRID_PIHA
Adafruit_MQTT_Publish sensor_temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/villaastrid.ulko-temp");
Adafruit_MQTT_Publish sensor_humidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/villaastrid.ulko-hum");
Adafruit_MQTT_Publish sensor_ldr1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/villaastrid.ulko-ldr-1");
Adafruit_MQTT_Publish sensor_ldr2 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/villaastrid.ulko-ldr-2");
#endif
#ifdef VILLA_ASTRID_TUPA
Adafruit_MQTT_Publish sensor_temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/villaastrid.tupa-temp");
Adafruit_MQTT_Publish sensor_humidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/villaastrid.tupa-hum");
Adafruit_MQTT_Publish sensor_ldr1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/villaastrid.ulko-ldr-1");
Adafruit_MQTT_Publish sensor_ldr2 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/villaastrid.ulko-ldr-2");
#endif

my_pub_st my_pub[FEED_PUB_NBR_OF] =
{
    [FEED_PUB_TEMPERATURE] = {
            .pub_feed = &sensor_temperature,
            .name = "TEMP",
            .interval = 60,
            .next_sec = 0,
            .value = 0.0
          },
    [FEED_PUB_HUMIDITY] = {
            .pub_feed = &sensor_humidity,
            .name = "HUMI",
            .interval = 120,
            .next_sec = 0,
            .value = 0.0
          },
    [FEED_PUB_LDR1] = {
            .pub_feed = &sensor_ldr1,
            .name = "LDR1",
            .interval = 300,
            .next_sec = 0,
            .value = 0.0
          },
    [FEED_PUB_LDR2] = {
            .pub_feed = &sensor_ldr2,
            .name = "LDR2",
            .interval = 300,
            .next_sec = 0,
            .value = 0.0
          },
};

// Adafruit_MQTT_Subscribe set_temperature = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/villaastrid.khh-set-temperature");

control_st ctrl = 
{
    .publ_indx = 0,
    .set_temp = 18,
    .heat_on = false,
    .fault_cntr =
    {
        .mqtt_fault = 0,
        .bme_fault   = 0,
        .ldr_fault   = 0        
    }
};



void setup() 
{
    //Serial.begin(115200);
    Serial1.setTX(8);
    Serial1.setRX(9);
    Serial.begin(9600);
    Serial1.begin(9600);
    delay(4000);

    #ifdef VILLA_ASTRID_TUPA
    my_pub[FEED_PUB_LDR1].interval = 0;
    my_pub[FEED_PUB_LDR2].interval = 0;
    #endif
    #ifdef VILLA_ASTRID_PIHA
    #endif
    #ifdef VILLA_ASTRID_KHH
    my_pub[FEED_PUB_LDR1].interval = 0;
    my_pub[FEED_PUB_LDR2].interval = 0;
    #endif


    //while (!Serial) {
    //  ;  // wait for serial port to connect. Needed for native USB port only
    //}
    Serial.println(F("T2312_Pico_Sensor"));
    Serial.println(__DATE__);Serial.println(__TIME__);
    // Connect to WiFi access point.
    Serial.print(F("Connecting to "));
    Serial.println(WLAN_SSID);
    Wire1.setSCL(7);
    Wire1.setSDA(6);
    Wire1.begin();

    measure_initialize();

    WiFi.begin(WLAN_SSID, WLAN_PASS);
    uint32_t timeout = millis() + 15000;
    while ((WiFi.status() != WL_CONNECTED) && (millis() < timeout))
    {
        delay(500);
        Serial.print(F("."));
        // Watchdog.reset();
    }
    uint32_t count_down_ms = Watchdog.enable(6000);
    Watchdog.reset();

    Serial.println();
    Serial.println(F("WiFi connected"));
    Serial.println(F("IP address: "));
    Serial.println(WiFi.localIP());

    // connect to adafruit io
    //mqtt.subscribe(&set_temperature);
    connect();

    for (uint8_t indx = 0; indx < FEED_PUB_NBR_OF; indx++)
    {
        my_pub[indx].next_sec = ((indx+1)*10);
    }
    ctrl.next_sec   = millis() + 1000;
    ctrl.next_ping  = millis();
    ctrl.next_measure = millis();

    ctrl.sec_cntr   = 0;
    ctrl.publ_indx  = 0;
    #ifdef VILLA_ASTRID_PIHA
    while(0)
    {
        //Serial1.println("<OL1:2345.1>");
        //Serial.println("<OL1:2345.1>");
        int ldr1 = analogRead(A0);
        Serial.print("LDR1 = "); Serial.println(ldr1);
        int ldr2 = analogRead(A1);
        Serial.print("LDR2 = "); Serial.println(ldr2);
        delay(1000);
        Watchdog.reset();
    }
    #endif
    
    
}

// connect to adafruit io via MQTT
void connect() 
{
    Serial.print(F("Connecting to Adafruit IO… "));
    int8_t ret;
    while ((ret = mqtt.connect()) != 0) 
    {
        switch (ret) 
        {
            case 1: Serial.println(F("Wrong protocol")); break;
            case 2: Serial.println(F("ID rejected")); break;
            case 3: Serial.println(F("Server unavail")); break;
            case 4: Serial.println(F("Bad user/pass")); break;
            case 5: Serial.println(F("Not authed")); break;
            case 6: Serial.println(F("Failed to subscribe")); break;
            default: Serial.println(F("Connection failed")); break;
        }

        if(ret >= 0) mqtt.disconnect();
        Serial.println(F("Retrying connection…"));
        Watchdog.reset();
        delay(1000);
    }
    
    Serial.println(F("Adafruit IO Connected!"));
}



void report_publ_status(bool publ_status)
{
    if (publ_status)
    {                     
        Serial.println(F(" - Sent!"));
    }
    else 
    {
        Serial.println(F(" - Failed"));
    }
}

/**
  * Syntax example: <TEMP:-10.1>\n
  *   < = start char
  *   XXXX = ID 4 char
  *   : = separator
  *   signed float value 
  *   > = end char
  *   \n  = newline (makes testing easier)
 */

void send_meas_to_uart(const char *id_4, float value)
{
    char buff[40];
    sprintf(buff, "<#X1T:OD_1;%s;%.2f;->\n",id_4,value);
    Serial.print(buff);
    Serial1.print(buff);
}

void loop() 
{
    bool publ_status;
    char buff[80];

    if (millis() > ctrl.next_measure)
    {
        ctrl.next_measure += 60000;
        if ( measure_read_bme()) 
        {
            my_pub[FEED_PUB_TEMPERATURE].value  = measure_get_bme_temperature();
            my_pub[FEED_PUB_HUMIDITY].value     = measure_get_bme_humidity();
        }
        #ifdef VILLA_ASTRID_PIHA
        int ldr1 = analogRead(A0);
        int ldr2 = analogRead(A1);
        my_pub[FEED_PUB_LDR1].value = (float)ldr1;
        my_pub[FEED_PUB_LDR2].value = (float)ldr2;
        #endif
    }

    if (millis() > ctrl.next_ping)
    {
        ctrl.next_ping += 60000;
        if(! mqtt.ping(3)) 
        {
            if(! mqtt.connected()) connect();
        }
        if(!mqtt.connected())  
        {
            ctrl.fault_cntr.mqtt_fault++;      
        }
        else
        {
            ctrl.fault_cntr.mqtt_fault = 0;    
        }

    }
    
    if (millis() > ctrl.next_sec)
    {
        measure_state_machine();
        Watchdog.reset();
        ctrl.next_sec += 1000;
        ctrl.sec_cntr++;

        if (my_pub[ctrl.publ_indx].interval > 0)
        {
            if( ctrl.sec_cntr >= my_pub[ctrl.publ_indx].next_sec )
            {
                my_pub[ctrl.publ_indx].next_sec +=  my_pub[ctrl.publ_indx].interval;
                send_meas_to_uart(my_pub[ctrl.publ_indx].name, my_pub[ctrl.publ_indx].value);
                publ_status = my_pub[ctrl.publ_indx].pub_feed->publish(my_pub[ctrl.publ_indx].value); //Publish to Adafruit
                report_publ_status(publ_status);
                ctrl.fault_cntr.bme_fault = 0;
            }

        }
        if(++ctrl.publ_indx >= FEED_PUB_NBR_OF) ctrl.publ_indx = 0;

        // ping adafruit io a few times to make sure we remain connected

    }

}
  
