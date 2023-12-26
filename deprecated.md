

        if (millis() > next_publ)
        {
            next_publ = millis() + AIO_PUBLISH_INTERVAL_ms;
            switch(ctrl.publ_indx)
            {
                case 0:
                    if ( bme.performReading()) 
                    {
                        ctrl.temp = bme.temperature;
                        Serial.print("temperature = ");
                        Serial.print(ctrl.temp);
                        send_meas_to_uart("TEMP",ctrl.temp);
                        publ_status = sensor_temperature.publish(ctrl.temp); //Publish to Adafruit
                        report_publ_status(publ_status);
                        ctrl.fault_cntr.bme_fault = 0;
                    }
                    else
                    {
                        ctrl.fault_cntr.bme_fault++;
                        Serial.println("Error when reading BME680");                            
                    }
                    ctrl.publ_indx++;
                    break;
                case 1:
                    if ( bme.performReading()) 
                    {
                        ctrl.humidity = bme.humidity;
                        send_meas_to_uart("HUMI",ctrl.humidity);
                        publ_status = sensor_humidity.publish(ctrl.humidity); //Publish to Adafruit
                        report_publ_status(publ_status);
                        ctrl.fault_cntr.bme_fault = 0;
                    }
                    else
                    {                                                        
                        Serial.println("Error when reading BME680");
                    }
                    ctrl.publ_indx++;
                    break;
                 case 2:     
                    ctrl.ldr1 = (float) analogRead(PIN_LDR1);
                    send_meas_to_uart("LDR1",ctrl.ldr1);
                    publ_status = sensor_ldr1.publish(ctrl.ldr1); 
                    report_publ_status(publ_status);                    
                    ctrl.publ_indx++;
                    break;  
                case 3:     
                    ctrl.ldr2 = (float) analogRead(PIN_LDR2);
                    send_meas_to_uart("LDR2",ctrl.ldr2);
                    publ_status = sensor_ldr2.publish(ctrl.ldr2); 
                    report_publ_status(publ_status);                    
                    ctrl.publ_indx = 0;
                    break;  
                default:
                    ctrl.publ_indx = 0;
                    break;
            }
        }
        // sprintf(buff, "%d %d %d\n",ctrl.fault_cntr.mqtt_fault, ctrl.fault_cntr.bme_fault, ctrl.fault_cntr.ldr_fault);
        // Serial.print(buff);
        if ((ctrl.fault_cntr.mqtt_fault < 2) &&
            (ctrl.fault_cntr.bme_fault < 2) &&
            (ctrl.fault_cntr.ldr_fault < 4))
        {
            Watchdog.reset();
        }
        else
        {
            Serial.println("A Watchdog reset will occur");             
        }                               
    }

    /*
    Adafruit_MQTT_Subscribe *subscription;
    while ((subscription = mqtt.readSubscription(5000))) 
    {
        if (subscription == &set_temperature) 
        {
            Serial.print(F("Set temperature: "));
            Serial.println((char *)set_temperature.lastread);
            ctrl.set_temp = atoi((char *)set_temperature.lastread);
            Serial.println(ctrl.set_temp);
        }
    }
    */

