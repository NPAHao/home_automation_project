//Include essential libraries
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h>
#include <DHT.h>
#include "pv_define.h"


//Declare task handle variable
TaskHandle_t gpio_task_handle;
TaskHandle_t dht11_task_handle;
TaskHandle_t wifi_task_handle;


//Declare Object
Preferences pref;

//Declare semaphore
SemaphoreHandle_t   touch_1_smp = NULL;


                                //Callback Function
void IRAM_ATTR touch_pin_1_isr() {
    xSemaphoreGiveFromISR(touch_1_smp , NULL);
}

void IRAM_ATTR d_input_pin_1_isr() {
    // client.publish( (mqtt.user_name+ "d_input1").c_str(), "strigg");
}


                                //Task Define
void touch_pin_1_task(void *pvPara) {
    uint8_t time;
    while (1)
    {
        time = 0;
        if( xSemaphoreTake( touch_1_smp , portMAX_DELAY) == pdTRUE ) {
            time++;
            while ( xSemaphoreTake ( touch_1_smp , 500 / portTICK_PERIOD_MS) == pdTRUE) {
                time++;
                if(time == 3) break;
            }
            if( (time == 1) && digitalRead(TOUCH_PIN_1) ) {
                //client.publish( (mqtt.user_name + "/touch1").c_str(), "longtouch");
                Serial.println("long detect");
            } else {
                switch (time)
                {
                case 1:
                    //client.publish( (mqtt.user_name + "/touch1").c_str(), "singletouch");
                    Serial.println("single touch detect");
                    break;
                case 2:
                    //client.publish( (mqtt.user_name + "/touch1").c_str(), "doubletouch");
                    Serial.println("double touch detect");
                    break;
                case 3:
                    //client.publish( (mqtt.user_name + "/touch1").c_str(), "trippletouch");
                    Serial.println("triple touch detect");
                    break;               
                }
            }
        }
    }
}

void gpio_task(void *pvPara) {
    pinMode(D_OUTPUT_PIN_1, OUTPUT);
    pinMode(TOUCH_PIN_1, INPUT);
    attachInterrupt(digitalPinToInterrupt(TOUCH_PIN_1), touch_pin_1_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(D_INPUT_PIN_1), d_input_pin_1_isr, RISING);
    touch_1_smp = xSemaphoreCreateBinary();
    xTaskCreate(touch_pin_1_task, "touch 1", 2048, NULL, 10, NULL);
    vTaskDelete(NULL);
}

/**
 * @brief This task will setup DHT11 and send data periodly 10s
 * 
 * @param pvPara 
 */
void dht11_task(void *pvPara) {
    DHT dht(DHT11_PIN, DHT11);
    dht.begin();
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount ();
    while (1)
    {   
        vTaskDelayUntil( &xLastWakeTime, 10000 / portTICK_PERIOD_MS);
        // Reading temperature or humidity takes about 250 milliseconds!
        float h = dht.readHumidity();
        // Read temperature as Celsius (the default)
        float t = dht.readTemperature();
        // check
        if (isnan(h) || isnan(t)) {
            Serial.println(F("Failed to read from DHT sensor!"));
        }
        //publish
        // client.publish( (mqtt.device_name + "/dht11_humi").c_str(), String(h).c_str());
        // client.publish( (mqtt.device_name + "/dht11_temp").c_str(), String(t).c_str());
    }
}


/**
 * @brief setup task
 * 
 */
void setup() {
    Serial.begin(115200);
    xTaskCreate(gpio_task, "gpio", 2048, NULL, 10, NULL);
    vTaskDelete(NULL);
}


void loop(){}
