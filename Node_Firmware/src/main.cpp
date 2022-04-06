//Include essential libraries
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h>
#include <DHT.h>
#include "pv_define.h"
#include <ArduinoJson.h>


//Declare Object
Preferences pref;

//Declare semaphore
SemaphoreHandle_t   touch_1_smp = NULL;

//Variable
String  name = "esp32";
String  mac_addr;


                                //Callback Function
void IRAM_ATTR touch_pin_1_isr() {
    xSemaphoreGiveFromISR(touch_1_smp , NULL);
}

void IRAM_ATTR d_input_pin_1_isr() {
    // client.publish( (mqtt.user_name+ "d_input1").c_str(), "strigg");
    DynamicJsonDocument doc(250);
    String json;
    doc["from"] = "node";
    doc["purpose"] = "publish";
    doc["topic"] = name + '/' + "d_input1";
    doc["payload"] = "ACTIVE";
    serializeJson(doc, json);
    esp_now_send((uint8_t *)mac_addr.c_str(), (uint8_t *)json.c_str(), json.length());
}

void recv_cb(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
    DynamicJsonDocument doc(250);
    deserializeJson(doc, data);
    String from = doc["from"];
    String purpose = doc["purpose"];
    if(from == "hub") {
        if(purpose == "add node") {

        } else if(purpose == "add subnode") {

        } else if(purpose == "control") {

        }
    } else if(from == "subnode") {
        if(purpose == "send data") {
            
        } else if(purpose == "add success") {

        }
    }
}


                                //Task Define
void touch_pin_1_task(void *pvPara) {
    uint8_t time;
    while (1)
    {
        DynamicJsonDocument doc(250);
        String json;
        doc["from"] = "node";
        doc["purpose"] = "publish";
        doc["topic"] = name + '/' + "touch_1";
        time = 0;
        if( xSemaphoreTake( touch_1_smp , portMAX_DELAY) == pdTRUE ) {
            time++;
            while ( xSemaphoreTake ( touch_1_smp , 500 / portTICK_PERIOD_MS) == pdTRUE) {
                time++;
                if(time == 3) break;
            }
            if( (time == 1) && digitalRead(TOUCH_PIN_1) ) {
                //client.publish( (mqtt.user_name + "/touch1").c_str(), "longtouch");
                doc["payload"] = "long touch";
            } else {
                switch (time)
                {
                case 1:
                    //client.publish( (mqtt.user_name + "/touch1").c_str(), "singletouch");
                    doc["payload"] = "single touch";
                    break;
                case 2:
                    //client.publish( (mqtt.user_name + "/touch1").c_str(), "doubletouch");
                    doc["payload"] = "double touch";
                    break;
                case 3:
                    //client.publish( (mqtt.user_name + "/touch1").c_str(), "trippletouch");
                    doc["payload"] = "triple touch";
                    break;               
                }
            }
            serializeJson(doc, json);
            esp_now_send((uint8_t *)mac_addr.c_str(), (uint8_t *)json.c_str(), json.length());
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


void wireless_init(void *pv) {
    WiFi.mode(WIFI_MODE_STA);
    WiFi.disconnect();
    esp_now_init();
    esp_now_register_recv_cb(recv_cb);
    vTaskDelete(NULL);
}


void setup() {
    Serial.begin(115200);
    // xTaskCreate(wireless_init, "espnow init", 2048, NULL, 10, NULL);
    // // xTaskCreate(gpio_task, "gpio", 2048, NULL, 10, NULL);
    // vTaskDelete(NULL);
}


void loop(){}
