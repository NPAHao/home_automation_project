//Include necessary libraries
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

//Task handle
TaskHandle_t    touch_1_handle;
TaskHandle_t    d_input_1_handle;

//Queue handle
QueueHandle_t espnow_queue;

//Struct
struct incoming_msg_infor
{
    String mac;
    String data;
    int data_len;
};

//Variable
String  device_name;
String  mac_addr;



                                //Callback Function
void IRAM_ATTR touch_1_isr() {
    xSemaphoreGiveFromISR(touch_1_smp, pdFALSE);
}

void IRAM_ATTR d_input_1_isr() {
    vTaskNotifyGiveFromISR(d_input_1_handle, pdFALSE);
}

void now_send_isr(const uint8_t *mac_addr, esp_now_send_status_t status) {

}

void now_recv_isr(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
    incoming_msg_infor msg;
    msg.mac = String((char *)mac_addr);
    msg.data = String((char *)data);
    msg.data_len = data_len;
    xQueueSendFromISR(espnow_queue, &msg, pdFALSE);
}


                                //Task Define
void touch_1_task(void *pvPara) {
    uint8_t time;
    DynamicJsonDocument doc(250);
    String json;
    doc["from"] = "node";
    doc["purpose"] = "publish";
    doc["topic"] = device_name + '/' + "touch_1";
    while (1)
    {
        time = 0;
        json = "";
        if( xSemaphoreTake( touch_1_smp , portMAX_DELAY) == pdTRUE ) {
            time++;
            while ( xSemaphoreTake ( touch_1_smp , 500 / portTICK_PERIOD_MS) == pdTRUE) {
                time++;
                if(time == 3) break;
            }
            if( (time == 1) && digitalRead(TOUCH_PIN_1) ) {
                doc["payload"] = "long touch";
            } else {
                switch (time)
                {
                case 1:
                    doc["payload"] = "single touch";
                    break;
                case 2:
                    doc["payload"] = "double touch";
                    break;
                case 3:
                    doc["payload"] = "triple touch";
                    break;               
                }
            }
            // serializeJsonPretty(doc, Serial);
            serializeJson(doc, json);
            esp_now_send((uint8_t *)mac_addr.c_str(), (uint8_t *)json.c_str(), json.length());
        }
    }
}


void d_input_1_task(void *pv) {
    DynamicJsonDocument doc(250);
    String json;
    doc["from"] = "node";
    doc["purpose"] = "publish";
    doc["topic"] = device_name + '/' + "d_input1";
    doc["payload"] = "ACTIVE";
    serializeJson(doc, json);
    while(1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    esp_now_send((uint8_t *)mac_addr.c_str(), (uint8_t *)json.c_str(), json.length());
    }
}

void recv_cb_task(void *pv) {
    incoming_msg_infor msg;
    while (1)
    {
        xQueueReceive(espnow_queue, &msg, portMAX_DELAY);
        DynamicJsonDocument doc(250);
        deserializeJson(doc, msg.data);
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
}

void get_infor_task(void *pv) {
    pref.begin("infor", true);
    mac_addr = pref.getString("mac_addr", "");
    device_name = pref.getString("name", "");

}


void gpio_task(void *pvPara) {
    pinMode(D_OUTPUT_PIN_1, OUTPUT);
    pinMode(TOUCH_PIN_1, INPUT);
    attachInterrupt(digitalPinToInterrupt(TOUCH_PIN_1), touch_1_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(D_INPUT_PIN_1), d_input_1_isr, RISING);
    xTaskCreate(touch_1_task, "touch 1", 2048, NULL, 10, NULL);
    vTaskDelete(NULL);
}


void wireless_init(void *pv) {
    WiFi.mode(WIFI_MODE_STA);
    WiFi.disconnect();
    esp_now_init();
    esp_now_register_recv_cb(now_recv_isr);
    esp_now_register_send_cb(now_send_isr);
    vTaskDelete(NULL);
}


void setup() {
    Serial.begin(115200);
    espnow_queue = xQueueCreate(5, sizeof(incoming_msg_infor));
    touch_1_smp = xSemaphoreCreateBinary();
    vTaskDelete(NULL);
}


void loop(){}
