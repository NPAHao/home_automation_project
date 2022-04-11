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
TaskHandle_t    touch_1_task_handle;
TaskHandle_t    d_input_1_task_handle;
TaskHandle_t    recv_cb_task_handle;
TaskHandle_t    clear_eeprom_task_handle;
TaskHandle_t    wait_for_infor_task_handle;
TaskHandle_t    get_infor_task_handle;
TaskHandle_t    setup_gpio_task_handle;
TaskHandle_t    setup_espnow_task_handle;

//Queue handle
QueueHandle_t espnow_queue;

//Struct
struct recv_msg
{
    String mac;
    String data;
    int data_len;
};

//Variable
String  device_name;
String  mac_addr;
uint8_t broadcast[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
esp_now_peer_info peer;
bool    send_result;
bool    get_infor_result;


                                //Callback Function
void IRAM_ATTR touch_1_isr() {
    xSemaphoreGiveFromISR(touch_1_smp, pdFALSE);
}

void IRAM_ATTR d_input_1_isr() {
    vTaskNotifyGiveFromISR(d_input_1_task_handle, pdFALSE);
}

void IRAM_ATTR eeprom_pin_isr() {
    vTaskNotifyGiveFromISR(clear_eeprom_task_handle, pdFALSE);
}

void now_send_isr(const uint8_t *mac_addr, esp_now_send_status_t status) {
    send_result = (status == ESP_NOW_SEND_SUCCESS)?true:false;
}

void now_recv_isr(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
    recv_msg msg;
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
            serializeJson(doc, json);
            esp_now_send((uint8_t *)mac_addr.c_str(), (uint8_t *)json.c_str(), json.length());
        }
    }
}

void d_input_1_task(void *pvPara) {
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

void recv_cb_task(void *pvPara) {
    recv_msg msg;
    while (1)
    {
        xQueueReceive(espnow_queue, &msg, portMAX_DELAY);
        DynamicJsonDocument doc(250);
        deserializeJson(doc, msg.data);
        String from = doc["from"];
        String to = doc["to"];
        String purpose = doc["purpose"];
        if((from == "hub") && (to == "node")) {
            if(purpose == "add node") {
                pref.begin("infor", false);
                pref.putString("name", String((const char *)doc["name"]));
                pref.putString("mac_addr", msg.mac);
                pref.end();
            } else if(purpose == "add subnode") {

            } else if(purpose == "control") {

            }
        } else if((from == "subnode") && (to == "node")) {
            if(purpose == "send data") {
                
            } else if(purpose == "add success") {

            }
        }
    }
}

void clear_eeprom_task(void *pvPara) {
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        pref.begin("infor", false);
        pref.putString("mac_addr", "");
        pref.putString("name", "");
        pref.end();
    }
}

void wait_for_infor_task(void *pvPara) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}

void get_infor_task(void *pvPara) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        pref.begin("infor", true);
        mac_addr = pref.getString("mac_addr", "");
        device_name = pref.getString("name", "");
        pref.end();
        ((mac_addr == "") || (device_name == ""));
    }
}

void setup_gpio_task(void *pvPara) {
    while(1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        pinMode(D_OUTPUT_PIN_1, OUTPUT);
        pinMode(TOUCH_PIN_1, INPUT);
        pinMode(EEPROM_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(TOUCH_PIN_1), touch_1_isr, RISING);
        attachInterrupt(digitalPinToInterrupt(D_INPUT_PIN_1), d_input_1_isr, RISING);
        attachInterrupt(digitalPinToInterrupt(EEPROM_PIN), eeprom_pin_isr, FALLING);
    }
}

void setup_espnow_task(void *pvPara) {
    while(1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        WiFi.mode(WIFI_MODE_STA);
        WiFi.disconnect();
        esp_now_init();
        esp_now_register_recv_cb(now_recv_isr);
        esp_now_register_send_cb(now_send_isr);
        peer.channel = 1;
        peer.encrypt = false;
        memcpy(peer.peer_addr, broadcast, 6);
        esp_now_add_peer(&peer);
    }
}

void blink_led(int time) {
    pinMode(LED_BUILTIN, OUTPUT);
    for(int i = 0; i < time; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        vTaskDelay( 200 / portTICK_PERIOD_MS);
        digitalWrite(LED_BUILTIN, LOW);
        vTaskDelay( 200 / portTICK_PERIOD_MS);
    }
}


void setup() {
    Serial.begin(115200);
    touch_1_smp = xSemaphoreCreateBinary();
    espnow_queue = xQueueCreate(5, sizeof(recv_msg));
    xTaskCreate(touch_1_task, "touch 1 task", 2048, NULL, 10, &touch_1_task_handle);
    xTaskCreate(d_input_1_task, "d input 1 task", 2048, NULL, 10, &d_input_1_task_handle);
    xTaskCreate(recv_cb_task, "espnow recv task", 4096, NULL, 10, &recv_cb_task_handle);
    xTaskCreate(clear_eeprom_task, "clear eeprom task", 2048, NULL, 10, &clear_eeprom_task_handle);
    xTaskCreate(wait_for_infor_task, "wait infor task", 2048, NULL, 10, &wait_for_infor_task_handle);
    xTaskCreate(get_infor_task, "get infor task", 2048, NULL, 10, &get_infor_task_handle);
    xTaskCreate(setup_gpio_task, "setup gpio task", 2048, NULL, 10, &setup_gpio_task_handle);
    xTaskCreate(setup_espnow_task, "setup espnow task", 2048, NULL, 10, &setup_espnow_task_handle);

    // vTaskDelete(NULL);
}


void loop(){
    Serial.println("hello");
    vTaskDelay( 2000 / portTICK_PERIOD_MS);
}