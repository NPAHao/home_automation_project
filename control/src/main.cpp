#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <esp_now.h>
// #include <Preferences.h>
// #include <nvs_flash.h>

#define     FORWARD_PIN     39
#define     BACKWARD_PIN    36
#define     LEFT_PIN        35
#define     RIGHT_PIN       34

esp_now_peer_info peer;
uint8_t car_addr[] = {0x7c, 0x9e, 0xbd, 0x62, 0x74, 0x04};
String  json_msg;
uint16_t forward_val;
uint16_t backward_val;
uint16_t left_val;
uint16_t right_val;
SemaphoreHandle_t send_smp;
SemaphoreHandle_t init_smp;

void send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if(status == ESP_NOW_SEND_SUCCESS) {
        xSemaphoreGiveFromISR( send_smp, NULL);
    } else Serial.println("send fail");
}

void wireless_init(void *pv) {
    xSemaphoreTake(init_smp, portMAX_DELAY);
    WiFi.mode(WIFI_MODE_STA);
    WiFi.disconnect();
    while (esp_now_init() != ESP_OK) {
        Serial.println("Init error!");
        vTaskDelay(500);
    }
    memcpy(peer.peer_addr, car_addr, 6);
    peer.channel = 1;
    peer.encrypt = false;
    while(esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("add peer fail");
        vTaskDelay(500);        
    }
    while(esp_now_register_send_cb(send_cb) != ESP_OK) {
        Serial.println("send register fail");
        vTaskDelay(500);
    }
    xSemaphoreGive(init_smp);
    vTaskDelete(NULL);
}

void forward_task(void *pv) {
    pinMode(FORWARD_PIN, INPUT);
    forward_val = 0;
    while (1)
    {
        if((digitalRead(FORWARD_PIN) == 1) && (forward_val < 4095)) {
            forward_val += 35;
        }
        else if((digitalRead(FORWARD_PIN) == 0) && (forward_val > 0)) {
            if(forward_val >= 105){
                forward_val -= 105;
            } else forward_val = 0;
        }
        vTaskDelay( 10 / portTICK_PERIOD_MS);
    }
}

void backward_task(void *pv) {
    pinMode(BACKWARD_PIN, INPUT);
    backward_val = 0;
    while (1)
    {
        if((digitalRead(BACKWARD_PIN) == 1) && (backward_val < 4095)) {
            backward_val += 35;
        }
        else if((digitalRead(BACKWARD_PIN) == 0) && (backward_val > 0)) {
            if(backward_val >= 105) {
                backward_val -= 105;
            } else backward_val = 0;
        }
        vTaskDelay( 10 / portTICK_PERIOD_MS);
    }
}

void left_task(void *pv) {
    pinMode(LEFT_PIN, INPUT);
    left_val = 0;
    while (1)
    {
        if((digitalRead(LEFT_PIN) == 1) && (left_val < 4095)) {
            left_val += 35;
        }
        else if((digitalRead(LEFT_PIN) == 0) && (left_val > 0)) {
            if(left_val >= 105) {
                left_val -= 105;
            } else left_val = 0;
        }
        vTaskDelay( 10 / portTICK_PERIOD_MS);
    }
}

void right_task(void *pv) {
    pinMode(RIGHT_PIN, INPUT);
    right_val = 0;
    while (1)
    {
        if((digitalRead(RIGHT_PIN) == 1) && (right_val < 4095)) {
            right_val += 35;
        }
        else if((digitalRead(RIGHT_PIN) == 0) && (right_val > 0)) {
            if(right_val >= 105) {
                right_val -= 105;
            } else right_val = 0;
        }
        vTaskDelay( 10 / portTICK_PERIOD_MS);
    }
}

void send_task(void *para) {
    xSemaphoreTake(init_smp, portMAX_DELAY);
    TickType_t start;
    while (1)
    {
        xSemaphoreTake(send_smp, 1000 / portTICK_PERIOD_MS);
        start = xTaskGetTickCount();
        json_msg = "";
        DynamicJsonDocument doc(250);
        doc["forward"] = forward_val;
        doc["backward"] = backward_val;
        doc["left"] = left_val;
        doc["right"] = right_val;
        serializeJson(doc, json_msg);
        // Serial.println(json_msg);
        esp_now_send(car_addr, (uint8_t *)json_msg.c_str(), json_msg.length());
        vTaskDelayUntil(&start, 250 / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(115200);
    vSemaphoreCreateBinary(send_smp);
    vSemaphoreCreateBinary(init_smp);
    xTaskCreate(wireless_init, "esp now init", 2048, NULL, 10, NULL);
    xTaskCreate(forward_task, "forward task", 1024, NULL, 10, NULL);
    xTaskCreate(backward_task, "backward task", 1024, NULL, 1024, NULL);
    xTaskCreate(left_task, "left task", 1024, NULL, 10, NULL);
    xTaskCreate(right_task, "right task", 1024, NULL, 10, NULL);
    xTaskCreate(send_task, "send task", 2048, NULL, 10, NULL);
    vTaskDelete(NULL);
}

void loop() {
  
}