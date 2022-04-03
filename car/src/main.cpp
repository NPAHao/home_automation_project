#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <esp_now.h>

void recv_cb(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  DynamicJsonDocument doc(250);
  deserializeJson(doc, data);
  serializeJsonPretty(doc, Serial);
  Serial.println();
}

void esp_task(void *pv) {
  WiFi.mode(WIFI_MODE_STA);
  WiFi.disconnect();
  while (esp_now_init() != ESP_OK) {
    Serial.println("init fail");
    vTaskDelay(500);
  }
  while(esp_now_register_recv_cb(recv_cb) != ESP_OK) {
    Serial.println("register fail");
    vTaskDelay(500);
  }
  while (1)
  {

  }
}



void setup() {
  Serial.begin(115200);
  xTaskCreate(esp_task, "init task", 2048, NULL, 10, NULL);
  vTaskDelete(NULL);
}

void loop() {
  // put your main code here, to run repeatedly:
}