#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ArduinoJson.h>
//84

const uint8_t des[] = {0x9c, 0x9c, 0x1f, 0xc7, 0x20, 0x8c};
esp_now_peer_info_t infor;

void recv_cb(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  DynamicJsonDocument doc(250);
  deserializeJson(doc, data);
  serializeJson(doc, Serial2);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  WiFi.mode(WIFI_MODE_STA);
  WiFi.disconnect();
  esp_now_init();
  infor.channel = 1;
  infor.encrypt = false;
  memcpy(infor.peer_addr, des, 6);
  esp_now_add_peer(&infor);
  esp_now_register_recv_cb(recv_cb);
}

void loop() {
  if(Serial2.available()) {
    DynamicJsonDocument doc(250);
    deserializeJson(doc, Serial2);
    String buffer;
    serializeJson(doc, buffer);
    Serial.println(buffer);
    esp_now_send(des, (const uint8_t *)buffer.c_str(), buffer.length());
  }
}