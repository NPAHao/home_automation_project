#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <ArduinoJson.h>

u8 des[] = {0x7c, 0x9e, 0xbd, 0x62, 0xa1, 0x84};

void recv_cb(u8 *mac_addr, u8 *data, u8 len) {
  Serial.println(String((char *)data));
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_now_init();
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_add_peer(des, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  esp_now_register_recv_cb(recv_cb);
  DynamicJsonDocument doc(250);
  String buffer;
  doc["from"] = "node";
  doc["to"] = "hub";
  serializeJson(doc, buffer);
  esp_now_send(des, (u8 *)buffer.c_str(), buffer.length());
}

void loop() {
  // put your main code here, to run repeatedly:
}