#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ArduinoJson.h>

esp_now_peer_info peer;
uint8_t broadcast[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

void recv_cb(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  Serial.println((const char*)data);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Start!");
  WiFi.mode(WIFI_MODE_STA);
  WiFi.disconnect();
  esp_now_init();
  peer.channel = 1;
  peer.encrypt = false;
  memcpy(peer.peer_addr, broadcast, 6);
  esp_now_add_peer(&peer);
  esp_now_register_recv_cb(recv_cb);
}

void loop() {
  DynamicJsonDocument doc(250);
  doc["from"] = "hub";
  doc["to"] = "node";
  doc["purpose"] = "add node";
  doc["name"] = "room";
  String buffer;
  serializeJson(doc, buffer);
  esp_now_send(broadcast, (uint8_t *)buffer.c_str(), buffer.length());
  vTaskDelay(5000 / portTICK_PERIOD_MS);
}