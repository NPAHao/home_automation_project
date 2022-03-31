#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <esp_now.h>

uint8_t broadcast_addr[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
bool in = false;
String jsondata;
esp_now_peer_info_t peer_infor;
// void recv_cb(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
//   DynamicJsonDocument doc(512);
//   deserializeJson(doc, data);
//   serializeJsonPretty(doc, jsondata);
//   in = true;
// }


void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_err_t error;
  error = esp_now_init();
  if(error != ESP_OK) {
    Serial.println("error1");
  }
  peer_infor.channel = 1;
  memcpy(peer_infor.peer_addr, broadcast_addr, 6);
  peer_infor.encrypt = false;
  error = esp_now_add_peer(&peer_infor);
  if(error != ESP_OK) {
    Serial.println("error2");
  }
  DynamicJsonDocument doc(250);
  doc["msg type"] = "incoming infor";
  doc["node infor"]["type"] = "flame";
  doc["node infor"]["interval"] = 5;
  doc["publish topic"] = "flame sensor";
  serializeJson(doc, jsondata);
  Serial.println(jsondata);
  esp_now_send(broadcast_addr, (uint8_t *)jsondata.c_str(), jsondata.length());
}

void loop() {
  
}