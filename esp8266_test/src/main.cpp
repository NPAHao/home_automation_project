#include <Arduino.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <StreamUtils.h>
#include <espnow.h>
#include <ESP8266WiFi.h>

void send_cb(u8 *mac_addr, u8 status) {

}

void setup() {
  Serial.begin(115200);
  uint8_t des[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
  String ssid = "NPAHao";
  String password = "hao12345";
  DynamicJsonDocument doc(512);
  doc["code"] = 1;
  doc["credential"]["ssid"] = ssid;
  doc["credential"]["password"] = password;
  doc["mqtt infor"]["ip addr"] = "198.162.1.12";
  doc["mqtt infor"]["master name"] = "esp32";
  serializeJsonPretty(doc, Serial);
  String jsondata;
  serializeJson(doc, jsondata);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_now_init();
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_add_peer(des, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  esp_now_send(des, (uint8_t *)jsondata.c_str(), jsondata.length());
}

void loop() {

}