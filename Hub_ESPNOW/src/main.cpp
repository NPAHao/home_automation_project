#include <Arduino.h>
#include <espnow.h>
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>

bool add_result;
bool send_result;
uint8_t broadcast[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

void send_cb(u8 *mac_addr, u8 status) {
  send_result = (status == 0)?true:false;
}

void recv_cb(u8 *mac_addr, u8 *data, u8 len) {
  DynamicJsonDocument doc(250);
  deserializeJson(doc, data);
  String from = doc["from"];
  String purpose = doc["purpose"];
  if( (from == "node") && (purpose == "add success") ) {
    add_result = true;
  }
  if( (from == "node") && (purpose == "publish") ) {
    serializeJson(doc, Serial);
  }
}

void setup() {
  Serial.begin(115200);
  esp_now_init();
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_send_cb(send_cb);
  esp_now_register_recv_cb(recv_cb);
  esp_now_add_peer(broadcast, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
}

void loop() {
  if(Serial.available() > 0) {
    DynamicJsonDocument doc(300);
    DeserializationError err = deserializeJson(doc, Serial);
    if (!err)
    {
      String str;
      serializeJson(doc, str);
      if(doc["purpose"] == "add node") {
        add_result = false;
        while (!add_result) {
          esp_now_send(broadcast, (uint8_t *)str.c_str(), str.length());
        }
      } else {
        String mac = doc["mac"];
        esp_now_add_peer((uint8_t *)mac.c_str(), ESP_NOW_ROLE_COMBO, 1, NULL, 0);
        send_result = false;
        while(!send_result) {
          esp_now_send((uint8_t *)mac.c_str(), (uint8_t *)str.c_str(), str.length());
        }
        esp_now_del_peer((uint8_t *)mac.c_str());
      }
    }
  }
}