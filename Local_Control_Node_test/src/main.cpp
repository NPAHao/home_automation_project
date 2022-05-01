#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ArduinoJson.h>

void recv_cb(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  DynamicJsonDocument doc(250);
  deserializeJson(doc, data);
  serializeJson(doc, Serial);
}

typedef struct split
{
  String name;
  String topic;
} split;

split split_topic(String str) {
  split ret_str;
  ret_str.name = str.substring( 0 , str.indexOf('/') );
  ret_str.topic = str.substring( str.indexOf('/') + 1);
  return ret_str;
}

void setup() {
  Serial.begin(115200);
  // WiFi.mode(WIFI_MODE_STA);
  // WiFi.disconnect();
  // esp_now_init();
  // esp_now_register_recv_cb(recv_cb);
  String str = "esp32/input";
  split str2 = split_topic(str);
  Serial.println(str);
  Serial.println(str2.name);
  Serial.println(str2.topic);
}

void loop() {
  // put your main code here, to run repeatedly:
}