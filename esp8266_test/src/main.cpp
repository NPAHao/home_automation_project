#include <Arduino.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <StreamUtils.h>
#include <ESP8266WiFi.h>
#include <espnow.h>

uint8_t debug_mac[] = {0x7c, 0x9e, 0xbd, 0x62, 0x74, 0x04};
bool    send_result;

void send_msg_cb(u8 *mac_addr, u8 status) {
  send_result = (status==0)?true:false;
}

void setup() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_now_init();
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
    DynamicJsonDocument doc(256);
    String buffer;
    doc["from"] = "subnode";
    doc["to"] = "node";
    doc["purpose"] = "send data";
    doc["topic"] = "topic";
    doc["payload"] = "FLAME DETECTED!";
    serializeJson(doc, buffer);
    esp_now_add_peer(debug_mac, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
    esp_now_register_send_cb(send_msg_cb);
    send_result = false;
    while (send_result != true) {
        // esp_now_send((uint8_t *)mac_address, (uint8_t *)buffer.c_str(), buffer.length());
        esp_now_send(debug_mac, (uint8_t *)buffer.c_str(), buffer.length());
        delay(250);
    }
}

void loop() {

}