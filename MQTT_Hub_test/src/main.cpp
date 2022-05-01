#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

String ssid = "Phi Hung";
String pw = "26022000";
String mqtt_server = "192.168.1.12";
WiFiClient    espclient;
PubSubClient  client(espclient);

void config_wifi(String ssid, String pw) {
  WiFi.begin(ssid.c_str(), pw.c_str());
  while(WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void mqtt_cb()



void setup() {
  Serial2.begin(115200);
  config_wifi(ssid, pw);

  client.setServer(mqtt_server.c_str(), 1883);
  client.setCallback();
}

void loop() {
  if(!client.connected()) {
    if(client.connect("ESP32")) {
      client.subscribe("esp32/hub2node");
      client.subscribe("esp32/hub2subnode");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  } else {
    client.loop();
    if(Serial2.available()) {
      DynamicJsonDocument doc(250);
      deserializeJson(doc, Serial2);
      String buffer;
      serializeJson(doc, buffer);
      client.publish("esp32/local2hub", buffer.c_str());
    }
  }
}