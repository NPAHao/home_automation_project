#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <ArduinoJson.h>

TaskHandle_t wifi_task_handle;

Preferences pref;
String      ssid;
String      psk;

void on_disconnect_wifi(WiFiEvent_t event, WiFiEventInfo_t info) {
  WiFi.begin(ssid.c_str(), psk.c_str());
}

void on_connect_wifi(WiFiEvent_t event, WiFiEventInfo_t info) {
  WiFi.removeEvent(SYSTEM_EVENT_STA_CONNECTED);
  Serial.println("connected");
  vTaskDelete(wifi_task_handle);
}

void setup_wifi(void *pv) {
  pref.begin("credential", false);
  ssid = pref.getString("ssid", "");
  psk = pref.getString("psk", "");
  if( (ssid == "") || (psk == "")) {
    WiFi.beginSmartConfig();
    Serial.println("begin smartconfig"); 
    while (!WiFi.smartConfigDone()) {
      vTaskDelay(500 / portTICK_PERIOD_MS);
      Serial.print('.');
    }
    ssid = WiFi.SSID();
    psk = WiFi.psk();
    pref.putString("ssid", ssid);
    pref.putString("psk", psk);    
  } else {
    WiFi.begin(ssid.c_str(), psk.c_str());
  }
  pref.end();
  WiFi.onEvent(on_disconnect_wifi, SYSTEM_EVENT_STA_DISCONNECTED);
  WiFi.onEvent(on_connect_wifi, SYSTEM_EVENT_STA_CONNECTED);
  while (1)
  {
    Serial.print('.');
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  WiFi.mode(WIFI_MODE_STA);
  xTaskCreate(setup_wifi, "wifi task", 2048, NULL, 10, &wifi_task_handle);
  vTaskDelete(NULL);
}

void loop(){}