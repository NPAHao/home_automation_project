#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

//task handle
TaskHandle_t wifi_task_handle;

//global variable
Preferences pref;
String      ssid;
String      psk;
String      mqtt_server = "192.168.1.12";
WiFiClient  espClient;
PubSubClient client(espClient);


// callback function
void on_disconnect_wifi(WiFiEvent_t event, WiFiEventInfo_t info) {
  WiFi.begin(ssid.c_str(), psk.c_str());
}

void on_connect_wifi(WiFiEvent_t event, WiFiEventInfo_t info) {
  WiFi.removeEvent(SYSTEM_EVENT_STA_CONNECTED);
  Serial.println("connected");
  vTaskDelete(wifi_task_handle);
}

void mqtt_cb(char* topic, byte* message, unsigned int length) {

}


// task code
void wifi_task(void *pv) {
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

void mqtt_task(void *pv) {
  client.setServer(mqtt_server.c_str(), 1883);
  client.setCallback(mqtt_cb);
  while (1)
  {
    if(!client.connected()){
      if(client.connect("esp32")){
        client.subscribe("abc");
      }
    }
  }
}

void receive_uart_task() {
  Serial2.begin(115200);
  while (1)
  {
    if(Serial2.available() > 0) {
      DynamicJsonDocument doc(250);
      deserializeJson(doc, Serial2);
      String from = doc["from"];
      String purpose = doc["purpose"];
      String topic = doc["topic"];
      String payload = doc["payload"];

    }
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_MODE_STA);
  xTaskCreate(wifi_task, "wifi task", 2048, NULL, 10, &wifi_task_handle);
  vTaskDelete(NULL);
}

void loop(){}