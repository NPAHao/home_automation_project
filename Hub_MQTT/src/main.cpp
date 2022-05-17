#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#define EEPROM_PIN          0

Preferences pref;

AsyncWebServer server(80);

TaskHandle_t    clear_eeprom_task_handle;

const char* ap_ssid = "Gateway AP";

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <title>ESP Input Form</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    Gateway Config
  </head>
  <body>
    <form action="/get">
      SSID: <input type="text" name="ssid"><br>
      Password: <input type="text" name="psk"><br>
      MQTT server IP: <input type="text" name="mqtt_server"><br>
      <input type="submit" value="Submit">
    </form>
  </body>
</html>)rawliteral";

String      ssid;
String      psk;
String      mqtt_server;
String      client_name = "gateway";
TaskHandle_t wifi_task_handle;
TaskHandle_t mqtt_task_handle;
TaskHandle_t uart_task_handle;
WiFiClient  espClient;
PubSubClient client;

void blink_led(int time, int interval_ms) 
{
  pinMode(LED_BUILTIN, OUTPUT);
  for(int i = 0; i < time; i++) 
  {
    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay( interval_ms / portTICK_PERIOD_MS);
    digitalWrite(LED_BUILTIN, LOW);
    vTaskDelay( interval_ms / portTICK_PERIOD_MS);
  }
}

void IRAM_ATTR eeprom_pin_isr() 
{
  vTaskNotifyGiveFromISR(clear_eeprom_task_handle, NULL);
}
void clear_eeprom_task(void *pvParameters) 
{
  pinMode(EEPROM_PIN, INPUT_PULLUP);
  attachInterrupt(EEPROM_PIN, eeprom_pin_isr, FALLING);
  while (1)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    pref.begin("infor", false);
    pref.putString("ssid", "");
    pref.putString("psk", "");
    pref.putString("mqtt_server", "");
    pref.end();
    blink_led(5, 200);
    ESP.restart();
  }
}

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

void onGet(AsyncWebServerRequest *request) {
  String ssid_msg;
  String psk_msg;
  String mqtt_msg;
  if (request->hasParam("ssid")) {
    ssid_msg = request->getParam("ssid")->value();
  }
  if (request->hasParam("psk")) {
    psk_msg = request->getParam("psk")->value();
  }
  if (request->hasParam("mqtt_server")) {
    mqtt_msg = request->getParam("mqtt_server")->value();
  }
  if((ssid_msg == "") || (psk_msg == "") || (mqtt_msg == "")){
    request->send(200, "text/html", "Some input field is empty<br><a href=\"/\">Return to Home Page</a>");
  } else {
    request->send(200, "text/html", "HTTP GET request sent to your ESP on input field<br>" 
                                      + String("SSID with value: ") + ssid_msg + "<br>" 
                                      + " Password with value: " + psk_msg + "<br>"
                                      + " MQTT server IP with value: " + mqtt_msg + "<br>"
                                      + "Gateway will restart in a few second");
    pref.begin("infor", false);
    pref.putString("ssid", ssid_msg);
    pref.putString("psk", psk_msg);
    pref.putString("mqtt_server", mqtt_msg);
    pref.end();
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    ESP.restart();
  }
}

void onIP(AsyncWebServerRequest *request) {
  request->send_P(200, "text/html", index_html);
}

void get_config_webserver_task(void *pvParameters) {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid);
  server.on("/", HTTP_GET, onIP);
  server.on("/get", HTTP_GET, onGet);
  server.onNotFound(notFound);
  server.begin();
  while (1)
  {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void get_infor() {
  pref.begin("infor", true);
  ssid = pref.getString("ssid", "");
  psk = pref.getString("psk", "");
  mqtt_server = pref.getString("mqtt_server", "");
  pref.end();
  if((ssid == "") || (psk == "") || (mqtt_server == "")) {
    xTaskCreate(get_config_webserver_task, "webserver", 4096, NULL, 10, NULL);
    while (1)
    {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }
}

void config_wifi(String pv_ssid, String pv_psk) {
  WiFi.begin(pv_ssid.c_str(), pv_psk.c_str());
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void wifi_loop_task(void *pvParameters) {
  WiFi.mode(WIFI_MODE_STA);
  config_wifi(ssid, psk);
  xTaskNotifyGive(mqtt_task_handle);
  while(1) {
    if(!WiFi.isConnected()) {
      config_wifi(ssid, psk);
    } else {
      vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
  }
}

void mqtt_loop_task(void *pvParameters) {
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  client.setClient(espClient);
  client.setServer(mqtt_server.c_str(), 1883);
  client.setStream(Serial2);
  do {
    client.connect(client_name.c_str());
    vTaskDelay(100 / portTICK_PERIOD_MS);
  } while(!client.connected());
  client.subscribe("mqtt out");
  xTaskNotifyGive(uart_task_handle);
  while(1) {
    if(!client.connected()) {
      if(client.connect(client_name.c_str())) {
        client.subscribe("mqtt out");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
      }
    } else {
      client.loop();
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }
}

void uart_loop_task(void *pvParameters) {
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  Serial2.begin(115200);
  Serial2.setTimeout(200);
  DynamicJsonDocument doc(MQTT_MAX_PACKET_SIZE);
  while(1) {
    if(Serial2.available()) {
      if(!deserializeJson(doc, Serial2)) {
        String buffer;
        serializeJson(doc, buffer);
        client.publish("mqtt in", buffer.c_str());
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup() {
  xTaskCreate(clear_eeprom_task, "clear infor", 2048, NULL, 10, &clear_eeprom_task_handle);
  get_infor();
  xTaskCreate(wifi_loop_task, "wifi loop", 2048, NULL, 10, &wifi_task_handle);
  xTaskCreate(mqtt_loop_task, "mqtt loop", 2048, NULL, 10, &mqtt_task_handle);
  xTaskCreate(uart_loop_task, "uart loop", 2048, NULL, 10, &uart_task_handle);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  vTaskDelete(NULL);
}

void loop(){}