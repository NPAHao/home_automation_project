#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <nvs_flash.h>

Preferences pref;

AsyncWebServer server(80);

const char* ap_ssid = "ESP32";

String ssid;
String psk;
String mqtt_server;

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

void get_config_webserver(void *pvParameters) {
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
        xTaskCreate(get_config_webserver, "webserver", 4096, NULL, 10, NULL);
        while (1)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}

void setup() {
    Serial.begin(115200);
    get_infor();
    Serial.println(ssid);
    Serial.println(psk);
    Serial.println(mqtt_server);
}

void loop() {
  
}