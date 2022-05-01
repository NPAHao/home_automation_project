#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

#define   DEBUG   1

typedef struct mqtt_packet
{
  String topic;
  String message;
} mqtt_packet;

//global variable
#ifdef  DEBUG
String      ssid        = "Phi Hung";
String      psk         = "26022000";
String      mqtt_server = "192.168.1.12";
String      client_name = "gateway";
String      node        = "room";
#else
String      ssid;
String      psk;
String      mqtt_server;
#endif
QueueHandle_t mqtt_packet_queue;
WiFiClient  espClient;
PubSubClient client(espClient);

void mqtt_cb(char* topic, uint8_t* message, unsigned int length) {
  mqtt_packet packet;
  packet.topic = String(topic);
  packet.message = String((char *)message).substring(0, length);
  xQueueSendFromISR(mqtt_packet_queue, &packet, NULL);
}

void config_wifi(String pv_ssid, String pv_psk) {
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin(pv_ssid.c_str(), pv_psk.c_str());
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void wifi_loop_task(void *pvParameters) {
  while(1) {
    if(!WiFi.isConnected()) {
      WiFi.begin(ssid.c_str(), psk.c_str());
      vTaskDelay(5000 / portTICK_PERIOD_MS);
    } else {
      vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
  }
}

void config_mqtt(const char *pv_server, uint16_t port, void (*callback)(char*, uint8_t*, unsigned int)) {
  client.setServer(pv_server, port);
  client.setCallback(callback);
}

void mqtt_loop_task(void *pvParameters) {
  String name = *(String *)pvParameters;
  while(1) {
    if(!client.connected()) {
      if(client.connect(name.c_str())) {
        client.subscribe((name + "/add node").c_str());
        client.subscribe((name + "/delete node").c_str());
        client.subscribe((node + "/add subnode").c_str());
        client.subscribe((node + "/set config").c_str());
        client.subscribe((node + "/get config").c_str());
        client.subscribe((node + "/control output").c_str());
        client.subscribe((node + "/trigger buzzer").c_str());
        //subcribe here
      }
    } else {
      client.loop();
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }
}

void split_topic(String pv_topic,String *pv_destination, String *pv_object) {
  Serial.println(pv_topic);
  *pv_destination = pv_topic.substring(0, pv_topic.indexOf('/'));
  *pv_object = pv_topic.substring(pv_topic.indexOf('/') + 1);
}

String get_mac_add(String pv_node) {
  String ret;
  return ret;
}

void process_topic(String pv_destination, String pv_object, String pv_message) {
  Serial.println(pv_destination);
  Serial.println(pv_object);
  Serial.println(pv_message);
}

void mqtt_packet_process(void *pvParameters) {
  mqtt_packet packet;
  String topic = "";
  String message = "";
  String destination = "";
  String object = "";
  String mac = "";
  DynamicJsonDocument doc(256);
  while(1) {
    xQueueReceive(mqtt_packet_queue, &packet, portMAX_DELAY);
    deserializeJson(doc, packet.message);
    destination = packet.topic.substring(0, packet.topic.indexOf('/'));
    object = packet.topic.substring(packet.topic.indexOf('/') + 1);
    doc["object"] = object;
    doc["destination"] = destination;
    // if(object == "add node") {
    //   doc["object"] = "add node";
    //   doc["node"] = String((const char *)msg_doc["node"]);
    // } else if(object == "add subnode") {
    //   doc["object"] = "add subnode";
    //   doc["node"]
    //   doc[""]
    // } else if(object == "delete node") {

    // } else if(object == "set input config") {

    // } else if(object == "get input config") {

    // } else if(object == "control output") {

    // } else if(object == "trigger buzzer") {

    // }
    serializeJson(doc, Serial2);
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  mqtt_packet_queue = xQueueCreate(10, sizeof(mqtt_packet));
  config_wifi(ssid, psk);
  xTaskCreate(wifi_loop_task, "wifi loop", 1024, NULL, 10, NULL);
  config_mqtt(mqtt_server.c_str(), (uint16_t)1883, mqtt_cb);
  xTaskCreate(mqtt_loop_task, "mqtt loop", 2048, (void *)&client_name, 10, NULL);
  xTaskCreate(mqtt_packet_process, "mqtt process", 2048, NULL, 10, NULL);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  vTaskDelete(NULL);
}

void loop(){}


// callback function
// task code
// void mqtt_task(void *pv) {
//   client.setServer(mqtt_server.c_str(), 1883);
//   client.setCallback(mqtt_cb);
//   while (1)
//   {
//     if(!client.connected()){
//       if(client.connect("esp32")){
//         client.subscribe("abc");
//       }
//     }
//   }
// }
// void receive_uart_task() {
//   Serial2.begin(115200);
//   while (1)
//   {
//     if(Serial2.available() > 0) {
//       DynamicJsonDocument doc(250);
//       deserializeJson(doc, Serial2);
//       String from = doc["from"];
//       String purpose = doc["purpose"];
//       String topic = doc["topic"];
//       String payload = doc["payload"];
//     }
//   }
// }