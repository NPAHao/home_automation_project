#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ArduinoJson.h>

#define MQTT_MSG_MAX_LEN  256

typedef struct espnow_packet
{
  uint8_t mac[ESP_NOW_ETH_ALEN];
  uint8_t message[ESP_NOW_MAX_DATA_LEN];
} espnow_packet;

QueueHandle_t espnow_send_queue;
QueueHandle_t espnow_recv_queue;
QueueHandle_t uart_recv_queue;
QueueHandle_t uart_send_queue;

uint8_t broadcast_mac[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
String s_broadcast_mac = String("FF:FF:FF:FF:FF:FF");
String s_self_mac;
uint8_t self_mac[6];
esp_now_peer_info_t peer_infor;
bool espnow_send_success = false;

void esp_now_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if(status == ESP_NOW_SEND_SUCCESS) {
    espnow_send_success = true;
  }
}

void esp_now_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  espnow_packet packet;
  memcpy(packet.mac, mac_addr, ESP_NOW_ETH_ALEN);
  memcpy(packet.message, data, data_len);
  xQueueSendFromISR(espnow_recv_queue, &packet, NULL);
}

bool config_esp_now() {
  WiFi.mode(WIFI_MODE_STA);
  WiFi.disconnect();
  WiFi.macAddress(self_mac);
  s_self_mac = WiFi.macAddress();
  if(esp_now_init() != ESP_OK) {
    return false;
  }
  if(esp_now_register_send_cb(esp_now_send_cb) != ESP_OK) {
    return false;
  }
  if(esp_now_register_recv_cb(esp_now_recv_cb) != ESP_OK) {
    return false;
  }
  return true;
}

void send_espnow_packet_task(void *pvParameters) {
  peer_infor.channel = 1;
  peer_infor.encrypt = false;
  while(1) 
  {
    espnow_packet packet;
    xQueueReceive(espnow_send_queue, &packet, portMAX_DELAY);
    memcpy(peer_infor.peer_addr, packet.mac, ESP_NOW_ETH_ALEN);
    esp_now_add_peer(&peer_infor);
    espnow_send_success = false;
    esp_now_send(packet.mac, packet.message, strlen((char *)packet.message));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    if(espnow_send_success != true) {
      DynamicJsonDocument doc(MQTT_MSG_MAX_LEN);
      doc["object"] = String("send error");
      doc["error"] = String("send to node fail");
      serializeJson(doc, Serial2);
    }
    esp_now_del_peer(packet.mac);
  }
}

void process_espnow_packet_task(void *pvParameters) {
  DynamicJsonDocument doc(ESP_NOW_MAX_DATA_LEN);
  while (1)
  {
    espnow_packet packet;
    xQueueReceive(espnow_recv_queue, &packet, portMAX_DELAY);
    if(!deserializeJson(doc, packet.message)) {
      String object = String((const char *)doc["object"]);
      if((object != String("add node")) && (object != String("add subnode"))) {
        serializeJson(doc, Serial2);
      }
    }
  }
}

void process_uart_msg_task(void *pvParameters) {
  DynamicJsonDocument doc(MQTT_MSG_MAX_LEN);
  while (1)
  {
    if(Serial2.available()) {
      if(!(deserializeJson(doc, Serial2))) {
        espnow_packet packet;
        String buffer = String((const char *)doc["mac"]);
        const char *str = buffer.c_str();
        for(int i = 0; i < 6; i++) {
          uint8_t val = 0;
          uint8_t temp = 0;
          char first = *(str + 3*i);
          char second = *(str + 3*i + 1);
          temp = (first >= 'A') ? (first - 'A' + 10) : (first - '0');
          val += temp*16;
          temp = (second >= 'A') ? (second - 'A' + 10) : (second - '0');
          val += temp;
          packet.mac[i] = val;
        }
        serializeJson(doc, packet.message);
        xQueueSend(espnow_send_queue, &packet, portMAX_DELAY);
      } else {
      vTaskDelay(10 / portTICK_PERIOD_MS);
      }
    }
  }
}

void setup() {
  Serial2.begin(115200);
  Serial2.setTimeout(100);
  espnow_send_queue = xQueueCreate(10, sizeof(espnow_packet));
  espnow_recv_queue = xQueueCreate(10, sizeof(espnow_packet));
  uart_recv_queue = xQueueCreate(10, sizeof(String));
  while(!config_esp_now()) {
    vTaskDelay(200 / portTICK_PERIOD_MS);
  };
  xTaskCreate(process_uart_msg_task, "process uart", 4096, NULL, 10, NULL);
  xTaskCreate(process_espnow_packet_task, "process espnow", 4096, NULL, 10, NULL);
  xTaskCreate(send_espnow_packet_task, "send espnow", 4096, NULL, 10, NULL);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  vTaskDelete(NULL);
}

void loop() {

}