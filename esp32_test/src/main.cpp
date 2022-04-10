#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ArduinoJson.h>

esp_now_peer_info peer;
String recv_data;
uint8_t broadcast[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

TaskHandle_t input_task_handle;
TaskHandle_t recv_task_handle;

void IRAM_ATTR input_isr() {
  vTaskNotifyGiveFromISR(input_task_handle, pdFALSE);
}

void recv_cb(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  Serial.println(data_len, DEC);
  recv_data = String((const char *)data);
  vTaskNotifyGiveFromISR(recv_task_handle, pdFALSE);
}

void input_task(void *pv) {
  DynamicJsonDocument doc(250);
  String buffer;
  doc["from"] = "node";
  doc["to"] = "subnode";
  doc["purpose"] = "add subnode";
  doc["type"] = "flame";
  doc["interval"] = 5;
  doc["topic"] = "flame alert";
  serializeJson(doc, buffer);
  while(1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    esp_now_send(broadcast, (uint8_t *)buffer.c_str(), buffer.length());
    Serial.println("send");
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void recv_task(void *pv) {
  while(1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    Serial.println(recv_data);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Start!");
  pinMode(39, INPUT_PULLDOWN);
  attachInterrupt(39, input_isr, RISING);
  WiFi.mode(WIFI_MODE_STA);
  WiFi.disconnect();
  esp_now_init();
  peer.channel = 1;
  peer.encrypt = false;
  memcpy(peer.peer_addr, broadcast, 6);
  esp_now_add_peer(&peer);
  esp_now_register_recv_cb(recv_cb);
  xTaskCreate(input_task, "input task", 4096, NULL, 10, &input_task_handle);
  xTaskCreate(recv_task, "recv task", 4096, NULL, 10, &recv_task_handle);
  vTaskDelete(NULL);
}

void loop() {
  Serial.println("loop");
  vTaskDelay( 500 / portTICK_PERIOD_MS);
}