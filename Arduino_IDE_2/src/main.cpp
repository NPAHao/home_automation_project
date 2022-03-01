#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

typedef struct struct_message {
  uint8_t code;
} espnow_msg;

espnow_msg send_msg;
esp_now_peer_info_t peerInfo;
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
constexpr char WIFI_SSID[] = "Phi Hung";

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void setup_espnow();
int32_t getWiFiChannel(const char *ssid);

void setup() {
  Serial.begin(115200);
  send_msg.code = 0;
  int32_t channel = getWiFiChannel(WIFI_SSID);

  WiFi.printDiag(Serial); // Uncomment to verify channel number before
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  WiFi.printDiag(Serial); // Uncomment to verify channel change after
  setup_espnow();
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  delay(2000);
  send_msg.code = (send_msg.code)?0:1;
  digitalWrite(LED_BUILTIN,send_msg.code);
  esp_now_send(broadcastAddress, (uint8_t *)&send_msg, sizeof(send_msg));
}

void setup_espnow() {
  WiFi.mode(WIFI_STA);
  Serial.println(WiFi.macAddress());
  if(esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
  }
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if(status == 0)
    Serial.println("Send success.");
    else Serial.println("Send fail.");
}

int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
      for (uint8_t i=0; i<n; i++) {
          if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
              return WiFi.channel(i);
          }
      }
  }
  return 0;
}
