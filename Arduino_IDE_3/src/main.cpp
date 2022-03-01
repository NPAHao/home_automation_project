#include <Arduino.h>
#include <espnow.h>
#include <ESP8266WiFi.h>

typedef struct struct_message {
  uint8_t code;
} espnow_msg;

espnow_msg send_msg;
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
constexpr char WIFI_SSID[] = "Phi Hung";

int32_t getWiFiChannel(const char *ssid);
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus);

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  int32_t channel = getWiFiChannel(WIFI_SSID);

  WiFi.printDiag(Serial); // Uncomment to verify channel number before
  wifi_promiscuous_enable(1);
  wifi_set_channel(channel);
  wifi_promiscuous_enable(0);
  WiFi.printDiag(Serial); // Uncomment to verify channel change after

  if (esp_now_init() != 0) 
    Serial.println("Error initializing ESP-NOW");
    
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);

  esp_now_register_send_cb(OnDataSent);
  
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 0, NULL, 0);

  send_msg.code = 0;

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  delay(2000);
  send_msg.code = (send_msg.code)?0:1;
  digitalWrite(LED_BUILTIN, send_msg.code);
  esp_now_send(broadcastAddress, (uint8_t *)&send_msg, sizeof(send_msg));
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

void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    Serial.println("Delivery success");
  }
  else{
    Serial.println("Delivery fail");
  }
}