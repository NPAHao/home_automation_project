#include <Arduino.h>
#include <espnow.h>
#include <ESP8266WiFi.h>

#define PING_REQUEST        0x00
#define PING_REPLY          0x01
#define TRANS_ALERT         0x02
#define CONFIRM_ALERT       0x03

bool send_result;

void send_cb(u8 *mac_addr, u8 status) {

}

void recv_cb(u8 *mac_addr, u8 *data, u8 len) {
  uint8_t code = data[0];
  uint8_t payload[len-1];
  memcpy(payload, data + 1, len - 1);
  if(code == PING_REQUEST) {
    reply_to_ping_request(mac_addr);
  } else {
    forward_espnow_msg_to_uart(mac_addr, data, len);
  }
}

bool get_peer_list() {
  //outcome is created peer list
  return true;
}

void setup_espnow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_now_init();
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  //esp now add peer from peer list
  esp_now_register_send_cb(send_cb);
  esp_now_register_recv_cb(recv_cb);
}

void reply_to_ping_request(uint8_t *mac_addr) {
  if(esp_now_is_peer_exist(mac_addr) == 1) {
    struct msg {
      uint8_t code = PING_REPLY;
      uint8_t payload = random(0, 256);
    } msg;
    esp_now_send(mac_addr, (uint8_t *)&msg, sizeof(msg));
  }
}

void handle_rx_msg_from_uart() {
  //save peer infor to peer list
  //add peer
  //delete peer
  //send espnow msg
}

void forward_espnow_msg_to_uart(u8 *mac_addr, u8 *data, u8 len) {

}

void setup() {
  Serial.begin(115200);
  while(get_peer_list() != true);
  setup_espnow();
}

void loop() {
  // put your main code here, to run repeatedly:
}