#include <Arduino.h>
#include <espnow.h>
#include <ESP8266WiFi.h>

//esp now msg code
#define PING_REQUEST        0x00        //receive
#define PING_REPLY          0x01        //send
#define TRANS_ALERT         0x02        //receive
#define CONFIRM_ALERT       0x03        //send

//uart msg code
#define FW_MSG              0x00        //send
#define GET_PEER            0x01        //send
#define ADD_PEER            0x02        //receive
#define DEL_PEER            0x03        //receive
#define SEND_DONE           0x04        //receive
#define SEND_ESPNOW         0x05        //receive

bool send_result;
bool peer_list_done = false;

void send_cb(u8 *mac_addr, u8 status) {

}

void recv_cb(u8 *mac_addr, u8 *data, u8 len) {
  if( esp_now_is_peer_exist(mac_addr) == 1) {
    uint8_t code = data[0];
    uint8_t payload[len-1];
    memcpy(payload, data + 1, len - 1);
    if(code == PING_REQUEST) {
      reply_to_ping_request(mac_addr);
    } else {
      forward_espnow_msg_to_uart(mac_addr, data, len);
    }
  }
}

void add_peer_list() {
  //send get peer list cmd
  uint8_t msg[2];
  msg[0] = 1;
  msg[1] = GET_PEER;
  Serial.write(msg, 2);
  while (peer_list_done != true) {   //wait until get the entire peer list
    if(Serial.available() >0) {
      handle_rx_msg_from_uart();
    }
  }
}

void setup_espnow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_now_init();
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  add_peer_list();
  esp_now_register_send_cb(send_cb);
  esp_now_register_recv_cb(recv_cb);
}

void reply_to_ping_request(uint8_t *mac_addr) {
  struct msg 
  {
    uint8_t code = PING_REPLY;
    uint8_t payload = random(0, 256);
  } msg;
  esp_now_send(mac_addr, (uint8_t *)&msg, sizeof(msg));
}

void handle_rx_msg_from_uart() {
  uint8_t msg_len = Serial.read();
  uint8_t msg[msg_len];
  Serial.readBytes(msg, msg_len);
//UART msg :   [code][6 x MAC][data.....][data_len]
  uint8_t code = msg[0];
  switch (code)
  {
  case ADD_PEER:
    uint8_t mac_addr[6];
    memcpy(mac_addr, &msg[1], 6);
    if(esp_now_is_peer_exist(mac_addr) == 0) {
      esp_now_add_peer(mac_addr, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
    }
    break;
  case DEL_PEER:
    uint8_t mac_addr[6];
    memcpy(mac_addr, &msg[1], 6);
    if(esp_now_is_peer_exist(mac_addr) == 1) {
      esp_now_del_peer(mac_addr);
    }
    break;
  case SEND_DONE:
    peer_list_done = true;
    break;
  case SEND_ESPNOW:
    uint8_t mac_addr[6];
    memcpy(mac_addr, &msg[1], 6);

    uint8_t data_len = msg[msg_len - 1];

    uint8_t data[data_len];
    memcpy(data, &msg[7], data_len);

    esp_now_send(mac_addr, data, data_len);
    break;
  }
}

void forward_espnow_msg_to_uart(u8 *mac_addr, u8 *data, u8 len) {
  uint8_t fw_msg[1+1+6+len+1];      //msg length(1byte) + msg code(1byte) + mac addr (6byte) + data(len byte) +data length(1byte)
  fw_msg[0] = 1 + 6 + len + 1;
  fw_msg[1] = FW_MSG;
  memcpy(&fw_msg[2], mac_addr, 6);
  memcpy(&fw_msg[8] , data, len);
  fw_msg[8 + len] = len;
  Serial.write(fw_msg, 1+1+6+len+1);
}

void setup() {
  Serial.begin(115200);
  setup_espnow();
}

void loop() {
  if(Serial.available() >0 ) {
    handle_rx_msg_from_uart();
  }
}