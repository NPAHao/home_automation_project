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

//uart field length
#define PAYLOAD_LEN_BYTE    1
#define MSG_CODE            1
#define MAC_LEN             6
#define DATA_LEN_BYTE       1

bool send_result;
bool peer_list_done = false;

void send_cb(u8 *mac_addr, u8 status);

void recv_cb(u8 *mac_addr, u8 *data, u8 len);

void add_peer_list();

void setup_espnow();

void reply_to_ping_request(uint8_t *mac_addr);

void handle_rx_msg_from_uart();

void forward_espnow_msg_to_uart(u8 *mac_addr, u8 *data, u8 len);

void setup() {
  Serial.begin(115200);
  delay(2000);
  setup_espnow();
}

void loop() {
  if(Serial.available() >0 ) {
    handle_rx_msg_from_uart();
  }
}

void send_cb(u8 *mac_addr, u8 status) {
}

void recv_cb(u8 *mac_addr, u8 *data, u8 len) {
  if( esp_now_is_peer_exist(mac_addr) == 1) {
    //esp now data format: [code][content....]
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
    delay(100);
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
//UART msg format :   [payload_len][code][6 x MAC][data.....][data_len]
  uint8_t payload_len = Serial.read();
//UART msg now in buffer :         [code][6 x MAC][data.....][data_len]
  uint8_t msg[payload_len];
  Serial.readBytes(msg, payload_len);
  uint8_t code = msg[0];
  uint8_t mac_addr[6];  
  switch (code)
  {
  case ADD_PEER:
    memcpy(mac_addr, &msg[1], 6);
    if(esp_now_is_peer_exist(mac_addr) == 0) {
      esp_now_add_peer(mac_addr, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
    }
    break;
  case DEL_PEER:
    memcpy(mac_addr, &msg[1], 6);
    if(esp_now_is_peer_exist(mac_addr) == 1) {
      esp_now_del_peer(mac_addr);
    }
    break;
  case SEND_DONE:
    peer_list_done = true;
    break;
  case SEND_ESPNOW:
    memcpy(mac_addr, &msg[1], 6);

    uint8_t data_len = msg[payload_len - 1];

    uint8_t data[data_len];
    memcpy(data, &msg[7], data_len);

    esp_now_send(mac_addr, data, data_len);
    break;
  }
}

void forward_espnow_msg_to_uart(u8 *mac_addr, u8 *data, u8 len) {
  //[payload_len][code][6 x MAC][data.....][data_len]
  uint8_t msg_len = PAYLOAD_LEN_BYTE + MSG_CODE + MAC_LEN + len + DATA_LEN_BYTE;
  //             [code][6 x MAC][data.....][data_len]
  uint8_t payload_len =                MSG_CODE + MAC_LEN + len + DATA_LEN_BYTE;

  //msg format: [payload_len][msg_code][6xMAC][len x data][len]
  uint8_t msg[msg_len];               //uart msg

  msg[0] = payload_len;               //payload len byte

  msg[1] = FW_MSG;                    //msg code byte

  memcpy(&msg[2], mac_addr, 6);       //mac addr field

  memcpy(&msg[8] , data, len);        //data field

  msg[msg_len - 1] = len;             //data len byte

  Serial.write(msg, msg_len);
}