#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include <espnow.h>

#define CLEAR_EEPROM_PIN    4
#define SENSOR_SWICH        12
#define SENSOR_PIN          13
#define SENSOR_READY_DELAY  500
#define EEPROM_USAGE        32

//Message code
#define PING_REQUEST        0x00
#define PING_REPLY          0x01
#define TRANS_ALERT         0x02
#define CONFIRM_ALERT       0x03

uint8_t broadcast_addr[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
uint8_t para_exist_eeprom_addr = 0;
uint8_t des_mac_eeprom_addr = 1;
uint8_t des_mac[6];
bool ping_reply;
bool send_alert_result;
bool confirm_alert_result;

void blink_led(int times) {
  for(int i = 0; i<times; i++) {
    digitalWrite(LED_BUILTIN, LOW); //sáng
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
  }
}

void send_msg_cb(u8 *mac_addr, u8 status) {
  if(memcmp(mac_addr, des_mac, 6) == 0) {
    send_alert_result = (status==0)?true:false;
  }
}

void recv_msg_cb(u8 *mac_addr, u8 *data, u8 len) {
    uint8_t code = data[0];
    uint8_t payload[len - 1];
    memcpy(payload, &data[1], len - 1);
    switch (code)
    {
    case PING_REQUEST:
      break;
    case PING_REPLY:
      ping_reply = true;
      EEPROM.begin(EEPROM_USAGE);
      for (int i = 0; i < 6; i++) {
        EEPROM.write(des_mac_eeprom_addr + i, mac_addr[i]);
      }
      EEPROM.write(para_exist_eeprom_addr, 1);
      EEPROM.commit();
      EEPROM.end();
      break;
    case TRANS_ALERT:
      break;
    case CONFIRM_ALERT:
      confirm_alert_result = (memcmp(des_mac, mac_addr, 6) == 0)?true:false;
      break;
    }
}

void reset_eeprom() {
  EEPROM.begin(EEPROM_USAGE);
  for( int i = 0; i < EEPROM_USAGE; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.end();
  blink_led(3);
}

void setup_esp_now() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_now_init();
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_send_cb(send_msg_cb);
  esp_now_register_recv_cb(recv_msg_cb);
}

bool check_parameter() {
  uint8_t check_result = 0;
  EEPROM.begin(EEPROM_USAGE);
  check_result = EEPROM.read(para_exist_eeprom_addr);
  EEPROM.end();
  if(check_result == 0) {
    return false;
  }
  else return true;
}

void ping_master() {
  esp_now_add_peer(broadcast_addr, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  ping_reply = false;
  struct ping_msg
  {
    uint8_t code;
    uint8_t payload;
  } ping_msg;
  ping_msg.code = PING_REQUEST;
  ping_msg.payload = random(0, 256);
  while (ping_reply != true) {
    esp_now_send(broadcast_addr, (uint8_t *)&ping_msg, sizeof(ping_msg));
    delay(500);
  }
}

void get_mac_addr() {
  EEPROM.begin(EEPROM_USAGE);
  for (int i = 0; i < 6; i++) {
    des_mac[i] = EEPROM.read(des_mac_eeprom_addr + i);
  }
  EEPROM.end();
}

void send_esp_now() {
  esp_now_add_peer(des_mac, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  struct send_msg {
    uint8_t code;
    uint8_t data;
  } send_msg;
  send_msg.code = TRANS_ALERT;
  send_msg.data = random(0, 256);
  send_alert_result = false;
  confirm_alert_result = false;
  while (send_alert_result != true) {
    esp_now_send(des_mac, (uint8_t *)&send_msg, sizeof(send_msg));
    delay(500);
  }
}

void setup() {
  //set up pin
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(SENSOR_SWICH, OUTPUT);
  digitalWrite(SENSOR_SWICH, HIGH);         //power for sensor
  delay(SENSOR_READY_DELAY);
  pinMode(SENSOR_PIN, INPUT);
  pinMode(CLEAR_EEPROM_PIN, INPUT_PULLUP);
  //check up for clear eeprom mode
  if(digitalRead(CLEAR_EEPROM_PIN) == 0) {
    reset_eeprom();
    ESP.deepSleep(0);
  }
  //check up sensor is active
  if(digitalRead(SENSOR_PIN) != 0) {
    setup_esp_now();
    if(check_parameter() != true) {
      ping_master();
    }
    get_mac_addr();
    send_esp_now();
    while(confirm_alert_result != true)
      delay(500);
  }
  ESP.deepSleep(5e6);
}

void loop() {
  //never get here
}