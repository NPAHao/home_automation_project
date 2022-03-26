#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include <espnow.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>


//define device type
#define FLAME_SENSOR        0x01
#define FLOOD_SENSOR        0x02
#define PRESENT_SENSOR      0x03
#define DOOR_SENSOR         0x04
#define DHT11_SENSOR        0x05
#define GAS_SENSOR          0x06

//define pin
#define CLEAR_EEPROM_PIN    4
#define SENSOR_SWICH        12
#define SENSOR_PIN          13
#define SENSOR_READY_DELAY  500
#define EEPROM_USAGE        32
#define DHTTYPE             DHT11

//define esp now message code
#define PING_REQUEST        0x00
#define PING_REPLY          0x01
#define TRANS_ALERT         0x02
#define CONFIRM_ALERT       0x03

uint8_t device_type;
uint8_t broadcast_addr[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
uint8_t para_exist_eeprom_addr = 0;
uint8_t des_mac_eeprom_addr = 1;
uint8_t device_type_eeprom_addr = 7;
uint8_t des_mac[6];
bool ping_reply;
bool send_alert_result;
bool confirm_alert_result;

void blink_led(int times);

void send_msg_cb(u8 *mac_addr, u8 status);

void recv_msg_cb(u8 *mac_addr, u8 *data, u8 len);

void reset_eeprom();

void setup_esp_now();

void setup_sensor_gpio(uint8_t type);

bool check_parameter();

bool check_sensor_is_active(uint8_t type);

void ping_master();

void get_mac_addr(uint8_t *mac_addr);

void get_device_type(uint8_t *dv_type);

void send_esp_now(uint8_t type);

void setup() {
  device_type = FLAME_SENSOR;
  //set up pin
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(CLEAR_EEPROM_PIN, INPUT_PULLUP);
  //check up for clear eeprom mode
  if(digitalRead(CLEAR_EEPROM_PIN) == 0) {
    reset_eeprom();
    blink_led(3);
    ESP.deepSleep(0);
  }
  //check up for is parameter already exist
  if(check_parameter() != true) {
    setup_esp_now();
    ping_master();
  }
  get_device_type(&device_type);
  setup_sensor_gpio(device_type);
  //check up sensor is active
  if(check_sensor_is_active(device_type)) {
    setup_esp_now();
    get_mac_addr(des_mac);
    send_esp_now(device_type);
    while(confirm_alert_result != true)
      delay(500);
  }
  ESP.deepSleep(5e6);
}

void loop() {
  //never get here
}

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
  char *buff = (char *)data;
  DynamicJsonDocument doc(256);
  String jsondata = String(buff);
  DeserializationError error = deserializeJson(doc, jsondata);
  if(!error) {
    uint8_t code = doc["code"];
    switch (code)
    {
      case PING_REQUEST:
        break;
      case PING_REPLY:
      {
        ping_reply = true;
        EEPROM.begin(EEPROM_USAGE);
        for (int i = 0; i < 6; i++) {
          EEPROM.write(des_mac_eeprom_addr + i, mac_addr[i]);
        }
        uint8_t dv_type = doc["type"];
        EEPROM.write(device_type_eeprom_addr, dv_type);
        EEPROM.write(para_exist_eeprom_addr, 1);
        EEPROM.commit();
        EEPROM.end();
        break;
      }
      case TRANS_ALERT:

        break;
      case CONFIRM_ALERT:
      {
        confirm_alert_result = (memcmp(des_mac, mac_addr, 6) == 0)?true:false;
        break;
      }
    }
  }
}

void reset_eeprom() {
  EEPROM.begin(EEPROM_USAGE);
  for( int i = 0; i < EEPROM_USAGE; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.end();
}

void setup_esp_now() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_now_init();
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_send_cb(send_msg_cb);
  esp_now_register_recv_cb(recv_msg_cb);
}

void setup_sensor_gpio(uint8_t type) {
  switch (type)
  {
  case FLAME_SENSOR:
    //
    break;
  case FLOOD_SENSOR:
  //
    break;
  case PRESENT_SENSOR:
  //
    break;
  case DOOR_SENSOR:
  //
    break;
  case DHT11_SENSOR:
  //
    break;
  case GAS_SENSOR:
  //
    break;
  }
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

bool check_sensor_is_active(uint8_t type) {
  switch (type)
  {
  case FLAME_SENSOR:
    return false;
  case FLOOD_SENSOR:
    return false;
  case PRESENT_SENSOR:
    return false;
  case DOOR_SENSOR:
    return false;
    break;
  case DHT11_SENSOR:
    return true;
  case GAS_SENSOR:
    return false;
  }
  return false;
}

void ping_master() {
  esp_now_add_peer(broadcast_addr, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  ping_reply = false;
  DynamicJsonDocument doc(256);
  String jsondata = "";
  doc["code"] = PING_REQUEST;
  doc["payload"] = 0xff;
  serializeJson(doc, jsondata);
  while (ping_reply != true) {
    esp_now_send(broadcast_addr, (uint8_t *)jsondata.c_str(), sizeof(jsondata) + 2);
    delay(500);
  }
}

void get_mac_addr(uint8_t *mac_addr) {
  EEPROM.begin(EEPROM_USAGE);
  for (int i = 0; i < 6; i++) {
    *(mac_addr + i) = EEPROM.read(des_mac_eeprom_addr + i);
  }
  EEPROM.end();
}
void get_device_type(uint8_t *dv_type) {
  EEPROM.begin(EEPROM_USAGE);
  *dv_type = EEPROM.read(device_type_eeprom_addr);
  EEPROM.end();
}

void send_esp_now(uint8_t type) {
  esp_now_add_peer(des_mac, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  DynamicJsonDocument doc(256);
  String jsondata = "";
  doc["code"] = TRANS_ALERT;
  doc["payload"] = 0xff;
  serializeJson(doc, jsondata);
  send_alert_result = false;
  confirm_alert_result = false;
  while (send_alert_result != true) {
    esp_now_send(des_mac, (uint8_t *)jsondata.c_str(), sizeof(jsondata) + 2);
    delay(500);
  }
}