#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include <espnow.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <StreamUtils.h>


//define device type
#define FLAME_SENSOR        0x01
#define FLOOD_SENSOR        0x02
#define PRESENT_SENSOR      0x03
#define DOOR_SENSOR         0x04
#define DHT11_SENSOR        0x05
#define GAS_SENSOR          0x06

//define pin
#define EEPROM_PIN          4
#define SENSOR_SWICH_PIN    12
#define SENSOR_PIN          13
#define SENSOR_READY_DELAY  500
#define EEPROM_USAGE        512
#define DHTTYPE             DHT11

//define esp now message code
#define INFOR_REQUEST       0x00    //send
#define INFOR_RESPOND       0x01    //receive
#define MSG_SEND            0x02    //send
#define MSG_CONFIRM         0x03    //receive

uint8_t broadcast_addr[]          = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
uint8_t device_type;
uint8_t mac_address[6];
String  publish_topic_name;
bool    infor_responded;
bool    send_result;
bool    confirm_result;

EepromStream infor(0, EEPROM_USAGE);

void blink_led(int times);

bool check_eeprom_pin(uint8_t eeprom_pin);

bool check_infor();

void send_msg_cb(u8 *mac_addr, u8 status);

void recv_msg_cb(u8 *mac_addr, u8 *data, u8 len);

void reset_eeprom();

void setup_esp_now();

void setup_sensor_gpio(uint8_t type);

bool check_sensor_is_active(uint8_t type);

void send_infor_request_msg();

void get_device_infor();

void send_esp_now(uint8_t type, String topic);

void setup() {
  if(check_eeprom_pin(EEPROM_PIN)) {
    reset_eeprom();
  }
  if(check_infor() != true) {
    setup_esp_now();
    send_infor_request_msg();
  }
  //B2: (check infor)?(tiếp tục):(yêu cầu infor-chờ và ghi infor vào eeprom)
  //B3: cài đặt gpio theo kiểu cảm biến
  //B4: (cảm biến tích cực)?(setup esnow-gửi msg):(deepsleep theo số giây)
}

void loop() {
  //never get here
}

/**
 * @return true to clear, false to continue
 */
bool check_eeprom_pin(uint8_t eeprom_pin) {
  pinMode(eeprom_pin, INPUT_PULLUP);
  delay(5);
  bool check = digitalRead(eeprom_pin)?false:true;
  return check;
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
  if(memcmp(mac_addr, mac_address, 6) == 0) {
    send_result = (status==0)?true:false;
  }
}

void recv_msg_cb(u8 *mac_addr, u8 *data, u8 len) {
  String mac = String((char *)mac_addr);
  char *buff = (char *)data;
  DynamicJsonDocument doc(256);
  String jsondata = String(buff);
  DeserializationError error = deserializeJson(doc, jsondata);
  if(!error) {
    uint8_t code = doc["code"];
    switch (code)
    {
      case INFOR_RESPOND:
      {
        infor_responded = true;
        EEPROM.begin(EEPROM_USAGE);
        DynamicJsonDocument sudo_doc(256);
        sudo_doc["device_type"] = doc["device_type"];
        sudo_doc["mac_addr"] = 1;
        // lưu độ dài chuỗi json vào address 0
        // lưu chuỗi json về loại cảm biến, địa chỉ mac, publish topic vào eeprom   
        EEPROM.commit();
        EEPROM.end();
        break;
      }
      case MSG_CONFIRM:
      {
        confirm_result = (memcmp(mac_address, mac_addr, 6) == 0)?true:false;
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


/**
 * @return true if infor exists, false if infor is empty
 */
bool check_infor() {
  uint8_t check_result = 0;
  EEPROM.begin(EEPROM_USAGE);
  check_result = EEPROM.read(0);
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

void send_infor_request_msg() {
  esp_now_add_peer(broadcast_addr, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  infor_responded = false;
  DynamicJsonDocument doc(256);
  String jsondata;
  doc["code"] = INFOR_REQUEST;
  doc["payload"] = "infor";
  serializeJson(doc, jsondata);
  while (infor_responded != true) {
    esp_now_send(broadcast_addr, (uint8_t *)jsondata.c_str(), sizeof(jsondata) + 2);
    delay(500);
  }
}

void get_device_infor(uint8_t *mac_addr) {

}

void send_esp_now(uint8_t type, String topic) {
  esp_now_add_peer(mac_address, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  DynamicJsonDocument doc(256);
  String jsondata = "";
  doc["code"] = MSG_SEND;
  doc["topic"] = topic;
  serializeJson(doc, jsondata);
  send_result = false;
  confirm_result = false;
  while (send_result != true) {
    esp_now_send(mac_address, (uint8_t *)jsondata.c_str(), sizeof(jsondata) + 2);
    delay(500);
  }
}