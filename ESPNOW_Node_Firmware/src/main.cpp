#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include <espnow.h>
#include <ArduinoJson.h>
#include <StreamUtils.h>

#define ON                  1
#define OFF                 0

//define pin
#define EEPROM_PIN          14
#define SENSOR_SOURCE_PIN   12
#define SENSOR_DATA_PIN     13
#define EEPROM_USAGE        512

#define SEND_RETRY_TIME     10


uint8_t broadcast_addr[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
typedef struct infor 
{
  String mac;
  String type;
  uint8_t interval;
  String topic;
} infor;
infor   device_infor;
bool    get_infor_result;
bool    send_result;


void send_msg_isr(u8 *mac_addr, u8 status) {
  send_result = (status==0)?true:false;
}

void infor_recv_isr(u8 *mac_addr, u8 *data, u8 len) {
  DynamicJsonDocument doc(250);
  DeserializationError error = deserializeJson(doc, data);
  if(!error) {
    String from     =   String((const char *)doc["from"]);
    String to       =   String((const char *)doc["to"]);
    String purpose  =   String((const char *)doc["purpose"]);
    if( (from == "node") && (to == "subnode") && (purpose == "add subnode")) {
      DynamicJsonDocument infor_doc(EEPROM_USAGE);
      infor_doc["mac addr"] = String((const char *)mac_addr);
      infor_doc["type"] = String((const char*)doc["type"]);
      infor_doc["interval"] = (uint8_t)doc["interval"];
      infor_doc["topic"] = String((const char*)doc["topic"]);
      EepromStream infor(0, EEPROM_USAGE);
      EEPROM.begin(EEPROM_USAGE);
      serializeJson(infor_doc, infor);
      EEPROM.commit();
      EEPROM.end();
      get_infor_result = true;
    }
  }
}


bool check_eeprom_pin(uint8_t pin) {
  pinMode(pin, INPUT_PULLUP);
  delay(5);
  bool check = digitalRead(pin)?false:true;
  return check;
}

void reset_eeprom(int eeprom_size) {
  EEPROM.begin(eeprom_size);
  for( int i = 0; i < eeprom_size; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.end();
}

void blink_led(int times) {
  if(times == 0) {
    while (1)
    {
      digitalWrite(LED_BUILTIN, LOW); //sáng
      delay(200);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
    }
  }
  for(int i = 0; i<times; i++) {
    digitalWrite(LED_BUILTIN, LOW); //sáng
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
  }
}

void config_esp_now(esp_now_send_cb_t send, esp_now_recv_cb_t recv) {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_now_init();
  esp_now_register_send_cb(send);
  esp_now_register_recv_cb(recv);
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
}

void get_device_infor(int eeprom_size, infor *infor) {
  EEPROM.begin(eeprom_size);
  DynamicJsonDocument doc(eeprom_size);
  EepromStream infor_in_eeprom(0, eeprom_size);
  DeserializationError error = deserializeJson(doc, infor_in_eeprom);
  EEPROM.end();
  if(!error) {
    infor->type = String((const char *)doc["type"]);
    infor->interval = (uint8_t)doc["interval"];
    infor->mac = String((const char *)doc["mac addr"]);
    infor->topic = String((const char *)doc["topic"]);
  } else {
    infor->type = "";
    infor->interval = 0;
    infor->mac = "";
    infor->topic = "";
  }
}

void waiting_infor() {
  get_infor_result = false;
  while(!get_infor_result){
    blink_led(1);
  }
  esp_now_unregister_recv_cb();
  get_device_infor(EEPROM_USAGE, &device_infor);
}

void config_sensor_gpio(String type) {
  pinMode(SENSOR_SOURCE_PIN, OUTPUT);
  digitalWrite(SENSOR_SOURCE_PIN, ON);
  if(type == "flame") {
    pinMode(SENSOR_DATA_PIN, INPUT);
    delay(100);
  } else if(type == "flood") {
    pinMode(SENSOR_DATA_PIN, INPUT);
    delay(100);
  }
}

bool is_sensor_active(String type) {
  bool ret;
  if(type == "flame") {
    int value = digitalRead(SENSOR_DATA_PIN);
    ret = value?false:true;
    return ret;
  } else if(type == "flood") {
    int value = digitalRead(SENSOR_DATA_PIN);
    ret = value?false:true;
    return ret;
  } else return false;
}

void send_infor_msg() {
  esp_now_add_peer((uint8_t *)device_infor.mac.c_str(), ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  DynamicJsonDocument doc(250);
  String buffer;
  doc["from"] = "subnode";
  doc["to"] = "node";
  doc["purpose"] = "send data";
  doc["topic"] = device_infor.topic;
  doc["type"] = device_infor.type;
  if(device_infor.type == "flame") {
    doc["payload"] = "FLAME DETECTED!";
  } else if(device_infor.type == "flood") {
    doc["payload"] = "FLOOD DETECTED!";
  }
  serializeJson(doc, buffer);
  send_result = false;
  int retry_time = 0;
  while (send_result != true) {
    esp_now_send((uint8_t *)device_infor.mac.c_str(), (uint8_t *)buffer.c_str(), buffer.length());
    blink_led(1);
    retry_time++;
    if(retry_time >= SEND_RETRY_TIME) {
      blink_led(0);
    }
  }
}

void config_onboard_led(uint8_t pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
}

void config_reset_eeprom_pin(uint8_t pin) {
  pinMode(pin, INPUT_PULLUP);
  delay(5);
}

bool is_infor_valid(infor infor) {
  bool ret = false;
  if((infor.mac == "") || (infor.topic == "") || (infor.type == "")) {
    ret = false;
  } else {
    ret = true;
  }
  return ret;
}

void send_confirm_msg() {
  esp_now_add_peer((uint8_t *)device_infor.mac.c_str(), ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  DynamicJsonDocument doc(250);
  String buffer;
  doc["from"] = "subnode";
  doc["to"] = "node";
  doc["purpose"] = "add success";
  serializeJson(doc, buffer);
  send_result = false;
  int retry_time = 0;
  while ((!send_result)) {
    esp_now_send((uint8_t *)device_infor.mac.c_str(), (uint8_t *)buffer.c_str(), buffer.length());
    blink_led(1);
    retry_time++;
    if(retry_time >= SEND_RETRY_TIME) {
      blink_led(0);
    }
  }
}

void setup() {
  Serial.begin(115200);
  config_onboard_led(LED_BUILTIN);
  config_reset_eeprom_pin(EEPROM_PIN);
  if(digitalRead(EEPROM_PIN) == 0){
    reset_eeprom(EEPROM_USAGE);
    blink_led(5);
    ESP.restart();
  }
  get_device_infor(EEPROM_USAGE, &device_infor);
  if(is_infor_valid(device_infor) == false) {
    config_esp_now(send_msg_isr, infor_recv_isr);
    waiting_infor();
    get_device_infor(EEPROM_USAGE, &device_infor);
    send_confirm_msg();
    ESP.restart();
  }
  config_sensor_gpio(device_infor.type);
  if(!is_sensor_active(device_infor.type)) {
    blink_led(1);
    ESP.deepSleep(device_infor.interval*1e6);
  }
  config_esp_now(send_msg_isr, NULL);
  send_infor_msg();
  blink_led(1);
  if( (device_infor.type == "flame") || (device_infor.type == "flood") ) {
    ESP.deepSleep(0);
  }
  ESP.deepSleep( device_infor.interval*1e6 );
}

void loop() {
  //never get here
}


