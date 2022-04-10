#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include <espnow.h>
#include <ArduinoJson.h>
#include <StreamUtils.h>

#define ON                  1
#define OFF                 0

//define pin
#define EEPROM_PIN          4
#define SENSOR_SOURCE_PIN   12
#define SENSOR_DATA_PIN     13
#define EEPROM_USAGE        512


uint8_t broadcast_addr[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
String mac_address;
String device_type;
uint8_t interval;
String topic;
bool    get_infor_result;
bool    send_result;


void send_msg_cb(u8 *mac_addr, u8 status) {
  send_result = (status==0)?true:false;
}

void recv_msg_cb(u8 *mac_addr, u8 *data, u8 len) {
  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, data);
  if(!error) {
    String from     =   String((const char *)doc["from"]);
    String to       =   String((const char *)doc["to"]);
    String purpose  =   String((const char *)doc["purpose"]);
    if( (from == "node") && (to == "subnode") && (purpose == "add subnode")) {
      doc["mac addr"] = String((const char *)mac_addr);
      EepromStream infor(0, EEPROM_USAGE);
      EEPROM.begin(EEPROM_USAGE);
      serializeJson(doc, infor);
      EEPROM.commit();
      EEPROM.end();
      get_infor_result = true;
    }
  }
}


bool check_eeprom_pin() {
  pinMode(EEPROM_PIN, INPUT_PULLUP);
  delay(5);
  bool check = digitalRead(EEPROM_PIN)?false:true;
  return check;
}

void reset_eeprom() {
  EEPROM.begin(EEPROM_USAGE);
  for( int i = 0; i < EEPROM_USAGE; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.end();
}

void blink_led(int times) {
  for(int i = 0; i<times; i++) {
    digitalWrite(LED_BUILTIN, LOW); //sáng
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
  }
}

void setup_esp_now() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_now_init();
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
}

bool get_device_infor() {
  EEPROM.begin(EEPROM_USAGE);
  DynamicJsonDocument doc(512);
  EepromStream infor_in_eeprom(0, EEPROM_USAGE);
  DeserializationError error = deserializeJson(doc, infor_in_eeprom);
  EEPROM.end();
  if(!error) {
    device_type = String((const char *)doc["type"]);
    interval = doc["interval"];
    mac_address = String((const char *)doc["mac addr"]);
    topic = String((const char *)doc["topic"]);
  }
  bool ret = (device_type == "") || (mac_address == "") || (topic == "");
  return !ret;
}

void waiting_infor() {
  esp_now_register_recv_cb(recv_msg_cb);
  esp_now_register_send_cb(send_msg_cb);
  get_infor_result = false;
  while(!get_infor_result){
    blink_led(1);
  }
  get_device_infor();
  esp_now_add_peer((uint8_t *)mac_address.c_str(), ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  esp_now_register_send_cb(send_msg_cb);
  DynamicJsonDocument doc(250);
  String buffer;
  doc["from"] = "subnode";
  doc["to"] = "node";
  doc["purpose"] = "add success";
  serializeJson(doc, buffer);
  send_result = false;
  while (!send_result) {
    esp_now_send((uint8_t *)mac_address.c_str(), (uint8_t *)buffer.c_str(), buffer.length());
    blink_led(1);
  }
  esp_now_del_peer((uint8_t *)mac_address.c_str());
  esp_now_unregister_send_cb();
  esp_now_unregister_recv_cb();
}

void setup_sensor_gpio() {
  pinMode(SENSOR_SOURCE_PIN, OUTPUT);
  digitalWrite(SENSOR_SOURCE_PIN, ON);
  if(device_type == "flame") {
    pinMode(SENSOR_DATA_PIN, INPUT);
    delay(100);
  } else if(device_type == "flood") {
    pinMode(SENSOR_DATA_PIN, INPUT);
    delay(100);
  }
}

bool is_sensor_active() {
  bool ret;
  if(device_type == "flame") {
    int value = digitalRead(SENSOR_DATA_PIN);
    ret = value?false:true;
    return ret;
  } else if(device_type == "flood") {
    int value = digitalRead(SENSOR_DATA_PIN);
    ret = value?false:true;
    return ret;
  } else return false;
}

void send_esp_now() {
  esp_now_add_peer((uint8_t *)mac_address.c_str(), ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  esp_now_register_send_cb(send_msg_cb);
  DynamicJsonDocument doc(256);
  String buffer;
  doc["from"] = "subnode";
  doc["to"] = "node";
  doc["purpose"] = "send data";
  doc["topic"] = topic;
  if(device_type == "flame") {
    doc["payload"] = "FLAME DETECTED!";
  } else if(device_type == "flood") {
    doc["payload"] = "FLOOD DETECTED!";
  }
  serializeJson(doc, buffer);
  send_result = false;
  while (send_result != true) {
    esp_now_send((uint8_t *)mac_address.c_str(), (uint8_t *)buffer.c_str(), buffer.length());
    blink_led(1);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  if(check_eeprom_pin()){
    reset_eeprom();
    blink_led(5);
    ESP.restart();
  }
  bool result = get_device_infor();
  if(!result) {
    setup_esp_now();
    waiting_infor();
  }
  setup_sensor_gpio();
  if(!is_sensor_active()) {
    blink_led(1);
    ESP.deepSleep(interval*1e6);
  }
  setup_esp_now();
  send_esp_now();
  blink_led(1);
  if( (device_type == "flame") || (device_type == "flood") ) {
    ESP.deepSleep(0);
  }
  ESP.deepSleep( interval*1e6 );
}

void loop() {
  //never get here
}


