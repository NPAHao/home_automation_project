#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include <espnow.h>
#include <ArduinoJson.h>
#include <StreamUtils.h>
#include <DHT.h>

#define ON                  1
#define OFF                 0
#define SEND_RETRY_TIME     10

#define ESP_NOW_MAX_DATA_LEN    250
#define ESP_NOW_ETH_LEN         6     

//define pin
#define EEPROM_PIN          14
#define SENSOR_SOURCE_PIN   12
#define SENSOR_DATA_PIN     13
#define EEPROM_USAGE        512

String node;
uint8_t node_mac[6];
String subnode;
String type;
String s_interval;
int     interval = 0;
bool    get_infor_result;
bool    send_result;
DHT dht(SENSOR_DATA_PIN, DHT11);

void send_msg_isr(u8 *mac_addr, u8 status) {
  send_result = (status==0)?true:false;
}

void infor_recv_isr(u8 *mac_addr, u8 *data, u8 len) {
  DynamicJsonDocument doc(ESP_NOW_MAX_DATA_LEN);
  if(!deserializeJson(doc, data)) {
    String object = String((const char *)doc["object"]);
    if(object == String("add subnode")) {
      doc.remove("object");
      doc["mac"] = String((const char*)mac_addr);
      EepromStream infor(0, EEPROM_USAGE);
      EEPROM.begin(EEPROM_USAGE);
      serializeJson(doc, infor);
      EEPROM.commit();
      EEPROM.end();
      get_infor_result = true;
    }
  }
}

void reset_eeprom(int eeprom_size) {
  EEPROM.begin(eeprom_size);
  for( int i = 0; i < eeprom_size; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.end();
}

void blink_led(int times, int pv_interval) {
  if(times == 0) {
    while (1)
    {
      digitalWrite(LED_BUILTIN, LOW); //sáng
      delay(pv_interval);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(pv_interval);
    }
  }
  for(int i = 0; i<times; i++) {
    digitalWrite(LED_BUILTIN, LOW); //sáng
    delay(pv_interval);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(pv_interval);
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

void get_device_infor(int eeprom_size) {
  EEPROM.begin(eeprom_size);
  DynamicJsonDocument doc(eeprom_size);
  EepromStream infor_in_eeprom(0, eeprom_size);
  DeserializationError error = deserializeJson(doc, infor_in_eeprom);
  EEPROM.end();
  if(!error) {
    type = String((const char *)doc["type"]);
    s_interval = String((const char *)doc["interval"]);
    String s_mac = String((const char *)doc["mac"]);
    memcpy(node_mac, s_mac.c_str(), ESP_NOW_ETH_LEN);
    node = String((const char *)doc["node"]);
    subnode = String((const char *)doc["subnode"]);
    interval = atoi(s_interval.c_str());
  } else {
    type = "";
    s_interval = "";
    node = "";
    subnode = "";
  }
}

void waiting_infor() {
  get_infor_result = false;
  while(!get_infor_result){
    blink_led(1, 200);
  }
  esp_now_unregister_recv_cb();
  get_device_infor(EEPROM_USAGE);
}

void config_sensor_gpio(String type) {
  pinMode(SENSOR_SOURCE_PIN, OUTPUT);
  digitalWrite(SENSOR_SOURCE_PIN, ON);
  if(type == "flame") {
    pinMode(SENSOR_DATA_PIN, INPUT);
    delay(10);
  } else if(type == "flood") {
    pinMode(SENSOR_DATA_PIN, INPUT);
    delay(10);
  } else if(type == "dht11") {
    dht.begin();
    delay(2000);
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
  } else if(type == "dht11") {
    return true;
  } else return false;
}

void send_data_msg() {
  esp_now_add_peer(node_mac, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  DynamicJsonDocument doc(250);
  String buffer;
  doc["object"] = "subnode send";
  doc["node"] = node;
  doc["type"] = type;
  if(type == "flame") {
    doc["payload"] = "FLAME DETECTED!";
  } else if(type == "flood") {
    doc["payload"] = "FLOOD DETECTED!";
  } else if(type == "dht11") {
    float humi = dht.readHumidity();
    float temp = dht.readTemperature();
    if (isnan(humi) || isnan(temp)) {
      return;
    }
    doc["payload"]["humi"] = String(humi);
    doc["payload"]["temp"] = String(temp);
  }
  serializeJson(doc, buffer);
  send_result = false;
  int retry_time = 0;
  while (send_result != true) {
    esp_now_send(node_mac, (uint8_t *)buffer.c_str(), buffer.length());
    blink_led(1, 200);
    retry_time++;
    if(retry_time >= SEND_RETRY_TIME) {
      blink_led(0, 100);
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

bool is_infor_valid() {
  bool ret = false;
  if((node == "") || (String((char *)node_mac) == "") || (type == "") || (s_interval == "") || (subnode == "")) {
    ret = false;
  } else {
    ret = true;
  }
  return ret;
}

void send_confirm_msg() {
  esp_now_add_peer(node_mac, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  DynamicJsonDocument doc(ESP_NOW_MAX_DATA_LEN);
  String buffer;
  doc["object"] = "confirm add subnode";
  doc["node"] = node;
  doc["mac"] = WiFi.macAddress();
  doc["subnode"] = subnode;
  doc["type"] = type;
  doc["interval"] = s_interval;
  serializeJson(doc, buffer);
  send_result = false;
  int retry_time = 0;
  while ((!send_result)) {
    esp_now_send(node_mac, (uint8_t *)buffer.c_str(), buffer.length());
    blink_led(1, 200);
    retry_time++;
    if(retry_time >= SEND_RETRY_TIME) {
      blink_led(0, 200);
    }
  }
}

void setup() {
  Serial.begin(115200);
  config_onboard_led(LED_BUILTIN);
  config_reset_eeprom_pin(EEPROM_PIN);
  if(digitalRead(EEPROM_PIN) == 0){
    reset_eeprom(EEPROM_USAGE);
    blink_led(5, 200);
    ESP.restart();
  }
  get_device_infor(EEPROM_USAGE);
  if(is_infor_valid() == false) {
    config_esp_now(send_msg_isr, infor_recv_isr);
    waiting_infor();
    get_device_infor(EEPROM_USAGE);
    send_confirm_msg();
    ESP.restart();
  }
  config_sensor_gpio(type);
  if(!is_sensor_active(type)) {
    blink_led(1, 200);
    ESP.deepSleep(interval*1e6);
  }
  config_esp_now(send_msg_isr, NULL);
  send_data_msg();
  blink_led(1, 200);
  if( (type == "flame") || (type == "flood") ) {
    ESP.deepSleep(0);
  }
  ESP.deepSleep(interval*1e6);
}

void loop() {
  //never get here
}


