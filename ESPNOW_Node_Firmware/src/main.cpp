#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include <espnow.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <StreamUtils.h>

#define ON                  1
#define OFF                 0

//define pin
#define EEPROM_PIN          4
#define SENSOR_SOURCE_PIN   12
#define SENSOR_DATA_PIN     13
#define EEPROM_USAGE        512
#define DHTTYPE             DHT11


uint8_t broadcast_addr[]          = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
String  device_type;
uint8_t interval; 
uint8_t last_value;
uint8_t current_value;
String  mac_address;
String  publish_topic_name;               //deepsleep interval
bool    get_infor_result;
bool    send_result;


void send_msg_cb(u8 *mac_addr, u8 status) {
  send_result = (status==0)?true:false;
}

void recv_msg_cb(u8 *mac_addr, u8 *data, u8 len) {
  String mac = String( (char *)mac_addr );
  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, data);
  if(!error) {
    String msg_type = doc["msg type"];
    if( msg_type == "incoming infor") {
      DynamicJsonDocument docTemp(512);
      docTemp["node infor"]["type"] = doc["node infor"]["type"];
      docTemp["node infor"]["interval"] = doc["node infor"]["interval"];
      docTemp["mac addr"] = mac;
      docTemp["publish topic"] = doc["publish topic"];
      EepromStream infor(0, EEPROM_USAGE);
      EEPROM.begin(EEPROM_USAGE);
      serializeJson(docTemp, infor);
      EEPROM.commit();
      EEPROM.end();
      get_infor_result = true;
    };
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

void setup_esp_now() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_now_init();
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
}

void waiting_infor() {
  esp_now_register_recv_cb(recv_msg_cb);
  get_infor_result = false;
  while (!get_infor_result) {
    blink_led(1);
  }
  esp_now_add_peer((uint8_t *)mac_address.c_str(), ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  DynamicJsonDocument doc(250);
  String str;
  doc["msg type"] = "receive success";
  serializeJson(doc, str);
  esp_now_register_send_cb(send_msg_cb);
  send_result = false;
  while (!send_result) {
    esp_now_send((uint8_t *)mac_address.c_str(),(uint8_t *)str.c_str(), str.length());
    blink_led(1);
  }
  esp_now_del_peer((uint8_t *)mac_address.c_str());
  esp_now_unregister_send_cb();
  esp_now_unregister_recv_cb();
}

bool get_device_infor() {
  EEPROM.begin(EEPROM_USAGE);
  DynamicJsonDocument doc(512);
  EepromStream infor_in_eeprom(0, EEPROM_USAGE);
  DeserializationError error = deserializeJson(doc, infor_in_eeprom);
  EEPROM.end();
  if(!error) {
    device_type = (String)doc["node infor"]["type"];
    interval = doc["node infor"]["interval"];
    last_value = doc["node infor"]["last value"];
    mac_address = (String)doc["mac addr"];
    publish_topic_name = (String)doc["publish topic"];
    //nếu type MQ-2 lấy thêm giá trị RO được lưu lần đầu tiên sau khi reset eeprom
    return true;
  } else return false;
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
  } else if(device_type == "DHT11") {
    pinMode(SENSOR_DATA_PIN, INPUT);
    delay(2000);
  } else if(device_type == "MQ-2") {
    //chờ tìm hiểu
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
  } else if(device_type == "DHT11") {
    return true;
  } else if(device_type == "MQ-2") {
    return true;
  } else return false;
}

void send_esp_now() {
  DynamicJsonDocument doc(256);
  String jsondata;
  doc["msg type"] = "msg send";
  doc["publish topic"] = publish_topic_name;
  doc["node infor"]["type"] = device_type;
  if(device_type == "flame") {
    doc["payload"]["text"] = "FLAME DETECTED!";
  } else if(device_type == "flood") {
    doc["payload"]["text"] = "FLOOD DETECTED!";
  } else if(device_type == "DHT11") {
    String temperature, huminity;
    //đọc cảm biến
    temperature = "31";
    huminity = "80";
    doc["payload"]["temp"] = temperature;
    doc["payload"]["humi"] = huminity;
  } else if(device_type == "MQ-2") {
    String co_val, lpg_val, ch4_val;
    //đọc cảm biến
    co_val = "200";
    lpg_val = "200";
    ch4_val = "200";
    doc["payload"]["CO"] = co_val;
    doc["payload"]["LPG"] = lpg_val;
    doc["payload"]["CH4"] = ch4_val;
  }
  serializeJson(doc, jsondata);
  esp_now_add_peer((uint8_t *)mac_address.c_str(), ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  esp_now_register_send_cb(send_msg_cb);
  send_result = false;
  while (send_result != true) {
    esp_now_send((uint8_t *)mac_address.c_str(), (uint8_t *)jsondata.c_str(), jsondata.length());
    delay(250);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  if(!check_eeprom_pin()){
    reset_eeprom();
    blink_led(5);
    ESP.deepSleep(0);
  }
  if(!check_infor()) {
    setup_esp_now();
    waiting_infor();
  }
  bool result = get_device_infor();
  if(!result) {
    blink_led(3);
    ESP.deepSleep(0);
  }
  setup_sensor_gpio();
  if(!is_sensor_active()) {
    blink_led(1);
    ESP.deepSleep(interval*1e6);
  }
  setup_esp_now();
  send_esp_now();
  blink_led(1);
  if( (device_type == "flame") || (device_type == "flood") ){
    ESP.deepSleep(0);
  }
  ESP.deepSleep( interval*1e6 );
}

void loop() {
  //never get here
}


