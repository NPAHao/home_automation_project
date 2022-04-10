#include <Arduino.h>
#include <ArduinoJson.h>
#include <StreamUtils.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <EEPROM.h>

#define ON                  1
#define OFF                 0

//define pin
#define EEPROM_PIN          4
#define SENSOR_SOURCE_PIN   12
#define SENSOR_DATA_PIN     13
#define EEPROM_USAGE        512

uint8_t broadcast_addr[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
const char *mac_address = nullptr;
const char *device_type = nullptr;
uint8_t interval;
const char *topic = nullptr;           
bool    get_infor_result;
bool    send_result;

void setup() {

}

void loop() {

}