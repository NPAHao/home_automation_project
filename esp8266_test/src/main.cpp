#include <Arduino.h>
#include <ArduinoJson.h>

void setup() {
  Serial.begin(115200);
  u8 *p = nullptr;
  p = (u8 *)malloc(3);
  *p = 65;
  *(p+1) = 66;
  *(p+2) = 67;
  char *p1 = (char *)p;
  String str = String(p1);
  String str1 = str;
  DynamicJsonDocument doc(256);
  String jsondata;
  doc["code"] = 1;
  doc["string"] = str1;
  serializeJson(doc, jsondata);
  Serial.println(jsondata);
}

void loop() {

}