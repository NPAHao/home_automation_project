#include <Arduino.h>

void setup() {
  Serial.begin(115200);
}

void loop() {
  // while (Serial.available() > 0)
  // {
  //   Serial.println(Serial.available());
  //   Serial.readStringUntil('\0');
  // }
  delay(2000);
  Serial.println('A', HEX);
}