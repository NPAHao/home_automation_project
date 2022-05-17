#include <Arduino.h>

void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.println("hello world");
  vTaskDelay(2000 / portTICK_PERIOD_MS);
}