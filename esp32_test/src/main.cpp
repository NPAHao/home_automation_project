#include <Arduino.h>

uint8_t str[] = "hello";

void read_task(void *para) {
  while (1) {
    if(Serial.available() > 0) {
      Serial.readStringUntil(0);
      Serial.print("Byte in buffer: ");
      Serial.println(Serial.available(), DEC);
    }
    vTaskDelay(100);
  }
}

void write_task(void *para) {
  while (1)
  {
    vTaskDelay( 1000 / portTICK_PERIOD_MS);
    Serial.write(str, sizeof(str));
  }
}

void setup() {
  Serial.begin(115200);
  xTaskCreate(read_task, "read task", 1024, NULL, 10, NULL);
  xTaskCreate(write_task, "write task", 1024, NULL, 10, NULL);
  vTaskDelete(NULL);
}

void loop() {
}