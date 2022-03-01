// Flame sensor
// Device MAC_address: 48:3F:DA:77:CB:63
// Custom new device Mac_address: 9C:9C:1F:C6:FB:BF
// Master MAC_address: 9C:9C:1F:C6:FB:40 (Kitchen)
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>

#define FLAME_ALERT_CODE            0x01 

// Typedef
typedef enum state
{
  FLAME_DETECTED,
  WAIT_FOR_CONFIRM,
  ALERT_CONFIRMED,
  PROCESS_NOW_RECV_PACK
} state;
typedef struct send_data
{
  uint8_t alert_code;
  bool flame_alert_flag=false;
} send_data;
typedef struct recv_data
{
  uint8_t alert_code;
  bool alert_confirm=false;
} recv_data;

// Variable declare
// uint8_t new_mac_Address[] = {0x9C,0x9C,0x1F,0xC6,0xFB,0xBF};
uint8_t kitchen_Address[] = {0x9C,0x9C,0x1F,0xC7,0x20,0x8C};
send_data send_msg;
recv_data recv_msg;
volatile state device_State = FLAME_DETECTED;
uint8_t send_data_len = sizeof(send_data);
uint8_t recv_data_len = sizeof(recv_data);

// Function delare
void send_callback(uint8_t *mac_addr, uint8_t status);
void recv_callback(uint8_t * mac_add, uint8_t *data, uint8_t len);
void process_recv_pack_func();
void led_flash_setup_success();
void led_flash_setup_fail();

// Setup
void setup() {
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if(esp_now_init() !=0) 
    led_flash_setup_fail();
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_add_peer(kitchen_Address, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  esp_now_register_send_cb(send_callback);
  esp_now_register_recv_cb(recv_callback);
  led_flash_setup_success();
}

// Infinity loop
void loop() {
  switch (device_State)
  {
  case FLAME_DETECTED:
    send_msg.alert_code = FLAME_ALERT_CODE;
    send_msg.flame_alert_flag = true;
    esp_now_send(kitchen_Address,(uint8_t *) &send_msg,send_data_len);
    delay(2000);
    break;
  case WAIT_FOR_CONFIRM:
    break;
  case PROCESS_NOW_RECV_PACK:
    process_recv_pack_func();
    break;
  case ALERT_CONFIRMED:
    send_msg.flame_alert_flag = false;
    recv_msg.alert_confirm = false;
    ESP.deepSleep(0);
    break;
  }
}

// Function description
void send_callback(uint8_t *mac_addr, uint8_t status)
{
  if(status==0)
    device_State = WAIT_FOR_CONFIRM;
}
void recv_callback(uint8_t * mac_add, uint8_t *data, uint8_t len)
{
  if(memcmp(mac_add, kitchen_Address,6) == 0)
    {
      memcpy(&recv_msg,data,recv_data_len);
      device_State=PROCESS_NOW_RECV_PACK;
    }
}
void process_recv_pack_func()
{
  if( (recv_msg.alert_code == FLAME_ALERT_CODE) && recv_msg.alert_confirm )
    device_State = ALERT_CONFIRMED;
}
void led_flash_setup_success()
{
  for(int i = 0; i<3 ; i++)
  {
  digitalWrite(LED_BUILTIN,LOW);
  delay(500);
  digitalWrite(LED_BUILTIN,HIGH);
  delay(500);
  }
}
void led_flash_setup_fail()
{
  for(int i = 0; i<3 ; i++)
  {
    digitalWrite(LED_BUILTIN,LOW);
    delay(200);
    digitalWrite(LED_BUILTIN,HIGH);
    delay(200);
  }
}